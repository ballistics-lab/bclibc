/* tiny_bclibc_wasm.c -- flat, numbers-only WebAssembly ABI over tiny_bclibc.
 *
 * Why a wrapper at all: a WebAssembly host that is *only* a JavaScript engine (Pythonista's
 * JSContext, a browser without Emscripten glue) can call exports with plain numbers, but has no
 * way to hand tiny_bclibc a C struct, a pointer to a callback, or a struct-by-pointer out-param.
 * So everything crosses the boundary through two flat `double` buffers in linear memory:
 *
 *   in   (host -> wasm): one serialized shot, laid out as described under "Input layout" below.
 *                        The host asks for it with tbw_input(n_doubles) and writes into it.
 *   out  (wasm -> host): a result header followed by rows, see "Output layout" below.
 *                        The host reads it back after a call via tbw_output().
 *
 * I/O is always `double`, whatever `real_t` is: a TINY_BCLIBC_SINGLE_PRECISION build converts
 * at this boundary, so a host binding never needs a second struct layout per precision (which
 * a ctypes binding over the shared library does have to carry).
 *
 * Build: ../build_wasm.sh (both precisions -> build/wasm/tiny_bclibc_{dp,sp}.wasm).
 * Consumer: py-ballisticcalc's examples/tiny_bclibc_wasm (a py_ballisticcalc engine that runs
 * this module in JavaScriptCore on Pythonista, or in Node on a desktop).
 *
 * The module imports nothing (no WASI, no Emscripten runtime): libm comes from the toolchain's
 * static libc, and memory for the two buffers comes from a small bump allocator over
 * `memory.grow` below -- so `new WebAssembly.Instance(module, {})` is all a host needs.
 *
 * Input layout (doubles, see TBW_IN_* below):
 *   [ 0..16]  bc, weight_grain, diameter_inch, length_inch, muzzle_velocity_fps,
 *             sight_height_ft, twist_inch, temp_c, pressure_hpa, altitude_ft, humidity,
 *             look_angle_rad, barrel_elevation_rad, barrel_azimuth_rad, cant_angle_rad,
 *             latitude_deg (NaN = no Coriolis), azimuth_deg (NaN = flat-fire Coriolis only)
 *   [17..23]  config: cStepMultiplier, cZeroFindingAccuracy, cMinimumVelocity, cMaximumDrop,
 *             cMaxIterations, cGravityConstant, cMinimumAltitude
 *   [24]      n_drag (>= 2),  [25] n_wind (>= 0)
 *   [26..]    mach[n_drag], cd[n_drag], then n_wind x
 *             (velocity_fps, direction_from_rad, until_distance_ft, max_distance_ft)
 *
 * Output layout (doubles):
 *   [0] status (TINY_BCLIBC_Status, 0 = OK)
 *   tbw_integrate:        [1] termination reason, [2] total rows, [3] n rows stored,
 *                         [4..11] final raw state (time, px, py, pz, vx, vy, vz, mach),
 *                         [12..] n x TBW_ROW rows
 *   tbw_find_zero_point:  [1] angle_rad, [2..17] one TBW_ROW row
 *   TBW_ROW row = the 15 real_t fields of TINY_BCLIBC_TrajectoryData in declaration order,
 *                 then `flag`.
 */

#define TINY_BCLIBC_NO_THREAD_LOCAL
#include "tiny_bclibc.h"

#define TBW_EXPORT(name) __attribute__((export_name(#name))) name

enum
{
    TBW_IN_HEADER = 26,
    TBW_ROW = 16,
    TBW_OUT_INTEGRATE_HEADER = 12,
    TBW_OUT_ZERO_HEADER = 2,
};

/* ── Memory: a bump allocator over memory.grow ─────────────────────────────────────────────
 * Only two buffers ever exist (in/out) and both only ever grow, so nothing is freed: a buffer
 * that outgrows its block moves to a fresh block at the top of the heap. Capacities double, so
 * the abandoned blocks add up to less than the live one. */

extern unsigned char __heap_base;
static uintptr_t tbw__heap_top = 0;

static void *tbw__alloc(size_t bytes)
{
    if (!tbw__heap_top)
        tbw__heap_top = (uintptr_t)&__heap_base;
    uintptr_t start = (tbw__heap_top + 15u) & ~(uintptr_t)15u;
    uintptr_t end = start + bytes;
    uintptr_t have = (uintptr_t)__builtin_wasm_memory_size(0) * 65536u;
    if (end > have)
    {
        size_t pages = (size_t)((end - have + 65535u) / 65536u);
        if (__builtin_wasm_memory_grow(0, pages) == (size_t)-1)
            return NULL;
    }
    tbw__heap_top = end;
    return (void *)start;
}

typedef struct
{
    double *data;
    int32_t cap; /* in doubles */
} tbw__buf;

static tbw__buf tbw__in = {0, 0};
static tbw__buf tbw__out = {0, 0};

static int tbw__reserve(tbw__buf *b, int32_t n, int keep)
{
    if (n <= b->cap)
        return 1;
    int32_t cap = b->cap ? b->cap : 256;
    while (cap < n)
        cap *= 2;
    double *p = (double *)tbw__alloc((size_t)cap * sizeof(double));
    if (!p)
        return 0;
    if (keep && b->data)
        memcpy(p, b->data, (size_t)b->cap * sizeof(double));
    b->data = p;
    b->cap = cap;
    return 1;
}

/* ── Unpacking the input buffer ──────────────────────────────────────────────────────────── */

/* Scratch for one call: tiny_bclibc_build_shot_props only stores pointers into these. */
typedef struct
{
    TINY_BCLIBC_Shot shot;
    TINY_BCLIBC_ShotProps props;
    real_t *mach;
    real_t *cd;
    TINY_BCLIBC_Wind *winds;
    TINY_BCLIBC_CurvePoint *curve;
} tbw__call;

static int32_t tbw__fail(int32_t status, const char *msg)
{
    tiny_bclibc__set_error(msg);
    if (tbw__reserve(&tbw__out, 1, 0))
        tbw__out.data[0] = (double)status;
    return status;
}

static int32_t tbw__load(tbw__call *c)
{
    const double *in = tbw__in.data;
    if (!in || tbw__in.cap < TBW_IN_HEADER)
        return tbw__fail(TINY_BCLIBC_ERR_INVALID_ARG, "tbw: input buffer not written");
    int32_t n_drag = (int32_t)in[24];
    int32_t n_wind = (int32_t)in[25];
    if (n_drag < 2 || n_wind < 0 || TBW_IN_HEADER + 2 * n_drag + 4 * n_wind > tbw__in.cap)
        return tbw__fail(TINY_BCLIBC_ERR_INVALID_ARG, "tbw: bad drag/wind counts in input buffer");

    /* Scratch lives on the heap after the in/out buffers; it is re-bumped on every call, so
     * reset the bump pointer to just past the live buffers first. */
    static uintptr_t scratch_base = 0;
    uintptr_t live_top = (uintptr_t)tbw__in.data + (uintptr_t)tbw__in.cap * sizeof(double);
    uintptr_t out_top = (uintptr_t)tbw__out.data + (uintptr_t)tbw__out.cap * sizeof(double);
    if (out_top > live_top)
        live_top = out_top;
    if (scratch_base < live_top)
        scratch_base = live_top;
    tbw__heap_top = scratch_base;

    c->mach = (real_t *)tbw__alloc((size_t)n_drag * sizeof(real_t));
    c->cd = (real_t *)tbw__alloc((size_t)n_drag * sizeof(real_t));
    c->curve = (TINY_BCLIBC_CurvePoint *)tbw__alloc((size_t)n_drag * sizeof(TINY_BCLIBC_CurvePoint));
    c->winds = (TINY_BCLIBC_Wind *)tbw__alloc((size_t)(n_wind ? n_wind : 1) * sizeof(TINY_BCLIBC_Wind));
    if (!c->mach || !c->cd || !c->curve || !c->winds)
        return tbw__fail(TINY_BCLIBC_ERR_RUNTIME, "tbw: out of memory");

    const double *mach = in + TBW_IN_HEADER;
    const double *cd = mach + n_drag;
    const double *wind = cd + n_drag;
    for (int32_t i = 0; i < n_drag; i++)
    {
        c->mach[i] = (real_t)mach[i];
        c->cd[i] = (real_t)cd[i];
    }
    for (int32_t i = 0; i < n_wind; i++)
    {
        c->winds[i].velocity_fps = (real_t)wind[4 * i + 0];
        c->winds[i].direction_from_rad = (real_t)wind[4 * i + 1];
        c->winds[i].until_distance_ft = (real_t)wind[4 * i + 2];
        c->winds[i].max_distance_ft = (real_t)wind[4 * i + 3];
    }

    TINY_BCLIBC_Shot *s = &c->shot;
    memset(s, 0, sizeof(*s));
    s->bc = (real_t)in[0];
    s->weight_grain = (real_t)in[1];
    s->diameter_inch = (real_t)in[2];
    s->length_inch = (real_t)in[3];
    s->muzzle_velocity_fps = (real_t)in[4];
    s->sight_height_ft = (real_t)in[5];
    s->twist_inch = (real_t)in[6];
    s->temp_c = (real_t)in[7];
    s->pressure_hpa = (real_t)in[8];
    s->altitude_ft = (real_t)in[9];
    s->humidity = (real_t)in[10];
    s->look_angle_rad = (real_t)in[11];
    s->barrel_elevation_rad = (real_t)in[12];
    s->barrel_azimuth_rad = (real_t)in[13];
    s->cant_angle_rad = (real_t)in[14];
    s->latitude_deg = (real_t)in[15];
    s->azimuth_deg = (real_t)in[16];
    s->config.cStepMultiplier = (real_t)in[17];
    s->config.cZeroFindingAccuracy = (real_t)in[18];
    s->config.cMinimumVelocity = (real_t)in[19];
    s->config.cMaximumDrop = (real_t)in[20];
    s->config.cMaxIterations = (int32_t)in[21];
    s->config.cGravityConstant = (real_t)in[22];
    s->config.cMinimumAltitude = (real_t)in[23];
    s->mach_data = c->mach;
    s->cd_data = c->cd;
    s->drag_table_size = n_drag;
    s->winds = c->winds;
    s->wind_count = n_wind;

    return tiny_bclibc_build_shot_props(s, c->curve, &c->props);
}

static void tbw__put_row(double *dst, const TINY_BCLIBC_TrajectoryData *p)
{
    dst[0] = p->time;
    dst[1] = p->distance_ft;
    dst[2] = p->velocity_fps;
    dst[3] = p->mach;
    dst[4] = p->height_ft;
    dst[5] = p->slant_height_ft;
    dst[6] = p->drop_angle_rad;
    dst[7] = p->windage_ft;
    dst[8] = p->windage_angle_rad;
    dst[9] = p->slant_distance_ft;
    dst[10] = p->angle_rad;
    dst[11] = p->density_ratio;
    dst[12] = p->drag;
    dst[13] = p->energy_ft_lb;
    dst[14] = p->ogw_lb;
    dst[15] = (double)p->flag;
}

/* ── Exports ─────────────────────────────────────────────────────────────────────────────── */

/* Size of real_t in bytes: 8 for a double-precision build, 4 for single. */
int32_t TBW_EXPORT(tbw_sizeof_real)(void) { return (int32_t)sizeof(real_t); }

/* Pointer to the NUL-terminated tiny_bclibc version string ("<git describe>-dp"/"-sp"). */
#ifndef TBW_VERSION
#define TBW_VERSION "unknown"
#endif
#if defined(TINY_BCLIBC_SINGLE_PRECISION)
#define TBW_VERSION_FULL TBW_VERSION "-sp"
#else
#define TBW_VERSION_FULL TBW_VERSION "-dp"
#endif
const char *TBW_EXPORT(tbw_version)(void) { return TBW_VERSION_FULL; }

/* Pointer to the NUL-terminated message for the last failed call. */
const char *TBW_EXPORT(tbw_last_error)(void) { return tiny_bclibc_last_error(); }

/* Make room for n doubles of input and return where to write them (0 = out of memory). */
double *TBW_EXPORT(tbw_input)(int32_t n)
{
    return tbw__reserve(&tbw__in, n < TBW_IN_HEADER ? TBW_IN_HEADER : n, 0) ? tbw__in.data : 0;
}

/* Where the last call's result starts (see "Output layout"). */
double *TBW_EXPORT(tbw_output)(void) { return tbw__out.data; }

/* Output length in doubles of the last call. */
static int32_t tbw__out_len = 0;
int32_t TBW_EXPORT(tbw_output_len)(void) { return tbw__out_len; }

typedef struct
{
    int32_t n;
    int32_t oom;
} tbw__stream_ctx;

static int32_t tbw__on_row(const TINY_BCLIBC_TrajectoryData *pt, void *vctx)
{
    tbw__stream_ctx *ctx = (tbw__stream_ctx *)vctx;
    int32_t need = TBW_OUT_INTEGRATE_HEADER + (ctx->n + 1) * TBW_ROW;
    if (!tbw__reserve(&tbw__out, need, 1))
    {
        ctx->oom = 1;
        return 1; /* stop */
    }
    tbw__put_row(tbw__out.data + TBW_OUT_INTEGRATE_HEADER + ctx->n * TBW_ROW, pt);
    ctx->n++;
    return 0;
}

/* tiny_bclibc_integrate_stream over the shot in the input buffer; rows land in the output. */
int32_t TBW_EXPORT(tbw_integrate)(double range_limit_ft, double range_step_ft, double time_step,
                                  int32_t filter_flags)
{
    tbw__out_len = 0;
    if (!tbw__reserve(&tbw__out, TBW_OUT_INTEGRATE_HEADER, 0))
        return TINY_BCLIBC_ERR_RUNTIME;
    tbw__call c;
    int32_t rc = tbw__load(&c);
    if (rc != TINY_BCLIBC_OK)
        return tbw__fail(rc, tiny_bclibc_last_error());

    TINY_BCLIBC_TrajectoryRequest req;
    req.range_limit_ft = (real_t)range_limit_ft;
    req.range_step_ft = (real_t)range_step_ft;
    req.time_step = (real_t)time_step;
    req.filter_flags = filter_flags;

    tbw__stream_ctx ctx = {0, 0};
    int32_t total = 0, reason = TINY_BCLIBC_TERM_NO_TERMINATE;
    TINY_BCLIBC_BaseTrajData fin;
    memset(&fin, 0, sizeof(fin));
    rc = tiny_bclibc_integrate_stream(&c.props, &req, tbw__on_row, &ctx, &total, &reason, &fin);
    if (ctx.oom)
        return tbw__fail(TINY_BCLIBC_ERR_RUNTIME, "tbw: out of memory while streaming rows");

    double *o = tbw__out.data; /* may have moved while streaming */
    o[0] = (double)rc;
    o[1] = (double)reason;
    o[2] = (double)total;
    o[3] = (double)ctx.n;
    o[4] = fin.time;
    o[5] = fin.px;
    o[6] = fin.py;
    o[7] = fin.pz;
    o[8] = fin.vx;
    o[9] = fin.vy;
    o[10] = fin.vz;
    o[11] = fin.mach;
    tbw__out_len = TBW_OUT_INTEGRATE_HEADER + ctx.n * TBW_ROW;
    return rc;
}

/* tiny_bclibc_find_zero_point over the shot in the input buffer. */
int32_t TBW_EXPORT(tbw_find_zero_point)(double distance_ft)
{
    tbw__out_len = 0;
    if (!tbw__reserve(&tbw__out, TBW_OUT_ZERO_HEADER + TBW_ROW, 0))
        return TINY_BCLIBC_ERR_RUNTIME;
    tbw__call c;
    int32_t rc = tbw__load(&c);
    if (rc != TINY_BCLIBC_OK)
        return tbw__fail(rc, tiny_bclibc_last_error());

    TINY_BCLIBC_ZeroPointResult res;
    memset(&res, 0, sizeof(res));
    rc = tiny_bclibc_find_zero_point(&c.props, (real_t)distance_ft, &res);
    double *o = tbw__out.data;
    o[0] = (double)rc;
    o[1] = res.angle_rad;
    tbw__put_row(o + TBW_OUT_ZERO_HEADER, &res.point);
    tbw__out_len = TBW_OUT_ZERO_HEADER + TBW_ROW;
    return rc;
}
