// test_hermite_single_point.cpp -- one interpolation convention for everything that works with integration steps.
//
// Every query that locates a point inside an integration step uses the cubic Hermite interpolant of that step (the
// positions and the velocities at its two ends; the velocity of the point is the derivative of the position
// polynomial, the Mach number is linear): the rows of the trajectory filter (range, time, apex, Mach, zero), find_apex,
// integrate_at / integrate_raw_at for every key, and the dense-output queries (get_at, interpolate_at,
// get_at_slant_height and their C ABI entry points). Before, the filter used it but find_apex and integrate_at (every key
// but POS_X) and the dense queries used a 3-point PCHIP of the raw points, which agrees with it for RK4 and Velocity
// Verlet but not for the wide steps of the adaptive methods and not for Euler (the apex of a vertical Euler shot was
// 6e-2 s away from the APEX row of the same trajectory). The 3-point PCHIP is only a fallback when no step brackets the
// query.
//
// Parts: (1) find_apex equals the APEX row of the filter, for the six integration methods; (2) integrate_at TIME and MACH
// equal the filter row at the same time; (3) dense get_at / get_at_slant_height on a wide adaptive step equal the
// Hermite of that step; (4) a hand-made step of known cubics, checked against the closed form through the core and the
// C ABI; (5) an Euler step with v0 > 0 > v1 (the one of a vertical shot of the Python engine) gives the Hermite apex of the
// Python engine.

#include "bclibc.hpp"
#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

using namespace bclibc;

static int failures = 0;

#define CHECK(cond)                                                                                  \
    do                                                                                               \
    {                                                                                                \
        if (!(cond))                                                                                 \
        {                                                                                            \
            std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);                   \
            ++failures;                                                                              \
        }                                                                                            \
    } while (0)

static const double kFt = 3.28084;

struct G7Point
{
    double mach, cd;
};
static const G7Point kG7[] = {
#include "g7_table.inc"
};
static const int kG7Size = static_cast<int>(sizeof kG7 / sizeof kG7[0]);
static double g_mach[sizeof kG7 / sizeof kG7[0]];
static double g_cd[sizeof kG7 / sizeof kG7[0]];

// 800 m/s, G7 0.31 (175 gr, .308, 1.24 in), 5 cm sight, the barrel elevated by elevation_deg. No minimum velocity: the
// apex of a steep shot is a slow point.
static BCLIBCFFI_Shot make_shot(int32_t method, double elevation_deg)
{
    for (int i = 0; i < kG7Size; ++i)
    {
        g_mach[i] = kG7[i].mach;
        g_cd[i] = kG7[i].cd;
    }
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = 0.31;
    s.weight_grain = 175.0;
    s.diameter_inch = 0.308;
    s.length_inch = 1.24;
    s.muzzle_velocity_fps = 800.0 * kFt;
    s.sight_height_ft = 0.05 * kFt;
    s.twist_inch = 0.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.humidity = 0.0;
    s.mach_data = g_mach;
    s.cd_data = g_cd;
    s.drag_table_size = kG7Size;
    s.barrel_elevation_rad = elevation_deg * M_PI / 180.0;
    s.latitude_deg = std::nan("");
    s.azimuth_deg = std::nan("");
    s.config.cStepMultiplier = 1.0;
    s.config.cZeroFindingAccuracy = 5e-6;
    s.config.cMinimumVelocity = 0.0;
    s.config.cMaximumDrop = -15000.0;
    s.config.cMaxIterations = 40;
    s.config.cGravityConstant = -32.17405;
    s.config.cMinimumAltitude = -1500.0;
    s.method = static_cast<BCLIBCFFI_IntegrationMethod>(method);
    return s;
}

struct Engine
{
    const char *name;
    int32_t method;
    bool adaptive;
};
static const Engine kEngines[] = {
    {"rk4", BCLIBCFFI_INTEGRATION_RK4, false},
    {"euler", BCLIBCFFI_INTEGRATION_EULER, false},
    {"verlet", BCLIBCFFI_INTEGRATION_VELOCITY_VERLET, false},
    {"rkck", BCLIBCFFI_INTEGRATION_CASH_KARP, true},
    {"dopri", BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, true},
    {"tsitouras", BCLIBCFFI_INTEGRATION_TSITOURAS, true},
};
static const double kRelTol = 1e-6, kAbsTol = 1e-6; // wide steps: the interpolation is what is under test

static BCLIBCFFI_Shot shot_of(const Engine &e, double elevation_deg)
{
    return make_shot(e.adaptive ? BCLIBCFFI_INTEGRATION_RK4 : e.method, elevation_deg);
}

static int32_t apex(const Engine &e, const BCLIBCFFI_Shot &shot, BCLIBCFFI_TrajectoryData *out, BCLIBCFFI_Error *err)
{
    return e.adaptive ? BCLIBCFFI_find_apex_adaptive_shot(&shot, e.method, kRelTol, kAbsTol, out, nullptr, err)
                      : BCLIBCFFI_find_apex_shot(&shot, out, err);
}

static int32_t at(const Engine &e, const BCLIBCFFI_Shot &shot, int32_t key, double value, BCLIBCFFI_Interception *out,
                  BCLIBCFFI_Error *err)
{
    return e.adaptive ? BCLIBCFFI_integrate_at_adaptive_shot(&shot, e.method, kRelTol, kAbsTol, key, value, out, nullptr, err)
                      : BCLIBCFFI_integrate_at_shot(&shot, key, value, out, err);
}

// The records of the trajectory filter (rows of the request), and the raw stream when want_dense.
struct Run
{
    BCLIBCFFI_TrajectoryData *records = nullptr;
    int32_t count = 0;
    BCLIBCFFI_DenseStep *dense = nullptr;
    int32_t dense_count = 0;
    ~Run()
    {
        BCLIBCFFI_free_trajectory(records);
        BCLIBCFFI_free_base_trajectory(dense);
    }
};

static int32_t run_filter(const Engine &e, const BCLIBCFFI_Shot &shot, double range_limit_ft, double time_step, int32_t flags,
                          bool want_dense, Run &r, BCLIBCFFI_Error *err)
{
    BCLIBCFFI_TrajectoryRequest req;
    std::memset(&req, 0, sizeof req);
    req.range_limit_ft = range_limit_ft;
    req.time_step = time_step;
    req.filter_flags = flags;
    int32_t reason = 0;
    if (e.adaptive)
        return BCLIBCFFI_integrate_adaptive_shot(&shot, e.method, kRelTol, kAbsTol, &req, want_dense ? 1 : 0, &r.records,
                                                 &r.count, &r.dense, &r.dense_count, &reason, nullptr, err);
    return BCLIBCFFI_integrate_dense_shot(&shot, &req, want_dense ? 1 : 0, &r.records, &r.count, &r.dense, &r.dense_count,
                                          &reason, err);
}

static bool near(double a, double b, double tol)
{
    return std::fabs(a - b) <= tol * std::fmax(1.0, std::fabs(b));
}

// ---- (1) find_apex is the APEX row of the filter.
static void test_find_apex_is_filter_row()
{
    for (const Engine &e : kEngines)
        for (double el : {5.0, 45.0, 89.99})
        {
            const BCLIBCFFI_Shot shot = shot_of(e, el);
            BCLIBCFFI_Error err;
            BCLIBCFFI_TrajectoryData a;
            std::memset(&a, 0, sizeof a);
            const int32_t rc = apex(e, shot, &a, &err);
            CHECK(rc == BCLIBCFFI_OK);
            Run r;
            const int32_t frc = run_filter(e, shot, 1e7, 0.0, BCLIBCFFI_TRAJ_FLAG_APEX, false, r, &err);
            CHECK(frc == BCLIBCFFI_OK);
            if (rc != BCLIBCFFI_OK || frc != BCLIBCFFI_OK)
                continue;
            const BCLIBCFFI_TrajectoryData *row = nullptr;
            for (int32_t i = 0; i < r.count; ++i)
                if (r.records[i].flag & BCLIBCFFI_TRAJ_FLAG_APEX)
                    row = &r.records[i];
            CHECK(row != nullptr);
            if (!row)
                continue;
            const bool same = near(a.time, row->time, 1e-9) && near(a.height_ft, row->height_ft, 1e-9) &&
                              near(a.distance_ft, row->distance_ft, 1e-9) && near(a.velocity_fps, row->velocity_fps, 1e-9);
            if (!same)
                std::fprintf(stderr, "  %-9s el %5.2f: find_apex t %.9f h %.9f x %.9f, filter row t %.9f h %.9f x %.9f\n",
                             e.name, el, a.time, a.height_ft, a.distance_ft, row->time, row->height_ft, row->distance_ft);
            CHECK(same);
        }
}

// ---- (2) integrate_at TIME and MACH equal the filter row at the same time.
static void expect_row_equals_interception(const char *what, const Engine &e, double el, const BCLIBCFFI_Interception &q,
                                           const BCLIBCFFI_TrajectoryData &row)
{
    const BCLIBCFFI_TrajectoryData &f = q.full_data;
    const bool same = near(f.time, row.time, 1e-9) && near(f.distance_ft, row.distance_ft, 1e-9) &&
                      near(f.height_ft, row.height_ft, 1e-9) && near(f.velocity_fps, row.velocity_fps, 1e-9) &&
                      near(f.mach, row.mach, 1e-9);
    if (!same)
        std::fprintf(stderr,
                     "  %s %-9s el %.1f: integrate_at t %.9f x %.9f h %.9f v %.9f mach %.9f | filter t %.9f x %.9f h %.9f v %.9f "
                     "mach %.9f\n",
                     what, e.name, el, f.time, f.distance_ft, f.height_ft, f.velocity_fps, f.mach, row.time, row.distance_ft,
                     row.height_ft, row.velocity_fps, row.mach);
    CHECK(same);
}

static const BCLIBCFFI_TrajectoryData *row_at_time(const Run &r, double t)
{
    for (int32_t i = 1; i < r.count; ++i)
        if (std::fabs(r.records[i].time - t) < 1e-9)
            return &r.records[i];
    return nullptr;
}

static void test_integrate_at_is_filter_row()
{
    for (const Engine &e : kEngines)
        for (double el : {3.0, 45.0})
        {
            const BCLIBCFFI_Shot shot = shot_of(e, el);
            BCLIBCFFI_Error err;
            // TIME: the row the filter samples at t.
            for (double t : {0.37, 1.1})
            {
                BCLIBCFFI_Interception q;
                std::memset(&q, 0, sizeof q);
                CHECK(at(e, shot, BCLIBCFFI_INTERP_KEY_TIME, t, &q, &err) == BCLIBCFFI_OK);
                Run r;
                CHECK(run_filter(e, shot, 1e7, t, BCLIBCFFI_TRAJ_FLAG_NONE, false, r, &err) == BCLIBCFFI_OK);
                const BCLIBCFFI_TrajectoryData *row = row_at_time(r, t);
                CHECK(row != nullptr);
                if (row)
                    expect_row_equals_interception("TIME", e, el, q, *row);
            }
            // MACH (the speed of sound of the point, in fps, which falls as the bullet climbs): the time at which it is
            // halfway between its value at t = 0 and at t = 1.1 s, then the row the filter samples at that time (Mach is
            // linear in time inside a step in both).
            {
                BCLIBCFFI_Interception q0, q1, q;
                std::memset(&q0, 0, sizeof q0);
                std::memset(&q1, 0, sizeof q1);
                std::memset(&q, 0, sizeof q);
                CHECK(at(e, shot, BCLIBCFFI_INTERP_KEY_TIME, 0.0, &q0, &err) == BCLIBCFFI_OK);
                CHECK(at(e, shot, BCLIBCFFI_INTERP_KEY_TIME, 1.1, &q1, &err) == BCLIBCFFI_OK);
                const double m = 0.5 * (q0.raw_data.mach + q1.raw_data.mach);
                CHECK(q1.raw_data.mach < q0.raw_data.mach);
                const int32_t rc = at(e, shot, BCLIBCFFI_INTERP_KEY_MACH, m, &q, &err);
                CHECK(rc == BCLIBCFFI_OK);
                if (rc == BCLIBCFFI_OK)
                {
                    CHECK(std::fabs(q.raw_data.mach - m) < 1e-9);
                    CHECK(q.raw_data.time > 0.0 && q.raw_data.time < 1.1);
                    Run r;
                    CHECK(run_filter(e, shot, 1e7, q.raw_data.time, BCLIBCFFI_TRAJ_FLAG_NONE, false, r, &err) == BCLIBCFFI_OK);
                    const BCLIBCFFI_TrajectoryData *row = row_at_time(r, q.raw_data.time);
                    CHECK(row != nullptr);
                    if (row)
                        expect_row_equals_interception("MACH", e, el, q, *row);
                }
            }
        }
}

// ---- (3) dense get_at on a wide adaptive step is the Hermite of the step.
struct Hermite
{
    // The Hermite state of step (a, b) at time t, written independently of the library.
    static double pos(double t, double t0, double t1, double p0, double p1, double v0, double v1)
    {
        const double h = t1 - t0, u = (t - t0) / h;
        return (2 * u * u * u - 3 * u * u + 1) * p0 + (u * u * u - 2 * u * u + u) * h * v0 +
               (-2 * u * u * u + 3 * u * u) * p1 + (u * u * u - u * u) * h * v1;
    }
    static double vel(double t, double t0, double t1, double p0, double p1, double v0, double v1)
    {
        const double h = t1 - t0, u = (t - t0) / h;
        return ((6 * u * u - 6 * u) * p0 + (-6 * u * u + 6 * u) * p1) / h + (3 * u * u - 4 * u + 1) * v0 +
               (3 * u * u - 2 * u) * v1;
    }
};

static void hermite_state(const BCLIBCFFI_DenseStep &a, const BCLIBCFFI_DenseStep &b, double t, BCLIBCFFI_DenseStep &o)
{
    o.time = t;
    o.px = Hermite::pos(t, a.time, b.time, a.px, b.px, a.vx, b.vx);
    o.py = Hermite::pos(t, a.time, b.time, a.py, b.py, a.vy, b.vy);
    o.pz = Hermite::pos(t, a.time, b.time, a.pz, b.pz, a.vz, b.vz);
    o.vx = Hermite::vel(t, a.time, b.time, a.px, b.px, a.vx, b.vx);
    o.vy = Hermite::vel(t, a.time, b.time, a.py, b.py, a.vy, b.vy);
    o.vz = Hermite::vel(t, a.time, b.time, a.pz, b.pz, a.vz, b.vz);
    o.mach = a.mach + (t - a.time) / (b.time - a.time) * (b.mach - a.mach);
}

static bool same_state(const BCLIBCFFI_DenseStep &a, const BCLIBCFFI_DenseStep &b, double tol)
{
    return near(a.time, b.time, tol) && near(a.px, b.px, tol) && near(a.py, b.py, tol) && near(a.pz, b.pz, tol) &&
           near(a.vx, b.vx, tol) && near(a.vy, b.vy, tol) && near(a.vz, b.vz, tol) && near(a.mach, b.mach, tol);
}

static void test_dense_get_at_is_hermite()
{
    for (const Engine &e : kEngines)
    {
        if (!e.adaptive)
            continue;
        const BCLIBCFFI_Shot shot = shot_of(e, 5.0);
        BCLIBCFFI_Error err;
        Run r;
        CHECK(run_filter(e, shot, 1e7, 0.0, BCLIBCFFI_TRAJ_FLAG_NONE, true, r, &err) == BCLIBCFFI_OK);
        if (r.dense_count < 4)
        {
            CHECK(false);
            continue;
        }
        // The widest accepted step on the way up, where height is monotone (the slant-height query needs it).
        int32_t apex_idx = 0;
        for (int32_t i = 1; i < r.dense_count; ++i)
            if (r.dense[i].py > r.dense[apex_idx].py)
                apex_idx = i;
        int32_t k = 1;
        for (int32_t i = 1; i + 1 <= apex_idx; ++i)
            if (r.dense[i + 1].time - r.dense[i].time > r.dense[k + 1].time - r.dense[k].time)
                k = i;
        const BCLIBCFFI_DenseStep &a = r.dense[k], &b = r.dense[k + 1];
        std::fprintf(stderr, "  %-9s %d steps, wide step %d: dt %.4f s\n", e.name, r.dense_count, k, b.time - a.time);
        CHECK(b.time - a.time > 0.05);

        // TIME
        {
            const double t = a.time + 0.43 * (b.time - a.time);
            BCLIBCFFI_DenseStep got, want;
            CHECK(BCLIBCFFI_dense_get_at(r.dense, r.dense_count, BCLIBCFFI_INTERP_KEY_TIME, t, 0.0, &got, &err) == BCLIBCFFI_OK);
            hermite_state(a, b, t, want);
            const bool ok = same_state(got, want, 1e-9);
            if (!ok)
                std::fprintf(stderr, "  %s TIME: x %.9f (hermite %.9f) y %.9f (%.9f) vy %.9f (%.9f)\n", e.name, got.px, want.px,
                             got.py, want.py, got.vy, want.vy);
            CHECK(ok);
        }
        // POS_X: the time where the Hermite x reaches the target, found by bisection here.
        {
            const double x = a.px + 0.61 * (b.px - a.px);
            double lo = a.time, hi = b.time;
            for (int i = 0; i < 100; ++i)
            {
                const double mid = 0.5 * (lo + hi);
                if (Hermite::pos(mid, a.time, b.time, a.px, b.px, a.vx, b.vx) < x)
                    lo = mid;
                else
                    hi = mid;
            }
            BCLIBCFFI_DenseStep got, want;
            CHECK(BCLIBCFFI_dense_get_at(r.dense, r.dense_count, BCLIBCFFI_INTERP_KEY_POS_X, x, 0.0, &got, &err) == BCLIBCFFI_OK);
            hermite_state(a, b, 0.5 * (lo + hi), want);
            want.px = x;
            const bool ok = same_state(got, want, 1e-9);
            if (!ok)
                std::fprintf(stderr, "  %s POS_X: t %.9f (hermite %.9f) y %.9f (%.9f) vy %.9f (%.9f)\n", e.name, got.time,
                             want.time, got.py, want.py, got.vy, want.vy);
            CHECK(ok);
        }
        // Slant height (look angle 0 is the height; the stream up to the apex is monotone in it).
        {
            const double t = a.time + 0.57 * (b.time - a.time);
            BCLIBCFFI_DenseStep want, got;
            hermite_state(a, b, t, want);
            CHECK(BCLIBCFFI_dense_get_at_slant_height(r.dense, apex_idx + 1, 0.0, want.py, &got, &err) == BCLIBCFFI_OK);
            const bool ok = same_state(got, want, 1e-9);
            if (!ok)
                std::fprintf(stderr, "  %s slant: t %.9f (hermite %.9f) x %.9f (%.9f)\n", e.name, got.time, want.time, got.px,
                             want.px);
            CHECK(ok);
        }
    }
}

// ---- (4) a hand-made step of known cubics: the Hermite reproduces them exactly.
//   px = 10 t + 2 t^2 - t^3           vx = 10 + 4 t - 3 t^2
//   py = 4 t - 4 t^2 - 5/3 t^3        vy = 4 - 8 t - 5 t^2     (zero at t = 0.4: 5 t^2 + 8 t - 4 = 0)
//   pz = 1 + t + t^2/2 + t^3/4        vz = 1 + t + 3/4 t^2
//   mach = 2 - t/2
static double G_px(double t) { return 10 * t + 2 * t * t - t * t * t; }
static double G_vx(double t) { return 10 + 4 * t - 3 * t * t; }
static double G_py(double t) { return 4 * t - 4 * t * t - 5.0 / 3.0 * t * t * t; }
static double G_vy(double t) { return 4 - 8 * t - 5 * t * t; }
static double G_pz(double t) { return 1 + t + 0.5 * t * t + 0.25 * t * t * t; }
static double G_vz(double t) { return 1 + t + 0.75 * t * t; }
static double G_mach(double t) { return 2 - 0.5 * t; }

static BCLIBC_BaseTrajData G_point(double t)
{
    return BCLIBC_BaseTrajData(t, G_px(t), G_py(t), G_pz(t), G_vx(t), G_vy(t), G_vz(t), G_mach(t));
}

// The root is found by Newton on the cubic of the step: exact to the roundoff of the step.
static const double kGoldenTol = 1e-12;

static bool is_golden_at(const BCLIBC_BaseTrajData &d, double t, double tol)
{
    return near(d.time, t, tol) && near(d.px, G_px(t), tol) && near(d.py, G_py(t), tol) && near(d.pz, G_pz(t), tol) &&
           near(d.vx, G_vx(t), tol) && near(d.vy, G_vy(t), tol) && near(d.vz, G_vz(t), tol) && near(d.mach, G_mach(t), tol);
}

struct GoldenQuery
{
    BCLIBC_BaseTrajData_InterpKey key;
    const char *name;
    double t; // the time of the point the key value belongs to
    double value;
};

static std::vector<GoldenQuery> golden_queries()
{
    return {
        {BCLIBC_BaseTrajData_InterpKey::TIME, "time", 0.3, 0.3},
        {BCLIBC_BaseTrajData_InterpKey::POS_X, "pos.x", 0.37, G_px(0.37)},
        {BCLIBC_BaseTrajData_InterpKey::POS_Z, "pos.z", 0.55, G_pz(0.55)},
        {BCLIBC_BaseTrajData_InterpKey::VEL_Y, "vel.y apex", 0.4, 0.0},
        {BCLIBC_BaseTrajData_InterpKey::VEL_Y, "vel.y", 0.65, G_vy(0.65)},
        {BCLIBC_BaseTrajData_InterpKey::MACH, "mach", 0.72, G_mach(0.72)},
    };
}

static void test_closed_form()
{
    const BCLIBC_BaseTrajData p0 = G_point(0.0), p1 = G_point(1.0), p2 = G_point(2.0);

    for (const GoldenQuery &q : golden_queries())
    {
        // The core: a sequence of three points, the query inside its first step.
        BCLIBC_BaseTrajSeq seq;
        seq.append(p0);
        seq.append(p1);
        seq.append(p2);
        BCLIBC_BaseTrajData out;
        const auto r = seq.get_at(q.key, q.value, 0.0, out);
        CHECK(!has_error(r));
        if (!has_error(r))
        {
            const bool ok = is_golden_at(out, q.t, kGoldenTol);
            if (!ok)
                std::fprintf(stderr, "  get_at %s: t %.15g (want %.15g) x %.15g (%.15g) vy %.15g (%.15g)\n", q.name, out.time,
                             q.t, out.px, G_px(q.t), out.vy, G_vy(q.t));
            CHECK(ok);
        }

        // interpolate over the three points.
        BCLIBC_BaseTrajData out3;
        const auto r3 = BCLIBC_BaseTrajData::interpolate(q.key, q.value, p0, p1, p2, out3);
        CHECK(!has_error(r3));
        if (!has_error(r3))
            CHECK(is_golden_at(out3, q.t, kGoldenTol));

        // The step handler (find_apex, integrate_at): one accepted step is enough.
        BCLIBC_TerminationReason reason = BCLIBC_TerminationReason::NO_TERMINATE;
        BCLIBC_SinglePointHandler h(q.key, q.value, &reason);
        CHECK(!has_error(h.handle(p0)));
        CHECK(!has_error(h.handle_step(p0, p1)));
        CHECK(h.found());
        if (h.found())
        {
            const auto res = h.get_result();
            CHECK(!has_error(res));
            if (!has_error(res))
                CHECK(is_golden_at(res.value().get(), q.t, kGoldenTol));
        }

        // The C ABI.
        BCLIBCFFI_BaseTrajData c0, c1, c2, co;
        auto to_c = [](const BCLIBC_BaseTrajData &d, BCLIBCFFI_BaseTrajData &c)
        { c = BCLIBCFFI_BaseTrajData{d.time, d.px, d.py, d.pz, d.vx, d.vy, d.vz, d.mach}; };
        to_c(p0, c0);
        to_c(p1, c1);
        to_c(p2, c2);
        BCLIBCFFI_Error err;
        const int32_t rc = BCLIBCFFI_base_traj_interpolate(static_cast<int32_t>(q.key), q.value, &c0, &c1, &c2, &co, &err);
        CHECK(rc == BCLIBCFFI_OK);
        if (rc == BCLIBCFFI_OK)
            CHECK(is_golden_at(BCLIBC_BaseTrajData(co.time, co.px, co.py, co.pz, co.vx, co.vy, co.vz, co.mach), q.t, kGoldenTol));
    }

    // interpolate_at(1) uses the points 0, 1, 2 -- the query in the second step (t in (1, 2)) is bracketed by (p1, p2).
    {
        BCLIBC_BaseTrajSeq seq;
        seq.append(p0);
        seq.append(p1);
        seq.append(p2);
        BCLIBC_BaseTrajData out;
        CHECK(!has_error(seq.interpolate_at(1, BCLIBC_BaseTrajData_InterpKey::TIME, 1.5, out)));
        CHECK(is_golden_at(out, 1.5, kGoldenTol));
        CHECK(!has_error(seq.interpolate_at(1, BCLIBC_BaseTrajData_InterpKey::POS_X, G_px(0.5), out)));
        CHECK(is_golden_at(out, 0.5, kGoldenTol));
    }
}

// ---- (5) the Euler step of the Python engine around the apex of a vertical shot (89.99 degrees, G7 .308 175 gr, 800 m/s,
// default step): the states are the two accepted steps py_ballisticcalc's Euler engine hands to its Hermite, whose apex
// row is t = 23.790192573917217 s, y = 12861.556433053374 ft. Euler's step is p1 = p0 + v1 dt, so a Hermite with the
// slope v0 at the start bows; the convention is the same one everywhere (the filter's APEX row has it too) and the apex it
// gives is the Python one.
static void test_euler_apex_step()
{
    const BCLIBC_BaseTrajData s0(23.769158253615814, 3.482012013673223, 12861.530952209254, 0.0, 0.10901699401662532,
                                 2.4938570153144086, 0.0, 1065.9384308658364);
    const BCLIBC_BaseTrajData s1(23.969459613096397, 3.503847672181756, 12860.73961827671, 0.0, 0.10901403048464878,
                                 -3.9507167330114834, 0.0, 1065.9416119942614);
    BCLIBC_TerminationReason reason = BCLIBC_TerminationReason::NO_TERMINATE;
    BCLIBC_SinglePointHandler h(BCLIBC_BaseTrajData_InterpKey::VEL_Y, 0.0, &reason);
    CHECK(!has_error(h.handle(s0)));
    CHECK(!has_error(h.handle_step(s0, s1)));
    CHECK(h.found());
    if (!h.found())
        return;
    const BCLIBC_BaseTrajData &a = h.get_result().value().get();
    std::fprintf(stderr, "  euler apex step: t %.9f (python 23.790192574), y %.9f (python 12861.556433053)\n", a.time, a.py);
    CHECK(std::fabs(a.time - 23.790192573917217) < 1e-9);
    CHECK(std::fabs(a.py - 12861.556433053374) < 1e-7);
    CHECK(std::fabs(a.px - 3.484305099638892) < 1e-9);
    CHECK(std::fabs(a.vx - 0.10901584721851307) < 1e-9);
    CHECK(std::fabs(a.vy) < 1e-9);
    CHECK(std::fabs(a.mach - 1065.9387649268451) < 1e-7);
}

int main()
{
    test_find_apex_is_filter_row();
    test_integrate_at_is_filter_row();
    test_dense_get_at_is_hermite();
    test_closed_form();
    test_euler_apex_step();
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_hermite_single_point: all checks passed");
    return EXIT_SUCCESS;
}
