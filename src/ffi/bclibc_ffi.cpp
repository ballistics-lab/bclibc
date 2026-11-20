/**
 * bclibc_ffi.cpp
 *
 * Thin C++ wrapper that implements the bclibc_ffi.h C API.
 * Mirrors the structure of the WASM bindings (wasm/bindings.cpp):
 *   - PCHIP curve building from a flat drag table
 *   - Engine initialisation from BCLIBCFFI_ShotProps
 *   - BCLIBC_Result → error-code conversion (has_error()/is_ok()), never a caught exception
 */

#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <functional>
#include <type_traits>
#include <vector>

#include "bclibc/base_types.hpp"
#include "bclibc/traj_data.hpp"
#include "bclibc/engine.hpp"
#include "bclibc/exceptions.hpp"
#include "bclibc/rk4.hpp"
#include "bclibc/euler.hpp"
#include "bclibc/velocity_verlet.hpp"
#include "bclibc/cash_karp.hpp"
#include "bclibc/dormand_prince.hpp"
#include "bclibc/tsitouras.hpp"
#include "bclibc/version.h" // This is the generated file

using namespace bclibc;

// ============================================================================
// Internal constants (same as WASM bindings)
// ============================================================================

static constexpr double APEX_IS_MAX_RANGE_RADIANS = 0.0003;
static constexpr double ALLOWED_ZERO_ERROR_FEET = 1e-2;

static BCLIBC_Result<BCLIBC_Curve> buildCurve(const BCLIBCFFI_DragPoint *dt, int n)
{
    if (n < 2)
        return BCLIBC_Error{BCLIBC_InvalidArgumentError{"Drag table requires at least 2 points"}};

    // 1. Prepare data (X and Y)
    std::vector<double> x(n), y(n);
    for (int i = 0; i < n; ++i)
    {
        x[i] = dt[i].Mach;
        y[i] = dt[i].CD;
    }

    // 2. Call universal build func
    return build_pchip_curve_from_arrays(x, y);
}

static BCLIBC_MachList buildMachList(const BCLIBCFFI_DragPoint *dt, int n)
{
    BCLIBC_MachList list;
    list.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
        list.push_back(dt[i].Mach);
    return list;
}

// ============================================================================
// Error helpers
// ============================================================================

static void clearError(BCLIBCFFI_Error *e)
{
    if (!e)
        return;
    e->code = BCLIBCFFI_OK;
    e->message[0] = '\0';
    e->f64_0 = e->f64_1 = e->f64_2 = 0.0;
    e->i32_0 = 0;
}

static void setError(BCLIBCFFI_Error *e, BCLIBCFFI_Status code, const char *msg)
{
    if (!e)
        return;
    e->code = static_cast<int32_t>(code);
    std::strncpy(e->message, msg, sizeof(e->message) - 1);
    e->message[sizeof(e->message) - 1] = '\0';
}

// Maps a BCLIBC_Error (returned by BCLIBC_BaseEngine instead of thrown) to an FFI
// status code, filling the per-kind diagnostic fields the same way the old
// exception-class catch blocks used to.
static int32_t setEngineError(BCLIBCFFI_Error *err, const BCLIBC_Error &error)
{
    int32_t code = BCLIBCFFI_ERR_GENERIC;
    switch (error.kind())
    {
    case BCLIBC_Error::Kind::SolverOutOfRange:
    {
        const auto *e = error.payload<BCLIBC_SolverOutOfRangeError>();
        if (err)
        {
            err->f64_0 = e->requested_distance_ft;
            err->f64_1 = e->max_range_ft;
            err->f64_2 = e->look_angle_rad;
        }
        code = BCLIBCFFI_ERR_OUT_OF_RANGE;
        break;
    }
    case BCLIBC_Error::Kind::SolverZeroFinding:
    {
        const auto *e = error.payload<BCLIBC_SolverZeroFindingError>();
        if (err)
        {
            err->f64_0 = e->zero_finding_error;
            err->f64_1 = e->last_barrel_elevation_rad;
            err->i32_0 = e->iterations_count;
        }
        code = BCLIBCFFI_ERR_ZERO_FINDING;
        break;
    }
    case BCLIBC_Error::Kind::SolverInterception:
        code = BCLIBCFFI_ERR_INTERCEPTION;
        break;
    case BCLIBC_Error::Kind::SolverRuntime:
        code = BCLIBCFFI_ERR_SOLVER_RUNTIME;
        break;
    case BCLIBC_Error::Kind::OutOfRange:
    {
        const auto *e = error.payload<BCLIBC_OutOfRangeError>();
        if (err)
        {
            err->f64_0 = e->requested;
            err->f64_1 = e->minimum;
            err->f64_2 = e->maximum;
        }
        code = BCLIBCFFI_ERR_OUT_OF_BOUNDS;
        break;
    }
    case BCLIBC_Error::Kind::Domain:
    {
        const auto *e = error.payload<BCLIBC_DomainError>();
        if (err)
        {
            err->f64_0 = e->lhs;
            err->f64_1 = e->rhs;
        }
        code = BCLIBCFFI_ERR_DOMAIN;
        break;
    }
    case BCLIBC_Error::Kind::InvalidArgument:
        code = BCLIBCFFI_ERR_INVALID_ARGUMENT;
        break;
    case BCLIBC_Error::Kind::Logic:
        code = BCLIBCFFI_ERR_LOGIC;
        break;
    case BCLIBC_Error::Kind::Runtime:
        code = BCLIBCFFI_ERR_RUNTIME;
        break;
    }
    setError(err, static_cast<BCLIBCFFI_Status>(code),
             error.what());
    return code;
}

// ============================================================================
// Engine initialisation (mirrors WASM initEngine + shotPropsFromVal)
// ============================================================================

static double calcStep(BCLIBCFFI_IntegrationMethod m, double multiplier)
{
    switch (m)
    {
    case BCLIBCFFI_INTEGRATION_RK4:
        return 0.0025 * multiplier;
    case BCLIBCFFI_INTEGRATION_VELOCITY_VERLET:
        // Velocity Verlet uses the 0.001 s base step of the Python and Cython engines
        // (VelocityVerletIntegrationEngine.DEFAULT_TIME_STEP), not RK4's 0.0025 s.
        return 0.001 * multiplier;
    case BCLIBCFFI_INTEGRATION_CASH_KARP:
        // This is the *base* step Cash-Karp starts from and can grow up to 64x
        // (or shrink down to 1/64x) via its own adaptive error control -- same
        // base as RK4's fixed step, not a separate tuning.
        return 0.0025 * multiplier;
    case BCLIBCFFI_INTEGRATION_DORMAND_PRINCE:
        return 0.0025 * multiplier;
    case BCLIBCFFI_INTEGRATION_TSITOURAS:
        return 0.0025 * multiplier;
    case BCLIBCFFI_INTEGRATION_EULER:
        return 0.5 * multiplier;
    default:
        // Unrecognized/out-of-range method value: fall back to RK4, not
        // Euler -- RK4 (enum value 0) is the library's documented default
        // and the one a zero-initialized/default-constructed caller gets.
        return 0.0025 * multiplier;
    }
}

static BCLIBC_IntegrateCallable selectIntegrateFunc(BCLIBCFFI_IntegrationMethod m)
{
    switch (m)
    {
    case BCLIBCFFI_INTEGRATION_RK4:
        return BCLIBC_IntegrateCallable(BCLIBC_integrateRK4);
    case BCLIBCFFI_INTEGRATION_VELOCITY_VERLET:
        return BCLIBC_IntegrateCallable(BCLIBC_integrateVELOCITY_VERLET);
    case BCLIBCFFI_INTEGRATION_CASH_KARP:
        return BCLIBC_IntegrateCallable(BCLIBC_integrateCashKarp);
    case BCLIBCFFI_INTEGRATION_DORMAND_PRINCE:
        return BCLIBC_IntegrateCallable(BCLIBC_integrateDormandPrince);
    case BCLIBCFFI_INTEGRATION_TSITOURAS:
        return BCLIBC_IntegrateCallable(BCLIBC_integrateTsitouras);
    case BCLIBCFFI_INTEGRATION_EULER:
        return BCLIBC_IntegrateCallable(BCLIBC_integrateEULER);
    default:
        // See calcStep's identical default: RK4, not Euler.
        return BCLIBC_IntegrateCallable(BCLIBC_integrateRK4);
    }
}

static int32_t initEngine(BCLIBC_BaseEngine &eng, const BCLIBCFFI_ShotProps *p, BCLIBCFFI_Error *err)
{
    BCLIBC_Config config(
        p->config.cStepMultiplier,
        p->config.cZeroFindingAccuracy,
        p->config.cMinimumVelocity,
        p->config.cMaximumDrop,
        p->config.cMaxIterations,
        p->config.cGravityConstant,
        p->config.cMinimumAltitude);

    BCLIBC_Atmosphere atmo(
        p->atmo.t0, p->atmo.a0, p->atmo.p0,
        p->atmo.mach, p->atmo.density_ratio, p->atmo.cLowestTempC);

    BCLIBC_Coriolis coriolis(
        p->coriolis.sin_lat, p->coriolis.cos_lat,
        p->coriolis.sin_az, p->coriolis.cos_az,
        p->coriolis.range_east, p->coriolis.range_north,
        p->coriolis.cross_east, p->coriolis.cross_north,
        p->coriolis.flat_fire_only,
        p->coriolis.muzzle_velocity_fps);

    BCLIBC_WindSock wind_sock;
    for (int i = 0; i < p->wind_count; ++i)
    {
        wind_sock.push(BCLIBC_Wind(
            p->winds[i].velocity_fps,
            p->winds[i].direction_from_rad,
            p->winds[i].until_distance_ft,
            p->winds[i].max_distance_ft));
    }
    wind_sock.update_cache();

    const auto curve_result = buildCurve(p->drag_table, p->drag_table_count);
    if (has_error(curve_result))
    {
        const auto &error = curve_result.error();
        return setEngineError(err, error);
    }
    eng.shot = BCLIBC_ShotProps(
        p->bc,
        p->look_angle_rad,
        p->twist_inch,
        p->length_inch,
        p->diameter_inch,
        p->weight_grain,
        p->barrel_elevation_rad,
        p->barrel_azimuth_rad,
        p->sight_height_ft,
        std::cos(p->cant_angle_rad),
        std::sin(p->cant_angle_rad),
        p->alt0_ft,
        calcStep(p->method, p->config.cStepMultiplier),
        p->muzzle_velocity_fps,
        0.0, // stability_coefficient (computed lazily)
        curve_result.value(),
        buildMachList(p->drag_table, p->drag_table_count),
        atmo,
        coriolis,
        wind_sock,
        BCLIBC_TRAJ_FLAG_NONE);

    eng.integrate_func = selectIntegrateFunc(p->method);
    eng.config = config;
    eng.gravity_vector = BCLIBC_V3dT(0.0, config.cGravityConstant, 0.0);
    return BCLIBCFFI_OK;
}

// Builds and initialises the engine from a BCLIBCFFI_Shot (user-facing natural units).
// mach_data/cd_data passed directly — same layout as BCLIBC_Shot, no copy needed.
// Winds are copied to BCLIBC_Wind; lifetime covers the to_shot_props() call.
static int32_t initEngineFromShot(BCLIBC_BaseEngine &eng, const BCLIBCFFI_Shot *s, BCLIBCFFI_Error *err)
{
    // Copy BCLIBCFFI_Wind → BCLIBC_Wind; lifetime covers to_shot_props() call.
    std::vector<BCLIBC_Wind> winds_v;
    winds_v.reserve(static_cast<size_t>(s->wind_count));
    for (int i = 0; i < s->wind_count; ++i)
        winds_v.emplace_back(
            s->winds[i].velocity_fps,
            s->winds[i].direction_from_rad,
            s->winds[i].until_distance_ft,
            s->winds[i].max_distance_ft);

    BCLIBC_Shot shot;
    shot.bc = s->bc;
    shot.weight_grain = s->weight_grain;
    shot.diameter_inch = s->diameter_inch;
    shot.length_inch = s->length_inch;
    shot.muzzle_velocity_fps = s->muzzle_velocity_fps;
    shot.stability_coefficient = 0.0;
    shot.sight_height_ft = s->sight_height_ft;
    shot.twist_inch = s->twist_inch;
    shot.temp_c = s->temp_c;
    shot.pressure_hpa = s->pressure_hpa;
    shot.altitude_ft = s->altitude_ft;
    shot.humidity = s->humidity;
    shot.mach_data = s->mach_data;
    shot.cd_data = s->cd_data;
    shot.drag_table_size = s->drag_table_size;
    shot.winds = winds_v.empty() ? nullptr : winds_v.data();
    shot.wind_count = s->wind_count;
    shot.look_angle_rad = s->look_angle_rad;
    shot.barrel_elevation_rad = s->barrel_elevation_rad;
    shot.barrel_azimuth_rad = s->barrel_azimuth_rad;
    shot.cant_angle_rad = s->cant_angle_rad;
    shot.latitude_deg = s->latitude_deg;
    shot.azimuth_deg = s->azimuth_deg;
    shot.calc_step = calcStep(s->method, s->config.cStepMultiplier);

    const auto props_result = shot.to_shot_props();
    if (has_error(props_result))
    {
        const auto &error = props_result.error();
        return setEngineError(err, error);
    }
    eng.shot = props_result.value();
    eng.integrate_func = selectIntegrateFunc(s->method);
    eng.config = BCLIBC_Config(
        s->config.cStepMultiplier,
        s->config.cZeroFindingAccuracy,
        s->config.cMinimumVelocity,
        s->config.cMaximumDrop,
        s->config.cMaxIterations,
        s->config.cGravityConstant,
        s->config.cMinimumAltitude);
    eng.gravity_vector = BCLIBC_V3dT(0.0, eng.config.cGravityConstant, 0.0);
    return BCLIBCFFI_OK;
}

// ============================================================================
// Output conversions
// ============================================================================

static void toC(const BCLIBC_TrajectoryData &s, BCLIBCFFI_TrajectoryData &d)
{
    d.time = s.time;
    d.distance_ft = s.distance_ft;
    d.velocity_fps = s.velocity_fps;
    d.mach = s.mach;
    d.height_ft = s.height_ft;
    d.slant_height_ft = s.slant_height_ft;
    d.drop_angle_rad = s.drop_angle_rad;
    d.windage_ft = s.windage_ft;
    d.windage_angle_rad = s.windage_angle_rad;
    d.slant_distance_ft = s.slant_distance_ft;
    d.angle_rad = s.angle_rad;
    d.density_ratio = s.density_ratio;
    d.drag = s.drag;
    d.energy_ft_lb = s.energy_ft_lb;
    d.ogw_lb = s.ogw_lb;
    d.flag = static_cast<int32_t>(s.flag);
}

static BCLIBC_TrajectoryData fromC(const BCLIBCFFI_TrajectoryData &s)
{
    BCLIBC_TrajectoryData d;
    d.time = s.time;
    d.distance_ft = s.distance_ft;
    d.velocity_fps = s.velocity_fps;
    d.mach = s.mach;
    d.height_ft = s.height_ft;
    d.slant_height_ft = s.slant_height_ft;
    d.drop_angle_rad = s.drop_angle_rad;
    d.windage_ft = s.windage_ft;
    d.windage_angle_rad = s.windage_angle_rad;
    d.slant_distance_ft = s.slant_distance_ft;
    d.angle_rad = s.angle_rad;
    d.density_ratio = s.density_ratio;
    d.drag = s.drag;
    d.energy_ft_lb = s.energy_ft_lb;
    d.ogw_lb = s.ogw_lb;
    d.flag = static_cast<BCLIBC_TrajFlag>(s.flag);
    return d;
}

static void toC(const BCLIBC_BaseTrajData &s, BCLIBCFFI_BaseTrajData &d)
{
    d.time = s.time;
    d.px = s.px;
    d.py = s.py;
    d.pz = s.pz;
    d.vx = s.vx;
    d.vy = s.vy;
    d.vz = s.vz;
    d.mach = s.mach;
}

static void toC(const BCLIBC_ZeroPointResult &s, BCLIBCFFI_ZeroPointResult &d)
{
    d.angle_rad = s.angle_rad;
    toC(s.point, d.point);
}

// ============================================================================
// Error wrapper (replaces BCLIBCFFI_CATCH macro)
// ============================================================================

// Nothing in bclibc throws: every fallible call returns a Result, checked with has_error()/is_ok()
// at its call site (see setEngineError above and the has_error() checks throughout this file).
// This wrapper only clears *err before dispatch; it is kept so every entry point has one place to
// change if that ever needs to do more. C++11 compatible: lambda with -> int32_t trailing return type.
template <typename Func>
static int32_t ffi_call(Func &&fn, BCLIBCFFI_Error *err) noexcept
{
    clearError(err);
    return fn();
}

// ============================================================================
// Adaptive integrators: one integrator per call, installed by reference
// ============================================================================

// Runs fn(eng, integrator) with a fresh stateful integrator at the given tolerances. The integrator is
// installed as the engine's integrate_func by reference; its step counts are read by the caller.
template <class Integ, class Fn>
static int32_t runAdaptive(const BCLIBCFFI_Shot *shot, double rtol, double atol, BCLIBCFFI_Error *err, Fn &&fn)
{
    Integ integ;
    const auto r = integ.set_relative_tolerance(rtol);
    if (has_error(r)) return setEngineError(err, r.error());
    const auto a = integ.set_absolute_tolerance(atol);
    if (has_error(a)) return setEngineError(err, a.error());
    BCLIBC_BaseEngine eng;
    if (const auto status = initEngineFromShot(eng, shot, err); status != BCLIBCFFI_OK) return status;
    eng.integrate_func = BCLIBC_IntegrateCallable(std::ref(integ));
    return fn(eng, integ);
}

template <class Fn>
static int32_t dispatchAdaptive(int32_t method, const BCLIBCFFI_Shot *shot, double rtol, double atol,
                                BCLIBCFFI_Error *err, Fn &&fn)
{
    switch (method)
    {
    case BCLIBCFFI_INTEGRATION_CASH_KARP:
        return runAdaptive<BCLIBC_CashKarpIntegrator>(shot, rtol, atol, err, fn);
    case BCLIBCFFI_INTEGRATION_DORMAND_PRINCE:
        return runAdaptive<BCLIBC_DormandPrinceIntegrator>(shot, rtol, atol, err, fn);
    case BCLIBCFFI_INTEGRATION_TSITOURAS:
        return runAdaptive<BCLIBC_TsitourasIntegrator>(shot, rtol, atol, err, fn);
    default:
        setError(err, BCLIBCFFI_ERR_GENERIC, "adaptive method required (cash-karp, dormand-prince, tsitouras)");
        return BCLIBCFFI_ERR_GENERIC;
    }
}

// Writes the step counts of a finished (or failed) adaptive run to out_stats, when given.
template <class Integ>
static void storeStats(const Integ &integ, BCLIBCFFI_StepStats *out_stats)
{
    if (!out_stats) return;
    int accepted = 0, rejected = 0;
    integ.get_stats(accepted, rejected);
    out_stats->accepted = accepted;
    out_stats->rejected = rejected;
}

// Integrates on eng and writes the filtered records and (when want_dense != 0) the accepted-step stream to the
// out_ pointers. Shared by the fixed-step and the adaptive entry points, so both return the same shapes.
static int32_t writeIntegration(BCLIBC_BaseEngine &eng, const BCLIBCFFI_TrajectoryRequest *request,
                                int32_t want_dense, BCLIBCFFI_TrajectoryData **out_records, int32_t *out_count,
                                BCLIBCFFI_DenseStep **out_dense, int32_t *out_dense_count, int32_t *out_reason,
                                BCLIBCFFI_Error *err)
{
    std::vector<BCLIBC_TrajectoryData> records;
    BCLIBC_TerminationReason reason;
    BCLIBC_BaseTrajSeq dense;
    // Leave the outputs defined on failure so a caller that frees them unconditionally is safe.
    *out_records = nullptr;
    *out_count = 0;
    *out_reason = 0;
    const auto res = eng.integrate_filtered(
        request->range_limit_ft, request->range_step_ft, request->time_step,
        static_cast<BCLIBC_TrajFlag>(request->filter_flags),
        records, reason, want_dense ? &dense : nullptr);
    if (has_error(res)) return setEngineError(err, res.error());

    const auto count = static_cast<int32_t>(records.size());
    BCLIBCFFI_TrajectoryData *arr = nullptr;
    if (count > 0)
    {
        arr = static_cast<BCLIBCFFI_TrajectoryData *>(
            std::malloc(sizeof(BCLIBCFFI_TrajectoryData) * static_cast<size_t>(count)));
        if (!arr)
        {
            setError(err, BCLIBCFFI_ERR_GENERIC, "Out of memory allocating trajectory");
            return BCLIBCFFI_ERR_GENERIC;
        }
        for (int32_t i = 0; i < count; ++i) toC(records[i], arr[i]);
    }
    *out_records = arr;
    *out_count = count;
    *out_reason = static_cast<int32_t>(reason);

    if (want_dense && out_dense)
    {
        const auto n = static_cast<int32_t>(dense.get_length());
        BCLIBCFFI_DenseStep *darr = nullptr;
        if (n > 0)
        {
            darr = static_cast<BCLIBCFFI_DenseStep *>(
                std::malloc(sizeof(BCLIBCFFI_DenseStep) * static_cast<size_t>(n)));
            if (!darr)
            {
                std::free(arr);
                setError(err, BCLIBCFFI_ERR_GENERIC, "Out of memory allocating dense output");
                return BCLIBCFFI_ERR_GENERIC;
            }
            for (int32_t i = 0; i < n; ++i)
            {
                const auto item = dense[i];
                if (has_error(item))
                {
                    std::free(darr);
                    std::free(arr);
                    return setEngineError(err, item.error());
                }
                toC(item.value().get(), darr[i]);
            }
        }
        *out_dense = darr;
        *out_dense_count = n;
    }
    return BCLIBCFFI_OK;
}

static bool isAdaptiveMethod(int32_t method)
{
    return method == BCLIBCFFI_INTEGRATION_CASH_KARP || method == BCLIBCFFI_INTEGRATION_DORMAND_PRINCE ||
           method == BCLIBCFFI_INTEGRATION_TSITOURAS;
}

// ============================================================================
// Solver building blocks and dense-output helpers (shared by the fixed-step and adaptive entry points)
// ============================================================================

static BCLIBC_BaseTrajData fromC(const BCLIBCFFI_BaseTrajData &s)
{
    return BCLIBC_BaseTrajData(s.time, s.px, s.py, s.pz, s.vx, s.vy, s.vz, s.mach);
}

// Builds the engine for a fixed-step call (shot->method) and runs fn(eng).
template <class Fn>
static int32_t runFixed(const BCLIBCFFI_Shot *shot, BCLIBCFFI_Error *err, Fn &&fn)
{
    BCLIBC_BaseEngine eng;
    if (const auto status = initEngineFromShot(eng, shot, err); status != BCLIBCFFI_OK) return status;
    return fn(eng);
}

static int32_t opErrorAtDistance(BCLIBC_BaseEngine &eng, double angle_rad, double x_ft, double y_ft,
                                 double *out_error, BCLIBCFFI_BaseTrajData *out_hit, BCLIBCFFI_Error *err)
{
    BCLIBC_BaseTrajData hit;
    const auto e = eng.error_at_distance(angle_rad, x_ft, y_ft, out_hit ? &hit : nullptr);
    if (has_error(e)) return setEngineError(err, e.error());
    *out_error = e.value();
    if (out_hit) toC(hit, *out_hit);
    return BCLIBCFFI_OK;
}

static int32_t opRangeForAngle(BCLIBC_BaseEngine &eng, double angle_rad, double *out_range)
{
    *out_range = eng.range_for_angle(angle_rad);
    return BCLIBCFFI_OK;
}

static int32_t opZeroNewton(BCLIBC_BaseEngine &eng, double distance_ft, double *out_angle, BCLIBCFFI_Error *err)
{
    const auto angle = eng.zero_angle_newton(distance_ft, APEX_IS_MAX_RANGE_RADIANS, ALLOWED_ZERO_ERROR_FEET);
    if (has_error(angle)) return setEngineError(err, angle.error());
    *out_angle = angle.value();
    return BCLIBCFFI_OK;
}

static int32_t opZeroInit(BCLIBC_BaseEngine &eng, double distance_ft, BCLIBCFFI_ZeroInit *out, BCLIBCFFI_Error *err)
{
    BCLIBC_ZeroInitialData data;
    const auto r = eng.init_zero_calculation(distance_ft, APEX_IS_MAX_RANGE_RADIANS, ALLOWED_ZERO_ERROR_FEET, data);
    if (has_error(r)) return setEngineError(err, r.error());
    out->status = data.status == BCLIBC_ZeroInitialStatus::DONE ? BCLIBCFFI_ZERO_INIT_DONE
                                                                : BCLIBCFFI_ZERO_INIT_CONTINUE;
    out->reserved = 0;
    out->look_angle_rad = data.look_angle_rad;
    out->slant_range_ft = data.slant_range_ft;
    out->target_x_ft = data.target_x_ft;
    out->target_y_ft = data.target_y_ft;
    out->start_height_ft = data.start_height_ft;
    return BCLIBCFFI_OK;
}

// Copies a C array of dense steps into a sequence the interpolators can search.
static BCLIBC_BaseTrajSeq denseSeq(const BCLIBCFFI_DenseStep *steps, int32_t count)
{
    BCLIBC_BaseTrajSeq seq;
    for (int32_t i = 0; i < count; ++i) seq.append(fromC(steps[i]));
    return seq;
}

// ============================================================================
// Public C API
// ============================================================================

extern "C"
{

    const char *BCLIBCFFI_get_version()
    {
        return BCLIBC_VERSION;
    }

    int32_t BCLIBCFFI_find_apex(
        const BCLIBCFFI_ShotProps *props,
        BCLIBCFFI_TrajectoryData *out,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngine(eng, props, err); status != BCLIBCFFI_OK) return status;
            BCLIBC_BaseTrajData apex;
            const auto apex_result = eng.find_apex(apex);
            if (has_error(apex_result)) return setEngineError(err, apex_result.error());
            auto traj_result = BCLIBC_TrajectoryData::from_base(eng.shot, apex, BCLIBC_TRAJ_FLAG_APEX);
            if (has_error(traj_result))
            {
                const auto &error = traj_result.error();
                return setEngineError(err, error);
            }
            toC(traj_result.value(), *out);
            return BCLIBCFFI_OK; }, err);
    }

    int32_t BCLIBCFFI_find_max_range(
        const BCLIBCFFI_ShotProps *props,
        double low_angle_deg,
        double high_angle_deg,
        BCLIBCFFI_MaxRangeResult *out,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngine(eng, props, err); status != BCLIBCFFI_OK) return status;
            const auto max_range_result = eng.find_max_range(
                low_angle_deg, high_angle_deg, APEX_IS_MAX_RANGE_RADIANS);
            if (has_error(max_range_result)) return setEngineError(err, max_range_result.error());
            const auto &r = max_range_result.value();
            out->max_range_ft = r.max_range_ft;
            out->angle_at_max_rad = r.angle_at_max_rad;
            return BCLIBCFFI_OK; }, err);
    }

    int32_t BCLIBCFFI_find_zero_angle(
        const BCLIBCFFI_ShotProps *props,
        double distance_ft,
        double *out_angle_rad,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngine(eng, props, err); status != BCLIBCFFI_OK) return status;
            const auto angle_result = eng.zero_angle_with_fallback(
                distance_ft, APEX_IS_MAX_RANGE_RADIANS, ALLOWED_ZERO_ERROR_FEET);
            if (has_error(angle_result)) return setEngineError(err, angle_result.error());
            *out_angle_rad = angle_result.value();
            return BCLIBCFFI_OK; }, err);
    }

    BCLIBC_API int32_t BCLIBCFFI_find_zero_point(
        const BCLIBCFFI_ShotProps *props,
        double distance_ft,
        BCLIBCFFI_ZeroPointResult *out,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngine(eng, props, err); status != BCLIBCFFI_OK) return status;
            const auto point_result = eng.zero_point_with_fallback(
                distance_ft, APEX_IS_MAX_RANGE_RADIANS, ALLOWED_ZERO_ERROR_FEET);
            if (has_error(point_result)) return setEngineError(err, point_result.error());
            toC(point_result.value(), *out);
            return BCLIBCFFI_OK; }, err);
    }

    int32_t BCLIBCFFI_integrate(
        const BCLIBCFFI_ShotProps *props,
        const BCLIBCFFI_TrajectoryRequest *request,
        BCLIBCFFI_TrajectoryData **out_records,
        int32_t *out_count,
        int32_t *out_reason,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngine(eng, props, err); status != BCLIBCFFI_OK) return status;

            std::vector<BCLIBC_TrajectoryData> records;
            BCLIBC_TerminationReason reason;

            eng.integrate_filtered(
                request->range_limit_ft,
                request->range_step_ft,
                request->time_step,
                static_cast<BCLIBC_TrajFlag>(request->filter_flags),
                records,
                reason,
                nullptr);

            auto count = static_cast<int32_t>(records.size());
            BCLIBCFFI_TrajectoryData *arr = nullptr;
            if (count > 0)
            {
                arr = static_cast<BCLIBCFFI_TrajectoryData *>(
                    std::malloc(sizeof(BCLIBCFFI_TrajectoryData) * static_cast<size_t>(count)));
                if (!arr)
                {
                    setError(err, BCLIBCFFI_ERR_GENERIC, "Out of memory allocating trajectory");
                    return BCLIBCFFI_ERR_GENERIC;
                }
                for (int32_t i = 0; i < count; ++i)
                    toC(records[i], arr[i]);
            }

            *out_records = arr;
            *out_count = count;
            *out_reason = static_cast<int32_t>(reason);
            return BCLIBCFFI_OK; }, err);
    }

    void BCLIBCFFI_free_trajectory(BCLIBCFFI_TrajectoryData *records)
    {
        std::free(records);
    }

    int32_t BCLIBCFFI_integrate_at(
        const BCLIBCFFI_ShotProps *props,
        int32_t key,
        double target_value,
        BCLIBCFFI_Interception *out,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngine(eng, props, err); status != BCLIBCFFI_OK) return status;

            BCLIBC_BaseTrajData raw;
            BCLIBC_TrajectoryData full;
            const auto integrate_at_result = eng.integrate_at(
                static_cast<BCLIBC_BaseTrajData_InterpKey>(key),
                target_value, raw, full);
            if (has_error(integrate_at_result)) return setEngineError(err, integrate_at_result.error());

            toC(raw, out->raw_data);
            toC(full, out->full_data);
            return BCLIBCFFI_OK; }, err);
    }

    double BCLIBCFFI_get_correction(double distance_ft, double offset_ft)
    {
        return BCLIBC_getCorrection(distance_ft, offset_ft);
    }

    double BCLIBCFFI_calculate_energy(double bullet_weight_grain, double velocity_fps)
    {
        return BCLIBC_calculateEnergy(bullet_weight_grain, velocity_fps);
    }

    double BCLIBCFFI_calculate_ogw(double bullet_weight_grain, double velocity_fps)
    {
        return BCLIBC_calculateOgw(bullet_weight_grain, velocity_fps);
    }

    double BCLIBCFFI_hermite(double x, double xk, double xk1, double yk, double yk1, double mk, double mk1)
    {
        return BCLIBC_hermite(x, xk, xk1, yk, yk1, mk, mk1);
    }

    double BCLIBCFFI_interpolate_3pt(double x, double x0, double x1, double x2, double y0, double y1, double y2)
    {
        return BCLIBC_interpolate3pt(x, x0, x1, x2, y0, y1, y2);
    }

    int32_t BCLIBCFFI_interpolate_2pt(double x, double x0, double y0, double x1, double y1, double *out, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            if (!out)
            {
                setError(err, BCLIBCFFI_ERR_GENERIC, "interpolate_2pt: a null output");
                return BCLIBCFFI_ERR_GENERIC;
            }
            if (BCLIBC_interpolate2pt(x, x0, y0, x1, y1, *out) != BCLIBC_InterpStatus::SUCCESS)
            {
                setError(err, BCLIBCFFI_ERR_GENERIC, "Zero division error during interpolation");
                return BCLIBCFFI_ERR_GENERIC;
            }
            return BCLIBCFFI_OK; },
                        err);
    }

    int32_t BCLIBCFFI_interpolate_trajectory_data(
        int32_t key,
        double value,
        const BCLIBCFFI_TrajectoryData *t0,
        const BCLIBCFFI_TrajectoryData *t1,
        const BCLIBCFFI_TrajectoryData *t2,
        int32_t flag,
        int32_t method,
        BCLIBCFFI_TrajectoryData *out,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            if (!t0 || !t1 || !t2 || !out)
            {
                setError(err, BCLIBCFFI_ERR_GENERIC, "interpolate_trajectory_data: a null point");
                return BCLIBCFFI_ERR_GENERIC;
            }
            if (key < 0 || key > static_cast<int32_t>(BCLIBC_TrajectoryData_InterpKey::FLAG))
            {
                setError(err, BCLIBCFFI_ERR_GENERIC, "interpolate_trajectory_data: unknown key");
                return BCLIBCFFI_ERR_GENERIC;
            }
            if (method != 0 && method != 1)
            {
                setError(err, BCLIBCFFI_ERR_GENERIC, "interpolate_trajectory_data: unknown method");
                return BCLIBCFFI_ERR_GENERIC;
            }
            const auto result = BCLIBC_TrajectoryData::interpolate(
                static_cast<BCLIBC_TrajectoryData_InterpKey>(key),
                value,
                fromC(*t0),
                fromC(*t1),
                fromC(*t2),
                static_cast<BCLIBC_TrajFlag>(flag),
                static_cast<BCLIBC_InterpMethod>(method));
            if (has_error(result))
            {
                return setEngineError(err, result.error());
            }
            toC(result.value(), *out);
            return BCLIBCFFI_OK; },
                        err);
    }

    // ============================================================================
    // BCLIBCFFI_Shot variants – all physics conversion in C++ via BCLIBC_Shot::to_shot_props()
    // ============================================================================

    int32_t BCLIBCFFI_find_apex_shot(
        const BCLIBCFFI_Shot *shot,
        BCLIBCFFI_TrajectoryData *out,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngineFromShot(eng, shot, err); status != BCLIBCFFI_OK) return status;
            BCLIBC_BaseTrajData apex;
            const auto apex_result = eng.find_apex(apex);
            if (has_error(apex_result)) return setEngineError(err, apex_result.error());
            auto traj_result = BCLIBC_TrajectoryData::from_base(eng.shot, apex, BCLIBC_TRAJ_FLAG_APEX);
            if (has_error(traj_result))
            {
                const auto &error = traj_result.error();
                return setEngineError(err, error);
            }
            toC(traj_result.value(), *out);
            return BCLIBCFFI_OK; }, err);
    }

    int32_t BCLIBCFFI_find_max_range_shot(
        const BCLIBCFFI_Shot *shot,
        double low_angle_deg,
        double high_angle_deg,
        BCLIBCFFI_MaxRangeResult *out,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngineFromShot(eng, shot, err); status != BCLIBCFFI_OK) return status;
            const auto max_range_result = eng.find_max_range(
                low_angle_deg, high_angle_deg, APEX_IS_MAX_RANGE_RADIANS);
            if (has_error(max_range_result)) return setEngineError(err, max_range_result.error());
            const auto &r = max_range_result.value();
            out->max_range_ft = r.max_range_ft;
            out->angle_at_max_rad = r.angle_at_max_rad;
            return BCLIBCFFI_OK; }, err);
    }

    int32_t BCLIBCFFI_find_zero_angle_shot(
        const BCLIBCFFI_Shot *shot,
        double distance_ft,
        double *out_angle_rad,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngineFromShot(eng, shot, err); status != BCLIBCFFI_OK) return status;
            const auto angle_result = eng.zero_angle_with_fallback(
                distance_ft, APEX_IS_MAX_RANGE_RADIANS, ALLOWED_ZERO_ERROR_FEET);
            if (has_error(angle_result)) return setEngineError(err, angle_result.error());
            *out_angle_rad = angle_result.value();
            return BCLIBCFFI_OK; }, err);
    }

    BCLIBC_API int32_t BCLIBCFFI_find_zero_point_shot(
        const BCLIBCFFI_Shot *shot,
        double distance_ft,
        BCLIBCFFI_ZeroPointResult *out,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngineFromShot(eng, shot, err); status != BCLIBCFFI_OK) return status;
            const auto point_result = eng.zero_point_with_fallback(
                distance_ft, APEX_IS_MAX_RANGE_RADIANS, ALLOWED_ZERO_ERROR_FEET);
            if (has_error(point_result)) return setEngineError(err, point_result.error());
            toC(point_result.value(), *out);
            return BCLIBCFFI_OK; }, err);
    }

    int32_t BCLIBCFFI_integrate_shot(
        const BCLIBCFFI_Shot *shot,
        const BCLIBCFFI_TrajectoryRequest *request,
        BCLIBCFFI_TrajectoryData **out_records,
        int32_t *out_count,
        int32_t *out_reason,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngineFromShot(eng, shot, err); status != BCLIBCFFI_OK) return status;

            std::vector<BCLIBC_TrajectoryData> records;
            BCLIBC_TerminationReason reason;

            eng.integrate_filtered(
                request->range_limit_ft,
                request->range_step_ft,
                request->time_step,
                static_cast<BCLIBC_TrajFlag>(request->filter_flags),
                records,
                reason,
                nullptr);

            auto count = static_cast<int32_t>(records.size());
            BCLIBCFFI_TrajectoryData *arr = nullptr;
            if (count > 0)
            {
                arr = static_cast<BCLIBCFFI_TrajectoryData *>(
                    std::malloc(sizeof(BCLIBCFFI_TrajectoryData) * static_cast<size_t>(count)));
                if (!arr)
                {
                    setError(err, BCLIBCFFI_ERR_GENERIC, "Out of memory allocating trajectory");
                    return BCLIBCFFI_ERR_GENERIC;
                }
                for (int32_t i = 0; i < count; ++i)
                    toC(records[i], arr[i]);
            }

            *out_records = arr;
            *out_count = count;
            *out_reason = static_cast<int32_t>(reason);
            return BCLIBCFFI_OK; }, err);
    }

    int32_t BCLIBCFFI_integrate_at_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t key,
        double target_value,
        BCLIBCFFI_Interception *out,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngineFromShot(eng, shot, err); status != BCLIBCFFI_OK) return status;

            BCLIBC_BaseTrajData raw;
            BCLIBC_TrajectoryData full;
            const auto integrate_at_result = eng.integrate_at(
                static_cast<BCLIBC_BaseTrajData_InterpKey>(key),
                target_value, raw, full);
            if (has_error(integrate_at_result)) return setEngineError(err, integrate_at_result.error());

            toC(raw, out->raw_data);
            toC(full, out->full_data);
            return BCLIBCFFI_OK; }, err);
    }

    int32_t BCLIBCFFI_integrate_adaptive_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t method,
        double relative_tolerance,
        double absolute_tolerance,
        const BCLIBCFFI_TrajectoryRequest *request,
        int32_t want_dense,
        BCLIBCFFI_TrajectoryData **out_records,
        int32_t *out_count,
        BCLIBCFFI_DenseStep **out_dense,
        int32_t *out_dense_count,
        int32_t *out_reason,
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err)
    {
        if (out_dense) *out_dense = nullptr;
        if (out_dense_count) *out_dense_count = 0;
        return ffi_call([&]() -> int32_t
                        { return dispatchAdaptive(method, shot, relative_tolerance, absolute_tolerance, err,
                                                  [&](BCLIBC_BaseEngine &eng, auto &integ) -> int32_t
                                                  {
                              const int32_t st = writeIntegration(eng, request, want_dense, out_records, out_count,
                                                                  out_dense, out_dense_count, out_reason, err);
                              storeStats(integ, out_stats);
                              return st; }); }, err);
    }

    int32_t BCLIBCFFI_integrate_dense_shot(
        const BCLIBCFFI_Shot *shot,
        const BCLIBCFFI_TrajectoryRequest *request,
        int32_t want_dense,
        BCLIBCFFI_TrajectoryData **out_records,
        int32_t *out_count,
        BCLIBCFFI_DenseStep **out_dense,
        int32_t *out_dense_count,
        int32_t *out_reason,
        BCLIBCFFI_Error *err)
    {
        if (out_dense) *out_dense = nullptr;
        if (out_dense_count) *out_dense_count = 0;
        return ffi_call([&]() -> int32_t
                        {
            if (isAdaptiveMethod(shot->method))
            {
                setError(err, BCLIBCFFI_ERR_GENERIC, "adaptive method: use BCLIBCFFI_integrate_adaptive_shot");
                return BCLIBCFFI_ERR_GENERIC;
            }
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngineFromShot(eng, shot, err); status != BCLIBCFFI_OK) return status;
            return writeIntegration(eng, request, want_dense, out_records, out_count, out_dense, out_dense_count,
                                    out_reason, err); }, err);
    }

    int32_t BCLIBCFFI_integrate_dense_shot_stats(
        const BCLIBCFFI_Shot *shot,
        const BCLIBCFFI_TrajectoryRequest *request,
        int32_t want_dense,
        BCLIBCFFI_TrajectoryData **out_records,
        int32_t *out_count,
        BCLIBCFFI_DenseStep **out_dense,
        int32_t *out_dense_count,
        int32_t *out_reason,
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err)
    {
        if (out_dense) *out_dense = nullptr;
        if (out_dense_count) *out_dense_count = 0;
        if (out_stats)
        {
            out_stats->accepted = 0;
            out_stats->rejected = 0;
        }
        return ffi_call([&]() -> int32_t
                        {
            if (isAdaptiveMethod(shot->method))
            {
                setError(err, BCLIBCFFI_ERR_GENERIC, "adaptive method: use BCLIBCFFI_integrate_adaptive_shot");
                return BCLIBCFFI_ERR_GENERIC;
            }
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngineFromShot(eng, shot, err); status != BCLIBCFFI_OK) return status;
            const int32_t st = writeIntegration(eng, request, want_dense, out_records, out_count, out_dense,
                                                out_dense_count, out_reason, err);
            if (out_stats)
            {
                out_stats->accepted = eng.integration_step_count;
                out_stats->rejected = 0;
            }
            return st; }, err);
    }

    void BCLIBCFFI_free_base_trajectory(BCLIBCFFI_DenseStep *steps)
    {
        std::free(steps);
    }

    int32_t BCLIBCFFI_find_zero_angle_adaptive_shot(
        const BCLIBCFFI_Shot *shot, int32_t method, double relative_tolerance, double absolute_tolerance,
        double distance_ft, double *out_angle_rad, BCLIBCFFI_StepStats *out_stats, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return dispatchAdaptive(method, shot, relative_tolerance, absolute_tolerance, err,
                                                  [&](BCLIBC_BaseEngine &eng, auto &integ) -> int32_t
                                                  {
                              const auto angle = eng.zero_angle_with_fallback(
                                  distance_ft, APEX_IS_MAX_RANGE_RADIANS, ALLOWED_ZERO_ERROR_FEET);
                              storeStats(integ, out_stats);
                              if (has_error(angle)) return setEngineError(err, angle.error());
                              *out_angle_rad = angle.value();
                              return BCLIBCFFI_OK; }); }, err);
    }

    int32_t BCLIBCFFI_find_zero_point_adaptive_shot(
        const BCLIBCFFI_Shot *shot, int32_t method, double relative_tolerance, double absolute_tolerance,
        double distance_ft, BCLIBCFFI_ZeroPointResult *out, BCLIBCFFI_StepStats *out_stats, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return dispatchAdaptive(method, shot, relative_tolerance, absolute_tolerance, err,
                                                  [&](BCLIBC_BaseEngine &eng, auto &integ) -> int32_t
                                                  {
                              const auto point = eng.zero_point_with_fallback(
                                  distance_ft, APEX_IS_MAX_RANGE_RADIANS, ALLOWED_ZERO_ERROR_FEET);
                              storeStats(integ, out_stats);
                              if (has_error(point)) return setEngineError(err, point.error());
                              toC(point.value(), *out);
                              return BCLIBCFFI_OK; }); }, err);
    }

    int32_t BCLIBCFFI_find_max_range_adaptive_shot(
        const BCLIBCFFI_Shot *shot, int32_t method, double relative_tolerance, double absolute_tolerance,
        double low_angle_deg, double high_angle_deg, BCLIBCFFI_MaxRangeResult *out,
        BCLIBCFFI_StepStats *out_stats, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return dispatchAdaptive(method, shot, relative_tolerance, absolute_tolerance, err,
                                                  [&](BCLIBC_BaseEngine &eng, auto &integ) -> int32_t
                                                  {
                              const auto r = eng.find_max_range(low_angle_deg, high_angle_deg, APEX_IS_MAX_RANGE_RADIANS);
                              storeStats(integ, out_stats);
                              if (has_error(r)) return setEngineError(err, r.error());
                              out->max_range_ft = r.value().max_range_ft;
                              out->angle_at_max_rad = r.value().angle_at_max_rad;
                              return BCLIBCFFI_OK; }); }, err);
    }

    int32_t BCLIBCFFI_find_apex_adaptive_shot(
        const BCLIBCFFI_Shot *shot, int32_t method, double relative_tolerance, double absolute_tolerance,
        BCLIBCFFI_TrajectoryData *out, BCLIBCFFI_StepStats *out_stats, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return dispatchAdaptive(method, shot, relative_tolerance, absolute_tolerance, err,
                                                  [&](BCLIBC_BaseEngine &eng, auto &integ) -> int32_t
                                                  {
                              BCLIBC_BaseTrajData apex;
                              const auto apex_result = eng.find_apex(apex);
                              storeStats(integ, out_stats);
                              if (has_error(apex_result)) return setEngineError(err, apex_result.error());
                              auto traj = BCLIBC_TrajectoryData::from_base(eng.shot, apex, BCLIBC_TRAJ_FLAG_APEX);
                              if (has_error(traj))
                              {
                                  return setEngineError(err, traj.error());
                              }
                              toC(traj.value(), *out);
                              return BCLIBCFFI_OK; }); }, err);
    }

    int32_t BCLIBCFFI_find_zero_angle_lofted_shot(
        const BCLIBCFFI_Shot *shot, double distance_ft, int32_t lofted, double *out_angle_rad, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngineFromShot(eng, shot, err); status != BCLIBCFFI_OK) return status;
            const auto angle = eng.find_zero_angle(distance_ft, lofted, APEX_IS_MAX_RANGE_RADIANS, ALLOWED_ZERO_ERROR_FEET);
            if (has_error(angle)) return setEngineError(err, angle.error());
            *out_angle_rad = angle.value();
            return BCLIBCFFI_OK; }, err);
    }

    int32_t BCLIBCFFI_find_zero_point_lofted_shot(
        const BCLIBCFFI_Shot *shot, double distance_ft, int32_t lofted, BCLIBCFFI_ZeroPointResult *out,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            BCLIBC_BaseEngine eng;
            if (const auto status = initEngineFromShot(eng, shot, err); status != BCLIBCFFI_OK) return status;
            const auto point = eng.find_zero_point(distance_ft, lofted, APEX_IS_MAX_RANGE_RADIANS, ALLOWED_ZERO_ERROR_FEET);
            if (has_error(point)) return setEngineError(err, point.error());
            toC(point.value(), *out);
            return BCLIBCFFI_OK; }, err);
    }

    int32_t BCLIBCFFI_find_zero_angle_lofted_adaptive_shot(
        const BCLIBCFFI_Shot *shot, int32_t method, double relative_tolerance, double absolute_tolerance,
        double distance_ft, int32_t lofted, double *out_angle_rad, BCLIBCFFI_StepStats *out_stats, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return dispatchAdaptive(method, shot, relative_tolerance, absolute_tolerance, err,
                                                  [&](BCLIBC_BaseEngine &eng, auto &integ) -> int32_t
                                                  {
                              const auto angle = eng.find_zero_angle(distance_ft, lofted, APEX_IS_MAX_RANGE_RADIANS,
                                                                     ALLOWED_ZERO_ERROR_FEET);
                              storeStats(integ, out_stats);
                              if (has_error(angle)) return setEngineError(err, angle.error());
                              *out_angle_rad = angle.value();
                              return BCLIBCFFI_OK; }); }, err);
    }

    int32_t BCLIBCFFI_find_zero_point_lofted_adaptive_shot(
        const BCLIBCFFI_Shot *shot, int32_t method, double relative_tolerance, double absolute_tolerance,
        double distance_ft, int32_t lofted, BCLIBCFFI_ZeroPointResult *out, BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return dispatchAdaptive(method, shot, relative_tolerance, absolute_tolerance, err,
                                                  [&](BCLIBC_BaseEngine &eng, auto &integ) -> int32_t
                                                  {
                              const auto point = eng.find_zero_point(distance_ft, lofted, APEX_IS_MAX_RANGE_RADIANS,
                                                                     ALLOWED_ZERO_ERROR_FEET);
                              storeStats(integ, out_stats);
                              if (has_error(point)) return setEngineError(err, point.error());
                              toC(point.value(), *out);
                              return BCLIBCFFI_OK; }); }, err);
    }

    int32_t BCLIBCFFI_integrate_at_adaptive_shot(
        const BCLIBCFFI_Shot *shot, int32_t method, double relative_tolerance, double absolute_tolerance,
        int32_t key, double target_value, BCLIBCFFI_Interception *out, BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return dispatchAdaptive(method, shot, relative_tolerance, absolute_tolerance, err,
                                                  [&](BCLIBC_BaseEngine &eng, auto &integ) -> int32_t
                                                  {
                              BCLIBC_BaseTrajData raw;
                              BCLIBC_TrajectoryData full;
                              const auto result = eng.integrate_at(static_cast<BCLIBC_BaseTrajData_InterpKey>(key),
                                                                   target_value, raw, full);
                              storeStats(integ, out_stats);
                              if (has_error(result)) return setEngineError(err, result.error());
                              toC(raw, out->raw_data);
                              toC(full, out->full_data);
                              return BCLIBCFFI_OK; }); }, err);
    }

    // ------------------------------------------------------------------------
    // Solver building blocks
    // ------------------------------------------------------------------------

    int32_t BCLIBCFFI_error_at_distance_shot(
        const BCLIBCFFI_Shot *shot, double angle_rad, double target_x_ft, double target_y_ft, double *out_error_ft,
        BCLIBCFFI_BaseTrajData *out_hit, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return runFixed(shot, err, [&](BCLIBC_BaseEngine &eng) -> int32_t
                                          { return opErrorAtDistance(eng, angle_rad, target_x_ft, target_y_ft,
                                                                     out_error_ft, out_hit, err); }); }, err);
    }

    int32_t BCLIBCFFI_error_at_distance_adaptive_shot(
        const BCLIBCFFI_Shot *shot, int32_t method, double relative_tolerance, double absolute_tolerance,
        double angle_rad, double target_x_ft, double target_y_ft, double *out_error_ft,
        BCLIBCFFI_BaseTrajData *out_hit, BCLIBCFFI_StepStats *out_stats, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return dispatchAdaptive(method, shot, relative_tolerance, absolute_tolerance, err,
                                                  [&](BCLIBC_BaseEngine &eng, auto &integ) -> int32_t
                                                  {
                              const int32_t st = opErrorAtDistance(eng, angle_rad, target_x_ft, target_y_ft,
                                                                   out_error_ft, out_hit, err);
                              storeStats(integ, out_stats);
                              return st; }); }, err);
    }

    int32_t BCLIBCFFI_range_for_angle_shot(
        const BCLIBCFFI_Shot *shot, double angle_rad, double *out_slant_range_ft, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return runFixed(shot, err, [&](BCLIBC_BaseEngine &eng) -> int32_t
                                          { return opRangeForAngle(eng, angle_rad, out_slant_range_ft); }); }, err);
    }

    int32_t BCLIBCFFI_range_for_angle_adaptive_shot(
        const BCLIBCFFI_Shot *shot, int32_t method, double relative_tolerance, double absolute_tolerance,
        double angle_rad, double *out_slant_range_ft, BCLIBCFFI_StepStats *out_stats, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return dispatchAdaptive(method, shot, relative_tolerance, absolute_tolerance, err,
                                                  [&](BCLIBC_BaseEngine &eng, auto &integ) -> int32_t
                                                  {
                              const int32_t st = opRangeForAngle(eng, angle_rad, out_slant_range_ft);
                              storeStats(integ, out_stats);
                              return st; }); }, err);
    }

    int32_t BCLIBCFFI_find_zero_angle_newton_shot(
        const BCLIBCFFI_Shot *shot, double distance_ft, double *out_angle_rad, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return runFixed(shot, err, [&](BCLIBC_BaseEngine &eng) -> int32_t
                                          { return opZeroNewton(eng, distance_ft, out_angle_rad, err); }); }, err);
    }

    int32_t BCLIBCFFI_find_zero_angle_newton_adaptive_shot(
        const BCLIBCFFI_Shot *shot, int32_t method, double relative_tolerance, double absolute_tolerance,
        double distance_ft, double *out_angle_rad, BCLIBCFFI_StepStats *out_stats, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return dispatchAdaptive(method, shot, relative_tolerance, absolute_tolerance, err,
                                                  [&](BCLIBC_BaseEngine &eng, auto &integ) -> int32_t
                                                  {
                              const int32_t st = opZeroNewton(eng, distance_ft, out_angle_rad, err);
                              storeStats(integ, out_stats);
                              return st; }); }, err);
    }

    int32_t BCLIBCFFI_zero_init_shot(
        const BCLIBCFFI_Shot *shot, double distance_ft, BCLIBCFFI_ZeroInit *out, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return runFixed(shot, err, [&](BCLIBC_BaseEngine &eng) -> int32_t
                                          { return opZeroInit(eng, distance_ft, out, err); }); }, err);
    }

    int32_t BCLIBCFFI_zero_init_adaptive_shot(
        const BCLIBCFFI_Shot *shot, int32_t method, double relative_tolerance, double absolute_tolerance,
        double distance_ft, BCLIBCFFI_ZeroInit *out, BCLIBCFFI_StepStats *out_stats, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return dispatchAdaptive(method, shot, relative_tolerance, absolute_tolerance, err,
                                                  [&](BCLIBC_BaseEngine &eng, auto &integ) -> int32_t
                                                  {
                              const int32_t st = opZeroInit(eng, distance_ft, out, err);
                              storeStats(integ, out_stats);
                              return st; }); }, err);
    }

    // ------------------------------------------------------------------------
    // Working with dense output
    // ------------------------------------------------------------------------

    int32_t BCLIBCFFI_dense_get_at(
        const BCLIBCFFI_DenseStep *steps, int32_t count, int32_t key, double value, double start_from_time,
        BCLIBCFFI_DenseStep *out, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            if (count < 0 || (!steps && count > 0))
            {
                setError(err, BCLIBCFFI_ERR_INVALID_ARGUMENT, "dense output: negative count or NULL steps");
                return BCLIBCFFI_ERR_INVALID_ARGUMENT;
            }
            const BCLIBC_BaseTrajSeq seq = denseSeq(steps, count);
            BCLIBC_BaseTrajData hit;
            const auto r = seq.get_at(static_cast<BCLIBC_BaseTrajData_InterpKey>(key), value, start_from_time, hit);
            if (has_error(r)) return setEngineError(err, r.error());
            toC(hit, *out);
            return BCLIBCFFI_OK; }, err);
    }

    int32_t BCLIBCFFI_dense_get_at_slant_height(
        const BCLIBCFFI_DenseStep *steps, int32_t count, double look_angle_rad, double value,
        BCLIBCFFI_DenseStep *out, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            if (count < 0 || (!steps && count > 0))
            {
                setError(err, BCLIBCFFI_ERR_INVALID_ARGUMENT, "dense output: negative count or NULL steps");
                return BCLIBCFFI_ERR_INVALID_ARGUMENT;
            }
            const BCLIBC_BaseTrajSeq seq = denseSeq(steps, count);
            BCLIBC_BaseTrajData hit;
            const auto r = seq.get_at_slant_height(look_angle_rad, value, hit);
            if (has_error(r)) return setEngineError(err, r.error());
            toC(hit, *out);
            return BCLIBCFFI_OK; }, err);
    }

    int32_t BCLIBCFFI_trajectory_data_from_step_shot(
        const BCLIBCFFI_Shot *shot, const BCLIBCFFI_DenseStep *step, int32_t flag, BCLIBCFFI_TrajectoryData *out,
        BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        { return runFixed(shot, err, [&](BCLIBC_BaseEngine &eng) -> int32_t
                                          {
            const auto traj = BCLIBC_TrajectoryData::from_base(eng.shot, fromC(*step), static_cast<BCLIBC_TrajFlag>(flag));
            if (has_error(traj))
            {
                return setEngineError(err, traj.error());
            }
            toC(traj.value(), *out);
            return BCLIBCFFI_OK; }); }, err);
    }

    // ------------------------------------------------------------------------
    // Growable buffer of base trajectory points, interpolation over points
    // ------------------------------------------------------------------------

    static bool validKey(int32_t key)
    {
        return key >= BCLIBCFFI_INTERP_KEY_TIME && key <= BCLIBCFFI_INTERP_KEY_VEL_Z;
    }

    static int32_t failWith(BCLIBCFFI_Error *err, BCLIBCFFI_Status code, const char *msg)
    {
        setError(err, code, msg);
        return code;
    }

    void BCLIBCFFI_base_traj_buf_free(BCLIBCFFI_BaseTrajBuf *buf)
    {
        if (!buf)
            return;
        std::free(buf->data);
        buf->data = nullptr;
        buf->count = 0;
        buf->capacity = 0;
    }

    int32_t BCLIBCFFI_base_traj_buf_append(BCLIBCFFI_BaseTrajBuf *buf, const BCLIBCFFI_BaseTrajData *item, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            if (!buf || !item)
                return failWith(err, BCLIBCFFI_ERR_INVALID_ARGUMENT, "NULL buffer or item");
            if (buf->count == buf->capacity)
            {
                if (buf->capacity > INT32_MAX / 2)
                    return failWith(err, BCLIBCFFI_ERR_RUNTIME, "Trajectory buffer is at its largest size");
                const int32_t capacity = buf->capacity == 0 ? 64 : buf->capacity * 2;
                void *grown = std::realloc(buf->data, static_cast<size_t>(capacity) * sizeof(BCLIBCFFI_BaseTrajData));
                if (!grown)
                    return failWith(err, BCLIBCFFI_ERR_RUNTIME, "Cannot grow the trajectory buffer: out of memory");
                buf->data = static_cast<BCLIBCFFI_BaseTrajData *>(grown);
                buf->capacity = capacity;
            }
            buf->data[buf->count++] = *item;
            return BCLIBCFFI_OK; }, err);
    }

    int32_t BCLIBCFFI_base_traj_interpolate(
        int32_t key, double value, const BCLIBCFFI_BaseTrajData *p0, const BCLIBCFFI_BaseTrajData *p1,
        const BCLIBCFFI_BaseTrajData *p2, BCLIBCFFI_BaseTrajData *out, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            if (!p0 || !p1 || !p2 || !out)
                return failWith(err, BCLIBCFFI_ERR_INVALID_ARGUMENT, "NULL point");
            if (!validKey(key))
                return failWith(err, BCLIBCFFI_ERR_INVALID_ARGUMENT, "Unknown interpolation key");
            BCLIBC_BaseTrajData r;
            const auto res = BCLIBC_BaseTrajData::interpolate(
                static_cast<BCLIBC_BaseTrajData_InterpKey>(key), value, fromC(*p0), fromC(*p1), fromC(*p2), r);
            if (has_error(res))
                return setEngineError(err, res.error());
            toC(r, *out);
            return BCLIBCFFI_OK; }, err);
    }

    int32_t BCLIBCFFI_base_traj_interpolate_at(
        const BCLIBCFFI_BaseTrajData *steps, int32_t count, int64_t idx, int32_t key, double value,
        BCLIBCFFI_BaseTrajData *out, BCLIBCFFI_Error *err)
    {
        return ffi_call([&]() -> int32_t
                        {
            if (!out || count < 0 || (!steps && count > 0))
                return failWith(err, BCLIBCFFI_ERR_INVALID_ARGUMENT, "NULL output or steps, or a negative count");
            if (!validKey(key))
                return failWith(err, BCLIBCFFI_ERR_INVALID_ARGUMENT, "Unknown interpolation key");
            // Only the three points around idx are read: the array is not copied (BCLIBC_BaseTrajSeq::interpolate_at
            // does the same range check and the same interpolation on its own buffer).
            if (idx < 0)
                idx += count;
            if (idx < 1 || idx >= static_cast<int64_t>(count) - 1)
            {
                const BCLIBC_Error e{BCLIBC_OutOfRangeError{
                    "Index outside valid interpolation range [1, n-2]",
                    static_cast<double>(idx), 1.0, static_cast<double>(count) - 2.0}};
                return setEngineError(err, e);
            }
            const auto k = static_cast<BCLIBC_BaseTrajData_InterpKey>(key);
            const BCLIBC_BaseTrajData p0 = fromC(steps[idx - 1]), p1 = fromC(steps[idx]), p2 = fromC(steps[idx + 1]);
            const double x0 = p0[k], x1 = p1[k], x2 = p2[k];
            if (x0 == x1 || x0 == x2 || x1 == x2)
            {
                const BCLIBC_Error e{BCLIBC_DomainError{"Duplicate key values: cannot interpolate", x0, x1}};
                return setEngineError(err, e);
            }
            BCLIBC_BaseTrajData r;
            BCLIBC_BaseTrajData::interpolate3pt_vectorized(value, x0, x1, x2, p0, p1, p2, r, k);
            toC(r, *out);
            return BCLIBCFFI_OK; }, err);
    }

    // ============================================================================
    // ABI layout introspection
    //
    // Fixed field order (must match the Dart-side reader in
    // lib/ffi/bclibc_ffi_web.dart _Layout exactly):
    //
    //   [ sizeof(Config),      7 field offsets ]
    //   [ sizeof(Wind),        4 field offsets ]
    //   [ sizeof(Shot),       24 field offsets ]
    //   [ sizeof(TrajectoryRequest), 4 field offsets ]
    //   [ sizeof(TrajectoryData),   16 field offsets ]
    //   [ sizeof(MaxRangeResult),    2 field offsets ]
    //   [ sizeof(BaseTrajData),      8 field offsets ]
    //   [ sizeof(Interception),      2 field offsets ]
    //   [ sizeof(ZeroPointResult),   2 field offsets ]
    //   [ sizeof(Error),             6 field offsets ]
    //   [ sizeof(StepStats),         2 field offsets ]
    //   [ sizeof(ZeroInit),          7 field offsets ]
    //   [ sizeof(BaseTrajBuf),       3 field offsets ]
    //
    // Append-only: a struct added to the ABI goes at the end, so every offset keeps its place and a reader that
    // knows the earlier ones only has to raise its expected count (BCLIBCFFI_get_layout() returns -1 when the
    // buffer is too small, and a reader that checks the exact count must be updated with the library).
    // ============================================================================

    int32_t BCLIBCFFI_get_layout(int32_t *out, int32_t out_len)
    {
        int32_t vals[] = {
            // Config
            (int32_t)sizeof(BCLIBCFFI_Config),
            (int32_t)offsetof(BCLIBCFFI_Config, cStepMultiplier),
            (int32_t)offsetof(BCLIBCFFI_Config, cZeroFindingAccuracy),
            (int32_t)offsetof(BCLIBCFFI_Config, cMinimumVelocity),
            (int32_t)offsetof(BCLIBCFFI_Config, cMaximumDrop),
            (int32_t)offsetof(BCLIBCFFI_Config, cMaxIterations),
            (int32_t)offsetof(BCLIBCFFI_Config, cGravityConstant),
            (int32_t)offsetof(BCLIBCFFI_Config, cMinimumAltitude),
            // Wind
            (int32_t)sizeof(BCLIBCFFI_Wind),
            (int32_t)offsetof(BCLIBCFFI_Wind, velocity_fps),
            (int32_t)offsetof(BCLIBCFFI_Wind, direction_from_rad),
            (int32_t)offsetof(BCLIBCFFI_Wind, until_distance_ft),
            (int32_t)offsetof(BCLIBCFFI_Wind, max_distance_ft),
            // Shot
            (int32_t)sizeof(BCLIBCFFI_Shot),
            (int32_t)offsetof(BCLIBCFFI_Shot, bc),
            (int32_t)offsetof(BCLIBCFFI_Shot, weight_grain),
            (int32_t)offsetof(BCLIBCFFI_Shot, diameter_inch),
            (int32_t)offsetof(BCLIBCFFI_Shot, length_inch),
            (int32_t)offsetof(BCLIBCFFI_Shot, muzzle_velocity_fps),
            (int32_t)offsetof(BCLIBCFFI_Shot, sight_height_ft),
            (int32_t)offsetof(BCLIBCFFI_Shot, twist_inch),
            (int32_t)offsetof(BCLIBCFFI_Shot, temp_c),
            (int32_t)offsetof(BCLIBCFFI_Shot, pressure_hpa),
            (int32_t)offsetof(BCLIBCFFI_Shot, altitude_ft),
            (int32_t)offsetof(BCLIBCFFI_Shot, humidity),
            (int32_t)offsetof(BCLIBCFFI_Shot, mach_data),
            (int32_t)offsetof(BCLIBCFFI_Shot, cd_data),
            (int32_t)offsetof(BCLIBCFFI_Shot, drag_table_size),
            (int32_t)offsetof(BCLIBCFFI_Shot, winds),
            (int32_t)offsetof(BCLIBCFFI_Shot, wind_count),
            (int32_t)offsetof(BCLIBCFFI_Shot, look_angle_rad),
            (int32_t)offsetof(BCLIBCFFI_Shot, barrel_elevation_rad),
            (int32_t)offsetof(BCLIBCFFI_Shot, barrel_azimuth_rad),
            (int32_t)offsetof(BCLIBCFFI_Shot, cant_angle_rad),
            (int32_t)offsetof(BCLIBCFFI_Shot, latitude_deg),
            (int32_t)offsetof(BCLIBCFFI_Shot, azimuth_deg),
            (int32_t)offsetof(BCLIBCFFI_Shot, config),
            (int32_t)offsetof(BCLIBCFFI_Shot, method),
            // TrajectoryRequest
            (int32_t)sizeof(BCLIBCFFI_TrajectoryRequest),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryRequest, range_limit_ft),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryRequest, range_step_ft),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryRequest, time_step),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryRequest, filter_flags),
            // TrajectoryData
            (int32_t)sizeof(BCLIBCFFI_TrajectoryData),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, time),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, distance_ft),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, velocity_fps),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, mach),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, height_ft),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, slant_height_ft),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, drop_angle_rad),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, windage_ft),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, windage_angle_rad),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, slant_distance_ft),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, angle_rad),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, density_ratio),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, drag),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, energy_ft_lb),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, ogw_lb),
            (int32_t)offsetof(BCLIBCFFI_TrajectoryData, flag),
            // MaxRangeResult
            (int32_t)sizeof(BCLIBCFFI_MaxRangeResult),
            (int32_t)offsetof(BCLIBCFFI_MaxRangeResult, max_range_ft),
            (int32_t)offsetof(BCLIBCFFI_MaxRangeResult, angle_at_max_rad),
            // BaseTrajData
            (int32_t)sizeof(BCLIBCFFI_BaseTrajData),
            (int32_t)offsetof(BCLIBCFFI_BaseTrajData, time),
            (int32_t)offsetof(BCLIBCFFI_BaseTrajData, px),
            (int32_t)offsetof(BCLIBCFFI_BaseTrajData, py),
            (int32_t)offsetof(BCLIBCFFI_BaseTrajData, pz),
            (int32_t)offsetof(BCLIBCFFI_BaseTrajData, vx),
            (int32_t)offsetof(BCLIBCFFI_BaseTrajData, vy),
            (int32_t)offsetof(BCLIBCFFI_BaseTrajData, vz),
            (int32_t)offsetof(BCLIBCFFI_BaseTrajData, mach),
            // Interception
            (int32_t)sizeof(BCLIBCFFI_Interception),
            (int32_t)offsetof(BCLIBCFFI_Interception, raw_data),
            (int32_t)offsetof(BCLIBCFFI_Interception, full_data),
            // ZeroPointResult
            (int32_t)sizeof(BCLIBCFFI_ZeroPointResult),
            (int32_t)offsetof(BCLIBCFFI_ZeroPointResult, angle_rad),
            (int32_t)offsetof(BCLIBCFFI_ZeroPointResult, point),
            // Error
            (int32_t)sizeof(BCLIBCFFI_Error),
            (int32_t)offsetof(BCLIBCFFI_Error, code),
            (int32_t)offsetof(BCLIBCFFI_Error, message),
            (int32_t)offsetof(BCLIBCFFI_Error, f64_0),
            (int32_t)offsetof(BCLIBCFFI_Error, f64_1),
            (int32_t)offsetof(BCLIBCFFI_Error, f64_2),
            (int32_t)offsetof(BCLIBCFFI_Error, i32_0),
            // StepStats
            (int32_t)sizeof(BCLIBCFFI_StepStats),
            (int32_t)offsetof(BCLIBCFFI_StepStats, accepted),
            (int32_t)offsetof(BCLIBCFFI_StepStats, rejected),
            // ZeroInit
            (int32_t)sizeof(BCLIBCFFI_ZeroInit),
            (int32_t)offsetof(BCLIBCFFI_ZeroInit, status),
            (int32_t)offsetof(BCLIBCFFI_ZeroInit, reserved),
            (int32_t)offsetof(BCLIBCFFI_ZeroInit, look_angle_rad),
            (int32_t)offsetof(BCLIBCFFI_ZeroInit, slant_range_ft),
            (int32_t)offsetof(BCLIBCFFI_ZeroInit, target_x_ft),
            (int32_t)offsetof(BCLIBCFFI_ZeroInit, target_y_ft),
            (int32_t)offsetof(BCLIBCFFI_ZeroInit, start_height_ft),
            // BaseTrajBuf
            (int32_t)sizeof(BCLIBCFFI_BaseTrajBuf),
            (int32_t)offsetof(BCLIBCFFI_BaseTrajBuf, data),
            (int32_t)offsetof(BCLIBCFFI_BaseTrajBuf, count),
            (int32_t)offsetof(BCLIBCFFI_BaseTrajBuf, capacity),
        };
        constexpr int32_t count = (int32_t)(sizeof(vals) / sizeof(vals[0]));
        if (out_len < count)
            return -1;
        std::memcpy(out, vals, sizeof(vals));
        return count;
    }

} // extern "C"
