/**
 * bclibc_ffi.cpp
 *
 * Thin C++ wrapper that implements the bclibc_ffi.h C API.
 * Mirrors the structure of the WASM bindings (wasm/bindings.cpp):
 *   - PCHIP curve building from a flat drag table
 *   - Engine initialisation from BCLIBCFFI_ShotProps
 *   - Exception → error-code conversion
 */

#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <vector>

#include "bclibc/base_types.hpp"
#include "bclibc/traj_data.hpp"
#include "bclibc/engine.hpp"
#include "bclibc/exceptions.hpp"
#include "bclibc/rk4.hpp"
#include "bclibc/euler.hpp"
#include "bclibc/version.h" // This is the generated file

using namespace bclibc;

// ============================================================================
// Internal constants (same as WASM bindings)
// ============================================================================

static constexpr double APEX_IS_MAX_RANGE_RADIANS = 0.0003;
static constexpr double ALLOWED_ZERO_ERROR_FEET = 1e-2;

static BCLIBC_Curve buildCurve(const BCLIBCFFI_DragPoint *dt, int n)
{
    if (n < 2)
        throw std::invalid_argument("Drag table requires at least 2 points");

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

// ============================================================================
// Engine initialisation (mirrors WASM initEngine + shotPropsFromVal)
// ============================================================================

static double calcStep(BCLIBCFFI_IntegrationMethod m, double multiplier)
{
    return (m == BCLIBCFFI_INTEGRATION_RK4 ? 0.0025 : 0.5) * multiplier;
}

static BCLIBC_IntegrateCallable selectIntegrateFunc(BCLIBCFFI_IntegrationMethod m)
{
    return (m == BCLIBCFFI_INTEGRATION_RK4) ? BCLIBC_IntegrateCallable(BCLIBC_integrateRK4)
                                            : BCLIBC_IntegrateCallable(BCLIBC_integrateEULER);
}

static void initEngine(BCLIBC_BaseEngine &eng, const BCLIBCFFI_ShotProps *p)
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
        buildCurve(p->drag_table, p->drag_table_count),
        buildMachList(p->drag_table, p->drag_table_count),
        atmo,
        coriolis,
        wind_sock,
        BCLIBC_TRAJ_FLAG_NONE);

    eng.integrate_func = selectIntegrateFunc(p->method);
    eng.config = config;
    eng.gravity_vector = BCLIBC_V3dT(0.0, config.cGravityConstant, 0.0);
}

// Builds and initialises the engine from a BCLIBCFFI_Shot (user-facing natural units).
// mach_data/cd_data passed directly — same layout as BCLIBC_Shot, no copy needed.
// Winds are copied to BCLIBC_Wind; lifetime covers the to_shot_props() call.
static void initEngineFromShot(BCLIBC_BaseEngine &eng, const BCLIBCFFI_Shot *s)
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

    eng.shot = shot.to_shot_props();
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

// ============================================================================
// Exception wrapper (replaces BCLIBCFFI_CATCH macro)
// ============================================================================

// Catches all exception types including non-std (catch(...)) across the FFI boundary.
// C++11 compatible: lambda with -> int32_t trailing return type.
template <typename Func>
static int32_t ffi_call(Func &&fn, BCLIBCFFI_Error *err) noexcept
{
    clearError(err);
    try
    {
        return fn();
    }
    catch (const BCLIBC_OutOfRangeError &e)
    {
        setError(err, BCLIBCFFI_ERR_OUT_OF_RANGE, e.what());
        if (err)
        {
            err->f64_0 = e.requested_distance_ft;
            err->f64_1 = e.max_range_ft;
            err->f64_2 = e.look_angle_rad;
        }
        return BCLIBCFFI_ERR_OUT_OF_RANGE;
    }
    catch (const BCLIBC_ZeroFindingError &e)
    {
        setError(err, BCLIBCFFI_ERR_ZERO_FINDING, e.what());
        if (err)
        {
            err->f64_0 = e.zero_finding_error;
            err->f64_1 = e.last_barrel_elevation_rad;
            err->i32_0 = e.iterations_count;
        }
        return BCLIBCFFI_ERR_ZERO_FINDING;
    }
    catch (const BCLIBC_InterceptionError &e)
    {
        setError(err, BCLIBCFFI_ERR_INTERCEPTION, e.what());
        return BCLIBCFFI_ERR_INTERCEPTION;
    }
    catch (const BCLIBC_SolverRuntimeError &e)
    {
        setError(err, BCLIBCFFI_ERR_SOLVER_RUNTIME, e.what());
        return BCLIBCFFI_ERR_SOLVER_RUNTIME;
    }
    catch (const std::exception &e)
    {
        setError(err, BCLIBCFFI_ERR_GENERIC, e.what());
        return BCLIBCFFI_ERR_GENERIC;
    }
    catch (...)
    {
        setError(err, BCLIBCFFI_ERR_GENERIC,
                 "Unknown non-std exception across FFI boundary");
        return BCLIBCFFI_ERR_GENERIC;
    }
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
            initEngine(eng, props);
            BCLIBC_BaseTrajData apex;
            eng.find_apex(apex);
            toC(BCLIBC_TrajectoryData(eng.shot, apex, BCLIBC_TRAJ_FLAG_APEX), *out);
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
            initEngine(eng, props);
            BCLIBC_MaxRangeResult r = eng.find_max_range(
                low_angle_deg, high_angle_deg, APEX_IS_MAX_RANGE_RADIANS);
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
            initEngine(eng, props);
            *out_angle_rad = eng.zero_angle_with_fallback(
                distance_ft, APEX_IS_MAX_RANGE_RADIANS, ALLOWED_ZERO_ERROR_FEET);
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
            initEngine(eng, props);

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
                try
                {
                    for (int32_t i = 0; i < count; ++i)
                        toC(records[i], arr[i]);
                }
                catch (...)
                {
                    std::free(arr);
                    throw; // re-throw — outer ffi_call catches and returns ERR_GENERIC
                }
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
            initEngine(eng, props);

            BCLIBC_BaseTrajData raw;
            BCLIBC_TrajectoryData full;
            eng.integrate_at(
                static_cast<BCLIBC_BaseTrajData_InterpKey>(key),
                target_value, raw, full);

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
            initEngineFromShot(eng, shot);
            BCLIBC_BaseTrajData apex;
            eng.find_apex(apex);
            toC(BCLIBC_TrajectoryData(eng.shot, apex, BCLIBC_TRAJ_FLAG_APEX), *out);
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
            initEngineFromShot(eng, shot);
            BCLIBC_MaxRangeResult r = eng.find_max_range(
                low_angle_deg, high_angle_deg, APEX_IS_MAX_RANGE_RADIANS);
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
            initEngineFromShot(eng, shot);
            *out_angle_rad = eng.zero_angle_with_fallback(
                distance_ft, APEX_IS_MAX_RANGE_RADIANS, ALLOWED_ZERO_ERROR_FEET);
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
            initEngineFromShot(eng, shot);

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
                try
                {
                    for (int32_t i = 0; i < count; ++i)
                        toC(records[i], arr[i]);
                }
                catch (...)
                {
                    std::free(arr);
                    throw;
                }
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
            initEngineFromShot(eng, shot);

            BCLIBC_BaseTrajData raw;
            BCLIBC_TrajectoryData full;
            eng.integrate_at(
                static_cast<BCLIBC_BaseTrajData_InterpKey>(key),
                target_value, raw, full);

            toC(raw, out->raw_data);
            toC(full, out->full_data);
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
    //   [ sizeof(Error),             6 field offsets ]
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
            // Error
            (int32_t)sizeof(BCLIBCFFI_Error),
            (int32_t)offsetof(BCLIBCFFI_Error, code),
            (int32_t)offsetof(BCLIBCFFI_Error, message),
            (int32_t)offsetof(BCLIBCFFI_Error, f64_0),
            (int32_t)offsetof(BCLIBCFFI_Error, f64_1),
            (int32_t)offsetof(BCLIBCFFI_Error, f64_2),
            (int32_t)offsetof(BCLIBCFFI_Error, i32_0),
        };
        constexpr int32_t count = (int32_t)(sizeof(vals) / sizeof(vals[0]));
        if (out_len < count)
            return -1;
        std::memcpy(out, vals, sizeof(vals));
        return count;
    }

} // extern "C"
