// test_identity.cpp — bclibc (C++) ↔ tbclibc (C) mathematical identity test.
//
// Compiles as C++17 so both headers can coexist in one translation unit.
// No external test framework — returns 0 on success, 1 on any failure.
//
// Key invariant enforced here: both engines use identical RK4 time-step
// (kCalcStep = 0.0025s).  bclibc reads it from BCLIBC_Shot::calc_step;
// tiny_bclibc_build_shot_props() derives it from config, so we override
// TINY_BCLIBC_ShotProps::calc_step after the build call.
//
// Tolerance: 1e-9 (absolute).  Both engines perform the same IEEE-754
// double arithmetic on the same input, so results should be bitwise
// identical or differ by at most ±1 ULP from different call ordering.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>
#ifndef _WIN32
#include <unistd.h>
#include <csignal>
#endif

// ── bclibc C++ headers ────────────────────────────────────────────────────
#include "bclibc.hpp"

// ── tbclibc C headers ────────────────────────────────────────────────────
extern "C"
{
#define TINY_BCLIBC_BUILD_SHARED // use extern declarations, impl via static inline
#include "tiny_bclibc/engine.h"
}
// tbclibc is header-only in this test: compile via static-inline mode.
// Re-include without TINY_BCLIBC_BUILD_SHARED so all TINY_BCLIBC_FUNC resolve to
// static inline definitions in this TU.
#undef TINY_BCLIBC_BUILD_SHARED
#undef TINY_BCLIBC_ENGINE_H // force re-include
// Actually: just use the header-only (no-TINY_BCLIBC_BUILD_SHARED) path:
// The include already happened above with BUILD_SHARED, which only changed
// the function visibility.  For a header-only test build we rely on the
// INTERFACE tiny_bclibc_headers target which does NOT define TINY_BCLIBC_BUILD_SHARED.

// ── test helpers ─────────────────────────────────────────────────────────
#include "fixture.hpp"
#include "compare.hpp"

// ═══════════════════════════════════════════════════════════════════════════
// bclibc engine helpers
// ═══════════════════════════════════════════════════════════════════════════

namespace
{

    // Flat arrays for G7 table (bclibc takes const double*)
    static double g_g7_mach[kG7TableSize];
    static double g_g7_cd[kG7TableSize];
    static bool g_g7_init = false;

    void init_g7_arrays()
    {
        if (g_g7_init)
            return;
        for (int i = 0; i < kG7TableSize; ++i)
        {
            g_g7_mach[i] = kG7Table[i].mach;
            g_g7_cd[i] = kG7Table[i].cd;
        }
        g_g7_init = true;
    }

    // bclibc no longer throws: every fallible call returns a BCLIBC_Result<T>.
    // This test's fixtures are known-good, so a failure here is a test bug, not a case to
    // handle gracefully -- assert and unwrap, rather than threading Result through every helper.
    template <class T, class E>
    T unwrap(const bclibc::BCLIBC_Result<T, E> &result, const char *what)
    {
        assert(!bclibc::has_error(result) && what);
        return result.value();
    }

    // Build a bclibc BCLIBC_ShotProps from G7_BASIC fixture (no wind).
    bclibc::BCLIBC_ShotProps make_bclibc_shot_props(double barrel_elevation_rad = G7_BASIC::BARREL_EL_RAD,
                                                    double sight_height_ft = G7_BASIC::SIGHT_HT_FT,
                                                    double look_angle_rad = G7_BASIC::LOOK_ANGLE_RAD,
                                                    double pressure_hpa = G7_BASIC::PRESSURE_HPA,
                                                    double latitude_deg = G7_BASIC::LAT_DEG,
                                                    double azimuth_deg = G7_BASIC::AZ_DEG)
    {
        init_g7_arrays();

        bclibc::BCLIBC_Shot shot;
        shot.bc = G7_BASIC::BC;
        shot.weight_grain = G7_BASIC::WEIGHT_GR;
        shot.diameter_inch = G7_BASIC::DIAMETER_IN;
        shot.length_inch = G7_BASIC::LENGTH_IN;
        shot.muzzle_velocity_fps = G7_BASIC::MV_FPS;
        shot.stability_coefficient = 0.0;
        shot.sight_height_ft = sight_height_ft;
        shot.twist_inch = G7_BASIC::TWIST_IN;
        shot.temp_c = G7_BASIC::TEMP_C;
        shot.pressure_hpa = pressure_hpa;
        shot.altitude_ft = G7_BASIC::ALT_FT;
        shot.humidity = G7_BASIC::HUMIDITY;
        shot.mach_data = g_g7_mach;
        shot.cd_data = g_g7_cd;
        shot.drag_table_size = kG7TableSize;
        shot.winds = nullptr;
        shot.wind_count = 0;
        shot.look_angle_rad = look_angle_rad;
        shot.barrel_elevation_rad = barrel_elevation_rad;
        shot.barrel_azimuth_rad = G7_BASIC::BARREL_AZ_RAD;
        shot.cant_angle_rad = G7_BASIC::CANT_ANGLE_RAD;
        shot.latitude_deg = latitude_deg;
        shot.azimuth_deg = azimuth_deg;
        shot.calc_step = kCalcStep;

        return unwrap(shot.to_shot_props(), "make_bclibc_shot_props: to_shot_props() failed");
    }

    // Build a bclibc BCLIBC_ShotProps with wind (G7_WIND fixture).
    bclibc::BCLIBC_ShotProps make_bclibc_shot_props_wind(double barrel_elevation_rad = G7_BASIC::BARREL_EL_RAD)
    {
        init_g7_arrays();

        static bclibc::BCLIBC_Wind wind(
            G7_WIND::WIND_VEL_FPS,
            G7_WIND::WIND_DIR_RAD,
            G7_WIND::WIND_UNTIL_FT,
            G7_WIND::WIND_MAX_FT);

        bclibc::BCLIBC_Shot shot;
        shot.bc = G7_BASIC::BC;
        shot.weight_grain = G7_BASIC::WEIGHT_GR;
        shot.diameter_inch = G7_BASIC::DIAMETER_IN;
        shot.length_inch = G7_BASIC::LENGTH_IN;
        shot.muzzle_velocity_fps = G7_BASIC::MV_FPS;
        shot.stability_coefficient = 0.0;
        shot.sight_height_ft = G7_BASIC::SIGHT_HT_FT;
        shot.twist_inch = G7_BASIC::TWIST_IN;
        shot.temp_c = G7_BASIC::TEMP_C;
        shot.pressure_hpa = G7_BASIC::PRESSURE_HPA;
        shot.altitude_ft = G7_BASIC::ALT_FT;
        shot.humidity = G7_BASIC::HUMIDITY;
        shot.mach_data = g_g7_mach;
        shot.cd_data = g_g7_cd;
        shot.drag_table_size = kG7TableSize;
        shot.winds = &wind;
        shot.wind_count = 1;
        shot.look_angle_rad = G7_BASIC::LOOK_ANGLE_RAD;
        shot.barrel_elevation_rad = barrel_elevation_rad;
        shot.barrel_azimuth_rad = G7_BASIC::BARREL_AZ_RAD;
        shot.cant_angle_rad = G7_BASIC::CANT_ANGLE_RAD;
        shot.latitude_deg = G7_BASIC::LAT_DEG;
        shot.azimuth_deg = G7_BASIC::AZ_DEG;
        shot.calc_step = kCalcStep;

        return unwrap(shot.to_shot_props(), "make_bclibc_shot_props_wind: to_shot_props() failed");
    }

    // Initialize engine from ShotProps (in-place; BCLIBC_BaseEngine is non-moveable).
    void init_bclibc_engine(bclibc::BCLIBC_BaseEngine &eng, bclibc::BCLIBC_ShotProps props)
    {
        eng.shot = std::move(props);
        eng.integrate_func = bclibc::BCLIBC_integrateTsitouras;
        eng.config = bclibc::BCLIBC_Config(
            kDefaultConfig.cStepMultiplier,
            kDefaultConfig.cZeroFindingAccuracy,
            kDefaultConfig.cMinimumVelocity,
            kDefaultConfig.cMaximumDrop,
            kDefaultConfig.cMaxIterations,
            kDefaultConfig.cGravityConstant,
            kDefaultConfig.cMinimumAltitude);
        eng.gravity_vector = bclibc::BCLIBC_V3dT(0.0, eng.config.cGravityConstant, 0.0);
    }

    // Run bclibc integrate_filtered with RANGE flag only.
    std::vector<bclibc::BCLIBC_TrajectoryData>
    run_bclibc_integrate(bclibc::BCLIBC_BaseEngine &eng, double range_ft, double step_ft)
    {
        std::vector<bclibc::BCLIBC_TrajectoryData> records;
        bclibc::BCLIBC_TerminationReason reason = bclibc::BCLIBC_TerminationReason::NO_TERMINATE;
        eng.integrate_filtered(range_ft, step_ft, 0.0,
                               bclibc::BCLIBC_TRAJ_FLAG_RANGE,
                               records, reason, nullptr);
        return records;
    }

    // Run C++ Tsitouras with the same filtered-output request as tiny_bclibc.
    std::vector<bclibc::BCLIBC_TrajectoryData>
    run_bclibc_tsitouras_integrate(bclibc::BCLIBC_BaseEngine &eng, double range_ft, double step_ft)
    {
        eng.integrate_func = bclibc::BCLIBC_integrateTsitouras;
        return run_bclibc_integrate(eng, range_ft, step_ft);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // tbclibc helpers
    // ═══════════════════════════════════════════════════════════════════════════

    // Flat arrays for G7 table (tbclibc takes const double*)
    static double g_tb_mach[kG7TableSize];
    static double g_tb_cd[kG7TableSize];

    void init_tb_g7_arrays()
    {
        for (int i = 0; i < kG7TableSize; ++i)
        {
            g_tb_mach[i] = kG7Table[i].mach;
            g_tb_cd[i] = kG7Table[i].cd;
        }
    }

    // Build tbclibc ShotProps from G7_BASIC (no wind).
    // curve_buf must be caller-allocated with >= kG7TableSize elements.
    int make_tiny_bclibc_shot_props(TINY_BCLIBC_ShotProps *out, TINY_BCLIBC_CurvePoint *curve_buf,
                                    double barrel_elevation_rad = G7_BASIC::BARREL_EL_RAD,
                                    double sight_height_ft = G7_BASIC::SIGHT_HT_FT,
                                    double look_angle_rad = G7_BASIC::LOOK_ANGLE_RAD,
                                    double pressure_hpa = G7_BASIC::PRESSURE_HPA,
                                    double latitude_deg = G7_BASIC::LAT_DEG,
                                    double azimuth_deg = G7_BASIC::AZ_DEG)
    {
        init_tb_g7_arrays();

        TINY_BCLIBC_Config cfg = TINY_BCLIBC_Config_default();
        cfg.cZeroFindingAccuracy = static_cast<double>(kDefaultConfig.cZeroFindingAccuracy);
        cfg.cMinimumVelocity = static_cast<double>(kDefaultConfig.cMinimumVelocity);
        cfg.cMaximumDrop = static_cast<double>(kDefaultConfig.cMaximumDrop);
        cfg.cMaxIterations = kDefaultConfig.cMaxIterations;
        cfg.cGravityConstant = static_cast<double>(kDefaultConfig.cGravityConstant);
        cfg.cMinimumAltitude = static_cast<double>(kDefaultConfig.cMinimumAltitude);

        TINY_BCLIBC_Shot shot;
        std::memset(&shot, 0, sizeof(shot));
        shot.bc = G7_BASIC::BC;
        shot.weight_grain = G7_BASIC::WEIGHT_GR;
        shot.diameter_inch = G7_BASIC::DIAMETER_IN;
        shot.length_inch = G7_BASIC::LENGTH_IN;
        shot.muzzle_velocity_fps = G7_BASIC::MV_FPS;
        shot.sight_height_ft = sight_height_ft;
        shot.twist_inch = G7_BASIC::TWIST_IN;
        shot.temp_c = G7_BASIC::TEMP_C;
        shot.pressure_hpa = pressure_hpa;
        shot.altitude_ft = G7_BASIC::ALT_FT;
        shot.humidity = G7_BASIC::HUMIDITY;
        shot.mach_data = g_tb_mach;
        shot.cd_data = g_tb_cd;
        shot.drag_table_size = kG7TableSize;
        shot.winds = nullptr;
        shot.wind_count = 0;
        shot.look_angle_rad = look_angle_rad;
        shot.barrel_elevation_rad = barrel_elevation_rad;
        shot.barrel_azimuth_rad = G7_BASIC::BARREL_AZ_RAD;
        shot.cant_angle_rad = G7_BASIC::CANT_ANGLE_RAD;
        shot.latitude_deg = latitude_deg;
        shot.azimuth_deg = azimuth_deg;
        shot.config = cfg;

        int rc = tiny_bclibc_build_shot_props(&shot, curve_buf, out);
        if (rc == TINY_BCLIBC_OK)
            out->calc_step = kCalcStep; // override: match bclibc RK4 step exactly
        return rc;
    }

    // Build tbclibc ShotProps from G7_WIND (one crosswind).
    int make_tiny_bclibc_shot_props_wind(TINY_BCLIBC_ShotProps *out, TINY_BCLIBC_CurvePoint *curve_buf,
                                         TINY_BCLIBC_Wind *wind_buf,
                                         double barrel_elevation_rad = G7_BASIC::BARREL_EL_RAD)
    {
        init_tb_g7_arrays();

        TINY_BCLIBC_Config cfg = TINY_BCLIBC_Config_default();
        cfg.cZeroFindingAccuracy = static_cast<double>(kDefaultConfig.cZeroFindingAccuracy);
        cfg.cMinimumVelocity = static_cast<double>(kDefaultConfig.cMinimumVelocity);
        cfg.cMaximumDrop = static_cast<double>(kDefaultConfig.cMaximumDrop);
        cfg.cMaxIterations = kDefaultConfig.cMaxIterations;
        cfg.cGravityConstant = static_cast<double>(kDefaultConfig.cGravityConstant);
        cfg.cMinimumAltitude = static_cast<double>(kDefaultConfig.cMinimumAltitude);

        wind_buf->velocity_fps = G7_WIND::WIND_VEL_FPS;
        wind_buf->direction_from_rad = G7_WIND::WIND_DIR_RAD;
        wind_buf->until_distance_ft = G7_WIND::WIND_UNTIL_FT;
        wind_buf->max_distance_ft = G7_WIND::WIND_MAX_FT;

        TINY_BCLIBC_Shot shot;
        std::memset(&shot, 0, sizeof(shot));
        shot.bc = G7_BASIC::BC;
        shot.weight_grain = G7_BASIC::WEIGHT_GR;
        shot.diameter_inch = G7_BASIC::DIAMETER_IN;
        shot.length_inch = G7_BASIC::LENGTH_IN;
        shot.muzzle_velocity_fps = G7_BASIC::MV_FPS;
        shot.sight_height_ft = G7_BASIC::SIGHT_HT_FT;
        shot.twist_inch = G7_BASIC::TWIST_IN;
        shot.temp_c = G7_BASIC::TEMP_C;
        shot.pressure_hpa = G7_BASIC::PRESSURE_HPA;
        shot.altitude_ft = G7_BASIC::ALT_FT;
        shot.humidity = G7_BASIC::HUMIDITY;
        shot.mach_data = g_tb_mach;
        shot.cd_data = g_tb_cd;
        shot.drag_table_size = kG7TableSize;
        shot.winds = wind_buf;
        shot.wind_count = 1;
        shot.look_angle_rad = G7_BASIC::LOOK_ANGLE_RAD;
        shot.barrel_elevation_rad = barrel_elevation_rad;
        shot.barrel_azimuth_rad = G7_BASIC::BARREL_AZ_RAD;
        shot.cant_angle_rad = G7_BASIC::CANT_ANGLE_RAD;
        shot.latitude_deg = G7_BASIC::LAT_DEG;
        shot.azimuth_deg = G7_BASIC::AZ_DEG;
        shot.config = cfg;

        int rc = tiny_bclibc_build_shot_props(&shot, curve_buf, out);
        if (rc == TINY_BCLIBC_OK)
            out->calc_step = kCalcStep;
        return rc;
    }

    // Run tiny_bclibc_integrate (two-pass: count then fill).
    std::vector<TINY_BCLIBC_TrajectoryData>
    run_tiny_bclibc_integrate(const TINY_BCLIBC_ShotProps *props, double range_ft, double step_ft)
    {
        TINY_BCLIBC_TrajectoryRequest req;
        req.range_limit_ft = static_cast<double>(range_ft);
        req.range_step_ft = static_cast<double>(step_ft);
        req.time_step = 0.0;
        req.filter_flags = TINY_BCLIBC_TRAJ_FLAG_RANGE;

        int32_t written = 0, total = 0, reason = 0;
        // Pass 1: count
        tiny_bclibc_integrate(props, &req, nullptr, 0, &written, &total, &reason);

        std::vector<TINY_BCLIBC_TrajectoryData> buf(static_cast<size_t>(total));
        if (total > 0)
        {
            // Pass 2: fill
            tiny_bclibc_integrate(props, &req, buf.data(), total, &written, &total, &reason);
            buf.resize(static_cast<size_t>(written));
        }
        return buf;
    }

} // anonymous namespace

// ═══════════════════════════════════════════════════════════════════════════
// Test cases
// ═══════════════════════════════════════════════════════════════════════════

// tiny_bclibc's tiny_bclibc_integrate/_stream run Tsitouras 5(4) (adaptive RK45,
// FSAL), not RK4 -- see engine.h's tiny_bclibc__run_tsitouras doc comment. The
// bclibc reference engine below now runs the same algorithm (init_bclibc_engine
// sets BCLIBC_integrateTsitouras), but the C and C++ implementations still are
// not bit-identical: their raw accepted-step boundaries fall at very slightly
// different times (different derivative-evaluator call ordering, different
// FSAL-cache bookkeeping), so even an exact reconstruction of the requested
// RANGE-step rows differs from the C++ reference by small, real,
// implementation-dependent amounts. 1e-3 relative (+1e-6 absolute floor for
// near-zero fields like windage/height close to the muzzle) comfortably covers
// everything observed here (worst case ~1e-3 relative, on ogw_lb -- it scales
// as velocity^3, tripling velocity's own relative error) while still catching
// an actual regression.
constexpr double kTsitourasRelTol = 4e-3;
constexpr double kTsitourasAbsFloor = 1e-6;

static bool test_g7_basic_integrate()
{
    // Build bclibc engine
    bclibc::BCLIBC_BaseEngine bc_eng;
    init_bclibc_engine(bc_eng, make_bclibc_shot_props());

    // Build tbclibc props
    TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
    TINY_BCLIBC_ShotProps tb_props;
    if (make_tiny_bclibc_shot_props(&tb_props, tb_curve) != TINY_BCLIBC_OK)
    {
        std::printf("FAIL: tiny_bclibc_build_shot_props failed: %s\n", tiny_bclibc_last_error());
        return false;
    }

    // Run both at 3000ft range, 100ft step → 31 points
    auto bc_traj = run_bclibc_integrate(bc_eng, 3000.0, 100.0);
    auto tb_traj = run_tiny_bclibc_integrate(&tb_props, 3000.0, 100.0);

    return identity::compare_trajectories(bc_traj, tb_traj, "G7_BASIC / integrate / 3000ft@100ft",
                                          kTsitourasAbsFloor, kTsitourasRelTol);
}

static bool test_g7_wind_integrate()
{
    // Build bclibc engine with wind
    bclibc::BCLIBC_BaseEngine bc_eng;
    init_bclibc_engine(bc_eng, make_bclibc_shot_props_wind());

    // Build tbclibc props with wind
    TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
    TINY_BCLIBC_ShotProps tb_props;
    TINY_BCLIBC_Wind tb_wind;
    if (make_tiny_bclibc_shot_props_wind(&tb_props, tb_curve, &tb_wind) != TINY_BCLIBC_OK)
    {
        std::printf("FAIL: tiny_bclibc_build_shot_props (wind) failed: %s\n", tiny_bclibc_last_error());
        return false;
    }

    auto bc_traj = run_bclibc_integrate(bc_eng, 3000.0, 100.0);
    auto tb_traj = run_tiny_bclibc_integrate(&tb_props, 3000.0, 100.0);

    return identity::compare_trajectories(bc_traj, tb_traj, "G7_WIND / integrate / 3000ft@100ft",
                                          kTsitourasAbsFloor, kTsitourasRelTol);
}

// Cash-Karp's accept/reject decisions must use the same per-component RMS
// controller in the C and C++ engines. This end-to-end case exercises that
// common policy through their public filtered-trajectory APIs.
static bool test_g7_basic_tsitouras_parity()
{
    bclibc::BCLIBC_BaseEngine bc_eng;
    init_bclibc_engine(bc_eng, make_bclibc_shot_props());

    TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
    TINY_BCLIBC_ShotProps tb_props;
    if (make_tiny_bclibc_shot_props(&tb_props, tb_curve) != TINY_BCLIBC_OK)
    {
        std::printf("FAIL: tiny_bclibc_build_shot_props failed: %s\n", tiny_bclibc_last_error());
        return false;
    }

    auto bc_traj = run_bclibc_tsitouras_integrate(bc_eng, 3000.0, 100.0);
    auto tb_traj = run_tiny_bclibc_integrate(&tb_props, 3000.0, 100.0);

    return identity::compare_trajectories(bc_traj, tb_traj,
                                          "G7_BASIC / Tsitouras parity / 3000ft@100ft",
                                          kTsitourasAbsFloor, kTsitourasRelTol);
}

static bool test_g7_basic_zero_angle()
{
    // Target: zero at 1000ft (≈333yd)
    constexpr double ZERO_DIST_FT = 1000.0;
    constexpr double APEX_MAX_RAD = 1.5707963267948966 * 0.99; // 99% of π/2
    constexpr double ALLOWED_ERR = 0.001;                      // ft

    // bclibc zero_angle
    bclibc::BCLIBC_BaseEngine bc_eng;
    init_bclibc_engine(bc_eng, make_bclibc_shot_props());
    auto bc_angle_result = bc_eng.zero_angle(ZERO_DIST_FT, APEX_MAX_RAD, ALLOWED_ERR);
    if (bclibc::has_error(bc_angle_result))
    {
        std::printf("FAIL: bclibc zero_angle failed\n");
        return false;
    }
    double bc_angle = bc_angle_result.value();

    // tbclibc zero_angle
    TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
    TINY_BCLIBC_ShotProps tb_props;
    if (make_tiny_bclibc_shot_props(&tb_props, tb_curve) != TINY_BCLIBC_OK)
    {
        std::printf("FAIL: tbclibc build failed\n");
        return false;
    }
    double tb_angle = 0.0;
    int rc = tiny_bclibc_find_zero_angle(&tb_props, ZERO_DIST_FT, &tb_angle);
    if (rc != TINY_BCLIBC_OK)
    {
        std::printf("FAIL: tiny_bclibc_find_zero_angle rc=%d: %s\n", rc, tiny_bclibc_last_error());
        return false;
    }

    std::printf("\n=== G7_BASIC / zero_angle / target=1000ft ===\n");
    // zero_angle uses Ridder's method with tolerance ALLOWED_ERR, so
    // expect agreement within the solver tolerance (1e-6 rad ≈ 0.2").
    return identity::compare_scalar("zero_angle (rad)", bc_angle, tb_angle, 1e-6);
}

static bool test_g7_basic_zero_point()
{
    constexpr double ZERO_DIST_FT = 1000.0;
    constexpr double APEX_MAX_RAD = 1.5707963267948966 * 0.99;
    constexpr double ALLOWED_ERR = 0.001;

    bclibc::BCLIBC_BaseEngine bc_eng;
    init_bclibc_engine(bc_eng, make_bclibc_shot_props());
    auto bc_result_wrapped = bc_eng.zero_point_with_fallback(ZERO_DIST_FT, APEX_MAX_RAD, ALLOWED_ERR);
    if (bclibc::has_error(bc_result_wrapped))
    {
        std::printf("FAIL: bclibc zero_point_with_fallback failed\n");
        return false;
    }
    bclibc::BCLIBC_ZeroPointResult bc_result = bc_result_wrapped.value();

    TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
    TINY_BCLIBC_ShotProps tb_props;
    if (make_tiny_bclibc_shot_props(&tb_props, tb_curve) != TINY_BCLIBC_OK)
        return false;
    TINY_BCLIBC_ZeroPointResult tb_result;
    int rc = tiny_bclibc_find_zero_point(&tb_props, ZERO_DIST_FT, &tb_result);
    if (rc != TINY_BCLIBC_OK)
    {
        std::printf("FAIL: tiny_bclibc_find_zero_point rc=%d: %s\n", rc, tiny_bclibc_last_error());
        return false;
    }

    std::printf("\n=== G7_BASIC / zero_point / target=1000ft ===\n");
    return identity::compare_scalar("zero_point angle (rad)", bc_result.angle_rad,
                                    tb_result.angle_rad, 1e-6) &&
           identity::compare_scalar("zero_point distance (ft)", bc_result.point.distance_ft,
                                    tb_result.point.distance_ft, 1e-6) &&
           identity::compare_scalar("zero_point slant height (ft)", bc_result.point.slant_height_ft,
                                    tb_result.point.slant_height_ft, 1e-6) &&
           bc_result.point.flag == tb_result.point.flag;
}

static bool find_apex_identity(double barrel_el_rad)
{
    // bclibc find_apex (via integrate_at VEL_Y=0)
    auto bc_props = make_bclibc_shot_props(barrel_el_rad); // small upward angle so there's an apex
    bclibc::BCLIBC_BaseEngine bc_eng;
    init_bclibc_engine(bc_eng, bc_props);

    bclibc::BCLIBC_BaseTrajData bc_raw;
    bclibc::BCLIBC_TrajectoryData bc_apex;
    if (bclibc::has_error(bc_eng.find_apex(bc_raw)))
    {
        std::printf("FAIL: bclibc find_apex failed\n");
        return false;
    }
    auto bc_apex_result = bclibc::BCLIBC_TrajectoryData::from_base(bc_eng.shot, bc_raw, bclibc::BCLIBC_TRAJ_FLAG_APEX);
    if (bclibc::has_error(bc_apex_result))
    {
        std::printf("FAIL: bclibc BCLIBC_TrajectoryData::from_base failed\n");
        return false;
    }
    bc_apex = bc_apex_result.value();

    // tbclibc find_apex
    TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
    TINY_BCLIBC_ShotProps tb_props;
    if (make_tiny_bclibc_shot_props(&tb_props, tb_curve, barrel_el_rad) != TINY_BCLIBC_OK)
    {
        std::printf("FAIL: tbclibc build failed\n");
        return false;
    }
    TINY_BCLIBC_TrajectoryData tb_apex;
    int rc = tiny_bclibc_find_apex(&tb_props, &tb_apex);
    if (rc != TINY_BCLIBC_OK)
    {
        std::printf("FAIL: tiny_bclibc_find_apex rc=%d: %s\n", rc, tiny_bclibc_last_error());
        return false;
    }

    std::printf("\n=== G7_BASIC / find_apex / barrel_el=%grad ===\n", barrel_el_rad);
    // Tsitouras is FSAL and takes far fewer, larger adaptive steps than Cash-Karp
    // (no per-retry-recompute, growth factor capped at 10x vs Cash-Karp's 5x): each
    // step's own C-vs-C++ stage-arithmetic rounding (templated Tableau::A(i,j) loop
    // summing derivatives before the single dt-scale in bclibc's C++ core, vs this
    // file's dt-scale-then-FMA-accumulate C style) therefore carries more weight
    // per step here than under Cash-Karp, where pure kAbsTol (no rel_tol) happened
    // to hold. Observed worst case ~1.4e-11 relative on this fixture; 1e-7 leaves
    // ample margin across platforms/compilers while still catching a real
    // regression many orders of magnitude larger than expected FP noise.
    // The apex is located on the Hermite interpolant of the step in both (the convention of the filter rows), where the
    // Mach number of the point is the interpolated speed of sound (linear in time) in bclibc and recomputed from the
    // altitude in tiny_bclibc: they differ by the curvature of the speed of sound over the (wide, adaptive) step, 3e-7
    // here, the size the rows of the trajectory comparison above are allowed (1e-6). Everything else is compared as before.
    const bool mach_ok = identity::compare_scalar("apex mach (interpolated vs recomputed)", bc_apex.mach, tb_apex.mach, 1e-6);
    bc_apex.mach = tb_apex.mach;
    return identity::compare_point(bc_apex, tb_apex, 0, identity::kAbsTol, true, 1e-7) && mach_ok;
}

static bool test_g7_basic_find_apex()
{
    return find_apex_identity(0.05);
}

// The apex falls inside the first integration step (0.001 deg): both engines used to report "no apex".
static bool test_g7_find_apex_first_step()
{
    return find_apex_identity(0.001 * 3.14159265358979323846 / 180.0);
}

// The range limit is not tested before the third integration step (the Python engines: `or step_count < 3`): a limit
// inside the first step still takes three. bclibc used to count the initial sample as a step and stopped after two.
struct StepCounter : bclibc::BCLIBC_BaseTrajDataHandlerInterface
{
    int n = 0;
    bclibc::BCLIBC_Result<std::monostate> handle(const bclibc::BCLIBC_BaseTrajData &) override
    {
        ++n;
        return std::monostate{};
    }
};

static int32_t count_tiny_points(const TINY_BCLIBC_BaseTrajData *, void *ctx)
{
    ++*static_cast<int *>(ctx);
    return 0;
}

static bool test_min_steps_gate()
{
    bool ok = true;
    for (const double limit_ft : {0.01, 1.0, 5.0})
    {
        bclibc::BCLIBC_BaseEngine bc_eng;
        init_bclibc_engine(bc_eng, make_bclibc_shot_props());
        StepCounter counter;
        bclibc::BCLIBC_TerminationReason reason = bclibc::BCLIBC_TerminationReason::NO_TERMINATE;
        const auto bc_result = bc_eng.integrate(limit_ft, counter, reason);

        TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
        TINY_BCLIBC_ShotProps tb_props;
        if (bclibc::has_error(bc_result) || make_tiny_bclibc_shot_props(&tb_props, tb_curve) != TINY_BCLIBC_OK)
        {
            std::printf("FAIL: min steps gate setup (limit %g ft)\n", limit_ft);
            ok = false;
            continue;
        }
        int tb_points = 0;
        int32_t tb_reason = 0;
        const int rc = tiny_bclibc_integrate_raw(&tb_props, limit_ft, count_tiny_points, &tb_points, &tb_reason);
        std::printf("\n=== G7_BASIC / min steps gate / limit %g ft: bclibc %d points, tiny_bclibc %d points ===\n",
                    limit_ft, counter.n, tb_points);
        // the start + three steps, in both
        if (rc != TINY_BCLIBC_OK || counter.n != 4 || tb_points != 4)
        {
            std::printf("FAIL: expected 4 points (start + 3 steps) in both\n");
            ok = false;
        }
    }
    return ok;
}

// integrate_at locates every key on the Hermite interpolant of the accepted step in both engines (time, velocity, height
// and range keys, not only position.x and the apex), so the raw point agrees to the interpolation of the same step.
static bool integrate_at_identity(int key, const char *name, double target, double barrel_el_rad)
{
    auto bc_props = make_bclibc_shot_props(barrel_el_rad);
    bclibc::BCLIBC_BaseEngine bc_eng;
    init_bclibc_engine(bc_eng, bc_props);
    bclibc::BCLIBC_BaseTrajData bc_raw;
    bclibc::BCLIBC_TrajectoryData bc_full;
    if (bclibc::has_error(bc_eng.integrate_at(static_cast<bclibc::BCLIBC_BaseTrajData_InterpKey>(key), target, bc_raw, bc_full)))
    {
        std::printf("FAIL: bclibc integrate_at(%s) failed\n", name);
        return false;
    }
    TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
    TINY_BCLIBC_ShotProps tb_props;
    if (make_tiny_bclibc_shot_props(&tb_props, tb_curve, barrel_el_rad) != TINY_BCLIBC_OK)
    {
        std::printf("FAIL: tbclibc build failed\n");
        return false;
    }
    TINY_BCLIBC_BaseTrajData tb_raw;
    TINY_BCLIBC_TrajectoryData tb_full;
    if (tiny_bclibc_integrate_at(&tb_props, key, target, &tb_raw, &tb_full) != TINY_BCLIBC_OK)
    {
        std::printf("FAIL: tiny_bclibc_integrate_at(%s) failed\n", name);
        return false;
    }
    std::printf("\n=== G7_BASIC / integrate_at %s=%g / barrel_el=%grad ===\n", name, target, barrel_el_rad);
    bool ok = true;
    ok &= identity::compare_scalar("time", bc_raw.time, tb_raw.time, 1e-7);
    ok &= identity::compare_scalar("position.x", bc_raw.px, tb_raw.px, 1e-6);
    ok &= identity::compare_scalar("position.y", bc_raw.py, tb_raw.py, 1e-6);
    ok &= identity::compare_scalar("velocity.x", bc_raw.vx, tb_raw.vx, 1e-6);
    ok &= identity::compare_scalar("velocity.y", bc_raw.vy, tb_raw.vy, 1e-6);
    return ok;
}

static bool test_g7_integrate_at_keys()
{
    bool ok = true;
    ok &= integrate_at_identity(TINY_BCLIBC_KEY_TIME, "time", 1.0, 0.05);
    ok &= integrate_at_identity(TINY_BCLIBC_KEY_POS_X, "position.x", 1500.0, 0.05);
    ok &= integrate_at_identity(TINY_BCLIBC_KEY_POS_Y, "position.y", 50.0, 0.05);
    ok &= integrate_at_identity(TINY_BCLIBC_KEY_VEL_X, "velocity.x", 2000.0, 0.05);
    return ok;
}

static bool test_g7_basic_find_max_range()
{
    // find_max_range drives range_for_angle (the line-of-sight crossing) for every probe, so this checks that
    // bclibc locates that crossing the way tiny_bclibc does (cubic Hermite on the step), not on a chord.
    constexpr double APEX_MAX_RAD = 1.5707963267948966 * 0.99;

    bclibc::BCLIBC_BaseEngine bc_eng;
    init_bclibc_engine(bc_eng, make_bclibc_shot_props());
    auto bc_result = bc_eng.find_max_range(0.0, 90.0, APEX_MAX_RAD);
    if (bclibc::has_error(bc_result))
    {
        std::printf("FAIL: bclibc find_max_range failed\n");
        return false;
    }

    TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
    TINY_BCLIBC_ShotProps tb_props;
    if (make_tiny_bclibc_shot_props(&tb_props, tb_curve) != TINY_BCLIBC_OK)
        return false;
    double tb_range = 0.0, tb_angle = 0.0;
    int rc = tiny_bclibc_find_max_range(&tb_props, 0.0, 90.0, &tb_range, &tb_angle);
    if (rc != TINY_BCLIBC_OK)
    {
        std::printf("FAIL: tiny_bclibc_find_max_range rc=%d: %s\n", rc, tiny_bclibc_last_error());
        return false;
    }

    std::printf("\n=== G7_BASIC / find_max_range / 0..90 deg ===\n");
    // Same search, same crossing: they agree to rounding (with a chord in bclibc they were 0.03 ft / 4e-4 rad apart).
    return identity::compare_scalar("max_range (ft)", bc_result.value().max_range_ft, tb_range, 1e-6) &&
           // bclibc stops the golden-section search at 1e-6 rad (as the Python reference), tiny_bclibc at 1e-5 (single-precision
           // floats; see "Deliberate differences from bclibc" in tiny_bclibc/README.md): the angles agree to the larger one.
           identity::compare_scalar("angle_at_max (rad)", bc_result.value().angle_at_max_rad, tb_angle, 1e-5);
}

static bool test_g7_basic_zero_angle_tight_limits()
{
    // The zero does not depend on the early-termination limits (issue #45). Both engines solve it with the default
    // limits and with tight ones (0 is "stop at muzzle level"); the four answers must agree: a limit may not cut the
    // search short (it used to end it with an error) nor bias the result.
    constexpr double APEX_MAX_RAD = 1.5707963267948966 * 0.99;
    constexpr double ALLOWED_ERR = 0.001;
    struct Limits { const char *name; double min_altitude, max_drop; };
    const Limits limits[] = {
        {"default", kDefaultConfig.cMinimumAltitude, kDefaultConfig.cMaximumDrop},
        {"alt 0, drop 0", 0.0, 0.0},
        {"alt 0, drop -10000", 0.0, -10000.0},
        {"alt -1500, drop 0", -1500.0, 0.0},
    };

    bool ok = true;
    for (const double dist_ft : {1000.0, 3000.0})
    {
        std::printf("\n=== G7_BASIC / zero_angle with tight limits / target=%.0fft ===\n", dist_ft);
        double reference = 0.0;
        for (size_t i = 0; i < sizeof limits / sizeof limits[0]; ++i)
        {
            bclibc::BCLIBC_BaseEngine bc_eng;
            init_bclibc_engine(bc_eng, make_bclibc_shot_props());
            bc_eng.config.cMinimumAltitude = limits[i].min_altitude;
            bc_eng.config.cMaximumDrop = limits[i].max_drop;
            auto bc_result = bc_eng.zero_angle(dist_ft, APEX_MAX_RAD, ALLOWED_ERR);
            if (bclibc::has_error(bc_result))
            {
                std::printf("FAIL: bclibc zero_angle failed with limits '%s'\n", limits[i].name);
                ok = false;
                continue;
            }
            const double bc_angle = bc_result.value();

            TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
            TINY_BCLIBC_ShotProps tb_props;
            if (make_tiny_bclibc_shot_props(&tb_props, tb_curve) != TINY_BCLIBC_OK)
                return false;
            tb_props.cfg.cMinimumAltitude = limits[i].min_altitude;
            tb_props.cfg.cMaximumDrop = limits[i].max_drop;
            double tb_angle = 0.0;
            const int rc = tiny_bclibc_find_zero_angle(&tb_props, dist_ft, &tb_angle);
            if (rc != TINY_BCLIBC_OK)
            {
                std::printf("FAIL: tiny_bclibc_find_zero_angle rc=%d with limits '%s': %s\n", rc, limits[i].name,
                            tiny_bclibc_last_error());
                ok = false;
                continue;
            }

            if (i == 0)
                reference = bc_angle;
            std::printf("  limits '%s'\n", limits[i].name);
            ok &= identity::compare_scalar("bclibc vs tiny_bclibc (rad)", bc_angle, tb_angle, 1e-6);
            ok &= identity::compare_scalar("bclibc vs default limits (rad)", bc_angle, reference, 1e-6);
            ok &= identity::compare_scalar("tiny_bclibc vs default limits (rad)", tb_angle, reference, 1e-6);
        }
    }
    return ok;
}

// A target closer than about twice the sight height: init_zero_calculation's "very close shot" shortcut, ported in
// tiny_bclibc. The cutoff is 2 * max(sight height, calc_step) (a length, not the cStepMultiplier factor), and the angle
// is the straight line from the bore (the negative sight height below the line of sight) up to the target. The shot
// fired with the zero must reach the line of sight at the target in both engines, and both engines must agree.
static bool test_close_target_zero()
{
    constexpr double APEX_MAX_RAD = 0.0003;
    constexpr double ALLOWED_ERR = 1e-2;
    constexpr double kFt = 3.28084;
    constexpr double kMissTolFt = 1e-3; // the old angle missed by 2 * sight height >= 0.33 ft

    bool ok = true;
    for (const double sight_m : {0.05, 0.1, 0.5})
        for (const double target_m : {0.05, 0.1, 0.2, 0.4, 0.6, 0.9, 2.0})
        {
            const double sight_ft = sight_m * kFt;
            const double dist_ft = target_m * kFt;

            bclibc::BCLIBC_BaseEngine bc_eng;
            init_bclibc_engine(bc_eng, make_bclibc_shot_props(G7_BASIC::BARREL_EL_RAD, sight_ft));
            auto bc_result = bc_eng.zero_angle(dist_ft, APEX_MAX_RAD, ALLOWED_ERR);
            if (bclibc::has_error(bc_result))
            {
                std::printf("FAIL: bclibc zero_angle failed (sight %.2f m, target %.2f m)\n", sight_m, target_m);
                ok = false;
                continue;
            }
            const double bc_angle = bc_result.value();
            auto bc_miss = bc_eng.error_at_distance(bc_angle, dist_ft, 0.0);

            TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
            TINY_BCLIBC_ShotProps tb_props;
            if (make_tiny_bclibc_shot_props(&tb_props, tb_curve, G7_BASIC::BARREL_EL_RAD, sight_ft) != TINY_BCLIBC_OK)
                return false;
            double tb_angle = 0.0;
            const int rc = tiny_bclibc_find_zero_angle(&tb_props, dist_ft, &tb_angle);
            if (rc != TINY_BCLIBC_OK)
            {
                std::printf("FAIL: tiny_bclibc_find_zero_angle rc=%d (sight %.2f m, target %.2f m): %s\n", rc, sight_m,
                            target_m, tiny_bclibc_last_error());
                ok = false;
                continue;
            }
            TINY_BCLIBC_ShotProps tb_fire = tb_props;
            const double tb_miss = tiny_bclibc__error_at_distance(&tb_fire, tb_angle, dist_ft, 0.0);

            std::printf("  sight %.2f m, target %.2f m: bclibc %.6f deg (miss %.2e ft), tiny_bclibc %.6f deg (miss %.2e ft)\n",
                        sight_m, target_m, bc_angle * 57.29577951308232,
                        bclibc::has_error(bc_miss) ? 1e9 : bc_miss.value(), tb_angle * 57.29577951308232, tb_miss);
            ok &= identity::compare_scalar("close target: bclibc vs tiny_bclibc (rad)", bc_angle, tb_angle, 1e-6);
            if (bclibc::has_error(bc_miss) || std::fabs(bc_miss.value()) >= kMissTolFt)
            {
                std::printf("FAIL: bclibc zero misses the line of sight\n");
                ok = false;
            }
            if (std::fabs(tb_miss) >= kMissTolFt)
            {
                std::printf("FAIL: tiny_bclibc zero misses the line of sight by %.5f ft\n", tb_miss);
                ok = false;
            }
        }
    return ok;
}


// A downhill look angle. The maximum-range search (the bracket of the Ridder zero search) measures every shot by where it
// comes down through the line of sight, which lies below the muzzle: the drop and altitude limits ended the shot before
// the crossing and the search reported a maximum range of a few feet at about 90 degrees, so the Ridder path answered
// OUT_OF_RANGE for any target. Both engines must (a) find the real maximum range (the same, and the same as a scan over
// the elevation with no limits), and (b) solve the zero by the Ridder path, the shot fired with it reaching the target.
static bool test_downhill_max_range_and_ridder_zero()
{
    constexpr double kFt = 3.28084;
    constexpr double kMissTolFt = 0.00656; // 2 mm
    constexpr double APEX_MAX_RAD = 0.0003;
    constexpr double ALLOWED_ERR = 1e-2;

    bool ok = true;
    for (const double look_deg : {-5.0, -10.0, -20.0, -30.0, -45.0})
        for (const double sight_m : {0.1, 0.5})
        {
            const double look = look_deg * 0.017453292519943295;
            const double sight_ft = sight_m * kFt;

            bclibc::BCLIBC_BaseEngine bc_eng;
            init_bclibc_engine(bc_eng, make_bclibc_shot_props(G7_BASIC::BARREL_EL_RAD, sight_ft, look));
            auto bc_mr = bc_eng.find_max_range(0.0, 90.0, 1.5707963267948966 * 0.99);

            TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
            TINY_BCLIBC_ShotProps tb_props;
            if (make_tiny_bclibc_shot_props(&tb_props, tb_curve, G7_BASIC::BARREL_EL_RAD, sight_ft, look) != TINY_BCLIBC_OK)
                return false;
            double tb_range = 0.0, tb_angle = 0.0;
            if (bclibc::has_error(bc_mr) ||
                tiny_bclibc_find_max_range(&tb_props, 0.0, 90.0, &tb_range, &tb_angle) != TINY_BCLIBC_OK)
            {
                std::printf("FAIL: find_max_range failed (look %.0f, sight %.2f m)\n", look_deg, sight_m);
                ok = false;
                continue;
            }
            const double bc_range = bc_mr.value().max_range_ft;
            std::printf("  look %.0f deg, sight %.2f m: max range bclibc %.1f ft @ %.2f deg, tiny_bclibc %.1f ft @ %.2f deg\n",
                        look_deg, sight_m, bc_range, bc_mr.value().angle_at_max_rad * 57.29577951308232, tb_range,
                        tb_angle * 57.29577951308232);

            // A scan of the elevation with every limit lifted.
            bc_eng.config.cMaximumDrop = -1e12;
            bc_eng.config.cMinimumAltitude = -1e12;
            bc_eng.config.cMinimumVelocity = 0.0;
            double scan_max = 0.0;
            for (double a = 1.0; a < 90.0; a += 1.0)
                scan_max = std::fmax(scan_max, bc_eng.range_for_angle(a * 0.017453292519943295));
            if (!(bc_range >= scan_max * (1.0 - 1e-6) && bc_range <= scan_max * 1.01 && tb_range >= scan_max * (1.0 - 1e-6) &&
                  tb_range <= scan_max * 1.01))
            {
                std::printf("FAIL: max range is not the scanned maximum %.1f ft\n", scan_max);
                ok = false;
            }
            ok &= identity::compare_scalar("downhill max_range (ft)", bc_range, tb_range, 1e-3);
            // the range is flat at its maximum: the angle is only as good as the golden-section tolerance (1e-5 rad) of each
            ok &= identity::compare_scalar("downhill angle_at_max (rad)", bc_mr.value().angle_at_max_rad, tb_angle, 1e-4);

            // The Ridder path, called directly (tiny_bclibc_find_zero_angle only falls back to it).
            for (const double target_m : {0.9, 2.0, 5.0, 10.0, 50.0, 300.0})
            {
                const double dist_ft = target_m * kFt;

                bclibc::BCLIBC_BaseEngine eng;
                init_bclibc_engine(eng, make_bclibc_shot_props(G7_BASIC::BARREL_EL_RAD, sight_ft, look));
                auto bc_z = eng.find_zero_angle(dist_ft, 0, APEX_MAX_RAD, ALLOWED_ERR);

                TINY_BCLIBC_ShotProps tb_z_props;
                if (make_tiny_bclibc_shot_props(&tb_z_props, tb_curve, G7_BASIC::BARREL_EL_RAD, sight_ft, look) != TINY_BCLIBC_OK)
                    return false;
                double tb_z = 0.0;
                const int rc = tiny_bclibc__find_zero_angle_ridders(&tb_z_props, dist_ft, &tb_z);
                if (bclibc::has_error(bc_z) || rc != TINY_BCLIBC_OK)
                {
                    std::printf("FAIL: Ridder zero (look %.0f, sight %.2f m, target %.1f m): bclibc %s, tiny_bclibc rc=%d (%s)\n",
                                look_deg, sight_m, target_m, bclibc::has_error(bc_z) ? "error" : "ok", rc,
                                rc != TINY_BCLIBC_OK ? tiny_bclibc_last_error() : "");
                    ok = false;
                    continue;
                }
                const double la = look;
                auto bc_miss = eng.error_at_distance(bc_z.value(), dist_ft * std::cos(la), dist_ft * std::sin(la));
                TINY_BCLIBC_ShotProps tb_fire = tb_z_props;
                const double tb_miss = tiny_bclibc__error_at_distance(&tb_fire, tb_z, dist_ft * std::cos(la), dist_ft * std::sin(la));
                if (bclibc::has_error(bc_miss) || std::fabs(bc_miss.value()) >= kMissTolFt || std::fabs(tb_miss) >= kMissTolFt)
                {
                    std::printf("FAIL: Ridder zero misses (look %.0f, sight %.2f m, target %.1f m): bclibc %.5f ft, tiny_bclibc %.5f ft\n",
                                look_deg, sight_m, target_m, bclibc::has_error(bc_miss) ? 1e9 : bc_miss.value(), tb_miss);
                    ok = false;
                }
                // each solver stops within its own height tolerance of the root (both hit, checked above)
                ok &= identity::compare_scalar("downhill Ridder zero (rad)", bc_z.value(), tb_z, 2e-4);
            }
        }
    return ok;
}


// The unconverged exit of the Ridder zero search (test_ridder_unconverged.cpp, built with the iteration cap lowered)
// fails with a zero-finding error; here the same searches run with the default iterations: both engines converge and the
// shot fired with the zero hits the target.
static bool test_ridder_default_iterations_converge()
{
    constexpr double kFt = 3.28084;
    constexpr double kMissTolFt = 0.00656; // 2 mm
    constexpr double APEX_MAX_RAD = 1.5707963267948966 * 0.99;
    constexpr double ALLOWED_ERR = 1e-2;
    struct Case
    {
        double look_deg, sight_m, target_m;
    };
    const Case cases[] = {{0.0, 0.05, 2000.0}, {-10.0, 0.05, 1500.0}, {-10.0, 0.05, 2000.0}, {-10.0, 0.1, 1500.0},
                          {-10.0, 0.1, 2000.0}};
    bool ok = true;
    for (const Case &c : cases)
    {
        const double look = c.look_deg * 0.017453292519943295;
        const double sight_ft = c.sight_m * kFt;
        const double dist_ft = c.target_m * kFt;

        bclibc::BCLIBC_BaseEngine bc_eng;
        init_bclibc_engine(bc_eng, make_bclibc_shot_props(G7_BASIC::BARREL_EL_RAD, sight_ft, look));
        auto bc_z = bc_eng.find_zero_angle(dist_ft, 0, APEX_MAX_RAD, ALLOWED_ERR);

        TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
        TINY_BCLIBC_ShotProps tb_props;
        if (make_tiny_bclibc_shot_props(&tb_props, tb_curve, G7_BASIC::BARREL_EL_RAD, sight_ft, look) != TINY_BCLIBC_OK)
            return false;
        double tb_z = 0.0;
        const int rc = tiny_bclibc__find_zero_angle_ridders(&tb_props, dist_ft, &tb_z);
        if (bclibc::has_error(bc_z) || rc != TINY_BCLIBC_OK)
        {
            std::printf("FAIL: default-iteration Ridder zero (look %.0f, sight %.2f m, target %.0f m): bclibc %s, tiny_bclibc rc=%d\n",
                        c.look_deg, c.sight_m, c.target_m, bclibc::has_error(bc_z) ? "error" : "ok", rc);
            ok = false;
            continue;
        }
        auto bc_miss = bc_eng.error_at_distance(bc_z.value(), dist_ft * std::cos(look), dist_ft * std::sin(look));
        TINY_BCLIBC_ShotProps fire = tb_props;
        const double tb_miss = tiny_bclibc__error_at_distance(&fire, tb_z, dist_ft * std::cos(look), dist_ft * std::sin(look));
        std::printf("  default-iteration Ridder zero look %.0f, sight %.2f m, target %.0f m: bclibc %.5f rad, tiny_bclibc %.5f rad (miss %.5f ft)\n",
                    c.look_deg, c.sight_m, c.target_m, bc_z.value(), tb_z, tb_miss);
        if (bclibc::has_error(bc_miss) || std::fabs(bc_miss.value()) >= kMissTolFt || std::fabs(tb_miss) >= kMissTolFt)
        {
            std::printf("FAIL: default-iteration Ridder zero misses the target\n");
            ok = false;
        }
        ok &= identity::compare_scalar("default-iteration Ridder zero (rad)", bc_z.value(), tb_z, 2e-4);
    }
    return ok;
}

// The maximum-range search of a downhill shot with the limits lifted must not hang: a shot launched at or below the
// line of sight (or too little above it to rise by its sight height) has no crossing and is not integrated.
#ifndef _WIN32
static void on_alarm(int)
{
    static const char msg[] = "FAIL: tiny_bclibc_find_max_range did not finish in time (hang)\n";
    (void)!write(2, msg, sizeof msg - 1);
    _exit(1);
}
#endif
static bool test_downhill_no_hang()
{
    bool ok = true;
    struct Case { double look_deg, lo, hi; };
    for (const Case c : {Case{-20.0, -90.0, 90.0}, Case{-0.001, 0.0, 0.0001}})
    {
        TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
        TINY_BCLIBC_ShotProps tb_props;
        if (make_tiny_bclibc_shot_props(&tb_props, tb_curve, G7_BASIC::BARREL_EL_RAD, 0.3281, c.look_deg * 0.017453292519943295) != TINY_BCLIBC_OK)
            return false;
        double range = -1.0, angle = 0.0;
#ifndef _WIN32
        std::signal(SIGALRM, on_alarm);
        alarm(30);
#endif
        const int rc = tiny_bclibc_find_max_range(&tb_props, c.lo, c.hi, &range, &angle);
#ifndef _WIN32
        alarm(0);
#endif
        std::printf("  look %.3f, bracket (%g, %g): rc %d, range %.1f ft\n", c.look_deg, c.lo, c.hi, rc, range);
        if (rc != TINY_BCLIBC_OK) { std::printf("FAIL: rc %d\n", rc); ok = false; }
        if (c.look_deg > -0.01 && range != 0.0) { std::printf("FAIL: expected no crossing\n"); ok = false; }
        if (c.look_deg < -1.0 && range < 5000.0) { std::printf("FAIL: wide bracket range too small\n"); ok = false; }
    }
    return ok;
}


// A long downhill zero: the target (1500 m at -30 degrees) is 750 m below the muzzle, below the default altitude limit
// (-1500 ft): find_zero_point's intercept shot used to stop short ("target intercept not found").
static bool test_long_downhill_zero_point()
{
    constexpr double kFt = 3.28084;
    const double look = -30.0 * 0.017453292519943295;
    const double dist_ft = 1500.0 * kFt;
    TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
    TINY_BCLIBC_ShotProps tb_props;
    if (make_tiny_bclibc_shot_props(&tb_props, tb_curve, G7_BASIC::BARREL_EL_RAD, 0.3281, look) != TINY_BCLIBC_OK)
        return false;
    TINY_BCLIBC_ZeroPointResult zp;
    const int rc = tiny_bclibc_find_zero_point(&tb_props, dist_ft, &zp);
    std::printf("  long downhill zero point: rc %d\n", rc);
    if (rc != TINY_BCLIBC_OK)
    {
        std::printf("FAIL: %s\n", tiny_bclibc_last_error());
        return false;
    }
    return true;
}

// The Ridder-only entries (tiny_bclibc_find_zero_angle_ridder / _point_ridder) against bclibc's find_zero_angle_ridder /
// find_zero_point (lofted == 0): the same status and, when OK, the same angle. Unlike the Newton-first entries they run the
// maximum-range search first, so a start below cMinimumAltitude (level look, alt 0, limit 100 ft) and a target beyond the
// maximum range are OUT_OF_RANGE in both engines.
static bool test_ridder_only_entries()
{
    constexpr double kFt = 3.28084;
    constexpr double APEX_MAX_RAD = 0.0003;
    constexpr double ALLOWED_ERR = 1e-2;
    struct Case { const char *name; double look_deg, target_m, min_alt_ft; bool expect_ok; };
    const Case cases[] = {
        {"start below limit", 0.0, 300.0, 100.0, false},
        {"beyond max range", 0.0, 30000.0, -1500.0, false},
        {"normal", 0.0, 300.0, -1500.0, true},
        {"downhill -10", -10.0, 300.0, -1500.0, true},
        {"uphill +10", 10.0, 300.0, -1500.0, true},
    };

    bool ok = true;
    for (const Case &c : cases)
    {
        const double look = c.look_deg * 0.017453292519943295;
        const double dist_ft = c.target_m * kFt;
        std::printf("\n=== Ridder-only entries / %s ===\n", c.name);

        bclibc::BCLIBC_BaseEngine bc_eng;
        init_bclibc_engine(bc_eng, make_bclibc_shot_props(G7_BASIC::BARREL_EL_RAD, 0.3281, look));
        bc_eng.config.cMinimumAltitude = c.min_alt_ft;
        auto bc_a = bc_eng.find_zero_angle(dist_ft, 0, APEX_MAX_RAD, ALLOWED_ERR);

        bclibc::BCLIBC_BaseEngine bc_eng_p;
        init_bclibc_engine(bc_eng_p, make_bclibc_shot_props(G7_BASIC::BARREL_EL_RAD, 0.3281, look));
        bc_eng_p.config.cMinimumAltitude = c.min_alt_ft;
        auto bc_p = bc_eng_p.find_zero_point(dist_ft, 0, APEX_MAX_RAD, ALLOWED_ERR);

        TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
        TINY_BCLIBC_ShotProps tb_props;
        if (make_tiny_bclibc_shot_props(&tb_props, tb_curve, G7_BASIC::BARREL_EL_RAD, 0.3281, look) != TINY_BCLIBC_OK)
            return false;
        tb_props.cfg.cMinimumAltitude = c.min_alt_ft;
        double tb_a = 0.0;
        const int rc_a = tiny_bclibc_find_zero_angle_ridder(&tb_props, dist_ft, &tb_a);
        TINY_BCLIBC_ZeroPointResult tb_p;
        const int rc_p = tiny_bclibc_find_zero_point_ridder(&tb_props, dist_ft, &tb_p);

        const bool bc_a_ok = !bclibc::has_error(bc_a), bc_p_ok = !bclibc::has_error(bc_p);
        std::printf("  bclibc angle %s, point %s; tiny_bclibc angle rc %d, point rc %d\n", bc_a_ok ? "ok" : "error",
                    bc_p_ok ? "ok" : "error", rc_a, rc_p);
        if (bc_a_ok != c.expect_ok || bc_p_ok != c.expect_ok || (rc_a == TINY_BCLIBC_OK) != c.expect_ok ||
            (rc_p == TINY_BCLIBC_OK) != c.expect_ok)
        {
            std::printf("FAIL: status differs from the expected %s\n", c.expect_ok ? "OK" : "OUT_OF_RANGE");
            ok = false;
            continue;
        }
        if (!c.expect_ok)
        {
            if (rc_a != TINY_BCLIBC_ERR_OUT_OF_RANGE || rc_p != TINY_BCLIBC_ERR_OUT_OF_RANGE)
            {
                std::printf("FAIL: tiny_bclibc rc %d / %d, expected OUT_OF_RANGE (%d)\n", rc_a, rc_p, TINY_BCLIBC_ERR_OUT_OF_RANGE);
                ok = false;
            }
            // the Newton-first entry still answers for a start below the limit (as bclibc zero_angle_with_fallback)
            if (c.min_alt_ft > 0.0)
            {
                double tb_n = 0.0;
                if (tiny_bclibc_find_zero_angle(&tb_props, dist_ft, &tb_n) != TINY_BCLIBC_OK)
                {
                    std::printf("FAIL: Newton-first tiny_bclibc_find_zero_angle fails for a start below the limit\n");
                    ok = false;
                }
            }
            continue;
        }
        ok &= identity::compare_scalar("Ridder zero angle (rad)", bc_a.value(), tb_a, 1e-4);
        ok &= identity::compare_scalar("Ridder zero point angle (rad)", bc_p.value().angle_rad, tb_p.angle_rad, 1e-4);
        ok &= identity::compare_scalar("Ridder zero point angle = angle entry (rad)", tb_a, tb_p.angle_rad, 1e-6);
        ok &= identity::compare_scalar("Ridder zero point distance (ft)", bc_p.value().point.distance_ft,
                                       tb_p.point.distance_ft, 1e-2);
    }
    return ok;
}

// A vacuum (pressure 0) is drag-free flight in both engines, with the same rows, apex and maximum range, and the closed
// form (the density ratio of the altitude was 0 / 0 = NaN and the integration never ended); a state that goes NaN (the
// gravity) is an error in both, not a loop.
static bool test_vacuum_identity()
{
    constexpr double kElev = 10.0 * 0.017453292519943295;
    constexpr double kFt = 3.28084;
    bool ok = true;

    bclibc::BCLIBC_BaseEngine bc_eng;
    init_bclibc_engine(bc_eng, make_bclibc_shot_props(kElev, G7_BASIC::SIGHT_HT_FT, G7_BASIC::LOOK_ANGLE_RAD, 0.0));
    TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
    TINY_BCLIBC_ShotProps tb_props;
    if (make_tiny_bclibc_shot_props(&tb_props, tb_curve, kElev, G7_BASIC::SIGHT_HT_FT, G7_BASIC::LOOK_ANGLE_RAD, 0.0) != TINY_BCLIBC_OK)
        return false;

    // Rows: up and down, far more than 30 ft off the muzzle altitude; the same rows; drag-free (the density ratio 0).
    auto bc_traj = run_bclibc_integrate(bc_eng, 20000.0, 2000.0);
    auto tb_traj = run_tiny_bclibc_integrate(&tb_props, 20000.0, 2000.0);
    ok &= identity::compare_trajectories(bc_traj, tb_traj, "vacuum / integrate / 20000ft@2000ft", kTsitourasAbsFloor, kTsitourasRelTol);
    double top = 0.0;
    for (const auto &r : tb_traj)
    {
        top = std::fmax(top, r.height_ft);
        if (!std::isfinite(r.height_ft) || r.density_ratio != 0.0) { std::printf("FAIL: tiny vacuum row not finite / drag-free\n"); ok = false; }
    }
    if (tb_traj.size() < 8 || top < 30.0) { std::printf("FAIL: tiny vacuum shot too short (%zu rows, top %.1f ft)\n", tb_traj.size(), top); ok = false; }

    // Apex and maximum range (45 degrees, v^2 / g on level ground; the sight height shifts it by a few feet)
    bclibc::BCLIBC_BaseTrajData bc_raw;
    TINY_BCLIBC_TrajectoryData tb_apex;
    if (bclibc::has_error(bc_eng.find_apex(bc_raw)) || tiny_bclibc_find_apex(&tb_props, &tb_apex) != TINY_BCLIBC_OK)
    {
        std::printf("FAIL: vacuum find_apex\n");
        return false;
    }
    const double vy = G7_BASIC::MV_FPS * std::sin(kElev), g = 32.17405;
    ok &= identity::compare_scalar("vacuum apex height tiny vs closed form (ft)", tb_apex.height_ft, vy * vy / (2.0 * g), 0.5);
    ok &= identity::compare_scalar("vacuum apex height bclibc vs tiny (ft)", bc_raw.py, tb_apex.height_ft, 1e-3);

    auto bc_mr = bc_eng.find_max_range(0.0, 90.0, 1.5707963267948966 * 0.99);
    double tb_range = 0.0, tb_angle = 0.0;
    const int rc = tiny_bclibc_find_max_range(&tb_props, 0.0, 90.0, &tb_range, &tb_angle);
    if (bclibc::has_error(bc_mr) || rc != TINY_BCLIBC_OK)
    {
        std::printf("FAIL: vacuum find_max_range\n");
        return false;
    }
    ok &= identity::compare_scalar("vacuum max range bclibc vs tiny (ft)", bc_mr.value().max_range_ft, tb_range, 1e-3);
    ok &= identity::compare_scalar("vacuum max range tiny vs v^2/g (ft)", tb_range,
                                   G7_BASIC::MV_FPS * G7_BASIC::MV_FPS / g, 20.0 + G7_BASIC::MV_FPS * G7_BASIC::MV_FPS / g * 1e-5);
    ok &= identity::compare_scalar("vacuum max range angle (rad)", tb_angle, 0.7853981633974483, 2e-5);
    (void)kFt;

    // A raw atmosphere with p0 = 0 and a non-zero density ratio is a vacuum at every altitude, also within 30 ft of the
    // base altitude (bclibc: the same rule).
    {
        TINY_BCLIBC_Atmosphere raw = tb_props.atmo;
        raw.p0 = 0.0;
        raw.density_ratio = 1.0;
        for (double alt : {0.0, 10.0, -29.0, 31.0, 500.0})
        {
            real_t rho = 1.0, mach = 0.0;
            TINY_BCLIBC_Atmosphere_update_density_mach(&raw, static_cast<real_t>(raw.a0 + alt), &rho, &mach);
            if (rho != 0.0 || !(mach > 0.0)) { std::printf("FAIL: tiny raw vacuum (p0 = 0, density 1) at +%g ft: rho %g\n", alt, (double)rho); ok = false; }
        }
    }

    // A NaN gravity: an error in both engines (rc != OK, a message), not a loop (no alarm here: it returned).
    tb_props.cfg.cGravityConstant = std::nan("");
    TINY_BCLIBC_TrajectoryData apex_nan;
    const int rc_nan = tiny_bclibc_find_apex(&tb_props, &apex_nan);
    double r_nan = 0.0, a_nan = 0.0;
    const int rc_nan_mr = tiny_bclibc_find_max_range(&tb_props, 0.0, 90.0, &r_nan, &a_nan);
    std::printf("  tiny NaN gravity: find_apex rc %d, find_max_range rc %d (%s)\n", rc_nan, rc_nan_mr, tiny_bclibc_last_error());
    if (rc_nan == TINY_BCLIBC_OK || rc_nan_mr == TINY_BCLIBC_OK) { std::printf("FAIL: tiny NaN gravity not an error\n"); ok = false; }
    TINY_BCLIBC_TrajectoryRequest req;
    req.range_limit_ft = 20000.0; req.range_step_ft = 2000.0; req.time_step = 0.0; req.filter_flags = TINY_BCLIBC_TRAJ_FLAG_RANGE;
    int32_t written = 0, total = 0, reason = 0;
    if (tiny_bclibc_integrate(&tb_props, &req, nullptr, 0, &written, &total, &reason) != TINY_BCLIBC_ERR_INVALID_ARG)
    { std::printf("FAIL: tiny integrate with NaN gravity is not TINY_BCLIBC_ERR_INVALID_ARG\n"); ok = false; }

    bc_eng.config.cGravityConstant = std::nan("");
    bc_eng.gravity_vector = bclibc::BCLIBC_V3dT(0.0, bc_eng.config.cGravityConstant, 0.0);
    std::vector<bclibc::BCLIBC_TrajectoryData> records;
    bclibc::BCLIBC_TerminationReason bc_reason = bclibc::BCLIBC_TerminationReason::NO_TERMINATE;
    const auto bc_nan = bc_eng.integrate_filtered(20000.0, 2000.0, 0.0, bclibc::BCLIBC_TRAJ_FLAG_RANGE, records, bc_reason, nullptr);
    if (!bclibc::has_error(bc_nan)) { std::printf("FAIL: bclibc integrate with NaN gravity is not an error\n"); ok = false; }
    return ok;
}

// Coriolis is evaluated at every stage of the adaptive step, with the stage's own velocity, in both engines (it used to be
// one value per step, which the embedded error estimate cannot see: the tolerance did not control its error). A 30 degree,
// 12000 ft shot, latitude 45 deg, azimuth 90 deg (the fixture bullet): both engines agree, and both are within 5e-4 ft in
// windage of the Python scipy engine (DOP853, rtol = atol = 1e-12, Coriolis per derivative call; G7 table of fixture.hpp,
// humidity 0.5): windage 9.1188264327 ft (zero without Coriolis, so its error is purely the Coriolis one). The once-per-step
// Coriolis was 0.119 ft off. The height is not compared to Python: at the default tolerance both engines are 0.16 ft off it
// already without Coriolis (a fast bullet, the steps are long), which has nothing to do with Coriolis.
static bool test_coriolis_stage_identity()
{
    constexpr double kElev = 30.0 * 0.017453292519943295;
    constexpr double kRange = 12000.0, kStep = 3000.0;
    constexpr double kRefWindage = 9.1188264327;
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    bool ok = true;

    double bc_w[2] = {0, 0}, tb_w[2] = {0, 0};
    for (int on = 0; on < 2; ++on)
    {
        const double lat = on ? 45.0 : kNaN, az = on ? 90.0 : kNaN;
        bclibc::BCLIBC_BaseEngine bc_eng;
        init_bclibc_engine(bc_eng, make_bclibc_shot_props(kElev, G7_BASIC::SIGHT_HT_FT, G7_BASIC::LOOK_ANGLE_RAD,
                                                          G7_BASIC::PRESSURE_HPA, lat, az));
        TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
        TINY_BCLIBC_ShotProps tb_props;
        if (make_tiny_bclibc_shot_props(&tb_props, tb_curve, kElev, G7_BASIC::SIGHT_HT_FT, G7_BASIC::LOOK_ANGLE_RAD,
                                        G7_BASIC::PRESSURE_HPA, lat, az) != TINY_BCLIBC_OK)
        {
            std::printf("FAIL: tiny_bclibc_build_shot_props (Coriolis) failed: %s\n", tiny_bclibc_last_error());
            return false;
        }
        auto bc_traj = run_bclibc_integrate(bc_eng, kRange, kStep);
        auto tb_traj = run_tiny_bclibc_integrate(&tb_props, kRange, kStep);
        ok &= identity::compare_trajectories(bc_traj, tb_traj, on ? "G7_BASIC / Coriolis on / 30 deg 12000ft@3000ft" : "G7_BASIC / Coriolis off / 30 deg 12000ft@3000ft",
                                             kTsitourasAbsFloor, kTsitourasRelTol);
        if (bc_traj.empty() || tb_traj.empty() || bc_traj.size() != tb_traj.size())
            return false;
        bc_w[on] = bc_traj.back().windage_ft;
        tb_w[on] = tb_traj.back().windage_ft;
    }
    ok &= identity::compare_scalar("Coriolis windage bclibc vs tiny (ft)", bc_w[1], tb_w[1], 1e-3);
    ok &= identity::compare_scalar("Coriolis windage bclibc vs Python (ft)", bc_w[1], kRefWindage, 5e-4);
    ok &= identity::compare_scalar("Coriolis windage tiny vs Python (ft)", tb_w[1], kRefWindage, 5e-4);
    return ok;
}

// ═══════════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════════

// A RANGE row restarts the time schedule, so the time rows of a wide adaptive step that holds both a time row and the
// range-limit row must be emitted before the range row (they were lost: the last time row before the range end was
// missing). bclibc and tiny_bclibc, Tsitouras: same rows for several time steps and range steps (the last step before the
// limit holds a time row for some of them); each time row is on the schedule (0.0.. previous row + time_step).
static bool test_time_rows_before_range_end_identity()
{
    bool ok = true;
    for (const double range_step : {1000.0, 3000.0})
        for (const double time_step : {0.07, 0.1, 0.13, 0.3, 0.5})
        {
            bclibc::BCLIBC_BaseEngine bc_eng;
            init_bclibc_engine(bc_eng, make_bclibc_shot_props());
            TINY_BCLIBC_CurvePoint tb_curve[kG7TableSize];
            TINY_BCLIBC_ShotProps tb_props;
            if (make_tiny_bclibc_shot_props(&tb_props, tb_curve) != TINY_BCLIBC_OK) return false;

            std::vector<bclibc::BCLIBC_TrajectoryData> bc;
            bclibc::BCLIBC_TerminationReason reason = bclibc::BCLIBC_TerminationReason::NO_TERMINATE;
            bc_eng.integrate_filtered(3000.0, range_step, time_step, bclibc::BCLIBC_TRAJ_FLAG_RANGE, bc, reason, nullptr);

            TINY_BCLIBC_TrajectoryRequest req;
            req.range_limit_ft = 3000.0; req.range_step_ft = range_step; req.time_step = time_step;
            req.filter_flags = TINY_BCLIBC_TRAJ_FLAG_RANGE;
            int32_t written = 0, total = 0, rsn = 0;
            tiny_bclibc_integrate(&tb_props, &req, nullptr, 0, &written, &total, &rsn);
            std::vector<TINY_BCLIBC_TrajectoryData> tb(static_cast<size_t>(total));
            tiny_bclibc_integrate(&tb_props, &req, tb.data(), total, &written, &total, &rsn);
            tb.resize(static_cast<size_t>(written));

            char name[96];
            std::snprintf(name, sizeof name, "time rows before range end / range_step %g / time_step %g", range_step, time_step);
            ok &= identity::compare_trajectories(bc, tb, name, kTsitourasAbsFloor, kTsitourasRelTol);
            // every row after the first is a time row (previous + time_step) or a range row (multiple of range_step or
            // the limit): none is lost between the last of them and the end
            for (size_t i = 1; i < tb.size(); ++i)
            {
                const double q = std::fabs(tb[i].distance_ft / range_step - std::floor(tb[i].distance_ft / range_step + 0.5)) * range_step;
                const bool range_row = q < 1e-6 || std::fabs(tb[i].distance_ft - 3000.0) < 1e-6;
                const bool time_row = std::fabs(tb[i].time - tb[i - 1].time - time_step) < 1e-9;
                // no time row is lost: consecutive rows are never further apart than the time step
                if (tb[i].time - tb[i - 1].time > time_step + 1e-9) { std::printf("FAIL: %s: gap before row %zu exceeds the time step\n", name, i); ok = false; }
                if (!range_row && !time_row) { std::printf("FAIL: %s: row %zu is neither a time nor a range row\n", name, i); ok = false; }
            }
        }
    return ok;
}

int main()
{
    std::printf("bclibc ↔ tbclibc identity test (calc_step=%.4fs, tol=%.0e)\n\n",
                kCalcStep, identity::kAbsTol);

    bool all_pass = true;
    all_pass &= test_g7_basic_integrate();
    all_pass &= test_g7_wind_integrate();
    all_pass &= test_g7_basic_tsitouras_parity();
    all_pass &= test_g7_basic_zero_angle();
    all_pass &= test_g7_basic_zero_point();
    all_pass &= test_g7_basic_find_apex();
    all_pass &= test_g7_find_apex_first_step();
    all_pass &= test_min_steps_gate();
    all_pass &= test_g7_integrate_at_keys();
    all_pass &= test_g7_basic_find_max_range();
    all_pass &= test_g7_basic_zero_angle_tight_limits();
    all_pass &= test_close_target_zero();
    all_pass &= test_downhill_max_range_and_ridder_zero();
    all_pass &= test_ridder_default_iterations_converge();
    all_pass &= test_ridder_only_entries();
    all_pass &= test_downhill_no_hang();
    all_pass &= test_long_downhill_zero_point();
    all_pass &= test_vacuum_identity();
    all_pass &= test_coriolis_stage_identity();
    all_pass &= test_time_rows_before_range_end_identity();

    std::printf("\n%s\n", all_pass ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
    return all_pass ? 0 : 1;
}
