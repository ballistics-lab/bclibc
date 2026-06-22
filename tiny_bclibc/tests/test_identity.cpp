// test_identity.cpp — bclibc (C++) ↔ tbclibc (C) mathematical identity test.
//
// Compiles as C++17 so both headers can coexist in one translation unit.
// No external test framework — returns 0 on success, 1 on any failure.
//
// Key invariant enforced here: both engines use identical RK4 time-step
// (kCalcStep = 0.0025s).  bclibc reads it from BCLIBC_Shot::calc_step;
// tbclibc_build_shot_props() derives it from config, so we override
// TBCLIBC_ShotProps::calc_step after the build call.
//
// Tolerance: 1e-9 (absolute).  Both engines perform the same IEEE-754
// double arithmetic on the same input, so results should be bitwise
// identical or differ by at most ±1 ULP from different call ordering.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

// ── bclibc C++ headers ────────────────────────────────────────────────────
#include "bclibc.hpp"

// ── tbclibc C headers ────────────────────────────────────────────────────
extern "C" {
#define TBCLIBC_BUILD_SHARED  // use extern declarations, impl via static inline
#include "tbclibc/engine.h"
}
// tbclibc is header-only in this test: compile via static-inline mode.
// Re-include without TBCLIBC_BUILD_SHARED so all TBCLIBC_FUNC resolve to
// static inline definitions in this TU.
#undef TBCLIBC_BUILD_SHARED
#undef TBCLIBC_ENGINE_H        // force re-include
// Actually: just use the header-only (no-TBCLIBC_BUILD_SHARED) path:
// The include already happened above with BUILD_SHARED, which only changed
// the function visibility.  For a header-only test build we rely on the
// INTERFACE tbclibc_headers target which does NOT define TBCLIBC_BUILD_SHARED.

// ── test helpers ─────────────────────────────────────────────────────────
#include "fixture.hpp"
#include "compare.hpp"

// ═══════════════════════════════════════════════════════════════════════════
// bclibc engine helpers
// ═══════════════════════════════════════════════════════════════════════════

namespace {

// Flat arrays for G7 table (bclibc takes const double*)
static double g_g7_mach[kG7TableSize];
static double g_g7_cd[kG7TableSize];
static bool   g_g7_init = false;

void init_g7_arrays()
{
    if (g_g7_init) return;
    for (int i = 0; i < kG7TableSize; ++i) {
        g_g7_mach[i] = kG7Table[i].mach;
        g_g7_cd[i]   = kG7Table[i].cd;
    }
    g_g7_init = true;
}

// Build a bclibc BCLIBC_ShotProps from G7_BASIC fixture (no wind).
bclibc::BCLIBC_ShotProps make_bclibc_shot_props(double barrel_elevation_rad = G7_BASIC::BARREL_EL_RAD)
{
    init_g7_arrays();

    bclibc::BCLIBC_Shot shot;
    shot.bc                  = G7_BASIC::BC;
    shot.weight_grain        = G7_BASIC::WEIGHT_GR;
    shot.diameter_inch       = G7_BASIC::DIAMETER_IN;
    shot.length_inch         = G7_BASIC::LENGTH_IN;
    shot.muzzle_velocity_fps = G7_BASIC::MV_FPS;
    shot.stability_coefficient = 0.0;
    shot.sight_height_ft     = G7_BASIC::SIGHT_HT_FT;
    shot.twist_inch          = G7_BASIC::TWIST_IN;
    shot.temp_c              = G7_BASIC::TEMP_C;
    shot.pressure_hpa        = G7_BASIC::PRESSURE_HPA;
    shot.altitude_ft         = G7_BASIC::ALT_FT;
    shot.humidity            = G7_BASIC::HUMIDITY;
    shot.mach_data           = g_g7_mach;
    shot.cd_data             = g_g7_cd;
    shot.drag_table_size     = kG7TableSize;
    shot.winds               = nullptr;
    shot.wind_count          = 0;
    shot.look_angle_rad      = G7_BASIC::LOOK_ANGLE_RAD;
    shot.barrel_elevation_rad = barrel_elevation_rad;
    shot.barrel_azimuth_rad  = G7_BASIC::BARREL_AZ_RAD;
    shot.cant_angle_rad      = G7_BASIC::CANT_ANGLE_RAD;
    shot.latitude_deg        = G7_BASIC::LAT_DEG;
    shot.azimuth_deg         = G7_BASIC::AZ_DEG;
    shot.calc_step           = kCalcStep;

    return shot.to_shot_props();
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
    shot.bc                  = G7_BASIC::BC;
    shot.weight_grain        = G7_BASIC::WEIGHT_GR;
    shot.diameter_inch       = G7_BASIC::DIAMETER_IN;
    shot.length_inch         = G7_BASIC::LENGTH_IN;
    shot.muzzle_velocity_fps = G7_BASIC::MV_FPS;
    shot.stability_coefficient = 0.0;
    shot.sight_height_ft     = G7_BASIC::SIGHT_HT_FT;
    shot.twist_inch          = G7_BASIC::TWIST_IN;
    shot.temp_c              = G7_BASIC::TEMP_C;
    shot.pressure_hpa        = G7_BASIC::PRESSURE_HPA;
    shot.altitude_ft         = G7_BASIC::ALT_FT;
    shot.humidity            = G7_BASIC::HUMIDITY;
    shot.mach_data           = g_g7_mach;
    shot.cd_data             = g_g7_cd;
    shot.drag_table_size     = kG7TableSize;
    shot.winds               = &wind;
    shot.wind_count          = 1;
    shot.look_angle_rad      = G7_BASIC::LOOK_ANGLE_RAD;
    shot.barrel_elevation_rad = barrel_elevation_rad;
    shot.barrel_azimuth_rad  = G7_BASIC::BARREL_AZ_RAD;
    shot.cant_angle_rad      = G7_BASIC::CANT_ANGLE_RAD;
    shot.latitude_deg        = G7_BASIC::LAT_DEG;
    shot.azimuth_deg         = G7_BASIC::AZ_DEG;
    shot.calc_step           = kCalcStep;

    return shot.to_shot_props();
}

// Initialize engine from ShotProps (in-place; BCLIBC_BaseEngine is non-moveable).
void init_bclibc_engine(bclibc::BCLIBC_BaseEngine &eng, bclibc::BCLIBC_ShotProps props)
{
    eng.shot = std::move(props);
    eng.integrate_func = bclibc::BCLIBC_integrateRK4;
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

// ═══════════════════════════════════════════════════════════════════════════
// tbclibc helpers
// ═══════════════════════════════════════════════════════════════════════════

// Flat arrays for G7 table (tbclibc takes const double*)
static double g_tb_mach[kG7TableSize];
static double g_tb_cd[kG7TableSize];

void init_tb_g7_arrays()
{
    for (int i = 0; i < kG7TableSize; ++i) {
        g_tb_mach[i] = kG7Table[i].mach;
        g_tb_cd[i]   = kG7Table[i].cd;
    }
}

// Build tbclibc ShotProps from G7_BASIC (no wind).
// curve_buf must be caller-allocated with >= kG7TableSize elements.
int make_tbclibc_shot_props(TBCLIBC_ShotProps *out, TBCLIBC_CurvePoint *curve_buf,
                             double barrel_elevation_rad = G7_BASIC::BARREL_EL_RAD)
{
    init_tb_g7_arrays();

    TBCLIBC_Config cfg = TBCLIBC_Config_default();
    cfg.cZeroFindingAccuracy = static_cast<double>(kDefaultConfig.cZeroFindingAccuracy);
    cfg.cMinimumVelocity     = static_cast<double>(kDefaultConfig.cMinimumVelocity);
    cfg.cMaximumDrop         = static_cast<double>(kDefaultConfig.cMaximumDrop);
    cfg.cMaxIterations       = kDefaultConfig.cMaxIterations;
    cfg.cGravityConstant     = static_cast<double>(kDefaultConfig.cGravityConstant);
    cfg.cMinimumAltitude     = static_cast<double>(kDefaultConfig.cMinimumAltitude);

    TBCLIBC_Shot shot;
    std::memset(&shot, 0, sizeof(shot));
    shot.bc                  = G7_BASIC::BC;
    shot.weight_grain        = G7_BASIC::WEIGHT_GR;
    shot.diameter_inch       = G7_BASIC::DIAMETER_IN;
    shot.length_inch         = G7_BASIC::LENGTH_IN;
    shot.muzzle_velocity_fps = G7_BASIC::MV_FPS;
    shot.sight_height_ft     = G7_BASIC::SIGHT_HT_FT;
    shot.twist_inch          = G7_BASIC::TWIST_IN;
    shot.temp_c              = G7_BASIC::TEMP_C;
    shot.pressure_hpa        = G7_BASIC::PRESSURE_HPA;
    shot.altitude_ft         = G7_BASIC::ALT_FT;
    shot.humidity            = G7_BASIC::HUMIDITY;
    shot.mach_data           = g_tb_mach;
    shot.cd_data             = g_tb_cd;
    shot.drag_table_size     = kG7TableSize;
    shot.winds               = nullptr;
    shot.wind_count          = 0;
    shot.look_angle_rad      = G7_BASIC::LOOK_ANGLE_RAD;
    shot.barrel_elevation_rad = barrel_elevation_rad;
    shot.barrel_azimuth_rad  = G7_BASIC::BARREL_AZ_RAD;
    shot.cant_angle_rad      = G7_BASIC::CANT_ANGLE_RAD;
    shot.latitude_deg        = G7_BASIC::LAT_DEG;
    shot.azimuth_deg         = G7_BASIC::AZ_DEG;
    shot.config              = cfg;

    int rc = tbclibc_build_shot_props(&shot, curve_buf, out);
    if (rc == TBCLIBC_OK)
        out->calc_step = kCalcStep; // override: match bclibc RK4 step exactly
    return rc;
}

// Build tbclibc ShotProps from G7_WIND (one crosswind).
int make_tbclibc_shot_props_wind(TBCLIBC_ShotProps *out, TBCLIBC_CurvePoint *curve_buf,
                                  TBCLIBC_Wind *wind_buf,
                                  double barrel_elevation_rad = G7_BASIC::BARREL_EL_RAD)
{
    init_tb_g7_arrays();

    TBCLIBC_Config cfg = TBCLIBC_Config_default();
    cfg.cZeroFindingAccuracy = static_cast<double>(kDefaultConfig.cZeroFindingAccuracy);
    cfg.cMinimumVelocity     = static_cast<double>(kDefaultConfig.cMinimumVelocity);
    cfg.cMaximumDrop         = static_cast<double>(kDefaultConfig.cMaximumDrop);
    cfg.cMaxIterations       = kDefaultConfig.cMaxIterations;
    cfg.cGravityConstant     = static_cast<double>(kDefaultConfig.cGravityConstant);
    cfg.cMinimumAltitude     = static_cast<double>(kDefaultConfig.cMinimumAltitude);

    wind_buf->velocity_fps       = G7_WIND::WIND_VEL_FPS;
    wind_buf->direction_from_rad = G7_WIND::WIND_DIR_RAD;
    wind_buf->until_distance_ft  = G7_WIND::WIND_UNTIL_FT;
    wind_buf->max_distance_ft    = G7_WIND::WIND_MAX_FT;

    TBCLIBC_Shot shot;
    std::memset(&shot, 0, sizeof(shot));
    shot.bc                  = G7_BASIC::BC;
    shot.weight_grain        = G7_BASIC::WEIGHT_GR;
    shot.diameter_inch       = G7_BASIC::DIAMETER_IN;
    shot.length_inch         = G7_BASIC::LENGTH_IN;
    shot.muzzle_velocity_fps = G7_BASIC::MV_FPS;
    shot.sight_height_ft     = G7_BASIC::SIGHT_HT_FT;
    shot.twist_inch          = G7_BASIC::TWIST_IN;
    shot.temp_c              = G7_BASIC::TEMP_C;
    shot.pressure_hpa        = G7_BASIC::PRESSURE_HPA;
    shot.altitude_ft         = G7_BASIC::ALT_FT;
    shot.humidity            = G7_BASIC::HUMIDITY;
    shot.mach_data           = g_tb_mach;
    shot.cd_data             = g_tb_cd;
    shot.drag_table_size     = kG7TableSize;
    shot.winds               = wind_buf;
    shot.wind_count          = 1;
    shot.look_angle_rad      = G7_BASIC::LOOK_ANGLE_RAD;
    shot.barrel_elevation_rad = barrel_elevation_rad;
    shot.barrel_azimuth_rad  = G7_BASIC::BARREL_AZ_RAD;
    shot.cant_angle_rad      = G7_BASIC::CANT_ANGLE_RAD;
    shot.latitude_deg        = G7_BASIC::LAT_DEG;
    shot.azimuth_deg         = G7_BASIC::AZ_DEG;
    shot.config              = cfg;

    int rc = tbclibc_build_shot_props(&shot, curve_buf, out);
    if (rc == TBCLIBC_OK)
        out->calc_step = kCalcStep;
    return rc;
}

// Run tbclibc_integrate (two-pass: count then fill).
std::vector<TBCLIBC_TrajectoryData>
run_tbclibc_integrate(const TBCLIBC_ShotProps *props, double range_ft, double step_ft)
{
    TBCLIBC_TrajectoryRequest req;
    req.range_limit_ft = static_cast<double>(range_ft);
    req.range_step_ft  = static_cast<double>(step_ft);
    req.time_step      = 0.0;
    req.filter_flags   = TBCLIBC_TRAJ_FLAG_RANGE;

    int32_t written = 0, total = 0, reason = 0;
    // Pass 1: count
    tbclibc_integrate(props, &req, nullptr, 0, &written, &total, &reason);

    std::vector<TBCLIBC_TrajectoryData> buf(static_cast<size_t>(total));
    if (total > 0) {
        // Pass 2: fill
        tbclibc_integrate(props, &req, buf.data(), total, &written, &total, &reason);
        buf.resize(static_cast<size_t>(written));
    }
    return buf;
}

} // anonymous namespace

// ═══════════════════════════════════════════════════════════════════════════
// Test cases
// ═══════════════════════════════════════════════════════════════════════════

static bool test_g7_basic_integrate()
{
    // Build bclibc engine
    bclibc::BCLIBC_BaseEngine bc_eng;
    init_bclibc_engine(bc_eng, make_bclibc_shot_props());

    // Build tbclibc props
    TBCLIBC_CurvePoint tb_curve[kG7TableSize];
    TBCLIBC_ShotProps  tb_props;
    if (make_tbclibc_shot_props(&tb_props, tb_curve) != TBCLIBC_OK) {
        std::printf("FAIL: tbclibc_build_shot_props failed: %s\n", tbclibc_last_error());
        return false;
    }

    // Run both at 3000ft range, 100ft step → 31 points
    auto bc_traj = run_bclibc_integrate(bc_eng, 3000.0, 100.0);
    auto tb_traj = run_tbclibc_integrate(&tb_props, 3000.0, 100.0);

    return identity::compare_trajectories(bc_traj, tb_traj, "G7_BASIC / integrate / 3000ft@100ft");
}

static bool test_g7_wind_integrate()
{
    // Build bclibc engine with wind
    bclibc::BCLIBC_BaseEngine bc_eng;
    init_bclibc_engine(bc_eng, make_bclibc_shot_props_wind());

    // Build tbclibc props with wind
    TBCLIBC_CurvePoint tb_curve[kG7TableSize];
    TBCLIBC_ShotProps  tb_props;
    TBCLIBC_Wind       tb_wind;
    if (make_tbclibc_shot_props_wind(&tb_props, tb_curve, &tb_wind) != TBCLIBC_OK) {
        std::printf("FAIL: tbclibc_build_shot_props (wind) failed: %s\n", tbclibc_last_error());
        return false;
    }

    auto bc_traj = run_bclibc_integrate(bc_eng, 3000.0, 100.0);
    auto tb_traj = run_tbclibc_integrate(&tb_props, 3000.0, 100.0);

    return identity::compare_trajectories(bc_traj, tb_traj, "G7_WIND / integrate / 3000ft@100ft");
}

static bool test_g7_basic_zero_angle()
{
    // Target: zero at 1000ft (≈333yd)
    constexpr double ZERO_DIST_FT = 1000.0;
    constexpr double APEX_MAX_RAD = 1.5707963267948966 * 0.99; // 99% of π/2
    constexpr double ALLOWED_ERR  = 0.001; // ft

    // bclibc zero_angle
    bclibc::BCLIBC_BaseEngine bc_eng;
    init_bclibc_engine(bc_eng, make_bclibc_shot_props());
    double bc_angle = bc_eng.zero_angle(ZERO_DIST_FT, APEX_MAX_RAD, ALLOWED_ERR);

    // tbclibc zero_angle
    TBCLIBC_CurvePoint tb_curve[kG7TableSize];
    TBCLIBC_ShotProps  tb_props;
    if (make_tbclibc_shot_props(&tb_props, tb_curve) != TBCLIBC_OK) {
        std::printf("FAIL: tbclibc build failed\n");
        return false;
    }
    double tb_angle = 0.0;
    int rc = tbclibc_find_zero_angle(&tb_props, ZERO_DIST_FT, &tb_angle);
    if (rc != TBCLIBC_OK) {
        std::printf("FAIL: tbclibc_find_zero_angle rc=%d: %s\n", rc, tbclibc_last_error());
        return false;
    }

    std::printf("\n=== G7_BASIC / zero_angle / target=1000ft ===\n");
    // zero_angle uses Ridder's method with tolerance ALLOWED_ERR, so
    // expect agreement within the solver tolerance (1e-6 rad ≈ 0.2").
    return identity::compare_scalar("zero_angle (rad)", bc_angle, tb_angle, 1e-6);
}

static bool test_g7_basic_find_apex()
{
    // bclibc find_apex (via integrate_at VEL_Y=0)
    auto bc_props = make_bclibc_shot_props(0.05); // small upward angle so there's an apex
    bclibc::BCLIBC_BaseEngine bc_eng;
    init_bclibc_engine(bc_eng, bc_props);

    bclibc::BCLIBC_BaseTrajData  bc_raw;
    bclibc::BCLIBC_TrajectoryData bc_apex;
    bc_eng.find_apex(bc_raw);
    bc_apex = bclibc::BCLIBC_TrajectoryData(bc_eng.shot, bc_raw, bclibc::BCLIBC_TRAJ_FLAG_APEX);

    // tbclibc find_apex
    TBCLIBC_CurvePoint tb_curve[kG7TableSize];
    TBCLIBC_ShotProps  tb_props;
    if (make_tbclibc_shot_props(&tb_props, tb_curve, 0.05) != TBCLIBC_OK) {
        std::printf("FAIL: tbclibc build failed\n");
        return false;
    }
    TBCLIBC_TrajectoryData tb_apex;
    int rc = tbclibc_find_apex(&tb_props, &tb_apex);
    if (rc != TBCLIBC_OK) {
        std::printf("FAIL: tbclibc_find_apex rc=%d: %s\n", rc, tbclibc_last_error());
        return false;
    }

    std::printf("\n=== G7_BASIC / find_apex / barrel_el=0.05rad ===\n");
    return identity::compare_point(bc_apex, tb_apex, 0, identity::kAbsTol, true);
}

// ═══════════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════════

int main()
{
    std::printf("bclibc ↔ tbclibc identity test (calc_step=%.4fs, tol=%.0e)\n\n",
                kCalcStep, identity::kAbsTol);

    bool all_pass = true;
    all_pass &= test_g7_basic_integrate();
    all_pass &= test_g7_wind_integrate();
    all_pass &= test_g7_basic_zero_angle();
    all_pass &= test_g7_basic_find_apex();

    std::printf("\n%s\n", all_pass ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
    return all_pass ? 0 : 1;
}
