// test_zero_ridder_angle.cpp -- find_zero_angle() and find_zero_point() return the same angle for the same search.
//
// Both entry points run the same Ridder search (find_zero_angle_ridder); find_zero_point additionally hands back the
// terminal trajectory point of the last evaluation. The exit on the width of the bracket,
// |high_angle - low_angle| < tolerance, returned next_angle (the point's angle) when a point was asked for, but the
// midpoint of the bracket (low_angle + high_angle) / 2 when it was not, so the two calls differed by up to the width of
// the bracket (1.5690554737 vs 1.5690599498 rad for a 500 m target at a look angle of 89.9 deg). The Python reference
// returns next_angle (with the last point) from this exit; the spec is that the two angles are equal.
//
// The test compares the two angles bit for bit (C++ core and C ABI, fixed step and adaptive), for the shot of the
// reports and for a grid of look angles, targets and zeroing accuracies (a looser accuracy ends the search on the
// bracket width instead of on the height of the miss), and checks that the point is still a RANGE point at the target
// (the flags and the distance of the point are the same as before). The second part is a regression guard: the angles of
// searches that never took that exit did not change (recorded values, zero_ridder_angle_guard.inc, `--print-guard`).

#include "bclibc.hpp"
#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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

static const double kFt = 3.28084; // feet per metre

static const double kMach[] = {0.0, 0.8, 1.2, 3.0};
static const double kCd[] = {0.12, 0.13, 0.38, 0.30};

static BCLIBCFFI_Shot make_abi_shot(double sight_m, double v_mps, double look_deg, double accuracy)
{
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = 0.243;
    s.weight_grain = 175.0;
    s.diameter_inch = 0.3;
    s.length_inch = 1.26;
    s.muzzle_velocity_fps = v_mps * kFt;
    s.sight_height_ft = sight_m * kFt;
    s.twist_inch = 10.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.mach_data = kMach;
    s.cd_data = kCd;
    s.drag_table_size = 4;
    s.look_angle_rad = look_deg * M_PI / 180.0;
    s.latitude_deg = std::nan("");
    s.azimuth_deg = std::nan("");
    s.config.cStepMultiplier = 1.0;
    s.config.cZeroFindingAccuracy = accuracy;
    s.config.cMinimumVelocity = 50.0;
    s.config.cMaximumDrop = -15000.0;
    s.config.cMaxIterations = 40;
    s.config.cGravityConstant = -32.17405;
    s.config.cMinimumAltitude = -1500.0;
    s.method = BCLIBCFFI_INTEGRATION_RK4;
    return s;
}

static bclibc::BCLIBC_ShotProps core_props(double sight_m, double v_mps, double look_deg)
{
    bclibc::BCLIBC_Shot shot;
    shot.bc = 0.243;
    shot.weight_grain = 175.0;
    shot.diameter_inch = 0.3;
    shot.length_inch = 1.26;
    shot.muzzle_velocity_fps = v_mps * kFt;
    shot.stability_coefficient = 0.0;
    shot.sight_height_ft = sight_m * kFt;
    shot.twist_inch = 10.0;
    shot.temp_c = 15.0;
    shot.pressure_hpa = 1013.25;
    shot.altitude_ft = 0.0;
    shot.humidity = 0.0;
    shot.mach_data = kMach;
    shot.cd_data = kCd;
    shot.drag_table_size = 4;
    shot.winds = nullptr;
    shot.wind_count = 0;
    shot.look_angle_rad = look_deg * M_PI / 180.0;
    shot.barrel_elevation_rad = 0.0;
    shot.barrel_azimuth_rad = 0.0;
    shot.cant_angle_rad = 0.0;
    shot.latitude_deg = std::nan("");
    shot.azimuth_deg = std::nan("");
    shot.calc_step = 0.0025;
    auto r = shot.to_shot_props();
    if (bclibc::has_error(r))
    {
        std::fprintf(stderr, "to_shot_props failed\n");
        std::exit(EXIT_FAILURE);
    }
    return r.value();
}

static void core_init(bclibc::BCLIBC_BaseEngine &eng, double sight_m, double v_mps, double look_deg, double accuracy)
{
    eng.shot = core_props(sight_m, v_mps, look_deg);
    eng.integrate_func = bclibc::BCLIBC_integrateRK4;
    eng.config = bclibc::BCLIBC_Config(1.0, accuracy, 50.0, -15000.0, 40, -32.17405, -1500.0);
    eng.gravity_vector = bclibc::BCLIBC_V3dT(0.0, eng.config.cGravityConstant, 0.0);
}

static const double kApex = 0.0003;
static const double kAllowed = 1e-2;

struct Case
{
    double sight_m, v_mps, look_deg, target_m, accuracy;
};

// Returns true when the search succeeded (a search with no solution has nothing to compare).
static bool core_pair(const Case &c, double *angle, double *point_angle)
{
    bclibc::BCLIBC_BaseEngine eng;
    core_init(eng, c.sight_m, c.v_mps, c.look_deg, c.accuracy);
    const double d = c.target_m * kFt;
    auto a = eng.find_zero_angle(d, 0, kApex, kAllowed);
    auto p = eng.find_zero_point(d, 0, kApex, kAllowed);
    CHECK(bclibc::has_error(a) == bclibc::has_error(p));
    if (bclibc::has_error(a) || bclibc::has_error(p))
        return false;
    *angle = a.value();
    *point_angle = p.value().angle_rad;
    // the point is the RANGE point of the search, at the target: its flags and distance are unchanged by the fix
    CHECK(p.value().has_point && p.value().point.flag == bclibc::BCLIBC_TRAJ_FLAG_RANGE);
    return true;
}

static void check_core(const Case &c)
{
    double a = 0.0, pa = 0.0;
    if (!core_pair(c, &a, &pa))
        return;
    if (a != pa)
        std::fprintf(stderr,
                     "  core: sight %.2f m, v %.0f, look %.1f deg, target %.0f m, accuracy %g: find_zero_angle %.10f, "
                     "find_zero_point %.10f rad\n",
                     c.sight_m, c.v_mps, c.look_deg, c.target_m, c.accuracy, a, pa);
    CHECK(a == pa);
}

static void check_abi(const Case &c)
{
    const BCLIBCFFI_Shot shot = make_abi_shot(c.sight_m, c.v_mps, c.look_deg, c.accuracy);
    const double d = c.target_m * kFt;
    BCLIBCFFI_Error err;
    double angle = 0.0;
    BCLIBCFFI_ZeroPointResult zp;
    std::memset(&zp, 0, sizeof zp);
    const int32_t rc1 = BCLIBCFFI_find_zero_angle_lofted_shot(&shot, d, 0, &angle, &err);
    const int32_t rc2 = BCLIBCFFI_find_zero_point_lofted_shot(&shot, d, 0, &zp, &err);
    CHECK((rc1 == BCLIBCFFI_OK) == (rc2 == BCLIBCFFI_OK));
    if (rc1 == BCLIBCFFI_OK && rc2 == BCLIBCFFI_OK)
    {
        if (angle != zp.angle_rad)
            std::fprintf(stderr, "  abi lofted: look %.1f deg, target %.0f m, accuracy %g: %.10f vs %.10f rad\n",
                         c.look_deg, c.target_m, c.accuracy, angle, zp.angle_rad);
        CHECK(angle == zp.angle_rad);
    }
    // the adaptive variants run the same search
    BCLIBCFFI_StepStats stats;
    std::memset(&stats, 0, sizeof stats);
    const int32_t rc3 = BCLIBCFFI_find_zero_angle_lofted_adaptive_shot(
        &shot, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, 1e-8, 1e-8, d, 0, &angle, &stats, &err);
    const int32_t rc4 = BCLIBCFFI_find_zero_point_lofted_adaptive_shot(
        &shot, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, 1e-8, 1e-8, d, 0, &zp, &stats, &err);
    CHECK((rc3 == BCLIBCFFI_OK) == (rc4 == BCLIBCFFI_OK));
    if (rc3 == BCLIBCFFI_OK && rc4 == BCLIBCFFI_OK)
        CHECK(angle == zp.angle_rad);
}

static const double kAccuracies[] = {5e-6, 1e-4, 1e-3, 1e-2};

// The reported shot: look 89.9 deg, 500 m, the low arc; and the same shots at the accuracies that end on the bracket.
static void test_reported()
{
    for (double acc : kAccuracies)
        for (double v : {800.0, 300.0})
        {
            const Case c{0.05, v, 89.9, 500.0, acc};
            check_core(c);
            check_abi(c);
        }
}

static void test_grid()
{
    for (double acc : {5e-6, 1e-3, 1e-2})
        for (double v : {300.0, 800.0})
            for (double look : {-20.0, 10.0, 60.0, 89.9})
                for (double target : {100.0, 1000.0})
                {
                    const Case c{0.1, v, look, target, acc};
                    check_core(c);
                    if (acc == 1e-3)
                        check_abi(c);
                }
}

struct Guard
{
    double sight_m, v_mps, look_deg, target_m, accuracy;
    double value;
};

static const Guard kGuard[] = {
#include "zero_ridder_angle_guard.inc"
};

static void test_unchanged_results()
{
    for (const Guard &g : kGuard)
    {
        double a = std::nan(""), pa = std::nan("");
        const Case c{g.sight_m, g.v_mps, g.look_deg, g.target_m, g.accuracy};
        if (!core_pair(c, &a, &pa))
        {
            CHECK(std::isnan(g.value));
            continue;
        }
        const bool same = std::fabs(a - g.value) <= 1e-12;
        if (!same)
            std::fprintf(stderr, "  guard: sight %.2f, v %.0f, look %.1f, target %.0f, accuracy %g: %.17g, recorded %.17g\n",
                         g.sight_m, g.v_mps, g.look_deg, g.target_m, g.accuracy, a, g.value);
        CHECK(same);
    }
}

// Searches that do not end on the bracket width (they end on the height of the miss, the default accuracy and a close
// target): not touched by the fix.
static void print_guard()
{
    for (double sight : {0.05, 0.1})
        for (double v : {300.0, 800.0})
            for (double look : {-10.0, 0.0, 10.0, 30.0})
                for (double target : {50.0, 300.0, 1000.0})
                {
                    const Case c{sight, v, look, target, 5e-6};
                    double a = 0.0, pa = 0.0;
                    if (core_pair(c, &a, &pa))
                        std::printf("    {%.2f, %.0f, %.1f, %.0f, %g, %.17g},\n", sight, v, look, target, 5e-6, a);
                }
}

int main(int argc, char **argv)
{
    if (argc > 1 && std::strcmp(argv[1], "--print-guard") == 0)
    {
        print_guard();
        return EXIT_SUCCESS;
    }
    test_reported();
    test_grid();
    test_unchanged_results();
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_zero_ridder_angle: all checks passed");
    return EXIT_SUCCESS;
}
