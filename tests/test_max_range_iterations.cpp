// test_max_range_iterations.cpp -- the golden-section search for the maximum range follows the Python reference.
//
// find_max_range() brackets the angle of maximum range with a golden-section search. Python
// (BaseIntegrationEngine._find_max_range) runs cMaxIterations iterations and stops when the bracket is narrower than
// 1e-6 rad; the C++ core ran a fixed 100 iterations and stopped at 1e-5 rad. With the default config the angle differed
// by up to 4e-6 rad and the range by 0.1 ft when the maximum lies at an edge of the bracket (12144.710 against
// 12144.611 ft for the bracket 0..10 deg); with cMaxIterations 0, 1 or 5 the Python result is the coarse bracket (the
// initial bracket's centre for 0), the C++ one was refined to 1e-5 rad (18098 ft against 17726 ft at 0).
//
// The test compares the maximum range and its angle with the values recorded from the Python reference
// (max_range_iterations_guard.inc) to 1e-4 ft (the integrators differ by 2e-5 ft at the same angle) / 1e-9 rad, through the C ABI (find_max_range_shot) and the C++ core.
// The cases cover a flat, an uphill and a downhill line of sight, the edge-of-bracket maxima, and cMaxIterations 0, 1, 5.

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

struct Case
{
    const char *name;
    double look_deg;
    double low_deg, high_deg;
    int iterations;
    double range_ft;
    double angle_rad;
};
static const Case kCases[] = {
#include "max_range_iterations_guard.inc"
};

static const double kM = 1.0 / 0.3048; // feet per metre
static const double kAngleTol = 1e-9; // rad
static const double kRangeTol = 1e-4;  // ft: the same shot integrated by C++ and by Python differs by about 2e-5 ft

static BCLIBCFFI_Shot make_shot(double look_deg, int iterations)
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
    s.muzzle_velocity_fps = 800.0 * kM;
    s.sight_height_ft = 0.05 * kM;
    s.twist_inch = 11.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.humidity = 0.0;
    s.mach_data = g_mach;
    s.cd_data = g_cd;
    s.drag_table_size = kG7Size;
    s.look_angle_rad = look_deg * M_PI / 180.0;
    s.latitude_deg = std::nan("");
    s.azimuth_deg = std::nan("");
    s.config.cStepMultiplier = 1.0;
    s.config.cZeroFindingAccuracy = 5e-6;
    s.config.cMinimumVelocity = 50.0;
    s.config.cMaximumDrop = -10000.0;
    s.config.cMaxIterations = iterations;
    s.config.cGravityConstant = -32.17405;
    s.config.cMinimumAltitude = -1500.0;
    s.method = BCLIBCFFI_INTEGRATION_RK4;
    return s;
}

static void check(const char *via, const Case &c, double range_ft, double angle_rad)
{
    const bool ok = std::fabs(range_ft - c.range_ft) < kRangeTol && std::fabs(angle_rad - c.angle_rad) < kAngleTol;
    if (!ok)
        std::fprintf(stderr, "  %s %s: range %.12f (Python %.12f), angle %.15f (Python %.15f)\n", via, c.name, range_ft,
                     c.range_ft, angle_rad, c.angle_rad);
    CHECK(ok);
}

static void test_abi()
{
    for (const Case &c : kCases)
    {
        const BCLIBCFFI_Shot shot = make_shot(c.look_deg, c.iterations);
        BCLIBCFFI_MaxRangeResult mr;
        BCLIBCFFI_Error err;
        const int32_t rc = BCLIBCFFI_find_max_range_shot(&shot, c.low_deg, c.high_deg, &mr, &err);
        CHECK(rc == BCLIBCFFI_OK);
        if (rc == BCLIBCFFI_OK)
            check("abi", c, mr.max_range_ft, mr.angle_at_max_rad);
    }
}

static void test_core()
{
    for (const Case &c : kCases)
    {
        bclibc::BCLIBC_Shot shot;
        shot.bc = 0.31;
        shot.weight_grain = 175.0;
        shot.diameter_inch = 0.308;
        shot.length_inch = 1.24;
        shot.muzzle_velocity_fps = 800.0 * kM;
        shot.stability_coefficient = 0.0;
        shot.sight_height_ft = 0.05 * kM;
        shot.twist_inch = 11.0;
        shot.temp_c = 15.0;
        shot.pressure_hpa = 1013.25;
        shot.altitude_ft = 0.0;
        shot.humidity = 0.0;
        for (int i = 0; i < kG7Size; ++i)
        {
            g_mach[i] = kG7[i].mach;
            g_cd[i] = kG7[i].cd;
        }
        shot.mach_data = g_mach;
        shot.cd_data = g_cd;
        shot.drag_table_size = kG7Size;
        shot.winds = nullptr;
        shot.wind_count = 0;
        shot.look_angle_rad = c.look_deg * M_PI / 180.0;
        shot.barrel_elevation_rad = 0.0;
        shot.barrel_azimuth_rad = 0.0;
        shot.cant_angle_rad = 0.0;
        shot.latitude_deg = std::nan("");
        shot.azimuth_deg = std::nan("");
        shot.calc_step = 0.0025;
        auto props = shot.to_shot_props();
        CHECK(!bclibc::has_error(props));
        if (bclibc::has_error(props))
            continue;
        bclibc::BCLIBC_BaseEngine eng;
        eng.shot = std::move(props.value());
        eng.integrate_func = bclibc::BCLIBC_integrateRK4;
        eng.config = bclibc::BCLIBC_Config(1.0, 5e-6, 50.0, -10000.0, c.iterations, -32.17405, -1500.0);
        eng.gravity_vector = bclibc::BCLIBC_V3dT(0.0, eng.config.cGravityConstant, 0.0);
        auto r = eng.find_max_range(c.low_deg, c.high_deg, 0.0003);
        CHECK(!bclibc::has_error(r));
        if (!bclibc::has_error(r))
            check("core", c, r.value().max_range_ft, r.value().angle_at_max_rad);
    }
}

int main()
{
    test_abi();
    test_core();
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_max_range_iterations: all checks passed");
    return EXIT_SUCCESS;
}
