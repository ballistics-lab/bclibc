// test_lofted_bound.cpp -- the upper bound of the lofted zero search is 89.9 degrees.
//
// The lofted search brackets the elevation between the angle of maximum range and an upper bound of 89.9 degrees (the
// Python reference: math.radians(89.9) = 1.5690509975429023). The C++ core carried 1.5690308719637473, which is
// 89.8988 degrees. When a lofted zero cannot be reached the error reports the last barrel elevation tried, which is that
// bound (the bracket search ends after firing at it): BCLIBCFFI_Error::f64_1 of BCLIBCFFI_ERR_ZERO_FINDING. The test
// asks for a target nearer than the lofted range of the shot (5 m), on all six engines, through the lofted angle and point entry
// points (fixed-step for RK4, Euler and Velocity Verlet, adaptive for Cash-Karp, Dormand-Prince and Tsitouras).

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

static const double kFt = 3.28084;      // feet per metre
static const double kMissTolFt = 1e-3;  // 0.3 mm

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

static BCLIBCFFI_Shot make_shot(double bc, double weight_gr, double diameter_in, double length_in, double mv_fps,
                                double sight_ft, double look_rad, double step_multiplier)
{
    for (int i = 0; i < kG7Size; ++i)
    {
        g_mach[i] = kG7[i].mach;
        g_cd[i] = kG7[i].cd;
    }
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = bc;
    s.weight_grain = weight_gr;
    s.diameter_inch = diameter_in;
    s.length_inch = length_in;
    s.muzzle_velocity_fps = mv_fps;
    s.sight_height_ft = sight_ft;
    s.twist_inch = 0.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.humidity = 0.0;
    s.mach_data = g_mach;
    s.cd_data = g_cd;
    s.drag_table_size = kG7Size;
    s.look_angle_rad = look_rad;
    s.cant_angle_rad = 0.0;
    s.latitude_deg = std::nan("");
    s.azimuth_deg = std::nan("");
    // The defaults of the engine config.
    s.config.cStepMultiplier = step_multiplier;
    s.config.cZeroFindingAccuracy = 5e-6;
    s.config.cMinimumVelocity = 50.0;
    s.config.cMaximumDrop = -15000.0;
    s.config.cMaxIterations = 40;
    s.config.cGravityConstant = -32.17405;
    s.config.cMinimumAltitude = -1500.0;
    s.method = BCLIBCFFI_INTEGRATION_RK4;
    return s;
}

// The miss of a shot fired at angle_rad: its height above the line of sight where it reaches the target.
static double miss_at(const BCLIBCFFI_Shot &shot, double angle_rad, double distance_ft)
{
    BCLIBCFFI_Error err;
    double miss = 1e9;
    const double tx = distance_ft * std::cos(shot.look_angle_rad);
    const double ty = distance_ft * std::sin(shot.look_angle_rad);
    if (BCLIBCFFI_error_at_distance_shot(&shot, angle_rad, tx, ty, &miss, nullptr, &err) != BCLIBCFFI_OK)
        std::fprintf(stderr, "  error_at_distance failed: %s\n", err.message);
    return miss;
}


static const double kPythonBound = 1.5690509975429023; // math.radians(89.9)

static const BCLIBCFFI_IntegrationMethod kMethods[] = {
    BCLIBCFFI_INTEGRATION_RK4,         BCLIBCFFI_INTEGRATION_EULER,         BCLIBCFFI_INTEGRATION_VELOCITY_VERLET,
    BCLIBCFFI_INTEGRATION_CASH_KARP,   BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, BCLIBCFFI_INTEGRATION_TSITOURAS};
static const char *kNames[] = {"rk4", "euler", "verlet", "rkck", "dopri", "tsitouras"};

static bool is_adaptive(BCLIBCFFI_IntegrationMethod m) { return m >= BCLIBCFFI_INTEGRATION_CASH_KARP; }

static void check_error(const char *what, const char *engine, int32_t rc, const BCLIBCFFI_Error &err)
{
    const bool ok = rc == BCLIBCFFI_ERR_ZERO_FINDING && std::fabs(err.f64_1 - kPythonBound) <= 1e-15;
    if (!ok)
        std::fprintf(stderr, "  %s (%s): rc %d, last_barrel_elevation %.17g, expected %.17g\n", what, engine, (int)rc,
                     err.f64_1, kPythonBound);
    CHECK(ok);
}

// A lofted shot at 89.9 degrees comes down some 15 m from the muzzle (800 m/s, G7, flat line of sight); a target nearer
// than that is below the lofted range, so the bracket (maximum-range angle, 89.9 degrees) has no sign change and the
// search fails with "No lofted zero trajectory" and the bound as its last elevation. 5 m is also far enough out not to
// be taken by the close-target shortcut.
static void test_unreachable_lofted_zero()
{
    for (int i = 0; i < 6; ++i)
    {
        BCLIBCFFI_Shot shot = make_shot(0.243, 175.0, 0.308, 1.26, 800.0 * kFt, 0.05 * kFt, 0.0, 1.0);
        shot.method = kMethods[i];
        const double distance_ft = 5.0 * kFt;
        BCLIBCFFI_Error err;
        BCLIBCFFI_StepStats st{0, 0};
        double angle = 0.0;
        BCLIBCFFI_ZeroPointResult zp;
        std::memset(&zp, 0, sizeof zp);
        std::memset(&err, 0, sizeof err);
        int32_t rc;
        if (is_adaptive(kMethods[i]))
            rc = BCLIBCFFI_find_zero_angle_lofted_adaptive_shot(&shot, kMethods[i], 1e-8, 1e-8, distance_ft, 1, &angle, &st,
                                                                &err);
        else
            rc = BCLIBCFFI_find_zero_angle_lofted_shot(&shot, distance_ft, 1, &angle, &err);
        check_error("lofted angle", kNames[i], rc, err);

        std::memset(&err, 0, sizeof err);
        if (is_adaptive(kMethods[i]))
            rc = BCLIBCFFI_find_zero_point_lofted_adaptive_shot(&shot, kMethods[i], 1e-8, 1e-8, distance_ft, 1, &zp, &st,
                                                                &err);
        else
            rc = BCLIBCFFI_find_zero_point_lofted_shot(&shot, distance_ft, 1, &zp, &err);
        check_error("lofted point", kNames[i], rc, err);
    }
}

int main()
{
    test_unreachable_lofted_zero();
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_lofted_bound: all checks passed");
    return EXIT_SUCCESS;
}
