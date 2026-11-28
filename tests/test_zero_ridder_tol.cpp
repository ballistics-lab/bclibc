// test_zero_ridder_tol.cpp -- the angle tolerance of the Ridder zero search.
//
// The Ridder loop in find_zero_angle() stops on a height tolerance (cZeroFindingAccuracy, feet) or, when the bracket
// or the step has become that narrow, on an angle tolerance (radians). The angle tolerance was 1e-5 rad; at a long
// flight time 1e-5 rad is several feet at the target, so the search returned an angle that missed. The Python reference
// (py-ballisticcalc, base_engine.py) uses 1e-7 rad. The tests fire the returned angle and measure the miss with
// error_at_distance (the measure the solver minimises):
//   (a) a lofted zero at a flat line of sight, 50 .. 1000 m (the miss used to be 58, 1.52, 0.98, 0.76 ft);
//   (b) the nine high-elevation points of py-ballisticcalc issue #204: the search must not fail and must hit;
//   (c) the flat 300 m zero of a rifle bullet is where it always was.

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

// (a) the lofted zero of an ordinary shot (800 m/s, G7, 5 cm sight, flat line of sight), 50 .. 1000 m.
//
// A lofted zero is a near-vertical shot, and the miss changes by up to 6.4e6 ft per radian along it (a 1000 m target
// over a 50 s flight), so the angle tolerance decides the miss: 1e-5 rad meant feet. The point entry point returns the
// angle of the last shot fired (|miss| below the height accuracy where the search converged), and so does the angle entry
// point (both leave the bracket-width exit with next_angle), so both are held to the strict bound at every target.
// cMinimumVelocity is 0 for these shots: with the default 50 fps a near-vertical shot ends at its apex (it is slower
// than that there), the miss measured at its last point does not depend on the angle, and the search ends on that
// step, whatever the tolerance.
static void test_lofted_zero_hits()
{
    const double targets_m[] = {50.0, 100.0, 300.0, 1000.0};
    BCLIBCFFI_Shot shot = make_shot(0.243, 175.0, 0.308, 1.26, 800.0 * kFt, 0.05 * kFt, 0.0, 1.0);
    shot.config.cMinimumVelocity = 0.0;
    for (double t : targets_m)
    {
        BCLIBCFFI_Error err;
        BCLIBCFFI_ZeroPointResult zp;
        std::memset(&zp, 0, sizeof zp);
        int32_t rc = BCLIBCFFI_find_zero_point_lofted_shot(&shot, t * kFt, 1, &zp, &err);
        double miss = (rc == BCLIBCFFI_OK) ? miss_at(shot, zp.angle_rad, t * kFt) : 1e9;
        if (rc != BCLIBCFFI_OK || std::fabs(miss) >= kMissTolFt)
            std::fprintf(stderr, "  lofted zero point at %.0f m: rc %d, angle %.9f rad, miss %.5f ft\n", t, (int)rc,
                         zp.angle_rad, miss);
        CHECK(rc == BCLIBCFFI_OK && std::fabs(miss) < kMissTolFt);

        double angle = 0.0;
        rc = BCLIBCFFI_find_zero_angle_lofted_shot(&shot, t * kFt, 1, &angle, &err);
        miss = (rc == BCLIBCFFI_OK) ? miss_at(shot, angle, t * kFt) : 1e9;
        if (rc != BCLIBCFFI_OK || std::fabs(miss) >= kMissTolFt)
            std::fprintf(stderr, "  lofted zero angle at %.0f m: rc %d, angle %.9f rad, miss %.5f ft\n", t, (int)rc, angle,
                         miss);
        CHECK(rc == BCLIBCFFI_OK && std::fabs(miss) < kMissTolFt);
    }
}

// (b) issue #204: high-elevation targets (x, y in metres), a 10 g G7 bullet at 800 m/s, cStepMultiplier 5.
static void test_issue_204_points()
{
    const double pts[][2] = {
        {1266.55316094695, 2804.89754166274}, {1576.72944526049, 2767.49890777391},
        {1654.27351633887, 2786.19822471832}, {2300.47410865875, 2430.91120277438},
        {3256.85098529216, 1608.14125721997}, {1525.033398, 1458.546722},
        {25.84802369, 2075.624181},           {1111.465019, 2617.904372},
        {491.1124502, 3047.988662}};
    for (const auto &p : pts)
    {
        const double look = std::atan2(p[1], p[0]);
        const double dist_ft = std::hypot(p[0], p[1]) * kFt;
        // 10 g = 154.324 gr; 7.62 cm = 3.0 in; 3 cm = 1.1811 in.
        const BCLIBCFFI_Shot shot = make_shot(0.22, 154.3236, 3.0, 1.1811, 800.0 * kFt, 0.0, look, 5.0);
        BCLIBCFFI_Error err;
        double angle = 0.0;
        const int32_t rc = BCLIBCFFI_find_zero_angle_shot(&shot, dist_ft, &angle, &err);
        const double miss = (rc == BCLIBCFFI_OK) ? miss_at(shot, angle, dist_ft) : 1e9;
        if (rc != BCLIBCFFI_OK || std::fabs(miss) >= kMissTolFt)
            std::fprintf(stderr, "  issue 204 (%.3f, %.3f): rc %d (%s), angle %.9f rad, miss %.5f ft\n", p[0], p[1], (int)rc,
                         rc != BCLIBCFFI_OK ? err.message : "", angle, miss);
        CHECK(rc == BCLIBCFFI_OK && std::fabs(miss) < kMissTolFt);
    }
}

// (c) the flat zero of a rifle bullet at 300 m: G7 0.31, 168 gr, 2750 fps, 1.5 in sight.
static void test_flat_zero_300m()
{
    const BCLIBCFFI_Shot shot = make_shot(0.31, 168.0, 0.308, 1.2, 2750.0, 1.5 / 12.0, 0.0, 1.0);
    BCLIBCFFI_Error err;
    double angle = 0.0;
    const int32_t rc = BCLIBCFFI_find_zero_angle_shot(&shot, 300.0 * kFt, &angle, &err);
    CHECK(rc == BCLIBCFFI_OK);
    if (std::fabs(angle - 0.002502) >= 1e-4)
        std::fprintf(stderr, "  flat zero at 300 m: %.9f rad, expected 0.002502 +- 1e-4\n", angle);
    CHECK(std::fabs(angle - 0.002502) < 1e-4);
}

int main()
{
    test_lofted_zero_hits();
    test_issue_204_points();
    test_flat_zero_300m();
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_zero_ridder_tol: all checks passed");
    return EXIT_SUCCESS;
}
