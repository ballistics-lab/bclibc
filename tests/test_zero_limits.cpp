// test_zero_limits.cpp -- the zero search does not depend on the early-termination limits (issue #45).
//
// A zero search fires shots that fall well below the muzzle before they reach the target (the flat edge of a bracket,
// the first Newton guess). With a tight cMaximumDrop or cMinimumAltitude (0 means "stop at muzzle level") such a shot
// used to end short of the target and the whole search failed with "Trajectory too short" -- while the same inputs
// with the default limits worked. The zero is the same angle whatever the limits are, so these tests solve it with
// the default limits first and then with the limits of the issue, for all the combinations, and compare.
//
// Through the flat C ABI only.

#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

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
static const double kMach[] = {0.0, 0.8, 1.2, 3.0};
static const double kCd[] = {0.30, 0.32, 0.45, 0.35};

struct Limits
{
    const char *name;
    double min_altitude; // cMinimumAltitude
    double max_drop;     // cMaximumDrop
};

// The default limits first, then the rows of the table of the issue.
static const Limits kLimits[] = {
    {"default", -1500.0, -10000.0},
    {"alt 0, drop 0", 0.0, 0.0},
    {"alt 0, drop -10000", 0.0, -10000.0},
    {"alt -1500, drop 0", -1500.0, 0.0},
};

static BCLIBCFFI_Shot make_shot(int32_t method, double sight_height_ft, const Limits &lim, double min_velocity)
{
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = 0.243;
    s.weight_grain = 175.0;
    s.diameter_inch = 0.3;
    s.length_inch = 1.26;
    s.muzzle_velocity_fps = 800.0 * kFt; // the shot of the issue: 7.62 mm, 175 gr, 800 m/s
    s.sight_height_ft = sight_height_ft;
    s.twist_inch = 10.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.humidity = 0.0;
    s.mach_data = kMach;
    s.cd_data = kCd;
    s.drag_table_size = 4;
    s.latitude_deg = std::nan("");
    s.azimuth_deg = std::nan("");
    s.config.cStepMultiplier = 1.0;
    s.config.cZeroFindingAccuracy = 5e-6;
    s.config.cMinimumVelocity = min_velocity;
    s.config.cMaximumDrop = lim.max_drop;
    s.config.cMaxIterations = 40;
    s.config.cGravityConstant = -32.17405;
    s.config.cMinimumAltitude = lim.min_altitude;
    s.method = static_cast<BCLIBCFFI_IntegrationMethod>(method);
    return s;
}

static const int32_t kMethods[] = {BCLIBCFFI_INTEGRATION_RK4, BCLIBCFFI_INTEGRATION_EULER,
                                   BCLIBCFFI_INTEGRATION_VELOCITY_VERLET, BCLIBCFFI_INTEGRATION_CASH_KARP,
                                   BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, BCLIBCFFI_INTEGRATION_TSITOURAS};
static const double kSights[] = {0.0, 2.0 / 12.0};      // no sight height, and 2 in
static const double kDistancesYd[] = {100.0, 200.0, 500.0};

static void test_zero_matrix()
{
    for (int32_t method : kMethods)
        for (double sight : kSights)
            for (double yd : kDistancesYd)
                for (double min_velocity : {50.0, 0.0})
                {
                    const double d = yd * 3.0;
                    BCLIBCFFI_Error err;
                    double ref = 0.0;
                    {
                        const BCLIBCFFI_Shot s = make_shot(method, sight, kLimits[0], min_velocity);
                        CHECK(BCLIBCFFI_find_zero_angle_shot(&s, d, &ref, &err) == BCLIBCFFI_OK);
                    }
                    for (size_t i = 1; i < sizeof kLimits / sizeof kLimits[0]; ++i)
                    {
                        const BCLIBCFFI_Shot s = make_shot(method, sight, kLimits[i], min_velocity);
                        double angle = 0.0;
                        const int32_t rc = BCLIBCFFI_find_zero_angle_shot(&s, d, &angle, &err);
                        if (rc != BCLIBCFFI_OK)
                            std::fprintf(stderr, "  method %d, sight %.3f ft, %.0f yd, limits '%s', min v %.0f: %s\n", method,
                                         sight, yd, kLimits[i].name, min_velocity, err.message);
                        CHECK(rc == BCLIBCFFI_OK);
                        // The same zero, not a biased one: the limits must not change the answer.
                        CHECK(std::fabs(angle - ref) < 1e-6);
                    }
                }
}

static void test_zero_point_and_newton()
{
    // The zero point and the Newton-only solver take the same tight limits.
    const double d = 200.0 * 3.0;
    BCLIBCFFI_Error err;
    for (int32_t method : {BCLIBCFFI_INTEGRATION_RK4, BCLIBCFFI_INTEGRATION_TSITOURAS})
    {
        const BCLIBCFFI_Shot def = make_shot(method, 2.0 / 12.0, kLimits[0], 50.0);
        const BCLIBCFFI_Shot tight = make_shot(method, 2.0 / 12.0, kLimits[1], 50.0);
        double ref = 0.0, angle = 0.0;
        BCLIBCFFI_ZeroPointResult zp;
        CHECK(BCLIBCFFI_find_zero_angle_shot(&def, d, &ref, &err) == BCLIBCFFI_OK);
        CHECK(BCLIBCFFI_find_zero_point_shot(&tight, d, &zp, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(zp.angle_rad - ref) < 1e-6);
        CHECK(BCLIBCFFI_find_zero_angle_newton_shot(&tight, d, &angle, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(angle - ref) < 1e-6);
        CHECK(BCLIBCFFI_find_zero_angle_lofted_shot(&tight, d, 0, &angle, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(angle - ref) < 1e-6);
    }
}

static void test_long_range_altitude_limit()
{
    // The default cMinimumAltitude (-1500 ft) allows a descent of 1500 ft; a 3000 yd zero needs a drop of about
    // x/2 = 4500 ft to be followed. The altitude limit used to be widened only when it was *above* the muzzle by that
    // much (a sign error), i.e. never, so far zeros depended on cMinimumAltitude. They do not any more.
    const double d = 3000.0 * 3.0;
    BCLIBCFFI_Error err;
    const Limits wide = {"alt -1e9", -1e9, -10000.0};
    for (int32_t method : {BCLIBCFFI_INTEGRATION_RK4, BCLIBCFFI_INTEGRATION_TSITOURAS})
    {
        const BCLIBCFFI_Shot ref_shot = make_shot(method, 0.15, wide, 50.0);
        const BCLIBCFFI_Shot shot = make_shot(method, 0.15, kLimits[0], 50.0);
        double ref = 0.0, angle = 0.0;
        CHECK(BCLIBCFFI_find_zero_angle_shot(&ref_shot, d, &ref, &err) == BCLIBCFFI_OK);
        CHECK(BCLIBCFFI_find_zero_angle_shot(&shot, d, &angle, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(angle - ref) < 1e-6);
    }
}

static void test_limits_are_restored_for_the_caller()
{
    // The widening is only for the search: a trajectory fired afterwards stops where the caller's limits say.
    // (Each ABI call builds its own engine, so this is about one call not leaking into the next: the same shot with
    // tight limits, integrated, ends at the muzzle level.)
    BCLIBCFFI_Error err;
    const BCLIBCFFI_Shot tight = make_shot(BCLIBCFFI_INTEGRATION_RK4, 0.0, kLimits[1], 50.0);
    double angle = 0.0;
    CHECK(BCLIBCFFI_find_zero_angle_shot(&tight, 600.0, &angle, &err) == BCLIBCFFI_OK);
    BCLIBCFFI_Shot fired = tight;
    fired.barrel_elevation_rad = angle;
    BCLIBCFFI_TrajectoryRequest req{3000.0 * 3.0, 100.0, 0.0, 0};
    BCLIBCFFI_TrajectoryData *recs = nullptr;
    int32_t n = 0, reason = 0;
    CHECK(BCLIBCFFI_integrate_shot(&fired, &req, &recs, &n, &reason, &err) == BCLIBCFFI_OK);
    // cMaximumDrop = 0 and cMinimumAltitude = 0: the shot ends when it comes back down through the muzzle level.
    CHECK(reason == BCLIBCFFI_TERM_MAXIMUM_DROP_REACHED || reason == BCLIBCFFI_TERM_MINIMUM_ALTITUDE_REACHED);
    CHECK(n > 0 && recs[n - 1].distance_ft < 3000.0 * 3.0);
    BCLIBCFFI_free_trajectory(recs);
}

int main()
{
    test_zero_matrix();
    test_zero_point_and_newton();
    test_long_range_altitude_limit();
    test_limits_are_restored_for_the_caller();
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_zero_limits: all checks passed");
    return EXIT_SUCCESS;
}
