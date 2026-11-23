// test_zero_ridder_unconverged.cpp -- the Ridder zero search reports failure when it runs out of iterations.
//
// After the loop find_zero_angle_ridder() used to "accept" a bracket narrower than 10 * the angle tolerance, or an end
// of the bracket with |error| < 10 * cZeroFindingAccuracy, instead of failing. With few iterations (cMaxIterations 5)
// the lofted zeros of an 800 m/s G7-like shot at 100 m and 600 m came back as angles that missed by 424 ft and 43 ft.
// The Python reference raises ZeroFindingError "Ridder's method failed to converge." there. The search must now fail
// (BCLIBCFFI_ERR_ZERO_FINDING); and with the default 40 iterations the same searches converge and hit to 1e-3 ft.

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
static const double kMissTolFt = 1e-3;  // 0.3 mm; the old code missed by 2 * sight height >= 0.1 m = 0.33 ft

// Two drag tables: a G7-like one and a blunter, G1-like one.
static const double kMachG7[] = {0.0, 0.8, 1.2, 3.0};
static const double kCdG7[] = {0.12, 0.13, 0.38, 0.30};
static const double kMachG1[] = {0.0, 0.8, 1.2, 3.0};
static const double kCdG1[] = {0.26, 0.30, 0.55, 0.45};

struct Table
{
    const char *name;
    const double *mach;
    const double *cd;
    double bc;
};
static const Table kTables[] = {{"G7", kMachG7, kCdG7, 0.243}, {"G1", kMachG1, kCdG1, 0.45}};

// The fixed-step methods of shot->method; the adaptive ones are exercised through the *_adaptive_* entry points.
static const int32_t kMethods[] = {BCLIBCFFI_INTEGRATION_RK4, BCLIBCFFI_INTEGRATION_VELOCITY_VERLET};
static const double kSightsM[] = {0.05, 0.1, 0.5};
static const double kVelocitiesMps[] = {300.0, 800.0};
static const double kTargetsM[] = {0.05, 0.1, 0.2, 0.4, 0.6, 0.9};
static const double kLooksDeg[] = {0.0, 10.0, -10.0};

static BCLIBCFFI_Shot make_shot(int32_t method, const Table &t, double sight_m, double velocity_mps, double look_deg,
                                double cant_deg = 0.0, double step_multiplier = 1.0)
{
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = t.bc;
    s.weight_grain = 175.0;
    s.diameter_inch = 0.3;
    s.length_inch = 1.26;
    s.muzzle_velocity_fps = velocity_mps * kFt;
    s.sight_height_ft = sight_m * kFt;
    s.twist_inch = 10.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.humidity = 0.0;
    s.mach_data = t.mach;
    s.cd_data = t.cd;
    s.drag_table_size = 4;
    s.look_angle_rad = look_deg * M_PI / 180.0;
    s.cant_angle_rad = cant_deg * M_PI / 180.0;
    s.latitude_deg = std::nan("");
    s.azimuth_deg = std::nan("");
    s.config.cStepMultiplier = step_multiplier;
    s.config.cZeroFindingAccuracy = 5e-6;
    s.config.cMinimumVelocity = 50.0;
    s.config.cMaximumDrop = -15000.0;
    s.config.cMaxIterations = 40;
    s.config.cGravityConstant = -32.17405;
    s.config.cMinimumAltitude = -1500.0;
    s.method = static_cast<BCLIBCFFI_IntegrationMethod>(method);
    return s;
}

// The miss of a shot fired at angle_rad: its height above the line of sight where it reaches the target.
static double abi_miss(const BCLIBCFFI_Shot &shot, double angle_rad, double distance_ft)
{
    BCLIBCFFI_Error err;
    double miss = 1e9;
    const double tx = distance_ft * std::cos(shot.look_angle_rad);
    const double ty = distance_ft * std::sin(shot.look_angle_rad);
    if (BCLIBCFFI_error_at_distance_shot(&shot, angle_rad, tx, ty, &miss, nullptr, &err) != BCLIBCFFI_OK)
        std::fprintf(stderr, "  error_at_distance failed: %s\n", err.message);
    return miss;
}


// With few iterations a search that has not converged must fail, not return an angle that misses. Any success must hit.
// The (target, iterations) pairs below are ones the old fallback accepted with a miss of 0.08 .. 0.4 ft (800 m/s, G7,
// 5 cm sight, cStepMultiplier 1); the 100 m and 600 m targets with 5 iterations must fail as well.
static void test_lofted_zero_unconverged_fails()
{
    BCLIBCFFI_Shot shot = make_shot(BCLIBCFFI_INTEGRATION_RK4, kTables[0], 0.05, 800.0, 0.0);
    shot.config.cMinimumVelocity = 0.0;
    const double targets_m[] = {100.0, 200.0, 300.0, 500.0, 600.0, 650.0, 750.0, 850.0};
    int failed_searches = 0;
    for (double t : targets_m)
    {
        BCLIBCFFI_Error err;
        BCLIBCFFI_ZeroPointResult zp;
        double angle = 0.0;

        // Default iterations: converges and hits.
        shot.config.cMaxIterations = 40;
        std::memset(&zp, 0, sizeof zp);
        int32_t rc = BCLIBCFFI_find_zero_point_lofted_shot(&shot, t * kFt, 1, &zp, &err);
        double miss = (rc == BCLIBCFFI_OK) ? abi_miss(shot, zp.angle_rad, t * kFt) : 1e9;
        if (rc != BCLIBCFFI_OK || std::fabs(miss) >= kMissTolFt)
            std::fprintf(stderr, "  40 iterations, point at %.0f m: rc %d, miss %.5f ft\n", t, (int)rc, miss);
        CHECK(rc == BCLIBCFFI_OK && std::fabs(miss) < kMissTolFt);
        rc = BCLIBCFFI_find_zero_angle_lofted_shot(&shot, t * kFt, 1, &angle, &err);
        miss = (rc == BCLIBCFFI_OK) ? abi_miss(shot, angle, t * kFt) : 1e9;
        if (rc != BCLIBCFFI_OK || std::fabs(miss) >= kMissTolFt)
            std::fprintf(stderr, "  40 iterations, angle at %.0f m: rc %d, miss %.5f ft\n", t, (int)rc, miss);
        CHECK(rc == BCLIBCFFI_OK && std::fabs(miss) < kMissTolFt);

        // Few iterations: a failure (ZERO_FINDING) or an angle that hits, never an angle that misses.
        for (int n = 3; n <= 10; ++n)
        {
            shot.config.cMaxIterations = n;
            std::memset(&zp, 0, sizeof zp);
            rc = BCLIBCFFI_find_zero_point_lofted_shot(&shot, t * kFt, 1, &zp, &err);
            if (rc == BCLIBCFFI_ERR_ZERO_FINDING)
                ++failed_searches;
            else
            {
                miss = (rc == BCLIBCFFI_OK) ? abi_miss(shot, zp.angle_rad, t * kFt) : 1e9;
                if (rc != BCLIBCFFI_OK || std::fabs(miss) >= kMissTolFt)
                    std::fprintf(stderr, "  %d iterations, point at %.0f m: rc %d, angle %.9f rad, miss %.5f ft\n", n, t,
                                 (int)rc, zp.angle_rad, miss);
                CHECK(rc == BCLIBCFFI_OK && std::fabs(miss) < kMissTolFt);
            }
            rc = BCLIBCFFI_find_zero_angle_lofted_shot(&shot, t * kFt, 1, &angle, &err);
            if (rc != BCLIBCFFI_ERR_ZERO_FINDING)
            {
                miss = (rc == BCLIBCFFI_OK) ? abi_miss(shot, angle, t * kFt) : 1e9;
                if (rc != BCLIBCFFI_OK || std::fabs(miss) >= kMissTolFt)
                    std::fprintf(stderr, "  %d iterations, angle at %.0f m: rc %d, angle %.9f rad, miss %.5f ft\n", n, t,
                                 (int)rc, angle, miss);
                CHECK(rc == BCLIBCFFI_OK && std::fabs(miss) < kMissTolFt);
            }
        }

        // 5 iterations is not enough for 100 m and 600 m: a failure.
        if (t == 100.0 || t == 600.0)
        {
            shot.config.cMaxIterations = 5;
            rc = BCLIBCFFI_find_zero_angle_lofted_shot(&shot, t * kFt, 1, &angle, &err);
            CHECK(rc == BCLIBCFFI_ERR_ZERO_FINDING);
            rc = BCLIBCFFI_find_zero_point_lofted_shot(&shot, t * kFt, 1, &zp, &err);
            CHECK(rc == BCLIBCFFI_ERR_ZERO_FINDING);
        }
    }
    // Sanity: the iteration limit really does stop searches.
    CHECK(failed_searches > 0);
}

int main()
{
    test_lofted_zero_unconverged_fails();
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_zero_ridder_unconverged: all checks passed");
    return EXIT_SUCCESS;
}
