// test_ridder_unconverged.cpp -- tiny_bclibc's Ridder zero search fails when it runs out of iterations.
//
// tiny_bclibc__find_zero_angle_ridders() used to "accept" an unconverged bracket narrower than 10 * acc and return its
// midpoint, an angle that can miss the target by feet. It must now return TINY_BCLIBC_ERR_ZERO_FINDING, as bclibc's
// BCLIBC_BaseEngine::find_zero_angle_ridder does.
//
// The iteration cap of the tiny_bclibc Ridder loop is a local constant of the header (`int32_t max_iter = 50;`, it does not
// read cfg.cMaxIterations), and the search converges in about 3 iterations, so no input reaches the failing path with the
// stock cap. CMake therefore compiles this test against a copy of the headers whose cap is lowered to
// TINY_RIDDER_ITER_CAP (see tests/CMakeLists.txt); the library itself is not changed. With the cap of 2 the cases below
// leave a bracket narrower than 10 * acc = 0.01 rad, which the pre-fix code accepted: it answered an angle missing by
// 6.8 .. 14 ft; now it must fail. bclibc is run on the same input with a lowered cMaxIterations and must fail with the
// same code; with the default iterations both solve each case and hit (test_identity).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

#include "bclibc.hpp"

extern "C"
{
#include "tiny_bclibc/engine.h"
}

#include "fixture.hpp"

#ifndef TINY_RIDDER_ITER_CAP
#error "TINY_RIDDER_ITER_CAP must be defined by the build (the iteration cap the header was patched to)"
#endif

static int g_failures = 0;
#define CHECK(cond, ...)                                                    \
    do                                                                      \
    {                                                                       \
        if (!(cond))                                                        \
        {                                                                   \
            std::printf("FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond);   \
            std::printf(__VA_ARGS__);                                       \
            std::printf("\n");                                              \
            ++g_failures;                                                   \
        }                                                                   \
    } while (0)

namespace
{
    constexpr double kFt = 3.28084;
    constexpr double kDeg = 0.017453292519943295;
    constexpr double kApexMaxRad = 1.5707963267948966 * 0.99;
    constexpr double kAllowedErr = 1e-2;
    constexpr int kBclibcLowIterations = 1; // bclibc's Ridder loop is capped by cMaxIterations

    double g_mach[kG7TableSize], g_cd[kG7TableSize];

    void init_arrays()
    {
        for (int i = 0; i < kG7TableSize; ++i)
        {
            g_mach[i] = kG7Table[i].mach;
            g_cd[i] = kG7Table[i].cd;
        }
    }

    bclibc::BCLIBC_ShotProps make_bclibc_props(double sight_ft, double look)
    {
        init_arrays();
        bclibc::BCLIBC_Shot shot;
        shot.bc = G7_BASIC::BC;
        shot.weight_grain = G7_BASIC::WEIGHT_GR;
        shot.diameter_inch = G7_BASIC::DIAMETER_IN;
        shot.length_inch = G7_BASIC::LENGTH_IN;
        shot.muzzle_velocity_fps = G7_BASIC::MV_FPS;
        shot.stability_coefficient = 0.0;
        shot.sight_height_ft = sight_ft;
        shot.twist_inch = G7_BASIC::TWIST_IN;
        shot.temp_c = G7_BASIC::TEMP_C;
        shot.pressure_hpa = G7_BASIC::PRESSURE_HPA;
        shot.altitude_ft = G7_BASIC::ALT_FT;
        shot.humidity = G7_BASIC::HUMIDITY;
        shot.mach_data = g_mach;
        shot.cd_data = g_cd;
        shot.drag_table_size = kG7TableSize;
        shot.winds = nullptr;
        shot.wind_count = 0;
        shot.look_angle_rad = look;
        shot.barrel_elevation_rad = G7_BASIC::BARREL_EL_RAD;
        shot.barrel_azimuth_rad = G7_BASIC::BARREL_AZ_RAD;
        shot.cant_angle_rad = G7_BASIC::CANT_ANGLE_RAD;
        shot.latitude_deg = G7_BASIC::LAT_DEG;
        shot.azimuth_deg = G7_BASIC::AZ_DEG;
        shot.calc_step = kCalcStep;
        auto r = shot.to_shot_props();
        if (bclibc::has_error(r))
        {
            std::printf("FAIL: to_shot_props\n");
            std::exit(2);
        }
        return std::move(r).value();
    }

    void init_bclibc_engine(bclibc::BCLIBC_BaseEngine &eng, bclibc::BCLIBC_ShotProps props, int max_iterations)
    {
        eng.shot = std::move(props);
        eng.integrate_func = bclibc::BCLIBC_integrateTsitouras;
        eng.config = bclibc::BCLIBC_Config(kDefaultConfig.cStepMultiplier, kDefaultConfig.cZeroFindingAccuracy,
                                           kDefaultConfig.cMinimumVelocity, kDefaultConfig.cMaximumDrop, max_iterations,
                                           kDefaultConfig.cGravityConstant, kDefaultConfig.cMinimumAltitude);
        eng.gravity_vector = bclibc::BCLIBC_V3dT(0.0, eng.config.cGravityConstant, 0.0);
    }

    int make_tiny_props(TINY_BCLIBC_ShotProps *out, TINY_BCLIBC_CurvePoint *curve, double sight_ft, double look)
    {
        init_arrays();
        TINY_BCLIBC_Config cfg = TINY_BCLIBC_Config_default();
        cfg.cZeroFindingAccuracy = kDefaultConfig.cZeroFindingAccuracy;
        cfg.cMinimumVelocity = kDefaultConfig.cMinimumVelocity;
        cfg.cMaximumDrop = kDefaultConfig.cMaximumDrop;
        cfg.cMaxIterations = kDefaultConfig.cMaxIterations;
        cfg.cGravityConstant = kDefaultConfig.cGravityConstant;
        cfg.cMinimumAltitude = kDefaultConfig.cMinimumAltitude;
        TINY_BCLIBC_Shot shot;
        std::memset(&shot, 0, sizeof(shot));
        shot.bc = G7_BASIC::BC;
        shot.weight_grain = G7_BASIC::WEIGHT_GR;
        shot.diameter_inch = G7_BASIC::DIAMETER_IN;
        shot.length_inch = G7_BASIC::LENGTH_IN;
        shot.muzzle_velocity_fps = G7_BASIC::MV_FPS;
        shot.sight_height_ft = sight_ft;
        shot.twist_inch = G7_BASIC::TWIST_IN;
        shot.temp_c = G7_BASIC::TEMP_C;
        shot.pressure_hpa = G7_BASIC::PRESSURE_HPA;
        shot.altitude_ft = G7_BASIC::ALT_FT;
        shot.humidity = G7_BASIC::HUMIDITY;
        shot.mach_data = g_mach;
        shot.cd_data = g_cd;
        shot.drag_table_size = kG7TableSize;
        shot.look_angle_rad = look;
        shot.barrel_elevation_rad = G7_BASIC::BARREL_EL_RAD;
        shot.barrel_azimuth_rad = G7_BASIC::BARREL_AZ_RAD;
        shot.cant_angle_rad = G7_BASIC::CANT_ANGLE_RAD;
        shot.latitude_deg = G7_BASIC::LAT_DEG;
        shot.azimuth_deg = G7_BASIC::AZ_DEG;
        shot.config = cfg;
        const int rc = tiny_bclibc_build_shot_props(&shot, curve, out);
        if (rc == TINY_BCLIBC_OK)
            out->calc_step = kCalcStep;
        return rc;
    }

    struct Case
    {
        double look_deg, sight_m, target_m;
    };
    // With the cap of 2 iterations the pre-fix tiny_bclibc answered these with misses of 9.35, 6.78, 13.9, 6.79 and 13.9 ft.
    // (bclibc, whose loop is capped by cMaxIterations, also runs out of iterations at 1 on all of them.)
    const Case kCases[] = {{0.0, 0.05, 2000.0}, {-10.0, 0.05, 1500.0}, {-10.0, 0.05, 2000.0}, {-10.0, 0.1, 1500.0},
                           {-10.0, 0.1, 2000.0}};
} // namespace

int main()
{
    std::printf("tiny_bclibc Ridder zero search, unconverged (iteration cap %d)\n", TINY_RIDDER_ITER_CAP);
    for (const Case &c : kCases)
    {
        const double look = c.look_deg * kDeg;
        const double sight_ft = c.sight_m * kFt;
        const double dist_ft = c.target_m * kFt;

        TINY_BCLIBC_CurvePoint curve[kG7TableSize];
        TINY_BCLIBC_ShotProps props;
        if (make_tiny_props(&props, curve, sight_ft, look) != TINY_BCLIBC_OK)
        {
            std::printf("FAIL: tiny_bclibc build\n");
            return 1;
        }
        double angle = -123.0;
        const int rc = tiny_bclibc__find_zero_angle_ridders(&props, dist_ft, &angle);
        double miss = std::numeric_limits<double>::quiet_NaN();
        if (rc == TINY_BCLIBC_OK)
        {
            TINY_BCLIBC_ShotProps fire = props;
            miss = tiny_bclibc__error_at_distance(&fire, angle, dist_ft * std::cos(look), dist_ft * std::sin(look));
        }
        std::printf("  look %+.0f deg, sight %.2f m, target %.0f m: tiny_bclibc rc=%d%s", c.look_deg, c.sight_m, c.target_m, rc,
                    rc == TINY_BCLIBC_OK ? "" : " (not converged)");
        if (rc == TINY_BCLIBC_OK)
            std::printf(", returned angle %.6f rad that misses by %.3f ft", angle, miss);
        std::printf("\n");
        CHECK(rc == TINY_BCLIBC_ERR_ZERO_FINDING, "tiny_bclibc rc=%d, miss %.3f ft (an unconverged Ridder search must fail)",
              rc, miss);

        // bclibc on the same input with its Ridder loop cut short fails with a zero-finding error too.
        bclibc::BCLIBC_BaseEngine bc_eng;
        init_bclibc_engine(bc_eng, make_bclibc_props(sight_ft, look), kBclibcLowIterations);
        auto bc_z = bc_eng.find_zero_angle(dist_ft, 0, kApexMaxRad, kAllowedErr);
        CHECK(bclibc::has_error(bc_z) && bc_z.error().kind() == bclibc::BCLIBC_Error::Kind::SolverZeroFinding,
              "bclibc did not fail with a zero-finding error with cMaxIterations=%d", kBclibcLowIterations);
    }

    std::printf("\n%s\n", g_failures == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
    return g_failures == 0 ? 0 : 1;
}
