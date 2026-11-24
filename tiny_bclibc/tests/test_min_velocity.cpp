// test_min_velocity.cpp -- the search entries ignore the caller's cMinimumVelocity, as bclibc's do.
//
// The maximum-range search, the apex search and the zero searches must not stop on the minimum-velocity terminator
// (bclibc: @with_no_minimum_velocity / ValueGuard in find_max_range, find_apex, find_zero_angle_ridder), and the
// caller's configuration is unchanged afterwards (also after an error return). The intercept shot of find_zero_point
// takes its point from that search, so a minimum velocity above the muzzle velocity is not an interception failure.
// Each result is compared with bclibc's through its C++ API (same settings). The Python reference (15277.6 ft at
// 1500 fps in the report that led to this test) belongs to another bullet than this fixture, so it is not asserted
// here: the engine-to-engine difference to Python (bclibc 15153.6 vs Python 15277.6 ft, 0.8%) is a known, separate one.

#include <cmath>
#include <cstdio>
#include <cstring>

#include "bclibc.hpp"

extern "C"
{
#include "tiny_bclibc/engine.h"
}

#include "fixture.hpp"

static int g_failures = 0;
#define CHECK(cond, ...)                                                  \
    do                                                                    \
    {                                                                     \
        if (!(cond))                                                      \
        {                                                                 \
            std::printf("FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); \
            std::printf(__VA_ARGS__);                                     \
            std::printf("\n");                                            \
            ++g_failures;                                                 \
        }                                                                 \
    } while (0)

namespace
{
    constexpr double kApexMaxRad = 1.5707963267948966 * 0.99;
    // Same search, same crossing: bclibc stops its golden-section search at 1e-6 rad, tiny at 1e-5; the range is flat
    // at its maximum, so the two ranges agree far closer than the angles do.
    constexpr double kRangeTolFt = 0.05;
    constexpr double kAngleTolRad = 1e-5; // as test_identity: the solver tolerance of tiny
    constexpr double kZeroDistFt = 900.0;

    double g_mach[kG7TableSize], g_cd[kG7TableSize];

    void init_arrays()
    {
        for (int i = 0; i < kG7TableSize; ++i)
        {
            g_mach[i] = kG7Table[i].mach;
            g_cd[i] = kG7Table[i].cd;
        }
    }

    bclibc::BCLIBC_ShotProps bc_props(double elevation)
    {
        init_arrays();
        bclibc::BCLIBC_Shot shot;
        shot.bc = G7_BASIC::BC;
        shot.weight_grain = G7_BASIC::WEIGHT_GR;
        shot.diameter_inch = G7_BASIC::DIAMETER_IN;
        shot.length_inch = G7_BASIC::LENGTH_IN;
        shot.muzzle_velocity_fps = G7_BASIC::MV_FPS;
        shot.stability_coefficient = 0.0;
        shot.sight_height_ft = 0.1;
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
        shot.look_angle_rad = 0.0;
        shot.barrel_elevation_rad = elevation;
        shot.barrel_azimuth_rad = G7_BASIC::BARREL_AZ_RAD;
        shot.cant_angle_rad = G7_BASIC::CANT_ANGLE_RAD;
        shot.latitude_deg = G7_BASIC::LAT_DEG;
        shot.azimuth_deg = G7_BASIC::AZ_DEG;
        shot.calc_step = kCalcStep;
        auto r = shot.to_shot_props();
        if (bclibc::has_error(r))
        {
            std::printf("bclibc to_shot_props failed\n");
            std::exit(2);
        }
        return r.value();
    }

    void init_bc(bclibc::BCLIBC_BaseEngine &eng, double elevation, double min_velocity)
    {
        eng.shot = bc_props(elevation);
        eng.integrate_func = bclibc::BCLIBC_integrateTsitouras;
        eng.config = bclibc::BCLIBC_Config(kDefaultConfig.cStepMultiplier, kDefaultConfig.cZeroFindingAccuracy,
                                           min_velocity, kDefaultConfig.cMaximumDrop, kDefaultConfig.cMaxIterations,
                                           kDefaultConfig.cGravityConstant, kDefaultConfig.cMinimumAltitude);
        eng.gravity_vector = bclibc::BCLIBC_V3dT(0.0, eng.config.cGravityConstant, 0.0);
    }

    int tiny_props(TINY_BCLIBC_ShotProps *out, TINY_BCLIBC_CurvePoint *curve, double elevation, double min_velocity)
    {
        init_arrays();
        TINY_BCLIBC_Config cfg = TINY_BCLIBC_Config_default();
        cfg.cZeroFindingAccuracy = kDefaultConfig.cZeroFindingAccuracy;
        cfg.cMinimumVelocity = min_velocity;
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
        shot.sight_height_ft = 0.1;
        shot.twist_inch = G7_BASIC::TWIST_IN;
        shot.temp_c = G7_BASIC::TEMP_C;
        shot.pressure_hpa = G7_BASIC::PRESSURE_HPA;
        shot.altitude_ft = G7_BASIC::ALT_FT;
        shot.humidity = G7_BASIC::HUMIDITY;
        shot.mach_data = g_mach;
        shot.cd_data = g_cd;
        shot.drag_table_size = kG7TableSize;
        shot.look_angle_rad = 0.0;
        shot.barrel_elevation_rad = elevation;
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

    bool same_bytes(const TINY_BCLIBC_ShotProps &a, const TINY_BCLIBC_ShotProps &b)
    {
        return std::memcmp(&a, &b, sizeof(a)) == 0;
    }

    void test_max_range(double min_velocity)
    {
        bclibc::BCLIBC_BaseEngine bc;
        init_bc(bc, 0.0, min_velocity);
        auto bcr = bc.find_max_range(0.0, 90.0, kApexMaxRad);
        CHECK(!bclibc::has_error(bcr), "bclibc find_max_range");
        if (bclibc::has_error(bcr))
            return;

        TINY_BCLIBC_CurvePoint curve[kG7TableSize];
        TINY_BCLIBC_ShotProps props, before;
        CHECK(tiny_props(&props, curve, 0.0, min_velocity) == TINY_BCLIBC_OK, "build");
        before = props;
        real_t range = -1, angle = -1;
        const int rc = tiny_bclibc_find_max_range(&props, 0.0, 90.0, &range, &angle);
        std::printf("  min velocity %.0f fps: max range tiny %.2f ft (%.5f rad), bclibc %.2f ft (%.5f rad)\n",
                    min_velocity, (double)range, (double)angle, bcr.value().max_range_ft, bcr.value().angle_at_max_rad);
        CHECK(rc == TINY_BCLIBC_OK, "find_max_range rc=%d", rc);
        CHECK(std::fabs(range - bcr.value().max_range_ft) < kRangeTolFt, "range %g vs bclibc %g", (double)range,
              bcr.value().max_range_ft);
        CHECK(std::fabs(angle - bcr.value().angle_at_max_rad) < kAngleTolRad, "angle %g vs bclibc %g", (double)angle,
              bcr.value().angle_at_max_rad);
        CHECK(range > 10000.0, "range %g ft is not a real maximum range", (double)range);
        CHECK(same_bytes(props, before), "the caller's props changed (cMinimumVelocity %g)", (double)props.cfg.cMinimumVelocity);
        CHECK(props.cfg.cMinimumVelocity == min_velocity, "cMinimumVelocity %g", (double)props.cfg.cMinimumVelocity);
    }

    void test_restored_after_error()
    {
        // An error return (a gravity constant that is not a negative number) leaves the config as it was, too.
        TINY_BCLIBC_CurvePoint curve[kG7TableSize];
        TINY_BCLIBC_ShotProps props, before;
        CHECK(tiny_props(&props, curve, 0.01, 1500.0) == TINY_BCLIBC_OK, "build");
        props.cfg.cGravityConstant = REAL_C(0.0);
        before = props;
        real_t range, angle;
        CHECK(tiny_bclibc_find_max_range(&props, 0.0, 90.0, &range, &angle) == TINY_BCLIBC_ERR_INVALID_ARG, "max range error");
        TINY_BCLIBC_TrajectoryData apex;
        CHECK(tiny_bclibc_find_apex(&props, &apex) == TINY_BCLIBC_ERR_INVALID_ARG, "apex error");
        real_t a;
        CHECK(tiny_bclibc_find_zero_angle_ridder(&props, kZeroDistFt, &a) == TINY_BCLIBC_ERR_INVALID_ARG, "ridder error");
        TINY_BCLIBC_ZeroPointResult zp;
        CHECK(tiny_bclibc_find_zero_point(&props, kZeroDistFt, &zp) == TINY_BCLIBC_ERR_INVALID_ARG, "zero point error");
        CHECK(same_bytes(props, before), "the caller's props changed after an error return");
        CHECK(props.cfg.cMinimumVelocity == 1500.0, "cMinimumVelocity %g after an error", (double)props.cfg.cMinimumVelocity);
    }

    void test_zero_entries(double min_velocity)
    {
        std::printf("  zero entries, min velocity %.0f fps (muzzle %.0f)\n", min_velocity, G7_BASIC::MV_FPS);
        bclibc::BCLIBC_BaseEngine bc;
        init_bc(bc, 0.0, min_velocity);
        auto bc_pt = bc.find_zero_point(kZeroDistFt, 0, kApexMaxRad, 0.01);
        auto bc_ang = bc.find_zero_angle(kZeroDistFt, 0, kApexMaxRad, 0.01);
        CHECK(!bclibc::has_error(bc_pt) && !bclibc::has_error(bc_ang), "bclibc zero search");
        if (bclibc::has_error(bc_pt) || bclibc::has_error(bc_ang))
            return;

        TINY_BCLIBC_CurvePoint curve[kG7TableSize];
        TINY_BCLIBC_ShotProps props, before;
        CHECK(tiny_props(&props, curve, 0.0, min_velocity) == TINY_BCLIBC_OK, "build");
        before = props;

        TINY_BCLIBC_ZeroPointResult zp;
        int rc = tiny_bclibc_find_zero_point(&props, kZeroDistFt, &zp);
        CHECK(rc == TINY_BCLIBC_OK, "find_zero_point rc=%d", rc);
        if (rc == TINY_BCLIBC_OK)
        {
            std::printf("    find_zero_point: tiny %.6f rad, bclibc %.6f rad\n", (double)zp.angle_rad, bc_pt.value().angle_rad);
            CHECK(std::fabs(zp.angle_rad - bc_pt.value().angle_rad) < kAngleTolRad, "angle");
        }
        rc = tiny_bclibc_find_zero_point_ridder(&props, kZeroDistFt, &zp);
        CHECK(rc == TINY_BCLIBC_OK, "find_zero_point_ridder rc=%d", rc);
        if (rc == TINY_BCLIBC_OK)
            CHECK(std::fabs(zp.angle_rad - bc_pt.value().angle_rad) < kAngleTolRad, "ridder point angle");

        real_t a = 0;
        rc = tiny_bclibc_find_zero_angle(&props, kZeroDistFt, &a);
        CHECK(rc == TINY_BCLIBC_OK && std::fabs(a - bc_ang.value()) < kAngleTolRad, "find_zero_angle rc=%d %g vs %g", rc,
              (double)a, bc_ang.value());
        rc = tiny_bclibc_find_zero_angle_ridder(&props, kZeroDistFt, &a);
        CHECK(rc == TINY_BCLIBC_OK && std::fabs(a - bc_ang.value()) < kAngleTolRad, "find_zero_angle_ridder rc=%d %g vs %g",
              rc, (double)a, bc_ang.value());
        CHECK(same_bytes(props, before), "the caller's props changed");
    }

    void test_apex(double min_velocity)
    {
        bclibc::BCLIBC_BaseEngine bc;
        init_bc(bc, 0.02, min_velocity);
        bclibc::BCLIBC_BaseTrajData bc_apex;
        const bool bc_ok = !bclibc::has_error(bc.find_apex(bc_apex));
        CHECK(bc_ok, "bclibc find_apex");
        TINY_BCLIBC_CurvePoint curve[kG7TableSize];
        TINY_BCLIBC_ShotProps props, before;
        CHECK(tiny_props(&props, curve, 0.02, min_velocity) == TINY_BCLIBC_OK, "build");
        before = props;
        TINY_BCLIBC_TrajectoryData apex;
        const int rc = tiny_bclibc_find_apex(&props, &apex);
        CHECK(rc == TINY_BCLIBC_OK, "find_apex rc=%d at min velocity %g", rc, min_velocity);
        if (rc == TINY_BCLIBC_OK && bc_ok)
        {
            std::printf("  find_apex, min velocity %.0f: tiny x %.3f ft, bclibc x %.3f ft\n", min_velocity,
                        (double)apex.distance_ft, bc_apex.px);
            CHECK(std::fabs(apex.distance_ft - bc_apex.px) < 0.05, "apex distance");
        }
        CHECK(same_bytes(props, before), "the caller's props changed");
    }
} // namespace

int main()
{
    std::printf("max range\n");
    test_max_range(50.0);   // the default: unchanged
    test_max_range(1500.0); // a bullet that slows through 1500 fps
    test_max_range(3000.0); // above the muzzle velocity
    test_restored_after_error();
    test_zero_entries(50.0);
    test_zero_entries(3000.0);
    test_apex(1500.0);
    test_apex(3000.0);
    std::printf("%s\n", g_failures == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
    return g_failures == 0 ? 0 : 1;
}
