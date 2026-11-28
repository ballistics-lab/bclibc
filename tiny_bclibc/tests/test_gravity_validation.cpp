// test_gravity_validation.cpp -- tiny_bclibc rejects a gravity constant that is not a finite negative number.
//
// The y axis points up (the default is -GRAVITY_IMPERIAL). With 0 or a positive value the projectile never falls, no
// terminator is reached and the integration never returns; NaN and infinity are no gravity at all. Every entry point
// (integrate, find_apex, the zero searches, find_max_range) must return TINY_BCLIBC_ERR_INVALID_ARG at once, as bclibc
// returns its Value error. A sane value (Earth, the Moon) still works. The bad-gravity calls never returned before the
// check, so CMake gives the test a time limit.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>

extern "C"
{
#include "tiny_bclibc/engine.h"
}

#include "fixture.hpp"

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
    constexpr double kFtPerM = 3.28084;
    double g_mach[kG7TableSize], g_cd[kG7TableSize];

    int make_props(TINY_BCLIBC_ShotProps *out, TINY_BCLIBC_CurvePoint *curve, double gravity)
    {
        for (int i = 0; i < kG7TableSize; ++i)
        {
            g_mach[i] = kG7Table[i].mach;
            g_cd[i] = kG7Table[i].cd;
        }
        TINY_BCLIBC_Config cfg = TINY_BCLIBC_Config_default();
        cfg.cGravityConstant = static_cast<real_t>(gravity);
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
        shot.barrel_elevation_rad = 0.01; // find_apex needs a rising shot
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

    // Runs the entry classes; rcs[i] is the status of each.
    constexpr int kEntries = 7;
    const char *const kEntryNames[kEntries] = {"integrate", "integrate_at", "find_apex", "find_zero_angle",
                                               "find_zero_angle_ridder", "find_zero_point", "find_max_range"};

    void run_all(const TINY_BCLIBC_ShotProps *props, int rcs[kEntries])
    {
        {
            TINY_BCLIBC_TrajectoryRequest req;
            req.range_limit_ft = 3000.0;
            req.range_step_ft = 300.0;
            req.time_step = 0.0;
            req.filter_flags = TINY_BCLIBC_TRAJ_FLAG_RANGE;
            TINY_BCLIBC_TrajectoryData buf[64];
            int32_t written = 0, total = 0, reason = 0;
            rcs[0] = tiny_bclibc_integrate(props, &req, buf, 64, &written, &total, &reason);
        }
        {
            TINY_BCLIBC_BaseTrajData raw;
            TINY_BCLIBC_TrajectoryData full;
            rcs[1] = tiny_bclibc_integrate_at(props, TINY_BCLIBC_KEY_POS_X, 1500.0, &raw, &full);
        }
        {
            TINY_BCLIBC_TrajectoryData apex;
            rcs[2] = tiny_bclibc_find_apex(props, &apex);
        }
        {
            real_t angle;
            rcs[3] = tiny_bclibc_find_zero_angle(props, 900.0, &angle);
            rcs[4] = tiny_bclibc_find_zero_angle_ridder(props, 900.0, &angle);
        }
        {
            TINY_BCLIBC_ZeroPointResult z;
            rcs[5] = tiny_bclibc_find_zero_point(props, 900.0, &z);
        }
        {
            real_t range, angle;
            rcs[6] = tiny_bclibc_find_max_range(props, 0.0, 90.0, &range, &angle);
        }
    }
} // namespace

int main()
{
    const double bad[] = {0.0, -0.0, 9.80665, 32.17405, std::nan(""), INFINITY, -INFINITY};
    for (double g : bad)
    {
        TINY_BCLIBC_CurvePoint curve[kG7TableSize];
        TINY_BCLIBC_ShotProps props;
        // The config of a built shot is a plain field: both ways in (set before the build, patched after) are checked.
        for (int patched = 0; patched < 2; ++patched)
        {
            const int brc = make_props(&props, curve, patched ? -32.17405 : g);
            CHECK(brc == TINY_BCLIBC_OK || brc == TINY_BCLIBC_ERR_INVALID_ARG, "build rc=%d", brc);
            if (brc != TINY_BCLIBC_OK)
                continue; // rejected already at the build: also fine
            if (patched)
                props.cfg.cGravityConstant = static_cast<real_t>(g);
            int rcs[kEntries];
            run_all(&props, rcs);
            for (int i = 0; i < kEntries; ++i)
                CHECK(rcs[i] == TINY_BCLIBC_ERR_INVALID_ARG, "gravity %g (%s): %s rc=%d", g, patched ? "patched" : "built",
                      kEntryNames[i], rcs[i]);
        }
    }

    // Sane values still work: Earth and the Moon.
    double apex_x[2] = {0.0, 0.0};
    int idx = 0;
    for (double g : {-9.80665 * kFtPerM, -1.62 * kFtPerM})
    {
        TINY_BCLIBC_CurvePoint curve[kG7TableSize];
        TINY_BCLIBC_ShotProps props;
        CHECK(make_props(&props, curve, g) == TINY_BCLIBC_OK, "build");
        int rcs[kEntries];
        run_all(&props, rcs);
        for (int i = 0; i < kEntries; ++i)
            CHECK(rcs[i] == TINY_BCLIBC_OK, "gravity %g: %s rc=%d", g, kEntryNames[i], rcs[i]);
        TINY_BCLIBC_TrajectoryData apex;
        if (tiny_bclibc_find_apex(&props, &apex) == TINY_BCLIBC_OK)
            apex_x[idx] = apex.distance_ft;
        ++idx;
    }
    CHECK(apex_x[1] > 1.5 * apex_x[0] && apex_x[0] > 0.0, "the Moon's apex is farther: %g vs %g", apex_x[1], apex_x[0]);

    std::printf("%s\n", g_failures == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
    return g_failures == 0 ? 0 : 1;
}
