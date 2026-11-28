// test_cant_max_range_bracket.cpp -- tiny_bclibc: the max-range search of a canted shot brackets only the reachable
// elevations (class C38; mirrors bclibc's tests/test_cant_max_range_bracket.cpp).
//
// With a cant the elevation that tiny_bclibc__set_aim can reach is |E| <= asin(R), R = hypot(sin L, cos L cos cant) (30 deg at
// cant 60 and a horizontal look). The golden-section search of the maximum range used to bracket (0, 90 deg): its first probes
// were both above the limit, clamped to one state, the ranges were equal and the search walked away from the maximum, so the
// close zero (D = 0.1 and 0.2 m, sight height 0.05 m, look 0, cant 60 deg) failed with out of range. The upper bound is now
// clipped to asin(R) when cant != 0 (cant == 0 untouched).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <vector>

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
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kFt = 3.28084;
    real_t g_mach[kG7TableSize], g_cd[kG7TableSize];
    TINY_BCLIBC_CurvePoint g_curve[kG7TableSize + 8];
    bool g_print = false;

    int make_props(TINY_BCLIBC_ShotProps *out, double look_deg, double cant_deg)
    {
        for (int i = 0; i < kG7TableSize; ++i)
        {
            g_mach[i] = static_cast<real_t>(kG7Table[i].mach);
            g_cd[i] = static_cast<real_t>(kG7Table[i].cd);
        }
        TINY_BCLIBC_Shot s;
        std::memset(&s, 0, sizeof s);
        s.bc = static_cast<real_t>(0.31);
        s.weight_grain = 175;
        s.diameter_inch = static_cast<real_t>(0.308);
        s.length_inch = static_cast<real_t>(1.24);
        s.muzzle_velocity_fps = static_cast<real_t>(800.0 * kFt);
        s.sight_height_ft = static_cast<real_t>(0.05 * kFt);
        s.twist_inch = 11;
        s.temp_c = 15;
        s.pressure_hpa = static_cast<real_t>(1013.25);
        s.humidity = 0;
        s.mach_data = g_mach;
        s.cd_data = g_cd;
        s.drag_table_size = kG7TableSize;
        s.latitude_deg = std::numeric_limits<real_t>::quiet_NaN();
        s.azimuth_deg = std::numeric_limits<real_t>::quiet_NaN();
        s.look_angle_rad = static_cast<real_t>(look_deg * kPi / 180.0);
        s.cant_angle_rad = static_cast<real_t>(cant_deg * kPi / 180.0);
        s.config.cStepMultiplier = 1;
        s.config.cZeroFindingAccuracy = static_cast<real_t>(5e-6);
        s.config.cMinimumVelocity = 50;
        s.config.cMaximumDrop = -15000;
        s.config.cMaxIterations = 40;
        s.config.cGravityConstant = static_cast<real_t>(-32.17405);
        s.config.cMinimumAltitude = -1500;
        return tiny_bclibc_build_shot_props(&s, g_curve, out);
    }

    void test_close_zero_cant60()
    {
        const double tol_ft = sizeof(real_t) == sizeof(float) ? 5e-3 : 1e-3;
        for (double d_m : {0.1, 0.2})
        {
            TINY_BCLIBC_ShotProps props;
            CHECK(make_props(&props, 0.0, 60.0) == TINY_BCLIBC_OK, "build");
            const double d_ft = d_m * kFt;
            for (int entry = 0; entry < 2; ++entry)
            {
                real_t angle = 0;
                const int rc = entry == 0 ? tiny_bclibc_find_zero_angle(&props, static_cast<real_t>(d_ft), &angle)
                                          : tiny_bclibc_find_zero_angle_ridder(&props, static_cast<real_t>(d_ft), &angle);
                CHECK(rc == TINY_BCLIBC_OK, "zero D=%g entry %d rc=%d", d_m, entry, rc);
                if (rc != TINY_BCLIBC_OK)
                    continue;
                TINY_BCLIBC_ShotProps p = props; // a zero at the target of a close shot: the limits of the shot
                const double miss = tiny_bclibc__error_at_distance(&p, angle, static_cast<real_t>(d_ft), 0);
                CHECK(std::fabs(miss) < tol_ft, "miss %g ft D=%g entry %d", miss, d_m, entry);
                CHECK(angle <= static_cast<real_t>(30.0 * kPi / 180.0) + static_cast<real_t>(1e-5), "reachable");
                if (d_m == 0.1)
                    CHECK(angle * 180.0 / kPi > 12.5 && angle * 180.0 / kPi < 13.5, "angle %g deg", angle * 180.0 / kPi);
                if (g_print)
                    std::printf("zero cant 60 D %g %s: %.9f deg miss %.3e ft\n", d_m, entry == 0 ? "find_zero_angle" : "ridder",
                                static_cast<double>(angle) * 180.0 / kPi, miss);
            }
        }
    }

    void test_max_range_scan()
    {
        for (double look : {0.0, 10.0, -10.0})
            for (double cant : {0.0, 45.0, 60.0, 75.0, 80.0})
            {
                TINY_BCLIBC_ShotProps props;
                CHECK(make_props(&props, look, cant) == TINY_BCLIBC_OK, "build");
                const double reach = std::asin(std::hypot(std::sin(look * kPi / 180.0), std::cos(look * kPi / 180.0) * std::cos(cant * kPi / 180.0)));
                real_t range = 0, angle = 0;
                const int rc = tiny_bclibc_find_max_range(&props, 0, 90, &range, &angle);
                CHECK(rc == TINY_BCLIBC_OK, "max range look %g cant %g rc=%d", look, cant, rc);
                if (rc != TINY_BCLIBC_OK)
                    continue;
                CHECK(angle <= reach + 1e-5, "angle %g above limit %g", static_cast<double>(angle), reach);
                CHECK(angle >= 0, "angle");
                TINY_BCLIBC_ShotProps p = props;
                (void)tiny_bclibc__lift_limits_if_downhill(&p); // the limits of the max-range search itself
                double best = 0, best_e = 0;
                const int n = 400; // the search brackets [0, limit]: the caller's lower bound is 0
                for (int i = 0; i <= n; ++i)
                {
                    const double e = reach * i / n;
                    real_t r = 0;
                    if (tiny_bclibc__range_for_angle(&p, static_cast<real_t>(e), &r) == TINY_BCLIBC_OK && r > best)
                    {
                        best = r;
                        best_e = e;
                    }
                }
                // tiny stops at a bracket of 1e-5 rad and the maximum is flat; single precision is coarser
                const double rel = sizeof(real_t) == sizeof(float) ? 2e-3 : 1e-4;
                CHECK(range >= best * (1.0 - rel), "range %g < scan %g (look %g cant %g)", static_cast<double>(range), best, look, cant);
                CHECK(range <= best * (1.0 + rel), "range %g > scan %g (look %g cant %g)", static_cast<double>(range), best, look, cant);
                if (g_print)
                    std::printf("maxrange look %g cant %g: %.9f ft at %.9f deg (limit %.6f deg; scan %.9f ft at %.6f deg)\n", look, cant,
                                static_cast<double>(range), static_cast<double>(angle) * 180.0 / kPi, reach * 180.0 / kPi, best,
                                best_e * 180.0 / kPi);
            }
    }
} // namespace

int main(int argc, char **argv)
{
    g_print = argc > 1 && std::strcmp(argv[1], "--print") == 0;
    test_close_zero_cant60();
    test_max_range_scan();
    if (g_failures)
    {
        std::printf("%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("OK\n");
    return 0;
}
