// test_continuous_root.cpp -- the Ridder exits on the angle tolerance check that the root is not a jump (class C18).
//
// bclibc and the Python reference raise a zero-finding error at the two angle-tolerance exits of Ridder's method (the step
// below the tolerance, the collapse of the bracket) when |f(root)| > 0.45 * min(|f(low)|, |f(high)|) of the initial bracket: a
// lofted zero near the vertical can converge on a jump of the error function. tiny_bclibc has no lofted search and the error
// function of the low zero is continuous, so with the real ratio the check never fires (measured: at most 2.5e-4 at those exits
// in double, 1.8e-3 in float; test_identity pins the unchanged angles). This test is built against a copy of the headers whose
// ratio is lowered to 1e-30 (see tests/CMakeLists.txt, as test_ridder_unconverged does for the iteration cap), which turns every
// angle-tolerance exit into a failure: it proves that both exits are checked and report TINY_BCLIBC_ERR_ZERO_FINDING with the
// "No continuous low zero" text. The cases are G7 .308 low zeros that end on each exit in double precision.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

extern "C"
{
#include "tiny_bclibc/engine.h"
}

#include "fixture.hpp"

static int g_failures = 0;

struct Case
{
    const char *exit_name;
    double mv_fps, look_deg, sight_ft, bc, target_m;
};

static int run(const Case &c, double *angle)
{
    static real_t mach[256], cd[256];
    for (int i = 0; i < kG7TableSize; i++)
    {
        mach[i] = static_cast<real_t>(kG7Table[i].mach);
        cd[i] = static_cast<real_t>(kG7Table[i].cd);
    }
    TINY_BCLIBC_Shot shot;
    std::memset(&shot, 0, sizeof shot);
    shot.bc = c.bc;
    shot.weight_grain = 168.0;
    shot.diameter_inch = 0.308;
    shot.length_inch = 1.282;
    shot.muzzle_velocity_fps = c.mv_fps;
    shot.sight_height_ft = c.sight_ft;
    shot.temp_c = 15.0;
    shot.pressure_hpa = 1013.25;
    shot.humidity = 0.5;
    shot.mach_data = mach;
    shot.cd_data = cd;
    shot.drag_table_size = kG7TableSize;
    shot.look_angle_rad = c.look_deg * 3.14159265358979323846 / 180.0;
    shot.latitude_deg = std::numeric_limits<double>::quiet_NaN();
    shot.azimuth_deg = std::numeric_limits<double>::quiet_NaN();
    shot.config = TINY_BCLIBC_Config_default();
    TINY_BCLIBC_CurvePoint curve[kG7TableSize];
    TINY_BCLIBC_ShotProps props;
    if (tiny_bclibc_build_shot_props(&shot, curve, &props) != TINY_BCLIBC_OK)
        return -1;
    props.calc_step = kCalcStep;
    real_t a = 0;
    const int rc = tiny_bclibc_find_zero_angle_ridder(&props, static_cast<real_t>(c.target_m * 3.28084), &a);
    *angle = static_cast<double>(a);
    return rc;
}

int main()
{
#ifdef TINY_BCLIBC_SINGLE_PRECISION
    std::puts("SKIP: the cases are chosen for double precision");
    return 0;
#else
    const Case cases[] = {
        {"collapse", 3500.0, 0.0, 0.0, 0.05, 1500.0},
        {"collapse", 3500.0, 10.0, 0.0, 0.05, 800.0},
        {"tol", 3500.0, -60.0, 0.1, 0.05, 3000.0},
    };
    for (const Case &c : cases)
    {
        double angle = 0.0;
        const int rc = run(c, &angle);
        const bool ok = rc == TINY_BCLIBC_ERR_ZERO_FINDING;
        std::printf("%s exit case (mv %.0f look %.0f d %.0f m): rc %d (%s)\n", c.exit_name, c.mv_fps, c.look_deg, c.target_m, rc,
                    ok ? "ok" : "EXPECTED ZERO_FINDING");
        if (!ok)
            ++g_failures;
#if !defined(TINY_BCLIBC_NO_ERR_BUF)
        if (ok && std::strstr(tiny_bclibc_last_error(), "No continuous low zero") == nullptr)
        {
            std::printf("FAIL: last_error text: %s\n", tiny_bclibc_last_error());
            ++g_failures;
        }
#endif
    }
    if (g_failures)
    {
        std::printf("%d check(s) failed\n", g_failures);
        return 1;
    }
    std::puts("OK: both angle-tolerance exits of the Ridder search check for a discontinuity");
    return 0;
#endif
}
