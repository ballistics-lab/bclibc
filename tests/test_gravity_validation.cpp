// test_gravity_validation.cpp -- a gravity constant that is not a finite negative number is an error, not a loop.
//
// The y axis points up, so the gravity constant of the config is negative (-32.17405 ft/s^2 by default). With 0 or a
// positive value the projectile never falls: no terminator is ever reached and integrate / find_apex / find_max_range /
// every zero search never returned. The engine entry points now check the value once, up front, and return a value error
// ("gravity_constant must be negative ...", BCLIBCFFI_ERR_INVALID_ARGUMENT) -- also for NaN and infinity.
//
// Through the flat C ABI only. The bad-gravity calls never returned before the check, so the test has a time limit in CMake.

#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

static int failures = 0;

#define CHECK(cond)                                                                    \
    do                                                                                 \
    {                                                                                  \
        if (!(cond))                                                                   \
        {                                                                              \
            std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            ++failures;                                                                \
        }                                                                              \
    } while (0)

static const double kMach[] = {0.0, 0.8, 1.2, 3.0};
static const double kCd[] = {0.26, 0.30, 0.55, 0.45};

static BCLIBCFFI_Shot make_shot(int32_t method, double gravity)
{
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = 0.243;
    s.weight_grain = 175.0;
    s.diameter_inch = 0.3;
    s.length_inch = 1.26;
    s.muzzle_velocity_fps = 2600.0;
    s.sight_height_ft = 1.5 / 12.0;
    s.twist_inch = 10.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.humidity = 0.0;
    s.mach_data = kMach;
    s.cd_data = kCd;
    s.drag_table_size = 4;
    s.barrel_elevation_rad = 0.01; // the apex needs a rising shot
    s.latitude_deg = std::nan("");
    s.azimuth_deg = std::nan("");
    s.config.cStepMultiplier = 1.0;
    s.config.cZeroFindingAccuracy = 5e-6;
    s.config.cMinimumVelocity = 50.0;
    s.config.cMaximumDrop = -15000.0;
    s.config.cMaxIterations = 40;
    s.config.cGravityConstant = gravity;
    s.config.cMinimumAltitude = -1500.0;
    s.method = static_cast<BCLIBCFFI_IntegrationMethod>(method);
    return s;
}

static const int32_t kMethods[] = {BCLIBCFFI_INTEGRATION_RK4, BCLIBCFFI_INTEGRATION_EULER,
                                   BCLIBCFFI_INTEGRATION_VELOCITY_VERLET, BCLIBCFFI_INTEGRATION_CASH_KARP,
                                   BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, BCLIBCFFI_INTEGRATION_TSITOURAS};

static bool is_adaptive(int32_t m)
{
    return m == BCLIBCFFI_INTEGRATION_CASH_KARP || m == BCLIBCFFI_INTEGRATION_DORMAND_PRINCE ||
           m == BCLIBCFFI_INTEGRATION_TSITOURAS;
}

// Runs the four entry classes (and the single-point integrate); returns the number of the ones that returned OK.
static int run_all(const BCLIBCFFI_Shot &s, BCLIBCFFI_Error errs[5], int32_t rcs[5])
{
    const bool ad = is_adaptive(s.method);
    BCLIBCFFI_TrajectoryRequest req{3000.0, 300.0, 0.0, BCLIBCFFI_TRAJ_FLAG_RANGE};
    for (int i = 0; i < 5; ++i)
        std::memset(&errs[i], 0, sizeof errs[i]);

    {
        BCLIBCFFI_TrajectoryData *rec = nullptr;
        int32_t n = 0, reason = 0;
        rcs[0] = ad ? BCLIBCFFI_integrate_adaptive_shot(&s, s.method, 1e-8, 1e-8, &req, 0, &rec, &n, nullptr, nullptr,
                                                        &reason, nullptr, &errs[0])
                    : BCLIBCFFI_integrate_shot(&s, &req, &rec, &n, &reason, &errs[0]);
        if (rcs[0] == BCLIBCFFI_OK)
            BCLIBCFFI_free_trajectory(rec);
    }
    {
        double angle = 0.0;
        rcs[1] = ad ? BCLIBCFFI_find_zero_angle_adaptive_shot(&s, s.method, 1e-8, 1e-8, 300.0, &angle, nullptr, &errs[1])
                    : BCLIBCFFI_find_zero_angle_shot(&s, 300.0, &angle, &errs[1]);
    }
    {
        BCLIBCFFI_MaxRangeResult r;
        rcs[2] = ad ? BCLIBCFFI_find_max_range_adaptive_shot(&s, s.method, 1e-8, 1e-8, 0.0, 90.0, &r, nullptr, &errs[2])
                    : BCLIBCFFI_find_max_range_shot(&s, 0.0, 90.0, &r, &errs[2]);
    }
    {
        BCLIBCFFI_TrajectoryData apex;
        rcs[3] = ad ? BCLIBCFFI_find_apex_adaptive_shot(&s, s.method, 1e-8, 1e-8, &apex, nullptr, &errs[3])
                    : BCLIBCFFI_find_apex_shot(&s, &apex, &errs[3]);
    }
    {
        BCLIBCFFI_ZeroPointResult z;
        rcs[4] = ad ? BCLIBCFFI_find_zero_point_adaptive_shot(&s, s.method, 1e-8, 1e-8, 300.0, &z, nullptr, &errs[4])
                    : BCLIBCFFI_find_zero_point_shot(&s, 300.0, &z, &errs[4]);
    }
    int ok = 0;
    for (int i = 0; i < 5; ++i)
        ok += rcs[i] == BCLIBCFFI_OK;
    return ok;
}

static void test_bad_gravity()
{
    const double bad[] = {0.0, -0.0, 9.80665, 32.17405, std::nan(""), INFINITY, -INFINITY};
    for (int32_t method : kMethods)
        for (double g : bad)
        {
            const BCLIBCFFI_Shot s = make_shot(method, g);
            BCLIBCFFI_Error errs[5];
            int32_t rcs[5];
            run_all(s, errs, rcs);
            for (int i = 0; i < 5; ++i)
            {
                if (rcs[i] != BCLIBCFFI_ERR_INVALID_ARGUMENT)
                    std::fprintf(stderr, "  method %d, gravity %g, entry %d: status %d (%s)\n", method, g, i, rcs[i],
                                 errs[i].message);
                CHECK(rcs[i] == BCLIBCFFI_ERR_INVALID_ARGUMENT);
                CHECK(std::strstr(errs[i].message, "Value error (") == errs[i].message);
                CHECK(std::strstr(errs[i].message, "gravity_constant must be negative") != nullptr);
            }
        }
}

static void test_sane_gravity()
{
    // Earth and the Moon: the calls work, and the Moon's lower gravity gives the longer flight.
    const double earth_g = -9.80665 * 3.280839895;
    const double moon_g = -1.62 * 3.280839895;
    for (int32_t method : kMethods)
    {
        double apex_x[2] = {0.0, 0.0};
        int idx = 0;
        for (double g : {earth_g, moon_g})
        {
            const BCLIBCFFI_Shot s = make_shot(method, g);
            BCLIBCFFI_Error errs[5];
            int32_t rcs[5];
            const int ok = run_all(s, errs, rcs);
            if (ok != 5)
                for (int i = 0; i < 5; ++i)
                    if (rcs[i] != BCLIBCFFI_OK)
                        std::fprintf(stderr, "  method %d, gravity %g, entry %d: status %d (%s)\n", method, g, i, rcs[i],
                                     errs[i].message);
            CHECK(ok == 5);
            BCLIBCFFI_TrajectoryData apex;
            BCLIBCFFI_Error err;
            const int32_t rc = is_adaptive(method)
                                   ? BCLIBCFFI_find_apex_adaptive_shot(&s, method, 1e-8, 1e-8, &apex, nullptr, &err)
                                   : BCLIBCFFI_find_apex_shot(&s, &apex, &err);
            CHECK(rc == BCLIBCFFI_OK);
            apex_x[idx++] = apex.distance_ft;
        }
        CHECK(apex_x[1] > 1.5 * apex_x[0]);
    }
}

int main()
{
    test_bad_gravity();
    test_sane_gravity();
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::puts("ok");
    return 0;
}
