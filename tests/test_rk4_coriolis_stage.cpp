// test_rk4_coriolis_stage.cpp -- RK4 evaluates the Coriolis acceleration in each of its four stages.
//
// The RK4 engine used to evaluate the Coriolis acceleration once per step, at the velocity the step starts with, and to
// reuse it in all four stages. A Runge-Kutta stage must evaluate the whole acceleration at the stage's own state; the
// Python reference (py_ballisticcalc/engines/rk4.py) calls coriolis_fn(ground_vel) in every stage, with that stage's
// velocity. On a long, high shot the once-per-step value was off by 1.5e-3 ft in height and 1.8e-3 ft in windage at
// 4000 m (more than the 4e-6 ft the rest of the engine differs by with Coriolis off).
//
// The shot: G7 .31, 168 gr, .308 in, 1.2 in, 800 m/s, 30 deg elevation, ICAO atmosphere, flat line of sight, no sight
// height, calc_step multiplier 1, range 4000 m. The expected values were recorded from the Python RK4 engine
// (rk4_engine) on that shot. With Coriolis off (no latitude) the result is the one the engine always gave.

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

static const double kFt = 3.28084;  // feet per metre, as the constants of the C++ core use

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

static BCLIBCFFI_Shot make_shot(double latitude_deg, double azimuth_deg)
{
    for (int i = 0; i < kG7Size; ++i)
    {
        g_mach[i] = kG7[i].mach;
        g_cd[i] = kG7[i].cd;
    }
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = 0.31;
    s.weight_grain = 168.0;
    s.diameter_inch = 0.308;
    s.length_inch = 1.2;
    s.muzzle_velocity_fps = 800.0 * kFt;
    s.sight_height_ft = 0.0;
    s.twist_inch = 0.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.humidity = 0.0;
    s.mach_data = g_mach;
    s.cd_data = g_cd;
    s.drag_table_size = kG7Size;
    s.look_angle_rad = 0.0;
    s.barrel_elevation_rad = 30.0 * M_PI / 180.0;
    s.cant_angle_rad = 0.0;
    s.latitude_deg = latitude_deg;
    s.azimuth_deg = azimuth_deg;
    s.config.cStepMultiplier = 1.0;
    s.config.cZeroFindingAccuracy = 5e-6;
    s.config.cMinimumVelocity = 50.0;
    s.config.cMaximumDrop = -15000.0;
    s.config.cMaxIterations = 40;
    s.config.cGravityConstant = -32.17405;
    s.config.cMinimumAltitude = -1500.0;
    s.method = BCLIBCFFI_INTEGRATION_RK4;
    return s;
}

// The record at 4000 m of the shot.
static bool fire(const BCLIBCFFI_Shot &shot, BCLIBCFFI_TrajectoryData *out)
{
    BCLIBCFFI_TrajectoryRequest req;
    std::memset(&req, 0, sizeof req);
    req.range_limit_ft = 4000.0 * kFt;
    req.range_step_ft = 4000.0 * kFt;
    req.time_step = 0.0;
    req.filter_flags = BCLIBCFFI_TRAJ_FLAG_RANGE;
    BCLIBCFFI_TrajectoryData *rec = nullptr;
    int32_t n = 0, reason = 0;
    BCLIBCFFI_Error err;
    const int32_t rc = BCLIBCFFI_integrate_shot(&shot, &req, &rec, &n, &reason, &err);
    if (rc != BCLIBCFFI_OK || n < 1)
    {
        std::fprintf(stderr, "  integrate_shot failed: rc %d (%s)\n", (int)rc, rc != BCLIBCFFI_OK ? err.message : "no record");
        if (rec)
            BCLIBCFFI_free_trajectory(rec);
        return false;
    }
    *out = rec[n - 1];
    BCLIBCFFI_free_trajectory(rec);
    return true;
}

// Python rk4_engine records on the shot, as (height ft, windage ft, time s).
static const double kPyNone[3] = {3812.6376660554083, 0.0, 18.027424003001826};      // no latitude
static const double kPyLat45Az90[3] = {3821.3722298252937, 9.5972698055716, 18.034165971032586};
static const double kPyLatM30Az0[3] = {3812.627652956408, -11.17549475175098, 18.02743560588093};

// The C++ core differs from the Python engine by 1.3e-5 ft in height at 4000 m with Coriolis off already (the unit
// constants and the interpolation of the record), so the Coriolis effect, the difference to the shot without it, is held
// to the Python one to 1e-6 ft, the windage (zero without Coriolis) is held to it absolutely, and the height absolutely to
// 5e-5 ft. The once-per-step Coriolis missed the Python effect by 1.5e-3 ft in height and 1.8e-3 ft in windage.
static void expect(const char *what, double lat, double az, const double py[3], const BCLIBCFFI_TrajectoryData &base)
{
    BCLIBCFFI_TrajectoryData p;
    CHECK(fire(make_shot(lat, az), &p));
    const double dh = (p.height_ft - base.height_ft) - (py[0] - kPyNone[0]);
    const double dw = p.windage_ft - py[1];
    const double dt = (p.time - base.time) - (py[2] - kPyNone[2]);
    std::fprintf(stderr, "  %s: height %.10f (vs Python %.3e, Coriolis effect %.3e) windage %.10f (vs Python %.3e) time %.10f (effect %.3e)\n",
                 what, p.height_ft, p.height_ft - py[0], dh, p.windage_ft, dw, p.time, dt);
    CHECK(std::fabs(p.distance_ft - 4000.0 * kFt) < 1.0);
    CHECK(std::fabs(dh) < 1e-6);
    CHECK(std::fabs(dw) < 1e-6);
    CHECK(std::fabs(dt) < 1e-6);
    CHECK(std::fabs(p.height_ft - py[0]) < 5e-5);
}

int main()
{
    // Coriolis off (no latitude): the result the engine gave before the fix, recorded from it.
    BCLIBCFFI_TrajectoryData base;
    CHECK(fire(make_shot(std::nan(""), std::nan("")), &base));
    std::fprintf(stderr, "  no Coriolis: height %.13f windage %.13f time %.13f\n", base.height_ft, base.windage_ft, base.time);
    CHECK(std::fabs(base.height_ft - 3812.6376527306379) < 1e-9);
    CHECK(base.windage_ft == 0.0);
    CHECK(std::fabs(base.time - 18.0274246731772) < 1e-9);
    CHECK(std::fabs(base.height_ft - kPyNone[0]) < 5e-5);

    expect("lat 45 az 90", 45.0, 90.0, kPyLat45Az90, base);
    expect("lat -30 az 0", -30.0, 0.0, kPyLatM30Az0, base);
    return failures ? 1 : 0;
}
