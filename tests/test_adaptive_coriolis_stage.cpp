// test_adaptive_coriolis_stage.cpp -- the embedded RK45 engines (Cash-Karp, Dormand-Prince, Tsitouras) evaluate the Coriolis
// acceleration at every stage, with the stage's own velocity: the tolerance controls its error.
//
// They used to evaluate the Coriolis acceleration once per step, at the velocity the step starts with, and to reuse it in
// every stage. Coriolis depends on the velocity, so frozen at the step start it is the same in all stages, cancels in the
// difference of the embedded 4th and 5th order solutions that the error estimate is made of, and the error it makes is
// invisible to the step-size control: tightening the tolerance from 1e-4 to 1e-10 left the Coriolis (windage) error of a
// 4000 m shot at 0.1 ft down to 0.045 ft, instead of making it vanish. A Runge-Kutta stage evaluates the whole
// acceleration at the stage's state (the Python references do: the scipy engine calls coriolis_fn per derivative call, the RK4
// engine per stage; RK4 of this library does since the fix before this one).
//
// The reference is the Python scipy engine (DOP853, rtol = atol = 1e-12, Coriolis per derivative call), an independent,
// converged solution; the values were recorded from it on the shots below (py_ballisticcalc scipy_engine, G7 .31, 168 gr,
// .308 in, 1.2 in, 800 m/s, ICAO atmosphere, no sight height, flat line of sight, lat 45 deg, azimuth 90 deg):
//   30 deg, 4000 m:  height 3821.6791931588 ft, windage 9.5970709458 ft (without Coriolis: height 3812.9444731466 ft)
//   85 deg,  300 m:  height 9894.2844453522 ft, windage 0.4482189391 ft (without Coriolis: height 9859.4201653090 ft)
//   60 deg, 3000 m:  height 9782.3085791674 ft, windage 10.1038649926 ft (without Coriolis: height 9776.9593287404 ft)
// The windage is zero without Coriolis (no wind, no spin drift), so its error is purely the Coriolis one. The height
// effect (the height with Coriolis minus the one without it, at the same tolerance) is compared to the reference effect.
// The C++ core differs from the Python one by about 1e-6 ft already (the unit constants), the floor of the bounds.
//
// The control: without Coriolis (no latitude, or a latitude without azimuth = flat fire) the two OFF routes agree exactly,
// and differ clearly from Coriolis ON (see check_coriolis_off_unchanged).

#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstdio>
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

static BCLIBCFFI_Shot make_shot(double latitude_deg, double azimuth_deg, double elevation_deg)
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
    s.barrel_elevation_rad = elevation_deg * M_PI / 180.0;
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

// The record at the range of the shot, integrated with the adaptive `method` at rtol = atol = tol.
static bool fire(const BCLIBCFFI_Shot &shot, int32_t method, double tol, double range_m, BCLIBCFFI_TrajectoryData *out)
{
    BCLIBCFFI_TrajectoryRequest req;
    std::memset(&req, 0, sizeof req);
    req.range_limit_ft = range_m * kFt;
    req.range_step_ft = range_m * kFt;
    req.time_step = 0.0;
    req.filter_flags = BCLIBCFFI_TRAJ_FLAG_RANGE;
    BCLIBCFFI_TrajectoryData *rec = nullptr;
    int32_t n = 0, dn = 0, reason = 0;
    BCLIBCFFI_Error err;
    const int32_t rc = BCLIBCFFI_integrate_adaptive_shot(&shot, method, tol, tol, &req, 0, &rec, &n, nullptr, &dn, &reason, nullptr, &err);
    if (rc != BCLIBCFFI_OK || n < 1)
    {
        std::fprintf(stderr, "  integrate_adaptive_shot failed: rc %d (%s)\n", (int)rc, rc != BCLIBCFFI_OK ? err.message : "no record");
        if (rec)
            BCLIBCFFI_free_trajectory(rec);
        return false;
    }
    *out = rec[n - 1];
    BCLIBCFFI_free_trajectory(rec);
    return true;
}

static const int32_t kMethods[3] = {BCLIBCFFI_INTEGRATION_CASH_KARP, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, BCLIBCFFI_INTEGRATION_TSITOURAS};
static const char *kMethodNames[3] = {"Cash-Karp", "Dormand-Prince", "Tsitouras"};
static const double kTols[4] = {1e-4, 1e-6, 1e-8, 1e-10};

// The Coriolis (windage) error bound at each tolerance, over the three shots and methods (ft). With the Coriolis acceleration
// at every stage the measured worst is 1.6e-4, 1.3e-4, 6e-6 and 8e-7 ft (the last is the floor of the unit constants); the
// once-per-step Coriolis had 0.11, 0.11, 0.08 and 0.045 ft, nearly independent of the tolerance.
static const double kWindageBound[4] = {1e-3, 1e-3, 1e-4, 1e-5};
// The height effect of Coriolis at the tightest tolerance (ft): measured worst 2.5e-4 ft, the once-per-step one 0.24 ft.
static const double kHeightEffectBound = 1e-3;

struct Shot
{
    const char *name;
    double elevation_deg, range_m;
    double ref_height, ref_windage, ref_height_off;
};

static const Shot kShots[3] = {
    {"30 deg 4000 m", 30.0, 4000.0, 3821.6791931588, 9.5970709458, 3812.9444731466},
    {"85 deg 300 m", 85.0, 300.0, 9894.2844453522, 0.4482189391, 9859.4201653090},
    {"60 deg 3000 m", 60.0, 3000.0, 9782.3085791674, 10.1038649926, 9776.9593287404},
};

static void check_error_vs_tolerance()
{
    for (const Shot &sh : kShots)
    {
        for (int m = 0; m < 3; ++m)
        {
            double werr[4] = {0, 0, 0, 0};
            double heff[4] = {0, 0, 0, 0};
            for (int t = 0; t < 4; ++t)
            {
                BCLIBCFFI_TrajectoryData on, off;
                CHECK(fire(make_shot(45.0, 90.0, sh.elevation_deg), kMethods[m], kTols[t], sh.range_m, &on));
                CHECK(fire(make_shot(std::nan(""), std::nan(""), sh.elevation_deg), kMethods[m], kTols[t], sh.range_m, &off));
                CHECK(std::fabs(on.distance_ft - sh.range_m * kFt) < 1.0);
                werr[t] = on.windage_ft - sh.ref_windage;
                heff[t] = (on.height_ft - off.height_ft) - (sh.ref_height - sh.ref_height_off);
                CHECK(std::fabs(werr[t]) < kWindageBound[t]);
            }
            CHECK(std::fabs(heff[3]) < kHeightEffectBound);
            // tolerance controls the error: the windage error at 1e-10 is clearly below the one at 1e-4
            CHECK(std::fabs(werr[3]) < 0.5 * std::fabs(werr[0]) || std::fabs(werr[3]) < 1e-6);
            std::fprintf(stderr, "  %-14s %-14s windage err (ft) 1e-4 %+.2e  1e-6 %+.2e  1e-8 %+.2e  1e-10 %+.2e | height effect err 1e-10 %+.2e\n",
                         sh.name, kMethodNames[m], werr[0], werr[1], werr[2], werr[3], heff[3]);
        }
    }
}

// Coriolis off, platform independent. A table recorded on one platform is not valid on another: the adaptive step controller
// amplifies libm / FMA differences beyond any tolerance that would still catch a leak. So both sides are computed here, in the
// same process on the same platform, and must agree bit for bit (1e-12 ft allowed):
//   A: no latitude          -> Coriolis structurally off (flat_fire_only set, all terms zero)
//   B: latitude, no azimuth -> flat fire: the shot API sets flat_fire_only itself when the azimuth is absent
//      (BCLIBC_Coriolis::from_lat_az), so the Coriolis acceleration is disabled by the flag while the latitude is used.
// (The Shot descriptor has no flat_fire_only field of its own; an azimuth together with the flag is not expressible here.)
// Any Coriolis leak into the OFF path changes B or A by far more than 1e-12 ft. The flat-fire windage offset (see below)
// is why the windage is compared at latitude 0 only.
static void check_coriolis_off_unchanged()
{
    for (int m = 0; m < 3; ++m)
    {
        for (int t = 0; t < 4; ++t)
        {
            BCLIBCFFI_TrajectoryData a, b45, b0;
            CHECK(fire(make_shot(std::nan(""), std::nan(""), 30.0), kMethods[m], kTols[t], 4000.0, &a));
            CHECK(fire(make_shot(45.0, std::nan(""), 30.0), kMethods[m], kTols[t], 4000.0, &b45));
            CHECK(fire(make_shot(0.0, std::nan(""), 30.0), kMethods[m], kTols[t], 4000.0, &b0));
            // Height is purely the integrated one (flat fire adds no vertical offset without an azimuth).
            CHECK(std::fabs(a.height_ft - b45.height_ft) < 1e-12);
            CHECK(std::fabs(a.height_ft - b0.height_ft) < 1e-12);
            // Flat fire adds a post-hoc horizontal drift omega * x * sin(lat) * t to the windage, not part of the integration;
            // it vanishes at latitude 0 (sin 0 == 0), where the windage must match exactly too.
            CHECK(std::fabs(a.windage_ft - b0.windage_ft) < 1e-12);
        }
    }
}

// Not vacuous: Coriolis ON (lat 45, azimuth 90) differs from OFF by far more than the bound above (measured ~9 ft in
// height-effect / windage on this shot).
static void check_coriolis_on_differs()
{
    for (int m = 0; m < 3; ++m)
    {
        for (int t = 0; t < 4; ++t)
        {
            BCLIBCFFI_TrajectoryData on, off;
            CHECK(fire(make_shot(45.0, 90.0, 30.0), kMethods[m], kTols[t], 4000.0, &on));
            CHECK(fire(make_shot(std::nan(""), std::nan(""), 30.0), kMethods[m], kTols[t], 4000.0, &off));
            CHECK(std::fabs(on.height_ft - off.height_ft) > 1e-2);
            CHECK(std::fabs(on.windage_ft - off.windage_ft) > 1e-2);
        }
    }
}

int main()
{
    check_error_vs_tolerance();
    check_coriolis_off_unchanged();
    check_coriolis_on_differs();
    return failures ? 1 : 0;
}
