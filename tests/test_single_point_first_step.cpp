// test_single_point_first_step.cpp -- a key crossing inside the first integration step.
//
// BCLIBC_SinglePointHandler (the handler of find_apex and integrate_at) keeps a window of three points and checked for
// the key crossing only between its 2nd and 3rd points, so the segment between the start point and the end of the first
// step was never examined:
//   * the apex of a shot with a tiny elevation (relative angle 0.001 deg: the apex is at about 1.4 ms, the first step of
//     RK4 ends at 2.5 ms) was reported as "No apex flagged";
//   * integrate_at(time, t) failed with an interception error for every t before the end of the first step, t = 0
//     included.
// The Python engines return both. The check is done through the C ABI entry points of the fixed-step methods (RK4, Euler,
// Velocity Verlet) and of an adaptive one (Tsitouras 5(4)).
//
// The truth is a trajectory of the same shot and method integrated with a calc_step 1000 times smaller, where the crossing
// is far from the first step; the answer for the default step is interpolated and is expected to agree with it to
// interpolation error.
//
// The second part is a regression guard: the points of crossings that are not in the first step (apex of a normal shot,
// time, Mach, velocity and height keys) are bit-for-bit what the code before the fix returned (zero_first_step_guard.inc).

#include "bclibc.hpp"
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

static const double kFt = 3.28084;

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

// 800 m/s, G7 0.31 (175 gr, .308, 1.24 in), 5 cm sight, the barrel elevated by elevation_deg above a level look angle.
static BCLIBCFFI_Shot make_shot(int32_t method, double elevation_deg, double step_multiplier = 1.0)
{
    for (int i = 0; i < kG7Size; ++i)
    {
        g_mach[i] = kG7[i].mach;
        g_cd[i] = kG7[i].cd;
    }
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = 0.31;
    s.weight_grain = 175.0;
    s.diameter_inch = 0.308;
    s.length_inch = 1.24;
    s.muzzle_velocity_fps = 800.0 * kFt;
    s.sight_height_ft = 0.05 * kFt;
    s.twist_inch = 0.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.humidity = 0.0;
    s.mach_data = g_mach;
    s.cd_data = g_cd;
    s.drag_table_size = kG7Size;
    s.barrel_elevation_rad = elevation_deg * M_PI / 180.0;
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

struct Engine
{
    const char *name;
    int32_t method;     // the shot's method (fixed-step), or the adaptive one
    bool adaptive;
};
static const Engine kEngines[] = {
    {"rk4", BCLIBCFFI_INTEGRATION_RK4, false},
    {"euler", BCLIBCFFI_INTEGRATION_EULER, false},
    {"verlet", BCLIBCFFI_INTEGRATION_VELOCITY_VERLET, false},
    {"tsitouras", BCLIBCFFI_INTEGRATION_TSITOURAS, true},
};
static const double kRelTol = 1e-9, kAbsTol = 1e-9;

static int32_t apex(const Engine &e, const BCLIBCFFI_Shot &shot, BCLIBCFFI_TrajectoryData *out, BCLIBCFFI_Error *err)
{
    return e.adaptive ? BCLIBCFFI_find_apex_adaptive_shot(&shot, e.method, kRelTol, kAbsTol, out, nullptr, err)
                      : BCLIBCFFI_find_apex_shot(&shot, out, err);
}

static int32_t at(const Engine &e, const BCLIBCFFI_Shot &shot, int32_t key, double value, BCLIBCFFI_Interception *out,
                  BCLIBCFFI_Error *err)
{
    return e.adaptive ? BCLIBCFFI_integrate_at_adaptive_shot(&shot, e.method, kRelTol, kAbsTol, key, value, out, nullptr, err)
                      : BCLIBCFFI_integrate_at_shot(&shot, key, value, out, err);
}

// The shot of an engine; the fixed-step methods read it in shot.method, the adaptive entry points take it as an argument.
static BCLIBCFFI_Shot shot_of(const Engine &e, double elevation_deg, double step_multiplier = 1.0)
{
    return make_shot(e.adaptive ? BCLIBCFFI_INTEGRATION_RK4 : e.method, elevation_deg, step_multiplier);
}

// The truth: the same method with a 1000 times smaller step (fixed-step) or the default adaptive controller (whose steps
// are not tied to calc_step, so the reference is the same call at a stricter tolerance).
static int32_t ref_at(const Engine &e, int32_t key, double value, double elevation_deg, BCLIBCFFI_Interception *out,
                      BCLIBCFFI_Error *err)
{
    if (e.adaptive)
    {
        const BCLIBCFFI_Shot shot = shot_of(e, elevation_deg);
        return BCLIBCFFI_integrate_at_adaptive_shot(&shot, e.method, 1e-12, 1e-12, key, value, out, nullptr, err);
    }
    const BCLIBCFFI_Shot shot = shot_of(e, elevation_deg, 0.001);
    return BCLIBCFFI_integrate_at_shot(&shot, key, value, out, err);
}

static void test_apex_in_first_step()
{
    const double elevations[] = {0.001, 0.003};
    for (const Engine &e : kEngines)
        for (double el : elevations)
        {
            const BCLIBCFFI_Shot shot = shot_of(e, el);
            BCLIBCFFI_TrajectoryData a;
            BCLIBCFFI_Error err;
            std::memset(&a, 0, sizeof a);
            const int32_t rc = apex(e, shot, &a, &err);
            if (rc != BCLIBCFFI_OK)
                std::fprintf(stderr, "  %s, elevation %.3f deg: find_apex failed: %s\n", e.name, el, err.message);
            CHECK(rc == BCLIBCFFI_OK);
            if (rc != BCLIBCFFI_OK)
                continue;

            // The apex of a 0.001 deg shot: vy0 = 0.0140 m/s, g = 9.81 -> 1.4 ms, 10 nm above the muzzle: the point is
            // known analytically to within drag, which acts on vy at the 1e-4 level here.
            BCLIBCFFI_Interception ref;
            const int32_t rrc = ref_at(e, BCLIBCFFI_INTERP_KEY_VEL_Y, 0.0, el, &ref, &err);
            CHECK(rrc == BCLIBCFFI_OK);
            if (rrc != BCLIBCFFI_OK)
                continue;
            const double dt = std::fabs(a.time - ref.raw_data.time);
            const double dh = std::fabs(a.height_ft - ref.full_data.height_ft);
            const double dx = std::fabs(a.distance_ft - ref.full_data.distance_ft);
            std::fprintf(stderr, "  %-9s el %.3f: apex t %.9f (ref %.9f), height %.9f ft (ref %.9f)\n", e.name, el, a.time,
                         ref.raw_data.time, a.height_ft, ref.full_data.height_ft);
            CHECK(a.time > 1e-4 && a.time < 5e-3);
            CHECK(dt < 5e-5);     // 3.5% of the apex time; the interpolation error of the window is far below
            CHECK(dh < 5e-5);     // ft (15 um): the 3-point interpolation error; the same size for an apex in the 2nd step (el 0.003)
            CHECK(dx < 2e-2);     // ft; the bullet travels 2600 ft/s, so dt = 5e-5 would be 0.13 ft
        }
}

static void test_time_in_first_step()
{
    for (const Engine &e : kEngines)
        for (double el : {0.0, 0.001, 1.0})
            for (double t : {0.0, 0.0005, 0.001, 0.002})
            {
                const BCLIBCFFI_Shot shot = shot_of(e, el);
                BCLIBCFFI_Interception r;
                BCLIBCFFI_Error err;
                std::memset(&r, 0, sizeof r);
                const int32_t rc = at(e, shot, BCLIBCFFI_INTERP_KEY_TIME, t, &r, &err);
                if (rc != BCLIBCFFI_OK)
                    std::fprintf(stderr, "  %s, elevation %.3f, t=%g: integrate_at failed (rc %d): %s\n", e.name, el, t, (int)rc,
                                 err.message);
                CHECK(rc == BCLIBCFFI_OK);
                if (rc != BCLIBCFFI_OK)
                    continue;
                BCLIBCFFI_Interception ref;
                const int32_t rrc = ref_at(e, BCLIBCFFI_INTERP_KEY_TIME, t, el, &ref, &err);
                CHECK(rrc == BCLIBCFFI_OK);
                if (rrc != BCLIBCFFI_OK)
                    continue;
                const BCLIBCFFI_BaseTrajData &a = r.raw_data, &b = ref.raw_data;
                CHECK(std::fabs(a.time - t) < 1e-12);
                if (t == 0.0)
                {
                    // The start state itself: position at the muzzle (below the sight line by the sight height), the
                    // muzzle velocity along the barrel.
                    CHECK(std::fabs(a.px) < 1e-12);
                    CHECK(std::fabs(a.py + 0.05 * kFt) < 1e-9 || std::fabs(a.py) < 1e-9);
                    CHECK(std::fabs(a.vx - b.vx) < 1e-9);
                    CHECK(std::fabs(a.vy - b.vy) < 1e-9);
                }
                const double tol_pos = 5e-3, tol_vel = 5e-3;   // ft, fps: interpolation error of the 2.5 ms step
                const bool ok = std::fabs(a.px - b.px) < tol_pos && std::fabs(a.py - b.py) < tol_pos &&
                                std::fabs(a.vx - b.vx) < tol_vel && std::fabs(a.vy - b.vy) < tol_vel;
                if (!ok)
                    std::fprintf(stderr, "  %s el %.3f t=%g: px %.9f/%.9f py %.9f/%.9f vx %.6f/%.6f vy %.6f/%.6f\n", e.name, el,
                                 t, a.px, b.px, a.py, b.py, a.vx, b.vx, a.vy, b.vy);
                CHECK(ok);
            }
}

// ---- Regression guard: crossings outside the first step are bit-for-bit those of the code before the fix.
struct Guard
{
    int engine;
    double elevation_deg;
    int key;
    double target;
    double time, px, py, pz, vx, vy, vz, mach;
};
static const Guard kGuard[] = {
#include "single_point_first_step_guard.inc"
};

struct GuardCase
{
    int key;
    double target;
};
// Keys and targets of crossings well after the first step: time, Mach, velocity components, height, and the apex.
static const GuardCase kGuardCases[] = {
    {BCLIBCFFI_INTERP_KEY_TIME, 0.5},     {BCLIBCFFI_INTERP_KEY_TIME, 1.5},     {BCLIBCFFI_INTERP_KEY_TIME, 0.0075},
    {BCLIBCFFI_INTERP_KEY_MACH, 1.8},     {BCLIBCFFI_INTERP_KEY_MACH, 1.1},     {BCLIBCFFI_INTERP_KEY_VEL_Y, 0.0},
    {BCLIBCFFI_INTERP_KEY_VEL_X, 1500.0}, {BCLIBCFFI_INTERP_KEY_POS_Y, 5.0},    {BCLIBCFFI_INTERP_KEY_POS_Y, -50.0},
};
static const double kGuardElevations[] = {1.0, 3.0};

// Fixed-step results are reproduced to rounding. The adaptive step-size controller amplifies the last-bit differences of
// other platforms (FMA contraction, libm), so its recorded values are compared to 1e-6 relative (the first-step fix moves a
// later crossing by far more than that, if it moves it at all).
static bool near(double a, double b, double rel)
{
    return std::fabs(a - b) <= rel * std::fmax(1.0, std::fabs(b));
}

static void test_later_crossings_unchanged()
{
    for (const Guard &g : kGuard)
    {
        const Engine &e = kEngines[g.engine];
        const BCLIBCFFI_Shot shot = shot_of(e, g.elevation_deg);
        BCLIBCFFI_Interception r;
        BCLIBCFFI_Error err;
        std::memset(&r, 0, sizeof r);
        const int32_t rc = at(e, shot, g.key, g.target, &r, &err);
        CHECK(rc == BCLIBCFFI_OK);
        if (rc != BCLIBCFFI_OK)
            continue;
        const BCLIBCFFI_BaseTrajData &a = r.raw_data;
        const double rel = e.adaptive ? 1e-6 : 1e-12;
        const bool same = near(a.time, g.time, rel) && near(a.px, g.px, rel) && near(a.py, g.py, rel) &&
                          near(a.pz, g.pz, rel) && near(a.vx, g.vx, rel) && near(a.vy, g.vy, rel) &&
                          near(a.vz, g.vz, rel) && near(a.mach, g.mach, rel);
        if (!same)
            std::fprintf(stderr, "  guard: %s el %.1f key %d target %g: t %.17g (rec %.17g) px %.17g (rec %.17g)\n", e.name,
                         g.elevation_deg, g.key, g.target, a.time, g.time, a.px, g.px);
        CHECK(same);
    }
}

// `test_single_point_first_step --print-guard` writes the lines of single_point_first_step_guard.inc.
static void print_guard()
{
    for (int ei = 0; ei < (int)(sizeof kEngines / sizeof kEngines[0]); ++ei)
        for (double el : kGuardElevations)
            for (const GuardCase &c : kGuardCases)
            {
                const Engine &e = kEngines[ei];
                const BCLIBCFFI_Shot shot = shot_of(e, el);
                BCLIBCFFI_Interception r;
                BCLIBCFFI_Error err;
                std::memset(&r, 0, sizeof r);
                if (at(e, shot, c.key, c.target, &r, &err) != BCLIBCFFI_OK)
                    continue;
                const BCLIBCFFI_BaseTrajData &a = r.raw_data;
                std::printf("    {%d, %.1f, %d, %.17g, %.17g, %.17g, %.17g, %.17g, %.17g, %.17g, %.17g, %.17g},\n", ei, el,
                            c.key, c.target, a.time, a.px, a.py, a.pz, a.vx, a.vy, a.vz, a.mach);
            }
}

int main(int argc, char **argv)
{
    if (argc > 1 && std::strcmp(argv[1], "--print-guard") == 0)
    {
        print_guard();
        return EXIT_SUCCESS;
    }
    test_apex_in_first_step();
    test_time_in_first_step();
    test_later_crossings_unchanged();
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_single_point_first_step: all checks passed");
    return EXIT_SUCCESS;
}
