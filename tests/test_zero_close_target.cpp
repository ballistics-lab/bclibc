// test_zero_close_target.cpp -- the zero angle of a target closer than about twice the sight height.
//
// init_zero_calculation() short-cuts a very close target ("ignore gravity and drag") and returns a geometric angle
// without a search. The shortcut had two defects:
//   1. its cutoff, 2 * max(|sight height|, cStepMultiplier), compared a length with a *multiplier* (1.0 by default,
//      so about 2 ft = 0.6 m whatever the sight height); the Python engine uses the shot's calc_step, which is what
//      this test pins down: a target further than twice the sight height is searched for, not guessed;
//   2. its angle, atan2(target_y + start_height, target_x), had the wrong sign of the sight height (start_height is the
//      *negative* sight height), so for a sight above the bore it pointed the bore *down*, away from the line of sight,
//      and the shot missed it by twice the sight height. The right angle is atan2(target_y - start_height, target_x).
//
// A zero angle is correct when the shot fired with it crosses the line of sight at the target. The test fires it and
// measures the miss with error_at_distance (the same measure the solvers minimise), for a grid of close targets, sight
// heights, velocities, drag tables and integrators; through the C ABI (every find_zero_* entry point, the lofted and
// adaptive variants, zero_init) and through the C++ core.
//
// The second part is a regression guard: the zero angle of a target the shortcut does not touch (further than the
// cutoff in both the old and the new code) did not change. The numbers were recorded with the code before the fix.

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

static void report(const char *what, int32_t method, const Table &t, double sight_m, double v, double look_deg,
                   double target_m, double angle_rad, double miss_ft)
{
    std::fprintf(stderr,
                 "  %s: method %d, %s, sight %.2f m, v %.0f m/s, look %.0f deg, target %.2f m -> %.3f deg, miss %.5f ft\n",
                 what, method, t.name, sight_m, v, look_deg, target_m, angle_rad * 180.0 / M_PI, miss_ft);
}

#define CHECK_ZERO(what, rc, angle, shot, d_ft)                                                         \
    do                                                                                                  \
    {                                                                                                   \
        CHECK((rc) == BCLIBCFFI_OK);                                                                    \
        const double miss_ = abi_miss((shot), (angle), (d_ft));                                         \
        const bool ok_ = (rc) == BCLIBCFFI_OK && std::fabs(miss_) < kMissTolFt;                         \
        if (!ok_)                                                                                       \
        {                                                                                               \
            report(what, method, t, sight, v, look, target, (angle), miss_);                            \
            if ((rc) != BCLIBCFFI_OK)                                                                   \
                std::fprintf(stderr, "    rc %d: %s\n", (int)(rc), err.message);                        \
        }                                                                                               \
        CHECK(std::fabs(miss_) < kMissTolFt);                                                           \
    } while (0)

// The higher (lofted) arc of a close target: a flat shot has none (the search reports it, as it does for any far
// target), but when it answers, the answer must hit. The same is accepted for the Ridder entry points on a downhill
// look angle, where the downhill maximum-range bracket used to report "out of range" for any target (see
// test_zero_downhill.cpp, which requires them to hit now); a wrong angle is still a failure here.
#define CHECK_LOFTED_ZERO(what, rc, angle, shot, d_ft)                                                  \
    do                                                                                                  \
    {                                                                                                   \
        if ((rc) == BCLIBCFFI_OK)                                                                       \
            CHECK_ZERO(what, rc, angle, shot, d_ft);                                                    \
        else                                                                                            \
            CHECK((rc) == BCLIBCFFI_ERR_ZERO_FINDING || (rc) == BCLIBCFFI_ERR_OUT_OF_RANGE);            \
    } while (0)

static void test_abi_close_targets()
{
    for (int32_t method : kMethods)
        for (const Table &t : kTables)
            for (double sight : kSightsM)
                for (double v : kVelocitiesMps)
                    for (double look : kLooksDeg)
                        for (double target : kTargetsM)
                        {
                            const BCLIBCFFI_Shot shot = make_shot(method, t, sight, v, look);
                            const double d = target * kFt;
                            BCLIBCFFI_Error err;
                            double angle = 0.0;
                            int32_t rc;

                            rc = BCLIBCFFI_find_zero_angle_shot(&shot, d, &angle, &err);
                            CHECK_ZERO("find_zero_angle_shot", rc, angle, shot, d);

                            BCLIBCFFI_ZeroPointResult zp;
                            std::memset(&zp, 0, sizeof zp);
                            rc = BCLIBCFFI_find_zero_point_shot(&shot, d, &zp, &err);
                            CHECK_ZERO("find_zero_point_shot", rc, zp.angle_rad, shot, d);

                            for (int32_t lofted : {0, 1})
                            {
                                // The lofted entry points search the whole maximum-range bracket (50 ms a call): a slice
                                // of the grid is enough, the shortcut they share with the others is the same.
                                if (method != BCLIBCFFI_INTEGRATION_RK4 || &t != &kTables[0] || v != 800.0 || look != 0.0)
                                    continue;
                                // lofted == 0 is the lower arc, the one a close target has; see CHECK_LOFTED_ZERO.
                                rc = BCLIBCFFI_find_zero_angle_lofted_shot(&shot, d, lofted, &angle, &err);
                                if (lofted || look < 0.0)
                                    CHECK_LOFTED_ZERO("find_zero_angle_lofted_shot", rc, angle, shot, d);
                                else
                                    CHECK_ZERO("find_zero_angle_lofted_shot", rc, angle, shot, d);
                                rc = BCLIBCFFI_find_zero_point_lofted_shot(&shot, d, lofted, &zp, &err);
                                if (lofted || look < 0.0)
                                    CHECK_LOFTED_ZERO("find_zero_point_lofted_shot", rc, zp.angle_rad, shot, d);
                                else
                                    CHECK_ZERO("find_zero_point_lofted_shot", rc, zp.angle_rad, shot, d);
                            }

                            BCLIBCFFI_ZeroInit zi;
                            std::memset(&zi, 0, sizeof zi);
                            rc = BCLIBCFFI_zero_init_shot(&shot, d, &zi, &err);
                            CHECK(rc == BCLIBCFFI_OK);
                            if (zi.status == BCLIBCFFI_ZERO_INIT_DONE)
                                CHECK_ZERO("zero_init_shot", rc, zi.look_angle_rad, shot, d);

                            // The adaptive entry points take a method of their own (the grid is cut: they are slow).
                            if (look != 0.0 || method != BCLIBCFFI_INTEGRATION_RK4)
                                continue;
                            BCLIBCFFI_StepStats stats;
                            std::memset(&stats, 0, sizeof stats);
                            rc = BCLIBCFFI_find_zero_angle_adaptive_shot(&shot, BCLIBCFFI_INTEGRATION_TSITOURAS, 1e-8,
                                                                         1e-8, d, &angle, &stats, &err);
                            CHECK_ZERO("find_zero_angle_adaptive_shot", rc, angle, shot, d);
                            rc = BCLIBCFFI_find_zero_point_adaptive_shot(&shot, BCLIBCFFI_INTEGRATION_CASH_KARP, 1e-8,
                                                                         1e-8, d, &zp, &stats, &err);
                            CHECK_ZERO("find_zero_point_adaptive_shot", rc, zp.angle_rad, shot, d);
                            rc = BCLIBCFFI_find_zero_angle_lofted_adaptive_shot(
                                &shot, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, 1e-8, 1e-8, d, 1, &angle, &stats, &err);
                            CHECK_LOFTED_ZERO("find_zero_angle_lofted_adaptive_shot", rc, angle, shot, d);
                            rc = BCLIBCFFI_find_zero_point_lofted_adaptive_shot(
                                &shot, BCLIBCFFI_INTEGRATION_TSITOURAS, 1e-8, 1e-8, d, 0, &zp, &stats, &err);
                            if (look < 0.0)
                                CHECK_LOFTED_ZERO("find_zero_point_lofted_adaptive_shot", rc, zp.angle_rad, shot, d);
                            else
                                CHECK_ZERO("find_zero_point_lofted_adaptive_shot", rc, zp.angle_rad, shot, d);
                        }
}

// The cutoff is twice the sight height (calc_step, 0.0025, is far below it; see test_abi_calc_step_branch): just inside it the geometric angle must
// hit, just outside it the search must. Both ends of the cutoff take the same hit, and the angles are continuous.
static void test_abi_cutoff_is_twice_the_sight_height()
{
    BCLIBCFFI_Error err;
    for (double sight : {0.05, 0.1, 0.5})
    {
        const Table &t = kTables[0];
        const double v = 800.0, look = 0.0;
        const BCLIBCFFI_Shot shot = make_shot(BCLIBCFFI_INTEGRATION_RK4, t, sight, v, look);
        const double cutoff_ft = 2.0 * shot.sight_height_ft;
        BCLIBCFFI_ZeroInit zi;
        std::memset(&zi, 0, sizeof zi);

        // Well inside: the shortcut (status DONE). Well outside, but closer than the old 2 * cStepMultiplier = 2 ft
        // cutoff whenever the sight is low: a search (status CONTINUE).
        CHECK(BCLIBCFFI_zero_init_shot(&shot, 0.9 * cutoff_ft, &zi, &err) == BCLIBCFFI_OK);
        CHECK(zi.status == BCLIBCFFI_ZERO_INIT_DONE);
        CHECK(BCLIBCFFI_zero_init_shot(&shot, 1.1 * cutoff_ft, &zi, &err) == BCLIBCFFI_OK);
        if (1.1 * cutoff_ft > 0.01)
            CHECK(zi.status == BCLIBCFFI_ZERO_INIT_CONTINUE);

        // The shortcut, with no gravity and no drag, is the straight line from the bore to the line of sight.
        CHECK(BCLIBCFFI_zero_init_shot(&shot, 0.5 * cutoff_ft, &zi, &err) == BCLIBCFFI_OK);
        CHECK(zi.status == BCLIBCFFI_ZERO_INIT_DONE);
        CHECK(std::fabs(zi.look_angle_rad - std::atan2(shot.sight_height_ft, 0.5 * cutoff_ft)) < 1e-12);
        CHECK(zi.look_angle_rad > 0.0); // the bore points up to a sight above it
    }
}

// A canted rifle: start_height = -sight * cos(cant), so the angle (and the cutoff) depend on the cant. The shot fired
// with the zero must still reach the line of sight.
static void test_abi_cant()
{
    BCLIBCFFI_Error err;
    const int32_t method = BCLIBCFFI_INTEGRATION_RK4;
    const Table &t = kTables[0];
    const double v = 800.0;
    for (double cant : {30.0, 60.0})
        for (double sight : {0.1, 0.5})
            for (double look : {0.0, 10.0})
                for (double target : {0.05, 0.1, 0.2, 0.4})
                {
                    const BCLIBCFFI_Shot shot = make_shot(method, t, sight, v, look, cant);
                    const double d = target * kFt;
                    double angle = 0.0;
                    int32_t rc = BCLIBCFFI_find_zero_angle_shot(&shot, d, &angle, &err);
                    BCLIBCFFI_ZeroPointResult zp;
                    std::memset(&zp, 0, sizeof zp);
                    // In the exact canted frame (class C37) the solvers' unknown is the elevation E, which the pitch r in the
                    // sight plane reaches only up to its maximum (r <= 90 deg - phi, phi = atan2(sin L, cos L cos c)); a
                    // target so close that the straight line from the bore needs a steeper pitch (a sight 0.5 m above the
                    // bore, 60 deg of cant, a target 5 or 10 cm away) has no elevation to answer with: the elevation is
                    // clamped to the reachable limit and the shot cannot hit. The call still answers.
                    const double kPiLocal = 3.14159265358979323846;
                    const double look_rad = look * kPiLocal / 180.0, cant_rad = cant * kPiLocal / 180.0;
                    const double r_needed = std::atan2(shot.sight_height_ft * std::cos(look_rad),
                                                       d + shot.sight_height_ft * std::cos(cant_rad) * std::sin(look_rad));
                    const double r_max = kPiLocal / 2.0 - std::atan2(std::sin(look_rad), std::cos(look_rad) * std::cos(cant_rad));
                    if (r_needed > r_max + 1e-9)
                    {
                        CHECK(rc == BCLIBCFFI_OK);
                        CHECK(BCLIBCFFI_find_zero_point_shot(&shot, d, &zp, &err) == BCLIBCFFI_OK);
                        continue;
                    }
                    CHECK_ZERO("cant find_zero_angle_shot", rc, angle, shot, d);
                    rc = BCLIBCFFI_find_zero_point_shot(&shot, d, &zp, &err);
                    CHECK_ZERO("cant find_zero_point_shot", rc, zp.angle_rad, shot, d);
                }
    // The shortcut itself, at look 0, and the cutoff is 2 * h * cos(cant). The bore is offset by h * cos(cant) up and
    // h * sin(cant) sideways from the line of sight, so the straight line to the target rises by
    // atan2(h * cos(cant), hypot(x, h * sin(cant))) in the exact canted frame (class C37; before the angle was
    // atan2(h * cos(cant), x): at cant 30 and sight 0.5 m, target 1.8 * cutoff/2, 29.05 to 27.84 degrees).
    for (double cant : {30.0, 60.0})
    {
        const BCLIBCFFI_Shot shot = make_shot(method, t, 0.5, v, 0.0, cant);
        const double h = shot.sight_height_ft * std::cos(shot.cant_angle_rad);
        const double lateral = shot.sight_height_ft * std::sin(shot.cant_angle_rad);
        BCLIBCFFI_ZeroInit zi;
        std::memset(&zi, 0, sizeof zi);
        CHECK(BCLIBCFFI_zero_init_shot(&shot, 1.8 * h, &zi, &err) == BCLIBCFFI_OK);
        CHECK(zi.status == BCLIBCFFI_ZERO_INIT_DONE);
        CHECK(std::fabs(zi.look_angle_rad - std::atan2(h, std::hypot(1.8 * h, lateral))) < 1e-12);
        CHECK(BCLIBCFFI_zero_init_shot(&shot, 2.2 * h, &zi, &err) == BCLIBCFFI_OK);
        CHECK(zi.status == BCLIBCFFI_ZERO_INIT_CONTINUE);
    }
}

// The cutoff is 2 * max(sight height, calc_step): the calc_step branch. With a large cStepMultiplier (RK4, 400:
// calc_step 1.0, cutoff 2 ft) and with Euler (calc_step 0.5, cutoff 1 ft), a low sight leaves calc_step the larger one.
// (The old cutoff, 2 * max(h, cStepMultiplier), was 800 ft for the first and 2 ft for the second.)
static void test_abi_calc_step_branch()
{
    BCLIBCFFI_Error err;
    const Table &t = kTables[0];
    BCLIBCFFI_ZeroInit zi;
    std::memset(&zi, 0, sizeof zi);
    const double h_m = 0.05;

    const BCLIBCFFI_Shot big = make_shot(BCLIBCFFI_INTEGRATION_RK4, t, h_m, 800.0, 0.0, 0.0, 400.0);
    CHECK(BCLIBCFFI_zero_init_shot(&big, 0.5 * kFt, &zi, &err) == BCLIBCFFI_OK); // 1.64 ft < 2 ft
    CHECK(zi.status == BCLIBCFFI_ZERO_INIT_DONE);
    CHECK(std::fabs(zi.look_angle_rad - std::atan2(h_m * kFt, 0.5 * kFt)) < 1e-12);
    CHECK(BCLIBCFFI_zero_init_shot(&big, 1.0 * kFt, &zi, &err) == BCLIBCFFI_OK); // 3.28 ft > 2 ft: a search
    CHECK(zi.status == BCLIBCFFI_ZERO_INIT_CONTINUE);

    const BCLIBCFFI_Shot euler = make_shot(BCLIBCFFI_INTEGRATION_EULER, t, h_m, 800.0, 0.0);
    CHECK(BCLIBCFFI_zero_init_shot(&euler, 0.2 * kFt, &zi, &err) == BCLIBCFFI_OK); // 0.66 ft < 1 ft
    CHECK(zi.status == BCLIBCFFI_ZERO_INIT_DONE);
    CHECK(std::fabs(zi.look_angle_rad - std::atan2(h_m * kFt, 0.2 * kFt)) < 1e-12);
    CHECK(BCLIBCFFI_zero_init_shot(&euler, 0.4 * kFt, &zi, &err) == BCLIBCFFI_OK); // 1.31 ft > 1 ft
    CHECK(zi.status == BCLIBCFFI_ZERO_INIT_CONTINUE);
}

// ---------------------------------------------------------------------------------------------------------------
// The C++ core, without the ABI in between.
// ---------------------------------------------------------------------------------------------------------------

static bclibc::BCLIBC_ShotProps core_props(const Table &t, double sight_m, double v, double look_deg)
{
    bclibc::BCLIBC_Shot shot;
    shot.bc = t.bc;
    shot.weight_grain = 175.0;
    shot.diameter_inch = 0.3;
    shot.length_inch = 1.26;
    shot.muzzle_velocity_fps = v * kFt;
    shot.stability_coefficient = 0.0;
    shot.sight_height_ft = sight_m * kFt;
    shot.twist_inch = 10.0;
    shot.temp_c = 15.0;
    shot.pressure_hpa = 1013.25;
    shot.altitude_ft = 0.0;
    shot.humidity = 0.0;
    shot.mach_data = t.mach;
    shot.cd_data = t.cd;
    shot.drag_table_size = 4;
    shot.winds = nullptr;
    shot.wind_count = 0;
    shot.look_angle_rad = look_deg * M_PI / 180.0;
    shot.barrel_elevation_rad = 0.0;
    shot.barrel_azimuth_rad = 0.0;
    shot.cant_angle_rad = 0.0;
    shot.latitude_deg = std::nan("");
    shot.azimuth_deg = std::nan("");
    shot.calc_step = 0.0025;
    auto r = shot.to_shot_props();
    if (bclibc::has_error(r))
    {
        std::fprintf(stderr, "to_shot_props failed\n");
        std::exit(EXIT_FAILURE);
    }
    return r.value();
}

static void core_init(bclibc::BCLIBC_BaseEngine &eng, bclibc::BCLIBC_ShotProps props)
{
    eng.shot = std::move(props);
    eng.integrate_func = bclibc::BCLIBC_integrateRK4;
    eng.config = bclibc::BCLIBC_Config(1.0, 5e-6, 50.0, -15000.0, 40, -32.17405, -1500.0);
    eng.gravity_vector = bclibc::BCLIBC_V3dT(0.0, eng.config.cGravityConstant, 0.0);
}

static const double kApex = 0.0003;
static const double kAllowed = 1e-2;

static double core_miss(bclibc::BCLIBC_BaseEngine &eng, double angle_rad, double d_ft)
{
    const double la = eng.shot.look_angle;
    auto r = eng.error_at_distance(angle_rad, d_ft * std::cos(la), d_ft * std::sin(la));
    return bclibc::has_error(r) ? 1e9 : r.value();
}

static void test_core_close_targets()
{
    for (const Table &t : kTables)
        for (double sight : kSightsM)
            for (double v : kVelocitiesMps)
                for (double look : kLooksDeg)
                    for (double target : kTargetsM)
                    {
                        const double d = target * kFt;
                        bclibc::BCLIBC_BaseEngine eng;
                        core_init(eng, core_props(t, sight, v, look));
                        const int32_t method = BCLIBCFFI_INTEGRATION_RK4;

                        auto a1 = eng.zero_angle(d, kApex, kAllowed);
                        CHECK(!bclibc::has_error(a1));
                        if (!bclibc::has_error(a1))
                        {
                            const double miss = core_miss(eng, a1.value(), d);
                            if (std::fabs(miss) >= kMissTolFt)
                                report("core zero_angle", method, t, sight, v, look, target, a1.value(), miss);
                            CHECK(std::fabs(miss) < kMissTolFt);
                        }

                        auto a2 = eng.zero_angle_with_fallback(d, kApex, kAllowed);
                        CHECK(!bclibc::has_error(a2));
                        if (!bclibc::has_error(a2))
                        {
                            const double miss = core_miss(eng, a2.value(), d);
                            if (std::fabs(miss) >= kMissTolFt)
                                report("core zero_angle_with_fallback", method, t, sight, v, look, target, a2.value(),
                                       miss);
                            CHECK(std::fabs(miss) < kMissTolFt);
                        }

                        for (int lofted : {0, 1})
                        {
                            if (&t != &kTables[0] || v != 800.0 || look != 0.0)
                                continue;
                            auto a3 = eng.find_zero_angle(d, lofted, kApex, kAllowed);
                            // A flat shot has no higher arc (an error, as for any far target); the lower one must hit.
                            CHECK(lofted || look < 0.0 || !bclibc::has_error(a3));
                            if (!bclibc::has_error(a3))
                            {
                                const double miss = core_miss(eng, a3.value(), d);
                                if (std::fabs(miss) >= kMissTolFt)
                                    report("core find_zero_angle", method, t, sight, v, look, target, a3.value(), miss);
                                CHECK(std::fabs(miss) < kMissTolFt);
                            }
                        }
                    }
}

// ---------------------------------------------------------------------------------------------------------------
// Regression guard: targets further than the cutoff of the old and the new code. Recorded before the fix.
// ---------------------------------------------------------------------------------------------------------------

struct Guard
{
    int32_t method;
    int table;
    double sight_m;
    double v_mps;
    double target_m;
    double angle_rad;
};

static const Guard kGuard[] = {
#include "zero_close_target_guard.inc"
};

static void test_far_targets_unchanged()
{
    BCLIBCFFI_Error err;
    for (const Guard &g : kGuard)
    {
        const BCLIBCFFI_Shot shot = make_shot(g.method, kTables[g.table], g.sight_m, g.v_mps, 0.0);
        double angle = 0.0;
        const int32_t rc = BCLIBCFFI_find_zero_angle_shot(&shot, g.target_m * kFt, &angle, &err);
        CHECK(rc == BCLIBCFFI_OK);
        const bool same = std::fabs(angle - g.angle_rad) < 1e-12;
        if (!same)
            std::fprintf(stderr, "  guard: method %d, %s, sight %.2f m, v %.0f, target %.2f m: %.15g, recorded %.15g\n",
                         g.method, kTables[g.table].name, g.sight_m, g.v_mps, g.target_m, angle, g.angle_rad);
        CHECK(same);
    }
}

// `test_zero_close_target --print-guard` writes the guard table (the lines of zero_close_target_guard.inc).
static void print_guard()
{
    BCLIBCFFI_Error err;
    for (int32_t method : {BCLIBCFFI_INTEGRATION_RK4, BCLIBCFFI_INTEGRATION_TSITOURAS,
                           BCLIBCFFI_INTEGRATION_VELOCITY_VERLET})
        for (int ti = 0; ti < 2; ++ti)
            for (double sight : kSightsM)
                for (double v : kVelocitiesMps)
                    for (double target : {2.0, 5.0, 25.0, 100.0, 300.0, 0.7, 1.2})
                    {
                        // 0.7 and 1.2 m are only guarded where the old and the new cutoff both leave them to the search.
                        if (target < 2.0 && (method != BCLIBCFFI_INTEGRATION_RK4 || sight > 0.1))
                            continue;
                        const BCLIBCFFI_Shot shot = make_shot(method, kTables[ti], sight, v, 0.0);
                        double angle = 0.0;
                        if (BCLIBCFFI_find_zero_angle_shot(&shot, target * kFt, &angle, &err) != BCLIBCFFI_OK)
                            continue;
                        std::printf("    {%d, %d, %.2f, %.0f, %.2f, %.17g},\n", method, ti, sight, v, target, angle);
                    }
}

int main(int argc, char **argv)
{
    if (argc > 1 && std::strcmp(argv[1], "--print-guard") == 0)
    {
        print_guard();
        return EXIT_SUCCESS;
    }
    test_abi_close_targets();
    test_abi_cutoff_is_twice_the_sight_height();
    test_abi_cant();
    test_abi_calc_step_branch();
    test_core_close_targets();
    test_far_targets_unchanged();
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_zero_close_target: all checks passed");
    return EXIT_SUCCESS;
}
