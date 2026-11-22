// test_zero_downhill.cpp -- the zero angle (Ridder path) and the maximum range of a shot with a DOWNHILL look angle.
//
// find_max_range() runs a golden-section search over the barrel elevation, measuring each shot by the slant distance at
// which it comes back down through the line of sight (range_for_angle). It runs with cMaximumDrop = 0 ("stop at muzzle
// level") and the caller's cMinimumAltitude. That is right for a flat or uphill line of sight, which a shot crosses
// above the muzzle. A downhill line of sight lies below the muzzle: a shot is stopped at muzzle level, or at the
// altitude limit, long before it reaches it; range_for_angle() answers 0 ("no crossing") for all but near-vertical
// elevations and the search reports a maximum range of a few feet at about 90 degrees. The Ridder zero search
// (find_zero_angle_lofted(lofted=0), find_zero_point_lofted, the adaptive variants, find_zero_angle/find_zero_point of
// the core) then answers OUT_OF_RANGE for a target a few metres away, however near it is.
//
// A zero angle is correct when the shot fired with it crosses the line of sight at the target (error_at_distance, the
// measure the solvers minimise, within 2 mm); the maximum range is correct when it matches a brute-force scan over the
// elevation with the early-termination limits switched off. Part two is a regression guard: results that did not
// depend on the defect did not change (recorded values, zero_downhill_guard.inc, `--print-guard`).

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
static const double kMissTolFt = 0.00656; // 2 mm

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
static const double kSightsM[] = {0.1, 0.5};
static const double kVelocitiesMps[] = {300.0, 800.0};
static const double kTargetsM[] = {0.5, 0.9, 2.0, 5.0, 10.0, 50.0, 300.0};
static const double kDownhillDeg[] = {-5.0, -10.0, -20.0, -30.0, -45.0};

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
        const double miss_ = (rc) == BCLIBCFFI_OK ? abi_miss((shot), (angle), (d_ft)) : 1e9;            \
        const bool ok_ = (rc) == BCLIBCFFI_OK && std::fabs(miss_) < kMissTolFt;                         \
        if (!ok_)                                                                                       \
        {                                                                                               \
            report(what, method, t, sight, v, look, target, (angle), miss_);                            \
            if ((rc) != BCLIBCFFI_OK)                                                                   \
                std::fprintf(stderr, "    rc %d: %s\n", (int)(rc), err.message);                        \
        }                                                                                               \
        CHECK(ok_);                                                                                     \
    } while (0)

// The Ridder entry points of the C ABI (lower arc, lofted == 0) on every downhill look angle.
static void test_abi_downhill_zero()
{
    for (int32_t method : kMethods)
        for (const Table &t : kTables)
            for (double sight : kSightsM)
                for (double v : kVelocitiesMps)
                    for (double look : kDownhillDeg)
                        for (double target : kTargetsM)
                        {
                            // Every Ridder call runs a golden-section search (~0.1 s): the full grid for RK4 with the G7 table, a slice for the others.
                            if ((method != BCLIBCFFI_INTEGRATION_RK4 || &t != &kTables[0]) && (sight != 0.1 || v != 800.0))
                                continue;
                            if (sight == 0.5 && v == 300.0) // the 0.5 m sight at 800 m/s covers it
                                continue;
                            const BCLIBCFFI_Shot shot = make_shot(method, t, sight, v, look);
                            const double d = target * kFt;
                            BCLIBCFFI_Error err;
                            double angle = 0.0;
                            int32_t rc;

                            rc = BCLIBCFFI_find_zero_angle_lofted_shot(&shot, d, 0, &angle, &err);
                            CHECK_ZERO("find_zero_angle_lofted_shot", rc, angle, shot, d);

                            BCLIBCFFI_ZeroPointResult zp;
                            std::memset(&zp, 0, sizeof zp);
                            rc = BCLIBCFFI_find_zero_point_lofted_shot(&shot, d, 0, &zp, &err);
                            CHECK_ZERO("find_zero_point_lofted_shot", rc, zp.angle_rad, shot, d);

                            // The Newton-first entry points already worked; they must keep working.
                            rc = BCLIBCFFI_find_zero_angle_shot(&shot, d, &angle, &err);
                            CHECK_ZERO("find_zero_angle_shot", rc, angle, shot, d);
                            rc = BCLIBCFFI_find_zero_point_shot(&shot, d, &zp, &err);
                            CHECK_ZERO("find_zero_point_shot", rc, zp.angle_rad, shot, d);

                            // The adaptive entry points take a method of their own (the grid is cut: they are slow).
                            if (method != BCLIBCFFI_INTEGRATION_RK4 || v != 800.0 || &t != &kTables[0])
                                continue;
                            BCLIBCFFI_StepStats stats;
                            std::memset(&stats, 0, sizeof stats);
                            rc = BCLIBCFFI_find_zero_angle_lofted_adaptive_shot(
                                &shot, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, 1e-8, 1e-8, d, 0, &angle, &stats, &err);
                            CHECK_ZERO("find_zero_angle_lofted_adaptive_shot", rc, angle, shot, d);
                            rc = BCLIBCFFI_find_zero_point_lofted_adaptive_shot(
                                &shot, BCLIBCFFI_INTEGRATION_TSITOURAS, 1e-8, 1e-8, d, 0, &zp, &stats, &err);
                            CHECK_ZERO("find_zero_point_lofted_adaptive_shot", rc, zp.angle_rad, shot, d);
                        }
}

// The reported repros, spelled out: 0.1 m sight, -20 deg at 0.9 / 2 / 50 m, -10 deg at 2 / 3 / 5 / 10 m, -30 deg at
// 0.7 m and beyond, 800 and 300 m/s, both tables.
static void test_abi_reported_repros()
{
    struct Repro
    {
        double look;
        double target;
    };
    static const Repro repros[] = {{-20, 0.9}, {-20, 2}, {-20, 50}, {-10, 2}, {-10, 3}, {-10, 5},
                                   {-10, 10},  {-30, 0.7}, {-30, 2}, {-30, 100}, {-30, 300}};
    for (const Table &t : kTables)
        for (double v : kVelocitiesMps)
            for (const Repro &r : repros)
            {
                const double sight = 0.1, look = r.look, target = r.target;
                const int32_t method = BCLIBCFFI_INTEGRATION_RK4;
                const BCLIBCFFI_Shot shot = make_shot(method, t, sight, v, look);
                BCLIBCFFI_Error err;
                double angle = 0.0;
                const int32_t rc = BCLIBCFFI_find_zero_angle_lofted_shot(&shot, target * kFt, 0, &angle, &err);
                CHECK_ZERO("repro find_zero_angle_lofted_shot", rc, angle, shot, target * kFt);
            }
}

// The maximum range of a downhill shot: not "a few feet at 90 degrees". It must be the real maximum slant range, the
// same as a scan over the elevation with no early termination at all.
static void test_core_downhill_max_range()
{
    for (const Table &t : kTables)
        for (double sight : kSightsM)
            for (double v : kVelocitiesMps)
                for (double look : kDownhillDeg)
                {
                    if (&t != &kTables[0] && sight != 0.1)
                        continue;
                    bclibc::BCLIBC_BaseEngine eng;
                    core_init(eng, core_props(t, sight, v, look));
                    const bclibc::BCLIBC_Config saved = eng.config;
                    auto mr = eng.find_max_range(0.0, 90.0, kApex);
                    CHECK(!bclibc::has_error(mr));
                    // the limits of the caller are restored
                    CHECK(eng.config.cMaximumDrop == saved.cMaximumDrop);
                    CHECK(eng.config.cMinimumAltitude == saved.cMinimumAltitude);
                    CHECK(eng.config.cMinimumVelocity == saved.cMinimumVelocity);
                    if (bclibc::has_error(mr))
                        continue;

                    eng.config.cMaximumDrop = -1e12;
                    eng.config.cMinimumAltitude = -1e12;
                    eng.config.cMinimumVelocity = 0.0;
                    double scan_max = 0.0;
                    for (double a = 2.0; a < 90.0; a += 2.0)
                        scan_max = std::fmax(scan_max, eng.range_for_angle(a * M_PI / 180.0));

                    const double got = mr.value().max_range_ft;
                    const double ang = mr.value().angle_at_max_rad * 180.0 / M_PI;
                    const bool ok = got >= scan_max * (1.0 - 1e-6) && got <= scan_max * 1.01 && ang > 1.0 && ang < 89.0;
                    if (!ok)
                        std::fprintf(stderr, "  max range: %s, sight %.2f, v %.0f, look %.0f: %.1f ft at %.2f deg, scan %.1f ft\n",
                                     t.name, sight, v, look, got, ang, scan_max);
                    CHECK(ok);
                }
}

static void test_core_downhill_zero()
{
    for (const Table &t : kTables)
        for (double sight : kSightsM)
            for (double v : kVelocitiesMps)
                for (double look : kDownhillDeg)
                    for (double target : kTargetsM)
                    {
                        if ((&t != &kTables[0] && (sight != 0.1 || v != 800.0)) || (sight == 0.5 && v == 300.0))
                            continue; // a slice of the second table
                        const double d = target * kFt;
                        const int32_t method = BCLIBCFFI_INTEGRATION_RK4;
                        bclibc::BCLIBC_BaseEngine eng;
                        core_init(eng, core_props(t, sight, v, look));

                        auto a1 = eng.find_zero_angle(d, 0, kApex, kAllowed);
                        CHECK(!bclibc::has_error(a1));
                        if (!bclibc::has_error(a1))
                        {
                            const double miss = core_miss(eng, a1.value(), d);
                            if (std::fabs(miss) >= kMissTolFt)
                                report("core find_zero_angle", method, t, sight, v, look, target, a1.value(), miss);
                            CHECK(std::fabs(miss) < kMissTolFt);
                        }
                        auto a2 = eng.find_zero_point(d, 0, kApex, kAllowed);
                        CHECK(!bclibc::has_error(a2));
                        if (!bclibc::has_error(a2))
                            CHECK(std::fabs(core_miss(eng, a2.value().angle_rad, d)) < kMissTolFt);

                        // The Newton-first paths worked before and must keep working.
                        auto a3 = eng.zero_angle_with_fallback(d, kApex, kAllowed);
                        CHECK(!bclibc::has_error(a3));
                        if (!bclibc::has_error(a3))
                            CHECK(std::fabs(core_miss(eng, a3.value(), d)) < kMissTolFt);
                        auto a4 = eng.zero_angle(d, kApex, kAllowed);
                        CHECK(!bclibc::has_error(a4));
                        if (!bclibc::has_error(a4))
                            CHECK(std::fabs(core_miss(eng, a4.value(), d)) < kMissTolFt);
                    }
}

// The target is really out of range: still reported, now with the real maximum range.
static void test_downhill_out_of_range_is_reported()
{
    const Table &t = kTables[0];
    const BCLIBCFFI_Shot shot = make_shot(BCLIBCFFI_INTEGRATION_RK4, t, 0.1, 300.0, -20.0);
    BCLIBCFFI_MaxRangeResult mr;
    BCLIBCFFI_Error err;
    CHECK(BCLIBCFFI_find_max_range_shot(&shot, 0.0, 90.0, &mr, &err) == BCLIBCFFI_OK);
    CHECK(mr.max_range_ft > 1000.0 * kFt);
    double angle = 0.0;
    int32_t rc = BCLIBCFFI_find_zero_angle_lofted_shot(&shot, 2.0 * mr.max_range_ft, 0, &angle, &err);
    CHECK(rc == BCLIBCFFI_ERR_OUT_OF_RANGE);
    CHECK(std::fabs(err.f64_1 - mr.max_range_ft) < 1e-6 * mr.max_range_ft); // reported max range
}

// ---------------------------------------------------------------------------------------------------------------
// Regression guard: results that did not depend on the defect. Recorded before the fix.
//   kind 0: find_zero_angle_shot (Newton, Ridder fallback), look in {-20, -10, 0, 10}
//   kind 1: find_zero_angle_lofted_shot(lofted == 0) for look >= 0
//   kind 2: find_max_range_shot for look >= 0 (angle_at_max_rad)
//   kind 3: find_max_range_shot for look >= 0 (max_range_ft)
// ---------------------------------------------------------------------------------------------------------------

struct Guard
{
    int kind;
    int table;
    double sight_m;
    double v_mps;
    double look_deg;
    double target_m;
    double value;
};

static const Guard kGuard[] = {
#include "zero_downhill_guard.inc"
};

static double guard_value(const Guard &g)
{
    const BCLIBCFFI_Shot shot = make_shot(BCLIBCFFI_INTEGRATION_RK4, kTables[g.table], g.sight_m, g.v_mps, g.look_deg);
    BCLIBCFFI_Error err;
    double angle = std::nan("");
    BCLIBCFFI_MaxRangeResult mr;
    switch (g.kind)
    {
    case 0:
        BCLIBCFFI_find_zero_angle_shot(&shot, g.target_m * kFt, &angle, &err);
        return angle;
    case 1:
        BCLIBCFFI_find_zero_angle_lofted_shot(&shot, g.target_m * kFt, 0, &angle, &err);
        return angle;
    default:
        if (BCLIBCFFI_find_max_range_shot(&shot, 0.0, 90.0, &mr, &err) != BCLIBCFFI_OK)
            return std::nan("");
        return g.kind == 2 ? mr.angle_at_max_rad : mr.max_range_ft;
    }
}

static void test_unchanged_results()
{
    for (const Guard &g : kGuard)
    {
        const double now = guard_value(g);
        const bool same = std::fabs(now - g.value) <= 1e-12 * std::fmax(1.0, std::fabs(g.value));
        if (!same)
            std::fprintf(stderr, "  guard: kind %d, %s, sight %.2f m, v %.0f, look %.0f, target %.2f m: %.17g, recorded %.17g\n",
                         g.kind, kTables[g.table].name, g.sight_m, g.v_mps, g.look_deg, g.target_m, now, g.value);
        CHECK(same);
    }
}

static void print_guard()
{
    BCLIBCFFI_Error err;
    for (int ti = 0; ti < 2; ++ti)
        for (double sight : kSightsM)
            for (double v : kVelocitiesMps)
            {
                for (double look : {-20.0, -10.0, 0.0, 10.0})
                    for (double target : {0.5, 2.0, 10.0, 50.0, 300.0})
                    {
                        Guard g{0, ti, sight, v, look, target, 0.0};
                        g.value = guard_value(g);
                        if (!std::isnan(g.value))
                            std::printf("    {0, %d, %.2f, %.0f, %.0f, %.2f, %.17g},\n", ti, sight, v, look, target, g.value);
                    }
                for (double look : {0.0, 10.0, 30.0})
                {
                    for (double target : {2.0, 10.0, 50.0, 300.0})
                    {
                        Guard g{1, ti, sight, v, look, target, 0.0};
                        g.value = guard_value(g);
                        if (!std::isnan(g.value))
                            std::printf("    {1, %d, %.2f, %.0f, %.0f, %.2f, %.17g},\n", ti, sight, v, look, target, g.value);
                    }
                    for (int kind : {2, 3})
                    {
                        Guard g{kind, ti, sight, v, look, 0.0, 0.0};
                        g.value = guard_value(g);
                        if (!std::isnan(g.value))
                            std::printf("    {%d, %d, %.2f, %.0f, %.0f, %.2f, %.17g},\n", kind, ti, sight, v, look, 0.0, g.value);
                    }
                }
            }
    (void)err;
}

int main(int argc, char **argv)
{
    if (argc > 1 && std::strcmp(argv[1], "--print-guard") == 0)
    {
        print_guard();
        return EXIT_SUCCESS;
    }
    test_abi_reported_repros();
    test_abi_downhill_zero();
    test_core_downhill_max_range();
    test_core_downhill_zero();
    test_downhill_out_of_range_is_reported();
    test_unchanged_results();
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_zero_downhill: all checks passed");
    return EXIT_SUCCESS;
}
