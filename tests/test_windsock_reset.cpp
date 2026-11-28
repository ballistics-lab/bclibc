// test_windsock_reset.cpp -- the wind sock is rewound at the start of every integration.
//
// The integrators advance eng.shot.wind_sock (current segment, next_range, cached vector) as the projectile passes the
// end of a wind segment, and nothing put it back at the start of the next integration of the same solve. Every trial
// shot of find_zero_angle / find_max_range / the lofted solves / the Newton zero started with the sock the previous
// trial had left behind (past the last segment, i.e. no wind at all), so a shot with more than one wind segment, or
// one finite segment, was solved with the wind of its first trial only. The Python engines build a fresh wind sock per
// integration; this test pins that down.
//
// Shot: 800 m/s, G7-like drag, BC 0.31, sight 5 cm, zero at 600 m.
//   * one wind of 30 m/s (direction 0) up to 1000 m: the trial shots pass 1000 m, so the sock ran off its end.
//   * two segments: 10 m/s cross wind up to 300 m, then 10 m/s (direction 0).
// A zero angle is correct when the shot fired with it crosses the line of sight at the target (the miss, measured on a
// fresh engine, is ~0) and when the Ridder and the Newton solvers agree on it. The maximum range with a head wind must
// differ from the one without a wind. In the C++ core, two consecutive integrations of the same engine must agree.
//
// Through the C ABI (fixed-step and adaptive methods) and through the C++ core.

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

static const double kFt = 3.28084;       // feet per metre
static const double kMissTolFt = 1e-3;   // 0.3 mm; the stale sock missed by 0.197 ft
static const double kAgreeRad = 2e-7;    // Ridder and Newton agree (their own tolerance is 5e-6 ft)
static const double kMaxWindFt = 1e8;    // BCLIBC_cMaxWindDistanceFeet

static const double kMach[] = {0.0, 0.8, 1.2, 3.0};
static const double kCd[] = {0.12, 0.13, 0.38, 0.30};

// Wind directions as in the Python reference repro: 0 is its "head" wind (the vector (+v, 0, 0)), pi the opposite one,
// pi/2 a cross wind.
static const BCLIBCFFI_Wind kHead1000[] = {{30.0 * kFt, 0.0, 1000.0 * kFt, kMaxWindFt}};
static const BCLIBCFFI_Wind kTwoSegments[] = {{10.0 * kFt, M_PI / 2.0, 300.0 * kFt, kMaxWindFt},
                                              {10.0 * kFt, 0.0, 5000.0 * kFt, kMaxWindFt}};

static BCLIBCFFI_Shot make_shot(int32_t method, const BCLIBCFFI_Wind *winds, int32_t wind_count)
{
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = 0.31;
    s.weight_grain = 175.0;
    s.diameter_inch = 0.3;
    s.length_inch = 1.26;
    s.muzzle_velocity_fps = 800.0 * kFt;
    s.sight_height_ft = 0.05 * kFt;
    s.twist_inch = 10.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.humidity = 0.0;
    s.mach_data = kMach;
    s.cd_data = kCd;
    s.drag_table_size = 4;
    s.winds = winds;
    s.wind_count = wind_count;
    s.latitude_deg = std::nan("");
    s.azimuth_deg = std::nan("");
    s.config.cStepMultiplier = 1.0;
    s.config.cZeroFindingAccuracy = 5e-6;
    s.config.cMinimumVelocity = 50.0;
    s.config.cMaximumDrop = -15000.0;
    s.config.cMaxIterations = 40;
    s.config.cGravityConstant = -32.17405;
    s.config.cMinimumAltitude = -1500.0;
    s.method = static_cast<BCLIBCFFI_IntegrationMethod>(method);
    return s;
}

static bool is_adaptive(int32_t method)
{
    return method == BCLIBCFFI_INTEGRATION_CASH_KARP || method == BCLIBCFFI_INTEGRATION_DORMAND_PRINCE ||
           method == BCLIBCFFI_INTEGRATION_TSITOURAS;
}

static const double kTol = 1e-8;

// The miss of a shot fired at angle_rad: a single integration on a fresh engine (the first trial, never stale).
static double abi_miss(const BCLIBCFFI_Shot &shot, int32_t method, double angle_rad, double d_ft)
{
    BCLIBCFFI_Error err;
    double miss = 1e9;
    BCLIBCFFI_StepStats stats;
    const int32_t rc = is_adaptive(method)
                           ? BCLIBCFFI_error_at_distance_adaptive_shot(&shot, method, kTol, kTol, angle_rad, d_ft, 0.0,
                                                                       &miss, nullptr, &stats, &err)
                           : BCLIBCFFI_error_at_distance_shot(&shot, angle_rad, d_ft, 0.0, &miss, nullptr, &err);
    if (rc != BCLIBCFFI_OK)
        std::fprintf(stderr, "  error_at_distance failed: %s\n", err.message);
    return miss;
}

static int32_t abi_ridder(const BCLIBCFFI_Shot &shot, int32_t method, double d_ft, double *angle, BCLIBCFFI_Error *err)
{
    BCLIBCFFI_StepStats stats;
    // The Ridder-only search (lofted = 0, the lower arc): it brackets with find_max_range, whose trials fly past every wind
    // segment. find_zero_angle_shot tries Newton first, which stops at the target and never runs the sock off its end.
    return is_adaptive(method)
               ? BCLIBCFFI_find_zero_angle_lofted_adaptive_shot(&shot, method, kTol, kTol, d_ft, 0, angle, &stats, err)
               : BCLIBCFFI_find_zero_angle_lofted_shot(&shot, d_ft, 0, angle, err);
}

static int32_t abi_newton(const BCLIBCFFI_Shot &shot, int32_t method, double d_ft, double *angle, BCLIBCFFI_Error *err)
{
    BCLIBCFFI_StepStats stats;
    return is_adaptive(method)
               ? BCLIBCFFI_find_zero_angle_newton_adaptive_shot(&shot, method, kTol, kTol, d_ft, angle, &stats, err)
               : BCLIBCFFI_find_zero_angle_newton_shot(&shot, d_ft, angle, err);
}

static const int32_t kMethods[] = {BCLIBCFFI_INTEGRATION_RK4, BCLIBCFFI_INTEGRATION_EULER,
                                   BCLIBCFFI_INTEGRATION_VELOCITY_VERLET, BCLIBCFFI_INTEGRATION_CASH_KARP,
                                   BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, BCLIBCFFI_INTEGRATION_TSITOURAS};

static void check_zero_with_wind(const char *what, const BCLIBCFFI_Wind *winds, int32_t n)
{
    const double d = 600.0 * kFt;
    for (int32_t method : kMethods)
    {
        const BCLIBCFFI_Shot shot = make_shot(method, winds, n);
        BCLIBCFFI_Error err;
        double ridder = 0.0, newton = 0.0;
        const int32_t rc1 = abi_ridder(shot, method, d, &ridder, &err);
        CHECK(rc1 == BCLIBCFFI_OK);
        const int32_t rc2 = abi_newton(shot, method, d, &newton, &err);
        CHECK(rc2 == BCLIBCFFI_OK);
        const double miss_r = abi_miss(shot, method, ridder, d);
        const double miss_n = abi_miss(shot, method, newton, d);
        const bool ok = rc1 == BCLIBCFFI_OK && rc2 == BCLIBCFFI_OK && std::fabs(miss_r) < kMissTolFt &&
                        std::fabs(miss_n) < kMissTolFt && std::fabs(ridder - newton) < kAgreeRad;
        if (!ok || getenv("V"))
            std::fprintf(stderr,
                         "  %s: method %d: ridder %.6f MOA (miss %+.5f ft), newton %.6f MOA (miss %+.5f ft)\n", what,
                         method, ridder * 180.0 / M_PI * 60.0, miss_r, newton * 180.0 / M_PI * 60.0, miss_n);
        CHECK(std::fabs(miss_r) < kMissTolFt);
        CHECK(std::fabs(miss_n) < kMissTolFt);
        CHECK(std::fabs(ridder - newton) < kAgreeRad);
    }
}

static void test_abi_zero_head_wind() { check_zero_with_wind("head 30 m/s to 1000 m", kHead1000, 1); }
static void test_abi_zero_two_segments() { check_zero_with_wind("cross then head", kTwoSegments, 2); }

// A wind of one finite segment changes the maximum range; with the stale sock the search ran in still air.
static void test_abi_max_range_with_wind()
{
    // A wind until 2000 m, so most of the flight (which is several km) is outside it.
    static const BCLIBCFFI_Wind dir0_2000[] = {{30.0 * kFt, 0.0, 2000.0 * kFt, kMaxWindFt}};
    static const BCLIBCFFI_Wind dirpi_2000[] = {{30.0 * kFt, M_PI, 2000.0 * kFt, kMaxWindFt}};
    for (int32_t method : kMethods)
    {
        BCLIBCFFI_Error err;
        BCLIBCFFI_MaxRangeResult calm, head, tail;
        std::memset(&calm, 0, sizeof calm);
        std::memset(&head, 0, sizeof head);
        std::memset(&tail, 0, sizeof tail);
        const BCLIBCFFI_Shot s0 = make_shot(method, nullptr, 0);
        const BCLIBCFFI_Shot s1 = make_shot(method, dir0_2000, 1);
        const BCLIBCFFI_Shot s2 = make_shot(method, dirpi_2000, 1);
        BCLIBCFFI_StepStats stats;
        int32_t rc[3];
        if (is_adaptive(method))
        {
            rc[0] = BCLIBCFFI_find_max_range_adaptive_shot(&s0, method, kTol, kTol, 0.0, 90.0, &calm, &stats, &err);
            rc[1] = BCLIBCFFI_find_max_range_adaptive_shot(&s1, method, kTol, kTol, 0.0, 90.0, &head, &stats, &err);
            rc[2] = BCLIBCFFI_find_max_range_adaptive_shot(&s2, method, kTol, kTol, 0.0, 90.0, &tail, &stats, &err);
        }
        else
        {
            rc[0] = BCLIBCFFI_find_max_range_shot(&s0, 0.0, 90.0, &calm, &err);
            rc[1] = BCLIBCFFI_find_max_range_shot(&s1, 0.0, 90.0, &head, &err);
            rc[2] = BCLIBCFFI_find_max_range_shot(&s2, 0.0, 90.0, &tail, &err);
        }
        CHECK(rc[0] == BCLIBCFFI_OK && rc[1] == BCLIBCFFI_OK && rc[2] == BCLIBCFFI_OK);
        // The two opposite winds move the range to either side of the still-air one by at least a metre (the stale sock
        // gave the same range for any wind).
        const double dh = head.max_range_ft - calm.max_range_ft, dt = tail.max_range_ft - calm.max_range_ft;
        const bool ok = dh * dt < 0.0 && std::fabs(dh) > 1.0 * kFt && std::fabs(dt) > 1.0 * kFt;
        if (!ok)
            std::fprintf(stderr, "  max range: method %d: calm %.3f m, head %.3f m, tail %.3f m\n", method,
                         calm.max_range_ft / kFt, head.max_range_ft / kFt, tail.max_range_ft / kFt);
        CHECK(ok);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// The C++ core: the same engine integrates the same shot twice with the same result.
// ---------------------------------------------------------------------------------------------------------------

static bclibc::BCLIBC_ShotProps core_props(const bclibc::BCLIBC_Wind *winds, int wind_count)
{
    bclibc::BCLIBC_Shot shot;
    shot.bc = 0.31;
    shot.weight_grain = 175.0;
    shot.diameter_inch = 0.3;
    shot.length_inch = 1.26;
    shot.muzzle_velocity_fps = 800.0 * kFt;
    shot.stability_coefficient = 0.0;
    shot.sight_height_ft = 0.05 * kFt;
    shot.twist_inch = 10.0;
    shot.temp_c = 15.0;
    shot.pressure_hpa = 1013.25;
    shot.altitude_ft = 0.0;
    shot.humidity = 0.0;
    shot.mach_data = kMach;
    shot.cd_data = kCd;
    shot.drag_table_size = 4;
    shot.winds = winds;
    shot.wind_count = wind_count;
    shot.look_angle_rad = 0.0;
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

static double core_miss(bclibc::BCLIBC_BaseEngine &eng, double angle_rad, double d_ft)
{
    auto r = eng.error_at_distance(angle_rad, d_ft, 0.0);
    return bclibc::has_error(r) ? 1e9 : r.value();
}

static void test_core(const char *name, bclibc::BCLIBC_IntegrateCallable fn, const bclibc::BCLIBC_Wind *winds, int n)
{
    const double d = 600.0 * kFt;
    bclibc::BCLIBC_BaseEngine eng;
    eng.shot = core_props(winds, n);
    eng.integrate_func = fn;
    eng.config = bclibc::BCLIBC_Config(1.0, 5e-6, 50.0, -15000.0, 40, -32.17405, -1500.0);
    eng.gravity_vector = bclibc::BCLIBC_V3dT(0.0, eng.config.cGravityConstant, 0.0);

    const double angle = 0.0035;
    const double first = core_miss(eng, angle, d);
    const double second = core_miss(eng, angle, d);
    const double third = core_miss(eng, angle, d);
    if (first != second || second != third)
        std::fprintf(stderr, "  core %s: repeated integration differs: %.9f %.9f %.9f ft\n", name, first, second, third);
    CHECK(first == second);
    CHECK(second == third);

    auto a = eng.zero_angle(d, 0.0003, 1e-2);
    CHECK(!bclibc::has_error(a));
    if (!bclibc::has_error(a))
    {
        const double miss = core_miss(eng, a.value(), d);
        if (std::fabs(miss) >= kMissTolFt)
            std::fprintf(stderr, "  core %s zero_angle: miss %+.5f ft\n", name, miss);
        CHECK(std::fabs(miss) < kMissTolFt);
    }
    auto b = eng.find_zero_angle(d, 0, 0.0003, 1e-2);
    CHECK(!bclibc::has_error(b));
    if (!bclibc::has_error(b))
    {
        const double miss = core_miss(eng, b.value(), d);
        if (std::fabs(miss) >= kMissTolFt)
            std::fprintf(stderr, "  core %s find_zero_angle: miss %+.5f ft\n", name, miss);
        CHECK(std::fabs(miss) < kMissTolFt);
    }
}

int main()
{
    test_abi_zero_head_wind();
    test_abi_zero_two_segments();
    test_abi_max_range_with_wind();
    const bclibc::BCLIBC_Wind two[] = {bclibc::BCLIBC_Wind(10.0 * kFt, M_PI / 2.0, 300.0 * kFt, kMaxWindFt),
                                       bclibc::BCLIBC_Wind(10.0 * kFt, 0.0, 5000.0 * kFt, kMaxWindFt)};
    const bclibc::BCLIBC_Wind one[] = {bclibc::BCLIBC_Wind(30.0 * kFt, 0.0, 1000.0 * kFt, kMaxWindFt)};
    for (int k = 0; k < 2; ++k)
    {
        const bclibc::BCLIBC_Wind *w = k ? one : two;
        const int n = k ? 1 : 2;
        test_core("rk4", bclibc::BCLIBC_integrateRK4, w, n);
        test_core("euler", bclibc::BCLIBC_integrateEULER, w, n);
        test_core("verlet", bclibc::BCLIBC_integrateVELOCITY_VERLET, w, n);
        test_core("cash_karp", bclibc::BCLIBC_integrateCashKarp, w, n);
        test_core("tsitouras", bclibc::BCLIBC_integrateTsitouras, w, n);
    }
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_windsock_reset: all checks passed");
    return EXIT_SUCCESS;
}
