// test_min_steps_gate.cpp -- the range limit is not tested before the third integration step.
//
// The Python engines loop `while x <= range_limit or step_count < 3` (rk4.py, euler.py, velocity_verlet.py): a shot whose
// range limit lies inside the first step still takes three steps, because the cubic interpolation of the recorded
// points needs three of them. The other termination limits (minimum velocity, maximum drop, minimum altitude) are not
// gated: they stop the shot after any step, the first one included (test_start_outside_limit.cpp).
//
// BCLIBC_EssentialTerminators counted the initial sample as well as the step ends, so its gate opened one step early
// (after two steps): the shot ended with start + 2 steps where the reference has start + 3 steps. The rows
// (interpolated at the requested ranges) are the same, the dense steps the C ABI returns are not: one step fewer, which
// is the data a caller interpolates later queries from. The adaptive methods have no Python reference; they follow the
// fixed-step convention and tiny_bclibc (which counts step ends only): three accepted steps.
//
// Expected dense point counts (start included) are those of the Python reference engines for the same shot.

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

static const double kMachG7[] = {0.0, 0.8, 1.2, 3.0};
static const double kCdG7[] = {0.12, 0.13, 0.38, 0.30};

static BCLIBCFFI_Shot make_shot(int32_t method, double sight_ft, double elevation_deg)
{
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = 0.243;
    s.weight_grain = 175.0;
    s.diameter_inch = 0.3;
    s.length_inch = 1.26;
    s.muzzle_velocity_fps = 2600.0;
    s.sight_height_ft = sight_ft;
    s.twist_inch = 10.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.humidity = 0.0;
    s.mach_data = kMachG7;
    s.cd_data = kCdG7;
    s.drag_table_size = 4;
    s.barrel_elevation_rad = elevation_deg * M_PI / 180.0;
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


struct Case
{
    double range_limit_ft;
    int32_t dense_points;
};

static int dense_points(const BCLIBCFFI_Shot &shot, bool adaptive, double range_limit_ft, int32_t *reason_out)
{
    BCLIBCFFI_TrajectoryRequest req;
    std::memset(&req, 0, sizeof req);
    req.range_limit_ft = range_limit_ft;
    req.range_step_ft = range_limit_ft;
    req.filter_flags = BCLIBCFFI_TRAJ_FLAG_RANGE;
    BCLIBCFFI_TrajectoryData *rows = nullptr;
    int32_t n = 0, reason = -1, n_dense = 0;
    BCLIBCFFI_DenseStep *dense = nullptr;
    BCLIBCFFI_Error err;
    std::memset(&err, 0, sizeof err);
    int32_t rc;
    if (adaptive)
        rc = BCLIBCFFI_integrate_adaptive_shot(&shot, shot.method, 1e-8, 1e-8, &req, 1, &rows, &n, &dense, &n_dense,
                                               &reason, nullptr, &err);
    else
        rc = BCLIBCFFI_integrate_dense_shot(&shot, &req, 1, &rows, &n, &dense, &n_dense, &reason, &err);
    if (rc != BCLIBCFFI_OK)
    {
        std::fprintf(stderr, "  rc %d: %s\n", (int)rc, err.message);
        CHECK(rc == BCLIBCFFI_OK);
        return -1;
    }
    BCLIBCFFI_free_trajectory(rows);
    BCLIBCFFI_free_base_trajectory(dense);
    *reason_out = reason;
    return n_dense;
}

static void check_cases(const char *name, int32_t method, bool adaptive, const Case *cases, int n)
{
    for (int i = 0; i < n; ++i)
    {
        const BCLIBCFFI_Shot s = make_shot(method, 0.0, 0.0);
        int32_t reason = -1;
        const int got = dense_points(s, adaptive, cases[i].range_limit_ft, &reason);
        if (got != cases[i].dense_points || reason != BCLIBCFFI_TERM_TARGET_RANGE_REACHED)
            std::fprintf(stderr, "  %s, range limit %g ft: %d dense points (want %d), reason %d\n", name,
                         cases[i].range_limit_ft, got, (int)cases[i].dense_points, (int)reason);
        CHECK(got == cases[i].dense_points);
        CHECK(reason == BCLIBCFFI_TERM_TARGET_RANGE_REACHED);
    }
}

int main()
{
    // rk4: a step is ~6.5 ft. Limits inside step 1, inside step 2, inside step 3 (all: start + 3 steps), past step 3
    // (the gate no longer matters: start + 4 steps, and 7 steps for 40 ft).
    const Case rk4[] = {{0.5, 4}, {6.0, 4}, {10.0, 4}, {15.0, 4}, {20.0, 5}, {40.0, 8}};
    check_cases("rk4", BCLIBCFFI_INTEGRATION_RK4, false, rk4, 6);
    // euler: a step is ~0.5 ft.
    const Case euler[] = {{0.5, 4}, {1.0, 4}, {2.0, 6}, {10.0, 22}};
    check_cases("euler", BCLIBCFFI_INTEGRATION_EULER, false, euler, 4);
    // verlet: a step is ~2.6 ft.
    const Case verlet[] = {{0.5, 4}, {6.0, 4}, {10.0, 5}, {20.0, 9}};
    check_cases("verlet", BCLIBCFFI_INTEGRATION_VELOCITY_VERLET, false, verlet, 4);
    // adaptive: no Python reference; three accepted steps (start + 3) for a limit inside the first steps.
    const Case adaptive[] = {{0.01, 4}};
    check_cases("cash_karp", BCLIBCFFI_INTEGRATION_CASH_KARP, true, adaptive, 1);
    check_cases("dormand_prince", BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, true, adaptive, 1);
    check_cases("tsitouras", BCLIBCFFI_INTEGRATION_TSITOURAS, true, adaptive, 1);
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_min_steps_gate: all checks passed");
    return EXIT_SUCCESS;
}
