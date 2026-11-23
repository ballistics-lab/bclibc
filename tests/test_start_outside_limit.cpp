// test_start_outside_limit.cpp -- a shot that starts outside a termination limit still ends with its terminal row.
//
// The essential terminators (minimum velocity, maximum drop, minimum altitude) ran on the *initial* point too. A shot
// that started outside a limit (a minimum velocity above the muzzle velocity, a minimum altitude above the muzzle with
// the bullet falling) was stopped before its first integration step, so the integrator never called handle_step():
// the filter's last point stayed the start point and finalize() had nothing to add. The result was the single row
// [RANGE t=0]. The Python engines take their first step before they test the limits and keep the terminal point:
// [RANGE t=0, NONE t=dt]. The expected rows below are the ones of the Python reference engines (rk4_engine,
// euler_engine, verlet_engine) for the same shot.
//
// Rows are checked through the C ABI (integrate_dense_shot for the fixed-step methods, integrate_adaptive_shot for the
// adaptive ones) with a RANGE request, which is what the exts engines issue.

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

struct Row
{
    int32_t flag;
    double time;
};

// Fires the shot with a 100 yd RANGE request to 300 yd and compares the rows with the expected ones.
// step_tol: absolute tolerance of the time of the terminal row (an adaptive step is not a number the reference fixes).
static void expect_rows(const char *what, const BCLIBCFFI_Shot &shot, bool adaptive, const Row *want, int n_want,
                        int32_t want_reason, double time_tol)
{
    BCLIBCFFI_TrajectoryRequest req;
    std::memset(&req, 0, sizeof req);
    req.range_limit_ft = 900.0;
    req.range_step_ft = 300.0;
    req.filter_flags = BCLIBCFFI_TRAJ_FLAG_RANGE;
    BCLIBCFFI_TrajectoryData *rows = nullptr;
    int32_t n = 0, reason = -1, n_dense = 0;
    BCLIBCFFI_DenseStep *dense = nullptr;
    BCLIBCFFI_Error err;
    std::memset(&err, 0, sizeof err);
    int32_t rc;
    if (adaptive)
        rc = BCLIBCFFI_integrate_adaptive_shot(&shot, shot.method, 1e-8, 1e-8, &req, 0, &rows, &n, &dense, &n_dense,
                                               &reason, nullptr, &err);
    else
        rc = BCLIBCFFI_integrate_dense_shot(&shot, &req, 0, &rows, &n, &dense, &n_dense, &reason, &err);
    if (rc != BCLIBCFFI_OK)
    {
        std::fprintf(stderr, "  %s: rc %d: %s\n", what, (int)rc, err.message);
        CHECK(rc == BCLIBCFFI_OK);
        return;
    }
    bool ok = (n == n_want) && reason == want_reason;
    for (int i = 0; ok && i < n; ++i)
        ok = rows[i].flag == want[i].flag && std::fabs(rows[i].time - want[i].time) <= (i == 0 ? 1e-12 : time_tol);
    if (!ok)
    {
        std::fprintf(stderr, "  %s: reason %d (want %d), %d rows (want %d):", what, (int)reason, (int)want_reason, (int)n,
                     n_want);
        for (int i = 0; i < n; ++i)
            std::fprintf(stderr, " [flag %d t=%.6f]", (int)rows[i].flag, rows[i].time);
        std::fprintf(stderr, "\n");
    }
    CHECK(ok);
    BCLIBCFFI_free_trajectory(rows);
    BCLIBCFFI_free_base_trajectory(dense);
}

struct Method
{
    const char *name;
    int32_t id;
    bool adaptive;
    double dt; // first step of the Python reference engine (fixed step); 0 where it has none
};
static const Method kMethods[] = {
    {"rk4", BCLIBCFFI_INTEGRATION_RK4, false, 0.0025},
    {"euler", BCLIBCFFI_INTEGRATION_EULER, false, 0.000192},
    {"verlet", BCLIBCFFI_INTEGRATION_VELOCITY_VERLET, false, 0.001},
    {"cash_karp", BCLIBCFFI_INTEGRATION_CASH_KARP, true, 0.0},
    {"dormand_prince", BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, true, 0.0},
};

static void test_start_outside(const char *limit, double sight_ft, double elevation_deg, void (*set)(BCLIBCFFI_Shot &),
                               int32_t reason)
{
    for (const Method &m : kMethods)
    {
        BCLIBCFFI_Shot s = make_shot(m.id, sight_ft, elevation_deg);
        set(s);
        char what[96];
        std::snprintf(what, sizeof what, "%s on %s", limit, m.name);
        // The terminal row is one accepted step after the start: exactly dt for a fixed step, some positive time
        // (below a tenth of a second) for an adaptive one.
        const Row want[] = {{BCLIBCFFI_TRAJ_FLAG_RANGE, 0.0}, {BCLIBCFFI_TRAJ_FLAG_NONE, m.adaptive ? 0.05 : m.dt}};
        expect_rows(what, s, m.adaptive, want, 2, reason, m.adaptive ? 0.0499 : 1e-5);
    }
}

// Minimum velocity above the muzzle velocity: the shot starts below the limit.
static void min_velocity(BCLIBCFFI_Shot &s) { s.config.cMinimumVelocity = 6000.0; }
// Minimum altitude of 0 with the bore 0.2 ft above the sight line start (the bullet starts at -0.2 ft, falling).
static void min_altitude(BCLIBCFFI_Shot &s) { s.config.cMinimumAltitude = 0.0; }
// Maximum drop of 0: the limit is the start height itself (the shot is not outside it, it falls through it).
static void max_drop(BCLIBCFFI_Shot &s) { s.config.cMaximumDrop = 0.0; }

int main()
{
    // enum values of the BCLIBCFFI_TerminationReason: see the header
    test_start_outside("minimum velocity", 0.2, 0.0, min_velocity, BCLIBCFFI_TERM_MINIMUM_VELOCITY_REACHED);
    test_start_outside("minimum velocity, elevated", 0.2, 1.0, min_velocity, BCLIBCFFI_TERM_MINIMUM_VELOCITY_REACHED);
    test_start_outside("minimum altitude", 0.2, 0.0, min_altitude, BCLIBCFFI_TERM_MINIMUM_ALTITUDE_REACHED);
    test_start_outside("maximum drop", 0.2, 0.0, max_drop, BCLIBCFFI_TERM_MAXIMUM_DROP_REACHED);
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_start_outside_limit: all checks passed");
    return EXIT_SUCCESS;
}
