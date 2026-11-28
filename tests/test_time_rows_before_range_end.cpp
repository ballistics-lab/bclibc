// test_time_rows_before_range_end.cpp -- the time rows of the last step before the range limit are kept.
//
// RANGE request with a time_step and a range_step, range limit 1500 m, time_step 0.3 s: rk4/euler/verlet keep the time row
// at t = 3.3 s (the last k*0.3 before the 1500 m end row, in the step that crosses the limit); the adaptive engines
// (Cash-Karp, Dormand-Prince, Tsitouras) dropped it: a wide adaptive step holds both the time row and the 1500 m row,
// and the range row restarted the time schedule before the time row was emitted. Checked here: same row count as rk4,
// every row a time row (0.3 s after the previous row, within 1e-9) or a range row, the last time row before the end
// row present, rows within 0.15 m and 1e-3 s of the rk4 row of the same index (the integrators differ ~0.03 m, ~2e-4 s).

#include "bclibc.hpp"
#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

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
static const double kFtPerM = 1.0 / 0.3048;
// rows of the adaptive method against the rk4 row of the same index: 0.15 m (the integrators differ ~0.03 m)
static const double kTolXFt = 0.15 * kFtPerM;

static BCLIBCFFI_Shot make_shot(int32_t method)
{
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = 0.243;
    s.weight_grain = 175.0;
    s.diameter_inch = 0.3;
    s.length_inch = 1.26;
    s.muzzle_velocity_fps = 800.0 * kFtPerM;
    s.sight_height_ft = 0.1;
    s.twist_inch = 10.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.humidity = 0.0;
    s.mach_data = kMachG7;
    s.cd_data = kCdG7;
    s.drag_table_size = 4;
    s.barrel_elevation_rad = 0.15 * M_PI / 180.0;
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
    double time, x, y;
};

static bool run(const BCLIBCFFI_Shot &shot, bool adaptive, double dist_step_m, std::vector<Row> &out)
{
    BCLIBCFFI_TrajectoryRequest req;
    std::memset(&req, 0, sizeof req);
    req.range_limit_ft = 1500.0 * kFtPerM;
    req.range_step_ft = dist_step_m * kFtPerM;
    req.time_step = 0.3;
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
        std::fprintf(stderr, "  rc %d: %s\n", (int)rc, err.message);
        return false;
    }
    for (int i = 0; i < n; ++i)
        out.push_back({rows[i].flag, rows[i].time, rows[i].distance_ft, rows[i].height_ft});
    BCLIBCFFI_free_trajectory(rows);
    BCLIBCFFI_free_base_trajectory(dense);
    return true;
}

static void dump(const char *what, const std::vector<Row> &r)
{
    std::fprintf(stderr, "  %s: %zu rows:", what, r.size());
    for (const Row &x : r)
        std::fprintf(stderr, " [t=%.5f x=%.2fm]", x.time, x.x / kFtPerM);
    std::fprintf(stderr, "\n");
}

static void check_method(const char *name, int32_t id, bool adaptive, double dist_step_m)
{
    std::vector<Row> ref, got;
    const bool ok = run(make_shot(BCLIBCFFI_INTEGRATION_RK4), false, dist_step_m, ref) &&
                    run(make_shot(id), adaptive, dist_step_m, got);
    CHECK(ok);
    if (!ok)
        return;
    const bool same = ref.size() == got.size();
    if (!same)
    {
        std::fprintf(stderr, "%s dist_step %g m: row count differs from rk4\n", name, dist_step_m);
        dump("rk4", ref);
        dump(name, got);
    }
    CHECK(same);
    // Every row after the first is a time row (0.3 s after the previous row, the schedule restarts from a range row) or a
    // range row (x on a multiple of dist_step or the 1500 m end); the time rows lie on that grid within 1e-9.
    int time_rows = 0;
    for (size_t i = 1; i < got.size(); ++i)
    {
        const double m = got[i].x / kFtPerM;
        const double q = std::fabs(m / dist_step_m - std::floor(m / dist_step_m + 0.5)) * dist_step_m;
        const bool range_row = q < 1e-6 || std::fabs(m - 1500.0) < 1e-6;
        const bool time_row = std::fabs(got[i].time - got[i - 1].time - 0.3) < 1e-9;
        CHECK(range_row || time_row);
        time_rows += time_row ? 1 : 0;
    }
    // the last time row before the end row is present: the row before the end row is a time row 0.3 s after its predecessor
    // (dist_step 500: t = 3.98169 after the 3.68169 row; dist_step 1500: t = 3.9 after 3.6) and ends before the end row
    CHECK(got.size() >= 3);
    if (got.size() >= 3)
    {
        const Row &pre = got[got.size() - 2], &end = got.back();
        CHECK(std::fabs(pre.time - got[got.size() - 3].time - 0.3) < 1e-9);
        CHECK(pre.time < end.time && std::fabs(end.x / kFtPerM - 1500.0) < 1e-6);
    }
    CHECK(time_rows >= 10);
    if (same)
        for (size_t i = 0; i < got.size(); ++i)
        {
            CHECK(std::fabs(got[i].x - ref[i].x) <= kTolXFt);
            CHECK(std::fabs(got[i].y - ref[i].y) <= kTolXFt);
            CHECK(std::fabs(got[i].time - ref[i].time) <= 1e-3);
        }
}

int main()
{
    for (const double step : {500.0, 1500.0})
    {
        check_method("rk4", BCLIBCFFI_INTEGRATION_RK4, false, step);
        check_method("euler", BCLIBCFFI_INTEGRATION_EULER, false, step);
        check_method("verlet", BCLIBCFFI_INTEGRATION_VELOCITY_VERLET, false, step);
        check_method("cash_karp", BCLIBCFFI_INTEGRATION_CASH_KARP, true, step);
        check_method("dormand_prince", BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, true, step);
        check_method("tsitouras", BCLIBCFFI_INTEGRATION_TSITOURAS, true, step);
    }
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_time_rows_before_range_end: all checks passed");
    return EXIT_SUCCESS;
}
