// test_ffi_fixed_stats.cpp -- C ABI checks for BCLIBCFFI_integrate_dense_shot_stats() in bclibc_ffi.h.
//
// The fixed-step integration with its step count: the same records as BCLIBCFFI_integrate_dense_shot(), the step count
// in out_stats->accepted (no step rejected), and the generic status for an adaptive method.

#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstdio>
#include <cstring>

static int failures = 0;

#define CHECK(cond)                                                                 \
    do                                                                              \
    {                                                                               \
        if (!(cond))                                                                \
        {                                                                           \
            std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++failures;                                                             \
        }                                                                           \
    } while (0)

static const double kMach[] = {0.0, 0.8, 1.2, 3.0};
static const double kCd[] = {0.30, 0.32, 0.45, 0.35};

static BCLIBCFFI_Shot make_shot(int32_t method)
{
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = 0.4;
    s.weight_grain = 175.0;
    s.diameter_inch = 0.308;
    s.length_inch = 1.2;
    s.muzzle_velocity_fps = 2700.0;
    s.sight_height_ft = 0.15;
    s.twist_inch = 10.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.mach_data = kMach;
    s.cd_data = kCd;
    s.drag_table_size = 4;
    s.latitude_deg = std::nan("");
    s.azimuth_deg = std::nan("");
    s.config.cStepMultiplier = 1.0;
    s.config.cZeroFindingAccuracy = 5e-6;
    s.config.cMinimumVelocity = 50.0;
    s.config.cMaximumDrop = -10000.0;
    s.config.cMaxIterations = 40;
    s.config.cGravityConstant = -32.17405;
    s.config.cMinimumAltitude = -1500.0;
    s.method = static_cast<BCLIBCFFI_IntegrationMethod>(method);
    return s;
}

static void test_same_records_and_step_count(int32_t method)
{
    BCLIBCFFI_Shot s = make_shot(method);
    BCLIBCFFI_TrajectoryRequest req{1000.0, 1000.0, 0.0, 0};
    BCLIBCFFI_TrajectoryData *recs_a = nullptr, *recs_b = nullptr;
    BCLIBCFFI_DenseStep *dense_a = nullptr, *dense_b = nullptr;
    int32_t n_a = -1, n_b = -1, dn_a = -1, dn_b = -1, reason_a = -1, reason_b = -1;
    BCLIBCFFI_StepStats stats{-1, -1};
    BCLIBCFFI_Error err_a, err_b;

    const int32_t st_a = BCLIBCFFI_integrate_dense_shot(&s, &req, 1, &recs_a, &n_a, &dense_a, &dn_a, &reason_a, &err_a);
    const int32_t st_b = BCLIBCFFI_integrate_dense_shot_stats(&s, &req, 1, &recs_b, &n_b, &dense_b, &dn_b, &reason_b,
                                                              &stats, &err_b);
    CHECK(st_a == BCLIBCFFI_OK);
    CHECK(st_b == BCLIBCFFI_OK);
    CHECK(n_a == n_b && n_a > 1);
    CHECK(dn_a == dn_b);
    CHECK(reason_a == reason_b);
    CHECK(stats.accepted > 0);     // the steps of the call
    CHECK(stats.rejected == 0);    // a fixed-step method rejects none
    if (recs_a && recs_b && n_a == n_b)
    {
        // field by field: the struct has padding after its int32 flag, which is not written
        bool same = true;
        for (int32_t i = 0; i < n_a; ++i)
        {
            const BCLIBCFFI_TrajectoryData &a = recs_a[i], &b = recs_b[i];
            same = same && a.time == b.time && a.distance_ft == b.distance_ft && a.velocity_fps == b.velocity_fps &&
                   a.mach == b.mach && a.height_ft == b.height_ft && a.slant_height_ft == b.slant_height_ft &&
                   a.drop_angle_rad == b.drop_angle_rad && a.windage_ft == b.windage_ft &&
                   a.windage_angle_rad == b.windage_angle_rad && a.slant_distance_ft == b.slant_distance_ft &&
                   a.angle_rad == b.angle_rad && a.density_ratio == b.density_ratio && a.drag == b.drag &&
                   a.energy_ft_lb == b.energy_ft_lb && a.ogw_lb == b.ogw_lb && a.flag == b.flag;
        }
        CHECK(same);
    }
    BCLIBCFFI_free_trajectory(recs_a);
    BCLIBCFFI_free_trajectory(recs_b);
    BCLIBCFFI_free_base_trajectory(dense_a);
    BCLIBCFFI_free_base_trajectory(dense_b);
}

static void test_adaptive_method_rejected()
{
    BCLIBCFFI_Shot s = make_shot(BCLIBCFFI_INTEGRATION_DORMAND_PRINCE);
    BCLIBCFFI_TrajectoryRequest req{1000.0, 1000.0, 0.0, 0};
    BCLIBCFFI_TrajectoryData *recs = nullptr;
    BCLIBCFFI_DenseStep *dense = nullptr;
    int32_t n = -1, dn = -1, reason = -1;
    BCLIBCFFI_StepStats stats{-1, -1};
    BCLIBCFFI_Error err;
    const int32_t st = BCLIBCFFI_integrate_dense_shot_stats(&s, &req, 0, &recs, &n, &dense, &dn, &reason, &stats, &err);
    CHECK(st == BCLIBCFFI_ERR_GENERIC);
    CHECK(stats.accepted == 0 && stats.rejected == 0);
}

int main()
{
    test_same_records_and_step_count(BCLIBCFFI_INTEGRATION_RK4);
    test_same_records_and_step_count(BCLIBCFFI_INTEGRATION_EULER);
    test_same_records_and_step_count(BCLIBCFFI_INTEGRATION_VELOCITY_VERLET);
    test_adaptive_method_rejected();
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::puts("test_ffi_fixed_stats: ok");
    return 0;
}
