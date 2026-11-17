// test_ffi_adaptive.cpp -- C ABI checks for the adaptive entry points in bclibc_ffi.h.
//
// Exercises status codes, per-call tolerances, step statistics and dense output through the flat C API
// only (no C++ types), using a small synthetic drag table.

#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

static int failures = 0;

#define CHECK(cond)                                                    \
    do                                                                 \
    {                                                                  \
        if (!(cond))                                                   \
        {                                                              \
            std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++failures;                                                \
        }                                                              \
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
    s.altitude_ft = 0.0;
    s.humidity = 0.0;
    s.mach_data = kMach;
    s.cd_data = kCd;
    s.drag_table_size = 4;
    s.winds = nullptr;
    s.wind_count = 0;
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

static void test_status_codes()
{
    BCLIBCFFI_Shot s = make_shot(BCLIBCFFI_INTEGRATION_DORMAND_PRINCE);
    BCLIBCFFI_TrajectoryRequest req{1000.0, 1000.0, 0.0, 0};
    BCLIBCFFI_TrajectoryData *recs = nullptr;
    BCLIBCFFI_DenseStep *dense = nullptr;
    int32_t n = -1, dn = -1, reason = -1;
    BCLIBCFFI_Error err;

    // RK4 is not an adaptive method: rejected with the generic status.
    CHECK(BCLIBCFFI_integrate_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_RK4, 1e-6, 1e-6, &req, 0, &recs, &n,
                                            &dense, &dn, &reason, nullptr, &err) == BCLIBCFFI_ERR_GENERIC);
    CHECK(err.code == BCLIBCFFI_ERR_GENERIC);

    // Negative relative tolerance: rejected by the integrator.
    CHECK(BCLIBCFFI_integrate_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, -1.0, 1e-6, &req, 0, &recs,
                                            &n, &dense, &dn, &reason, nullptr, &err) == BCLIBCFFI_ERR_GENERIC);
    CHECK(err.code == BCLIBCFFI_ERR_GENERIC);
}

static void test_tolerance_and_stats()
{
    BCLIBCFFI_Shot s = make_shot(BCLIBCFFI_INTEGRATION_DORMAND_PRINCE);
    BCLIBCFFI_TrajectoryRequest req{2000.0 * 3.28084, 100.0 * 3.28084, 0.0, 31};
    BCLIBCFFI_TrajectoryData *loose_recs = nullptr, *tight_recs = nullptr;
    int32_t loose_n = 0, tight_n = 0, reason = 0, dn = 0;
    BCLIBCFFI_StepStats loose{0, 0}, tight{0, 0};
    BCLIBCFFI_Error err;

    CHECK(BCLIBCFFI_integrate_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, 1e-3, 1e-3, &req, 0,
                                            &loose_recs, &loose_n, nullptr, &dn, &reason, &loose, &err) == BCLIBCFFI_OK);
    CHECK(BCLIBCFFI_integrate_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, 1e-9, 1e-9, &req, 0,
                                            &tight_recs, &tight_n, nullptr, &dn, &reason, &tight, &err) == BCLIBCFFI_OK);
    CHECK(loose.accepted > 0 && tight.accepted > 0);
    CHECK(tight.accepted + tight.rejected > loose.accepted + loose.rejected);
    CHECK(loose_n > 0 && tight_n > 0);
    BCLIBCFFI_free_trajectory(loose_recs);
    BCLIBCFFI_free_trajectory(tight_recs);
}

static void test_dense_output()
{
    BCLIBCFFI_Shot s = make_shot(BCLIBCFFI_INTEGRATION_TSITOURAS);
    BCLIBCFFI_TrajectoryRequest req{1000.0 * 3.28084, 100.0 * 3.28084, 0.0, 0};
    BCLIBCFFI_TrajectoryData *recs = nullptr;
    BCLIBCFFI_DenseStep *dense = nullptr;
    int32_t n = 0, dn = 0, reason = 0;
    BCLIBCFFI_StepStats stats{0, 0};
    BCLIBCFFI_Error err;

    CHECK(BCLIBCFFI_integrate_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_TSITOURAS, 1e-6, 1e-6, &req, 1, &recs, &n,
                                            &dense, &dn, &reason, &stats, &err) == BCLIBCFFI_OK);
    CHECK(dense != nullptr && dn > 2);
    // Dense steps are time-ordered and start at the muzzle.
    if (dense && dn > 1)
    {
        CHECK(dense[0].time == 0.0);
        CHECK(dense[dn - 1].time > dense[0].time);
    }
    // Stats are reported and match the accepted-step stream (one dense point per accepted step, plus start).
    CHECK(stats.accepted > 0);
    CHECK(dn == stats.accepted + 1);
    BCLIBCFFI_free_base_trajectory(dense);
    BCLIBCFFI_free_trajectory(recs);

    // Without dense output the pointer stays NULL.
    CHECK(BCLIBCFFI_integrate_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_TSITOURAS, 1e-6, 1e-6, &req, 0, &recs, &n,
                                            &dense, &dn, &reason, nullptr, &err) == BCLIBCFFI_OK);
    CHECK(dense == nullptr && dn == 0);
    BCLIBCFFI_free_trajectory(recs);
}

static void test_zero_and_apex()
{
    BCLIBCFFI_Shot s = make_shot(BCLIBCFFI_INTEGRATION_CASH_KARP);
    double angle = 0.0;
    BCLIBCFFI_StepStats stats{0, 0};
    BCLIBCFFI_Error err;
    CHECK(BCLIBCFFI_find_zero_angle_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_CASH_KARP, 1e-6, 1e-6, 300.0 * 3.28084,
                                                  &angle, &stats, &err) == BCLIBCFFI_OK);
    CHECK(std::isfinite(angle) && angle > 0.0 && angle < 0.5);
    CHECK(stats.accepted > 0);

    // Apex needs a positive barrel elevation (same rule as BCLIBCFFI_find_apex_shot).
    s.barrel_elevation_rad = 0.02;
    BCLIBCFFI_TrajectoryData apex;
    CHECK(BCLIBCFFI_find_apex_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_CASH_KARP, 1e-6, 1e-6, &apex, nullptr,
                                            &err) == BCLIBCFFI_OK);
    CHECK(apex.height_ft > 0.0);
}


static void test_fixed_dense()
{
    BCLIBCFFI_Shot s = make_shot(BCLIBCFFI_INTEGRATION_RK4);
    BCLIBCFFI_TrajectoryRequest req{1000.0 * 3.28084, 100.0 * 3.28084, 0.0, 0};
    BCLIBCFFI_TrajectoryData *recs = nullptr;
    BCLIBCFFI_DenseStep *dense = nullptr;
    int32_t n = 0, dn = 0, reason = 0;
    BCLIBCFFI_Error err;
    CHECK(BCLIBCFFI_integrate_dense_shot(&s, &req, 1, &recs, &n, &dense, &dn, &reason, &err) == BCLIBCFFI_OK);
    CHECK(dense != nullptr && dn > 2 && n > 0);
    if (dense && dn > 1)
    {
        CHECK(dense[0].time == 0.0);
        CHECK(dense[dn - 1].time > dense[0].time);
    }
    BCLIBCFFI_free_base_trajectory(dense);
    BCLIBCFFI_free_trajectory(recs);

    // Adaptive methods are rejected here.
    BCLIBCFFI_Shot d = make_shot(BCLIBCFFI_INTEGRATION_DORMAND_PRINCE);
    CHECK(BCLIBCFFI_integrate_dense_shot(&d, &req, 0, &recs, &n, nullptr, nullptr, &reason, &err) ==
          BCLIBCFFI_ERR_GENERIC);
    CHECK(err.code == BCLIBCFFI_ERR_GENERIC);
}

static void test_lofted_and_integrate_at()
{
    // The lofted solver reports "Trajectory too short" for the test shots (the Cython engines report the same error
    // for these shots); the entry points must return that status, not crash or return garbage.
    BCLIBCFFI_Shot s = make_shot(BCLIBCFFI_INTEGRATION_RK4);
    double angle = -1.0;
    BCLIBCFFI_Error err;
    CHECK(BCLIBCFFI_find_zero_angle_lofted_shot(&s, 300.0 * 3.28084, 0, &angle, &err) == BCLIBCFFI_OK);
    CHECK(std::isfinite(angle) && angle > 0.0);
    CHECK(BCLIBCFFI_find_zero_angle_lofted_shot(&s, 300.0 * 3.28084, 1, &angle, &err) == BCLIBCFFI_ERR_SOLVER_RUNTIME);
    CHECK(std::strstr(err.message, "too short") != nullptr);

    BCLIBCFFI_ZeroPointResult zp;
    CHECK(BCLIBCFFI_find_zero_point_lofted_shot(&s, 300.0 * 3.28084, 0, &zp, &err) == BCLIBCFFI_OK);
    CHECK(std::isfinite(zp.angle_rad) && zp.angle_rad > 0.0);

    // integrate_at: interpolated raw point at time 0.5 s, fixed-step and adaptive
    BCLIBCFFI_Interception hit;
    CHECK(BCLIBCFFI_integrate_at_shot(&s, 0 /* BCLIBCFFI_INTERP_KEY_TIME */, 0.5, &hit, &err) == BCLIBCFFI_OK);
    CHECK(std::fabs(hit.raw_data.time - 0.5) < 1e-6);

    BCLIBCFFI_Shot a = make_shot(BCLIBCFFI_INTEGRATION_CASH_KARP);
    BCLIBCFFI_StepStats st{0, 0};
    CHECK(BCLIBCFFI_integrate_at_adaptive_shot(&a, BCLIBCFFI_INTEGRATION_CASH_KARP, 1e-8, 1e-8, 0, 0.5, &hit, &st,
                                               &err) == BCLIBCFFI_OK);
    CHECK(std::fabs(hit.raw_data.time - 0.5) < 1e-6);
    CHECK(st.accepted > 0);
}

int main()
{
    test_status_codes();
    test_tolerance_and_stats();
    test_dense_output();
    test_zero_and_apex();
    test_fixed_dense();
    test_lofted_and_integrate_at();
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_ffi_adaptive: all checks passed");
    return EXIT_SUCCESS;
}
