// test_ffi_adaptive.cpp -- C ABI checks for the adaptive entry points in bclibc_ffi.h.
//
// Exercises status codes, per-call tolerances, step statistics and dense output through the flat C API
// only (no C++ types), using a small synthetic drag table.

#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

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
    // The lofted zero is the higher of the two angles that hit the target. Its search brackets between the angle of
    // the maximum range and 89.9 degrees; the near-vertical edge never gets as far as the target, which used to end
    // the search with "Trajectory too short" (the miss is now measured at the last point of that trajectory).
    BCLIBCFFI_Shot s = make_shot(BCLIBCFFI_INTEGRATION_RK4);
    double angle = -1.0, low = -1.0;
    BCLIBCFFI_Error err;
    CHECK(BCLIBCFFI_find_zero_angle_lofted_shot(&s, 300.0 * 3.28084, 0, &low, &err) == BCLIBCFFI_OK);
    CHECK(std::isfinite(low) && low > 0.0);
    CHECK(BCLIBCFFI_find_zero_angle_lofted_shot(&s, 300.0 * 3.28084, 1, &angle, &err) == BCLIBCFFI_OK);
    CHECK(std::isfinite(angle) && angle > low && angle < 1.5707963267948966);
    {
        // ... and it really is a zero: fired at that angle the shot comes back down through the sight line at 300 m.
        // The solvers fire with no minimum velocity (the shot slows to a crawl at the apex of a near-vertical flight),
        // so the check does too. Near vertical the miss is very sensitive to the angle (2.5 ft per 1e-6 rad).
        double miss = 1e9;
        BCLIBCFFI_Shot hi = make_shot(BCLIBCFFI_INTEGRATION_RK4);
        hi.config.cMinimumVelocity = 0.0;
        BCLIBCFFI_BaseTrajData hit;
        CHECK(BCLIBCFFI_error_at_distance_shot(&hi, angle, 300.0 * 3.28084, 0.0, &miss, &hit, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(miss) < 1.0);
        CHECK(hit.vy < 0.0); // on the way down
    }

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

static void test_all_methods_and_lofted_adaptive()
{
    const int32_t methods[] = {BCLIBCFFI_INTEGRATION_CASH_KARP, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE,
                               BCLIBCFFI_INTEGRATION_TSITOURAS};
    double first = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        BCLIBCFFI_Shot s = make_shot(methods[i]);
        BCLIBCFFI_Error err;
        BCLIBCFFI_StepStats st{0, 0};

        // Zero angle and zero point agree with each other and across the three methods.
        double angle = 0.0;
        CHECK(BCLIBCFFI_find_zero_angle_adaptive_shot(&s, methods[i], 1e-8, 1e-8, 300.0 * 3.28084, &angle, &st,
                                                      &err) == BCLIBCFFI_OK);
        if (i == 0) first = angle;
        CHECK(std::fabs(angle - first) < 1e-4);

        BCLIBCFFI_ZeroPointResult zp;
        st = {0, 0};
        CHECK(BCLIBCFFI_find_zero_point_adaptive_shot(&s, methods[i], 1e-8, 1e-8, 300.0 * 3.28084, &zp, &st,
                                                      &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(zp.angle_rad - angle) < 1e-6);
        CHECK(st.accepted > 0);

        // Lower (lofted == 0) variants match the plain adaptive solvers.
        double lo = 0.0;
        CHECK(BCLIBCFFI_find_zero_angle_lofted_adaptive_shot(&s, methods[i], 1e-8, 1e-8, 300.0 * 3.28084, 0, &lo,
                                                             &st, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(lo - angle) < 1e-6);
        CHECK(BCLIBCFFI_find_zero_point_lofted_adaptive_shot(&s, methods[i], 1e-8, 1e-8, 300.0 * 3.28084, 0, &zp,
                                                             &st, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(zp.angle_rad - angle) < 1e-6);

        // Max range: positive range at a sensible angle.
        BCLIBCFFI_MaxRangeResult mr;
        CHECK(BCLIBCFFI_find_max_range_adaptive_shot(&s, methods[i], 1e-6, 1e-6, 0.0, 90.0, &mr, &st, &err) ==
              BCLIBCFFI_OK);
        CHECK(mr.max_range_ft > 1000.0 && mr.angle_at_max_rad > 0.0 && mr.angle_at_max_rad < 1.6);
    }

    // The lofted (higher) zero with an adaptive method: found, and the step counts are reported.
    BCLIBCFFI_Shot s = make_shot(BCLIBCFFI_INTEGRATION_TSITOURAS);
    BCLIBCFFI_Error err;
    BCLIBCFFI_StepStats st{-1, -1};
    double angle = 0.0, low = 0.0;
    CHECK(BCLIBCFFI_find_zero_angle_lofted_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_TSITOURAS, 1e-6, 1e-6,
                                                         300.0 * 3.28084, 1, &angle, &st, &err) == BCLIBCFFI_OK);
    CHECK(BCLIBCFFI_find_zero_angle_lofted_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_TSITOURAS, 1e-6, 1e-6,
                                                         300.0 * 3.28084, 0, &low, &st, &err) == BCLIBCFFI_OK);
    CHECK(angle > low && angle < 1.5707963267948966);
    CHECK(st.accepted > 0);
}

static void test_invalid_arguments()
{
    BCLIBCFFI_Shot s = make_shot(BCLIBCFFI_INTEGRATION_DORMAND_PRINCE);
    BCLIBCFFI_Error err;
    double angle = 0.0;
    const double d = 300.0 * 3.28084;
    // Unknown method, zero / NaN / negative tolerances.
    CHECK(BCLIBCFFI_find_zero_angle_adaptive_shot(&s, 999, 1e-6, 1e-6, d, &angle, nullptr, &err) ==
          BCLIBCFFI_ERR_GENERIC);
    CHECK(BCLIBCFFI_find_zero_angle_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, 0.0, 1e-6, d, &angle,
                                                  nullptr, &err) == BCLIBCFFI_ERR_GENERIC);
    CHECK(BCLIBCFFI_find_zero_angle_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, std::nan(""), 1e-6, d,
                                                  &angle, nullptr, &err) == BCLIBCFFI_ERR_GENERIC);
    CHECK(BCLIBCFFI_find_zero_angle_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, 1e-6, -1.0, d, &angle,
                                                  nullptr, &err) == BCLIBCFFI_ERR_GENERIC);

    // On failure the integrate outputs are NULL/0, so an unconditional free is safe.
    BCLIBCFFI_TrajectoryRequest req{1000.0, 1000.0, 0.0, 0};
    BCLIBCFFI_TrajectoryData *recs = reinterpret_cast<BCLIBCFFI_TrajectoryData *>(0x1);
    BCLIBCFFI_DenseStep *dense = reinterpret_cast<BCLIBCFFI_DenseStep *>(0x1);
    int32_t n = -1, dn = -1, reason = -1;
    CHECK(BCLIBCFFI_integrate_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_RK4, 1e-6, 1e-6, &req, 1, &recs, &n, &dense,
                                            &dn, &reason, nullptr, &err) == BCLIBCFFI_ERR_GENERIC);
    CHECK(dense == nullptr && dn == 0);
    // Freeing NULL is a no-op.
    BCLIBCFFI_free_base_trajectory(nullptr);
    BCLIBCFFI_free_trajectory(nullptr);
}

static const double kFt = 3.28084;

static void test_error_at_distance_and_range()
{
    const double d = 300.0 * kFt;
    const int32_t methods[] = {BCLIBCFFI_INTEGRATION_RK4, BCLIBCFFI_INTEGRATION_CASH_KARP,
                               BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, BCLIBCFFI_INTEGRATION_TSITOURAS};
    for (int32_t m : methods)
    {
        BCLIBCFFI_Shot s = make_shot(m);
        BCLIBCFFI_Error err;
        double zero = 0.0;
        CHECK(BCLIBCFFI_find_zero_angle_shot(&s, d, &zero, &err) == BCLIBCFFI_OK);

        // At the zero angle the error at the target is ~0, and the hit point lies at the target distance.
        double e = 1.0;
        BCLIBCFFI_BaseTrajData hit;
        std::memset(&hit, 0, sizeof hit);
        CHECK(BCLIBCFFI_error_at_distance_shot(&s, zero, d, 0.0, &e, &hit, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(e) < 1e-2);
        CHECK(std::fabs(hit.px - d) < 1e-6 && hit.time > 0.0);

        // A steeper angle puts the shot higher (positive error), a flatter one lower.
        double up = 0.0, down = 0.0;
        CHECK(BCLIBCFFI_error_at_distance_shot(&s, zero + 0.002, d, 0.0, &up, nullptr, &err) == BCLIBCFFI_OK);
        CHECK(BCLIBCFFI_error_at_distance_shot(&s, zero - 0.002, d, 0.0, &down, nullptr, &err) == BCLIBCFFI_OK);
        CHECK(up > e && down < e);

        // The line-of-sight crossing for the zero angle is the zero distance, for every method: the crossing is
        // located on the Hermite interpolant of the step, so large adaptive steps do not cost accuracy.
        double range = 0.0;
        CHECK(BCLIBCFFI_range_for_angle_shot(&s, zero, &range, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(range - d) < 0.1);

        // Adaptive forms agree with the fixed-step ones to the tolerance, and report steps.
        if (m != BCLIBCFFI_INTEGRATION_RK4)
        {
            BCLIBCFFI_StepStats st{0, 0};
            double ea = 1.0, ra = 0.0;
            CHECK(BCLIBCFFI_error_at_distance_adaptive_shot(&s, m, 1e-9, 1e-9, zero, d, 0.0, &ea, nullptr, &st,
                                                            &err) == BCLIBCFFI_OK);
            CHECK(std::fabs(ea) < 1e-2 && st.accepted > 0);
            st = {0, 0};
            CHECK(BCLIBCFFI_range_for_angle_adaptive_shot(&s, m, 1e-9, 1e-9, zero, &ra, &st, &err) == BCLIBCFFI_OK);
            CHECK(std::fabs(ra - d) < 0.1 && st.accepted > 0);
        }
    }

    // A target the shot never reaches (it drops below the minimum velocity first): the miss is measured at the last
    // point of the trajectory -- a finite, very negative error (short), and that point is the hit.
    BCLIBCFFI_Shot s = make_shot(BCLIBCFFI_INTEGRATION_RK4);
    BCLIBCFFI_Error err;
    double e = 0.0;
    BCLIBCFFI_BaseTrajData last;
    std::memset(&last, 0, sizeof last);
    CHECK(BCLIBCFFI_error_at_distance_shot(&s, 0.01, 1e7, 0.0, &e, &last, &err) == BCLIBCFFI_OK);
    CHECK(std::isfinite(e) && e < -1e6);
    CHECK(last.time > 0.0 && last.px > 0.0 && last.px < 1e7);
    BCLIBCFFI_StepStats st{-1, -1};
    e = 0.0;
    CHECK(BCLIBCFFI_error_at_distance_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_TSITOURAS, 1e-6, 1e-6, 0.01, 1e7, 0.0,
                                                    &e, nullptr, &st, &err) == BCLIBCFFI_OK);
    CHECK(std::isfinite(e) && e < -1e6);
    CHECK(st.accepted > 0);

    // Invalid adaptive arguments are rejected like everywhere else.
    CHECK(BCLIBCFFI_error_at_distance_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_RK4, 1e-6, 1e-6, 0.01, d, 0.0, &e,
                                                    nullptr, nullptr, &err) == BCLIBCFFI_ERR_GENERIC);
    CHECK(BCLIBCFFI_range_for_angle_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_TSITOURAS, -1.0, 1e-6, 0.01, &e,
                                                  nullptr, &err) == BCLIBCFFI_ERR_GENERIC);
}

static void test_newton_and_zero_init()
{
    const double d = 300.0 * kFt;
    BCLIBCFFI_Error err;
    for (int32_t m : {BCLIBCFFI_INTEGRATION_RK4, BCLIBCFFI_INTEGRATION_TSITOURAS})
    {
        BCLIBCFFI_Shot s = make_shot(m);
        double ref = 0.0, newton = 0.0;
        CHECK(BCLIBCFFI_find_zero_angle_shot(&s, d, &ref, &err) == BCLIBCFFI_OK);
        CHECK(BCLIBCFFI_find_zero_angle_newton_shot(&s, d, &newton, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(newton - ref) < 1e-6);
    }
    BCLIBCFFI_Shot a = make_shot(BCLIBCFFI_INTEGRATION_DORMAND_PRINCE);
    BCLIBCFFI_StepStats st{0, 0};
    double ref = 0.0, newton = 0.0;
    CHECK(BCLIBCFFI_find_zero_angle_adaptive_shot(&a, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, 1e-9, 1e-9, d, &ref, nullptr,
                                                  &err) == BCLIBCFFI_OK);
    CHECK(BCLIBCFFI_find_zero_angle_newton_adaptive_shot(&a, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, 1e-9, 1e-9, d,
                                                         &newton, &st, &err) == BCLIBCFFI_OK);
    CHECK(std::fabs(newton - ref) < 1e-6 && st.accepted > 0);
    CHECK(BCLIBCFFI_find_zero_angle_newton_adaptive_shot(&a, 0, 1e-9, 1e-9, d, &newton, nullptr, &err) ==
          BCLIBCFFI_ERR_GENERIC);

    // zero_init: an ordinary shot needs a search; the target is d ahead, the muzzle sits a sight height below.
    BCLIBCFFI_Shot s = make_shot(BCLIBCFFI_INTEGRATION_RK4);
    BCLIBCFFI_ZeroInit zi;
    std::memset(&zi, 0xFF, sizeof zi);
    CHECK(BCLIBCFFI_zero_init_shot(&s, d, &zi, &err) == BCLIBCFFI_OK);
    CHECK(zi.status == BCLIBCFFI_ZERO_INIT_CONTINUE && zi.reserved == 0);
    CHECK(std::fabs(zi.slant_range_ft - d) < 1e-9 && std::fabs(zi.target_x_ft - d) < 1e-9);
    CHECK(std::fabs(zi.target_y_ft) < 1e-9 && std::fabs(zi.look_angle_rad) < 1e-12);
    CHECK(std::fabs(zi.start_height_ft + 0.15) < 1e-9);
    // A target at the muzzle is done at once.
    CHECK(BCLIBCFFI_zero_init_shot(&s, 1e-6, &zi, &err) == BCLIBCFFI_OK);
    CHECK(zi.status == BCLIBCFFI_ZERO_INIT_DONE);
    st = {0, 0};
    CHECK(BCLIBCFFI_zero_init_adaptive_shot(&a, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, 1e-6, 1e-6, d, &zi, &st, &err) ==
          BCLIBCFFI_OK);
    CHECK(zi.status == BCLIBCFFI_ZERO_INIT_CONTINUE);
    // A vertical shot whose apex is below the target: out of range.
    BCLIBCFFI_Shot v = make_shot(BCLIBCFFI_INTEGRATION_RK4);
    v.look_angle_rad = 1.5707963267948966;
    v.barrel_elevation_rad = 1.5707963267948966;
    CHECK(BCLIBCFFI_zero_init_shot(&v, 1e7, &zi, &err) == BCLIBCFFI_ERR_OUT_OF_RANGE);
}

static void test_dense_helpers()
{
    BCLIBCFFI_Shot s = make_shot(BCLIBCFFI_INTEGRATION_TSITOURAS);
    BCLIBCFFI_TrajectoryRequest req{1000.0 * kFt, 100.0 * kFt, 0.0, 0};
    BCLIBCFFI_TrajectoryData *recs = nullptr;
    BCLIBCFFI_DenseStep *dense = nullptr;
    int32_t n = 0, dn = 0, reason = 0;
    BCLIBCFFI_Error err;
    s.barrel_elevation_rad = 0.004;
    CHECK(BCLIBCFFI_integrate_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_TSITOURAS, 1e-9, 1e-9, &req, 1, &recs, &n,
                                            &dense, &dn, &reason, nullptr, &err) == BCLIBCFFI_OK);
    CHECK(dense != nullptr && dn > 4);
    if (dense && dn > 4)
    {
        // Interpolating in the dense stream at a time inside it matches integrate_at at that time.
        const double t = 0.5;
        BCLIBCFFI_DenseStep at;
        CHECK(BCLIBCFFI_dense_get_at(dense, dn, BCLIBCFFI_INTERP_KEY_TIME, t, 0.0, &at, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(at.time - t) < 1e-12);
        BCLIBCFFI_Interception hit;
        CHECK(BCLIBCFFI_integrate_at_adaptive_shot(&s, BCLIBCFFI_INTEGRATION_TSITOURAS, 1e-9, 1e-9, 0, t, &hit,
                                                   nullptr, &err) == BCLIBCFFI_OK);
        // Both interpolate the same stream, from slightly different brackets: they agree to a few millimetres.
        CHECK(std::fabs(at.px - hit.raw_data.px) < 0.05 && std::fabs(at.py - hit.raw_data.py) < 0.01);

        // An exact step is returned as it is.
        BCLIBCFFI_DenseStep exact;
        CHECK(BCLIBCFFI_dense_get_at(dense, dn, BCLIBCFFI_INTERP_KEY_TIME, dense[2].time, 0.0, &exact, &err) ==
              BCLIBCFFI_OK);
        CHECK(exact.px == dense[2].px && exact.vy == dense[2].vy);

        // By position, by slant height (the muzzle is 0.15 ft below the sight line, so it crosses 0 going up).
        BCLIBCFFI_DenseStep byx;
        CHECK(BCLIBCFFI_dense_get_at(dense, dn, BCLIBCFFI_INTERP_KEY_POS_X, 500.0, 0.0, &byx, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(byx.px - 500.0) < 1e-9);
        BCLIBCFFI_DenseStep slant;
        CHECK(BCLIBCFFI_dense_get_at_slant_height(dense, dn, 0.0, 0.0, &slant, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(slant.py) < 1e-6);

        // A value outside the stream is an interception error, not a crash.
        BCLIBCFFI_DenseStep none;
        CHECK(BCLIBCFFI_dense_get_at(dense, dn, BCLIBCFFI_INTERP_KEY_TIME, 1e6, 0.0, &none, &err) !=
              BCLIBCFFI_OK);
        // Empty input is rejected.
        CHECK(BCLIBCFFI_dense_get_at(nullptr, 0, BCLIBCFFI_INTERP_KEY_TIME, 0.5, 0.0, &none, &err) ==
              BCLIBCFFI_ERR_GENERIC);
        CHECK(BCLIBCFFI_dense_get_at_slant_height(dense, 0, 0.0, 0.0, &none, &err) == BCLIBCFFI_ERR_GENERIC);

        // A step becomes a full record: it matches the record the integrate call itself produced at that range
        // (3 x 100 m = 984.252 ft is one of the requested rows).
        const double row_ft = 3.0 * 100.0 * kFt;
        BCLIBCFFI_DenseStep byrow;
        CHECK(BCLIBCFFI_dense_get_at(dense, dn, BCLIBCFFI_INTERP_KEY_POS_X, row_ft, 0.0, &byrow, &err) == BCLIBCFFI_OK);
        BCLIBCFFI_TrajectoryData full;
        CHECK(BCLIBCFFI_trajectory_data_from_step_shot(&s, &byrow, BCLIBCFFI_TRAJ_FLAG_RANGE, &full, &err) ==
              BCLIBCFFI_OK);
        CHECK(full.flag == BCLIBCFFI_TRAJ_FLAG_RANGE);
        CHECK(std::fabs(full.distance_ft - row_ft) < 1e-9 && std::fabs(full.time - byrow.time) < 1e-12);
        CHECK(full.velocity_fps > 0.0 && full.energy_ft_lb > 0.0);
        bool matched = false;
        for (int32_t i = 0; i < n; ++i)
        {
            if (std::fabs(recs[i].distance_ft - row_ft) < 1e-6)
            {
                matched = true;
                CHECK(std::fabs(recs[i].height_ft - full.height_ft) < 0.01);
                CHECK(std::fabs(recs[i].velocity_fps - full.velocity_fps) < 0.5);
            }
        }
        CHECK(matched);
    }
    BCLIBCFFI_free_base_trajectory(dense);
    BCLIBCFFI_free_trajectory(recs);
}

static void test_layout()
{
    // BCLIBCFFI_get_layout() lists the structs in the order they joined the ABI, the newest at the end: StepStats and
    // ZeroInit are the last 3 + 8 values, and nothing before them moved. Readers that check the exact count (js-ballistics,
    // the web layer of dart-bclibc) have to follow it.
    int32_t buf[256];
    const int32_t count = BCLIBCFFI_get_layout(buf, 256);
    CHECK(count == 96);
    CHECK(BCLIBCFFI_get_layout(buf, count - 1) == -1); // one value short: refused, nothing half-written is "success"
    CHECK(BCLIBCFFI_get_layout(buf, 85) == -1);        // the size of the layout before StepStats and ZeroInit joined it
    CHECK(BCLIBCFFI_get_layout(buf, count) == count);
    // the tail: StepStats (size, accepted, rejected), ZeroInit (size, 7 fields)
    CHECK(buf[85] == (int32_t)sizeof(BCLIBCFFI_StepStats) && buf[86] == 0 && buf[87] == 4);
    CHECK(buf[88] == (int32_t)sizeof(BCLIBCFFI_ZeroInit) && buf[89] == 0 && buf[90] == 4 && buf[91] == 8 && buf[92] == 16 &&
          buf[93] == 24 && buf[94] == 32 && buf[95] == 40);
    // and the head is still where readers built before expect it: Config first, its size then its first offset
    CHECK(buf[0] == (int32_t)sizeof(BCLIBCFFI_Config) && buf[1] == 0);
}

int main()
{
    test_status_codes();
    test_tolerance_and_stats();
    test_dense_output();
    test_zero_and_apex();
    test_fixed_dense();
    test_lofted_and_integrate_at();
    test_all_methods_and_lofted_adaptive();
    test_invalid_arguments();
    test_error_at_distance_and_range();
    test_newton_and_zero_init();
    test_dense_helpers();
    test_layout();
    if (failures)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("bclibc_test_ffi_adaptive: all checks passed");
    return EXIT_SUCCESS;
}
