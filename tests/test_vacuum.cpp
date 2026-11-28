// test_vacuum.cpp -- a vacuum (pressure 0) is drag-free flight, and a state that goes non-finite stops with an error.
//
// A vacuum atmosphere (BCLIBCFFI_Shot::pressure_hpa == 0) has density ratio 0 at every altitude. The density-ratio-for-altitude
// computation divided by the base pressure, 0 / 0 = NaN once the shot was more than 30 ft from the muzzle altitude; the
// integrators carried the NaN, no terminator ever compared true, and integrate / find_apex / find_max_range / every zero
// search never returned (also Cash-Karp, Dormand-Prince and Tsitouras).
//
// Part 1: the vacuum flight is the closed-form parabola (g from the config, the muzzle velocity components): the rows of
// integrate, the apex, the maximum range (45 degrees, v^2 / g on level ground), a zero angle -- RK4, Euler, Verlet and the
// three adaptive methods, through the C ABI.
// Part 2: a state that becomes NaN by another route (a gravity or an atmosphere field that is NaN, a NaN muzzle velocity)
// is stopped by the finite-state check of the integration: an error, in every entry point, not a loop. These calls run
// with no time limit in the test: before the check they never returned.

#include "bclibc/base_types.hpp"
#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

static int failures = 0;

#define CHECK(cond)                                                                    \
    do                                                                                 \
    {                                                                                  \
        if (!(cond))                                                                   \
        {                                                                              \
            std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            ++failures;                                                                \
        }                                                                              \
    } while (0)

static const double kMach[] = {0.0, 0.8, 1.2, 3.0};
static const double kCd[] = {0.26, 0.30, 0.55, 0.45};

static const double kV = 800.0;                   // muzzle velocity (fps)
static const double kG = 32.17405;                // |gravity| (ft/s^2), the config's
static const double kElev = 30.0 * M_PI / 180.0;  // rises 2486 ft and flies 17.2 kft: far more than 30 ft

static BCLIBCFFI_Shot make_shot(int32_t method, double pressure_hpa = 0.0)
{
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = 0.243;
    s.weight_grain = 175.0;
    s.diameter_inch = 0.3;
    s.length_inch = 1.26;
    s.muzzle_velocity_fps = kV;
    s.sight_height_ft = 0.0;
    s.twist_inch = 10.0;
    s.temp_c = 15.0;
    s.pressure_hpa = pressure_hpa;
    s.altitude_ft = 0.0;
    s.humidity = 0.0;
    s.mach_data = kMach;
    s.cd_data = kCd;
    s.drag_table_size = 4;
    s.barrel_elevation_rad = kElev;
    s.latitude_deg = std::nan("");
    s.azimuth_deg = std::nan("");
    s.config.cStepMultiplier = 1.0;
    s.config.cZeroFindingAccuracy = 5e-6;
    s.config.cMinimumVelocity = 50.0;
    s.config.cMaximumDrop = -15000.0;
    s.config.cMaxIterations = 40;
    s.config.cGravityConstant = -kG;
    s.config.cMinimumAltitude = -1500.0;
    s.method = static_cast<BCLIBCFFI_IntegrationMethod>(method);
    return s;
}

struct Engine
{
    const char *name;
    int32_t method;
    bool adaptive;
    double step_multiplier; // the fixed-step engines: Euler is first order, a small step brings it to its limit
    double tol_ft;          // the closed-form tolerance of the rows and the apex
};

static const Engine kEngines[] = {
    {"rk4", BCLIBCFFI_INTEGRATION_RK4, false, 1.0, 1e-3},
    {"verlet", BCLIBCFFI_INTEGRATION_VELOCITY_VERLET, false, 1.0, 1e-3},
    // Euler is first order (its step is 0.5 ft of range, 0.6 ms here): its error is about g * dt * t / 2, 0.5 ft at the
    // end of this flight; its checks use that stated bound, not the 1e-3 ft of the methods that are exact for a parabola.
    {"euler", BCLIBCFFI_INTEGRATION_EULER, false, 1.0, 1.5},
    {"cash_karp", BCLIBCFFI_INTEGRATION_CASH_KARP, true, 1.0, 1e-3},
    {"dormand_prince", BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, true, 1.0, 1e-3},
    {"tsitouras", BCLIBCFFI_INTEGRATION_TSITOURAS, true, 1.0, 1e-3},
};

static const double kRelTol = 1e-9, kAbsTol = 1e-9;

static int32_t do_integrate(const Engine &e, const BCLIBCFFI_Shot &s, const BCLIBCFFI_TrajectoryRequest &req,
                            BCLIBCFFI_TrajectoryData **recs, int32_t *n, BCLIBCFFI_Error *err)
{
    int32_t reason = 0;
    if (e.adaptive)
    {
        int32_t dn = 0;
        return BCLIBCFFI_integrate_adaptive_shot(&s, e.method, kRelTol, kAbsTol, &req, 0, recs, n, nullptr, &dn,
                                                 &reason, nullptr, err);
    }
    return BCLIBCFFI_integrate_shot(&s, &req, recs, n, &reason, err);
}

static int32_t do_apex(const Engine &e, const BCLIBCFFI_Shot &s, BCLIBCFFI_TrajectoryData *out, BCLIBCFFI_Error *err)
{
    if (e.adaptive)
        return BCLIBCFFI_find_apex_adaptive_shot(&s, e.method, kRelTol, kAbsTol, out, nullptr, err);
    return BCLIBCFFI_find_apex_shot(&s, out, err);
}

static int32_t do_max_range(const Engine &e, const BCLIBCFFI_Shot &s, BCLIBCFFI_MaxRangeResult *out,
                            BCLIBCFFI_Error *err)
{
    if (e.adaptive)
        return BCLIBCFFI_find_max_range_adaptive_shot(&s, e.method, kRelTol, kAbsTol, 0.0, 90.0, out, nullptr, err);
    return BCLIBCFFI_find_max_range_shot(&s, 0.0, 90.0, out, err);
}

static int32_t do_zero(const Engine &e, const BCLIBCFFI_Shot &s, double d_ft, double *angle, BCLIBCFFI_Error *err)
{
    if (e.adaptive)
        return BCLIBCFFI_find_zero_angle_adaptive_shot(&s, e.method, kRelTol, kAbsTol, d_ft, angle, nullptr, err);
    return BCLIBCFFI_find_zero_angle_shot(&s, d_ft, angle, err);
}

// ---------------------------------------------------------------------------------------------------------------------
// Part 1: the closed forms
// ---------------------------------------------------------------------------------------------------------------------

static void test_vacuum_rows(const Engine &e)
{
    BCLIBCFFI_Shot s = make_shot(e.method);
    s.config.cStepMultiplier = e.step_multiplier;
    BCLIBCFFI_TrajectoryRequest req{12000.0, 1000.0, 0.0, BCLIBCFFI_TRAJ_FLAG_RANGE};
    BCLIBCFFI_TrajectoryData *recs = nullptr;
    int32_t n = 0;
    BCLIBCFFI_Error err;
    std::memset(&err, 0, sizeof err);

    const int32_t rc = do_integrate(e, s, req, &recs, &n, &err);
    if (rc != BCLIBCFFI_OK)
        std::fprintf(stderr, "  %s integrate rc %d: %s\n", e.name, (int)rc, err.message);
    CHECK(rc == BCLIBCFFI_OK);
    if (rc != BCLIBCFFI_OK) return;
    CHECK(n >= 12);

    const double vx = kV * std::cos(kElev), vy = kV * std::sin(kElev);
    double worst = 0.0;
    int rows_high = 0;
    for (int i = 0; i < n; ++i)
    {
        const BCLIBCFFI_TrajectoryData &r = recs[i];
        CHECK(std::isfinite(r.time) && std::isfinite(r.distance_ft) && std::isfinite(r.height_ft) &&
              std::isfinite(r.velocity_fps) && std::isfinite(r.mach) && std::isfinite(r.energy_ft_lb));
        CHECK(r.density_ratio == 0.0); // a vacuum
        // The parabola at the row's own time: x = vx t, y = vy t - g t^2 / 2.
        const double x = vx * r.time, y = vy * r.time - 0.5 * kG * r.time * r.time;
        const double dx = std::fabs(r.distance_ft - x), dy = std::fabs(r.height_ft - y);
        worst = std::fmax(worst, std::fmax(dx, dy));
        if (r.height_ft > 30.0) ++rows_high;
        // Drag-free: the horizontal speed is constant, the speed follows the vertical one.
        const double speed = std::sqrt(vx * vx + (vy - kG * r.time) * (vy - kG * r.time));
        CHECK(std::fabs(r.velocity_fps - speed) < 10.0 * e.tol_ft);
    }
    if (worst >= e.tol_ft)
        std::fprintf(stderr, "  %s: rows deviate from the parabola by %.6f ft (limit %.4f)\n", e.name, worst, e.tol_ft);
    CHECK(worst < e.tol_ft);
    CHECK(rows_high >= 6); // the shot really is more than 30 ft off the muzzle altitude, up and down
    BCLIBCFFI_free_trajectory(recs);
}

static void test_vacuum_apex(const Engine &e)
{
    BCLIBCFFI_Shot s = make_shot(e.method);
    s.config.cStepMultiplier = e.step_multiplier;
    BCLIBCFFI_TrajectoryData apex;
    BCLIBCFFI_Error err;
    std::memset(&apex, 0, sizeof apex);
    std::memset(&err, 0, sizeof err);
    const int32_t rc = do_apex(e, s, &apex, &err);
    if (rc != BCLIBCFFI_OK) std::fprintf(stderr, "  %s apex rc %d: %s\n", e.name, (int)rc, err.message);
    CHECK(rc == BCLIBCFFI_OK);
    if (rc != BCLIBCFFI_OK) return;
    const double vx = kV * std::cos(kElev), vy = kV * std::sin(kElev);
    const double t = vy / kG;
    CHECK(std::isfinite(apex.height_ft) && std::isfinite(apex.distance_ft));
    CHECK(std::fabs(apex.time - t) < e.tol_ft / 10.0 + 1e-3);
    CHECK(std::fabs(apex.height_ft - vy * vy / (2.0 * kG)) < e.tol_ft);
    CHECK(std::fabs(apex.distance_ft - vx * t) < e.tol_ft);
}

static void test_vacuum_max_range(const Engine &e)
{
    BCLIBCFFI_Shot s = make_shot(e.method);
    s.config.cStepMultiplier = e.step_multiplier;
    BCLIBCFFI_MaxRangeResult r;
    BCLIBCFFI_Error err;
    std::memset(&r, 0, sizeof r);
    std::memset(&err, 0, sizeof err);
    const int32_t rc = do_max_range(e, s, &r, &err);
    if (rc != BCLIBCFFI_OK) std::fprintf(stderr, "  %s max range rc %d: %s\n", e.name, (int)rc, err.message);
    CHECK(rc == BCLIBCFFI_OK);
    if (rc != BCLIBCFFI_OK) return;
    CHECK(std::isfinite(r.max_range_ft) && std::isfinite(r.angle_at_max_rad));
    // Level ground: 45 degrees, range v^2 / g. The golden section ends when its bracket is under 1e-6 rad.
    const double range = kV * kV / kG;
    const double angle_tol = e.tol_ft > 1.0 ? 5e-3 : 2e-6; // Euler's own optimum is off 45 deg by its first-order error
    if (std::fabs(r.angle_at_max_rad - M_PI / 4.0) >= angle_tol || std::fabs(r.max_range_ft - range) >= e.tol_ft * 4.0)
        std::fprintf(stderr, "  %s: max range %.4f ft at %.9f rad; closed form %.4f ft at %.9f rad\n", e.name,
                     r.max_range_ft, r.angle_at_max_rad, range, M_PI / 4.0);
    CHECK(std::fabs(r.angle_at_max_rad - M_PI / 4.0) < angle_tol);
    CHECK(std::fabs(r.max_range_ft - range) < e.tol_ft * 4.0);
}

// The elevation that puts the bore (starting sight_height below the line of sight) on the sight line at distance d.
static double closed_form_zero(double sight_ft, double d)
{
    double lo = 0.0, hi = 0.3; // rad: the lower arc
    for (int i = 0; i < 200; ++i)
    {
        const double mid = 0.5 * (lo + hi);
        const double c = std::cos(mid);
        const double y = -sight_ft + d * std::tan(mid) - kG * d * d / (2.0 * kV * kV * c * c);
        if (y < 0.0) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}

static void test_vacuum_zero(const Engine &e)
{
    BCLIBCFFI_Shot s = make_shot(e.method);
    s.config.cStepMultiplier = e.step_multiplier;
    s.sight_height_ft = 0.1 * 3.28084;
    const double d = 300.0 * 3.28084;
    double angle = 0.0;
    BCLIBCFFI_Error err;
    std::memset(&err, 0, sizeof err);
    const int32_t rc = do_zero(e, s, d, &angle, &err);
    if (rc != BCLIBCFFI_OK) std::fprintf(stderr, "  %s zero rc %d: %s\n", e.name, (int)rc, err.message);
    CHECK(rc == BCLIBCFFI_OK);
    if (rc != BCLIBCFFI_OK) return;
    const double want = closed_form_zero(s.sight_height_ft, d);
    // Euler's first-order error moves its zero by about (g dt t / 2) / d in angle
    const double tol = e.tol_ft > 1.0 ? 1e-4 : 1e-6;
    if (std::fabs(angle - want) >= tol)
        std::fprintf(stderr, "  %s: zero %.9f rad, closed form %.9f rad\n", e.name, angle, want);
    CHECK(std::fabs(angle - want) < tol);
}

// ---------------------------------------------------------------------------------------------------------------------
// Part 2: a state that goes non-finite stops with an error
// ---------------------------------------------------------------------------------------------------------------------

struct Bad
{
    const char *what;
    void (*apply)(BCLIBCFFI_Shot &);
};

static const Bad kBad[] = {
    {"gravity NaN", [](BCLIBCFFI_Shot &s) { s.config.cGravityConstant = std::nan(""); }},
    {"pressure NaN", [](BCLIBCFFI_Shot &s) { s.pressure_hpa = std::nan(""); }},
    {"temperature NaN", [](BCLIBCFFI_Shot &s) { s.temp_c = std::nan(""); }},
    {"muzzle velocity NaN", [](BCLIBCFFI_Shot &s) { s.muzzle_velocity_fps = std::nan(""); }},
    {"elevation NaN", [](BCLIBCFFI_Shot &s) { s.barrel_elevation_rad = std::nan(""); }},
};

static void check_error(const char *what, const Engine &e, const char *entry, int32_t rc, const BCLIBCFFI_Error &err)
{
    if (rc == BCLIBCFFI_OK || err.code != rc || err.message[0] == '\0')
        std::fprintf(stderr, "  %s, %s, %s: rc %d, code %d, message \"%s\"\n", what, e.name, entry, (int)rc, (int)err.code,
                     err.message);
    CHECK(rc != BCLIBCFFI_OK);
    CHECK(err.code == rc);
    CHECK(err.message[0] != '\0');
}

static void test_non_finite(const Engine &e)
{
    for (const Bad &b : kBad)
    {
        // Pressure NaN and temperature NaN need a shot that gets above the 30 ft band; the others are bad from the start.
        BCLIBCFFI_Shot s = make_shot(e.method, 1013.25);
        s.config.cStepMultiplier = e.step_multiplier;
        b.apply(s);
        BCLIBCFFI_Error err;

        BCLIBCFFI_TrajectoryRequest req{12000.0, 1000.0, 0.0, BCLIBCFFI_TRAJ_FLAG_RANGE};
        BCLIBCFFI_TrajectoryData *recs = nullptr;
        int32_t n = 0;
        std::memset(&err, 0, sizeof err);
        int32_t rc = do_integrate(e, s, req, &recs, &n, &err);
        check_error(b.what, e, "integrate", rc, err);
        if (rc == BCLIBCFFI_OK) BCLIBCFFI_free_trajectory(recs);

        BCLIBCFFI_TrajectoryData apex;
        std::memset(&err, 0, sizeof err);
        rc = do_apex(e, s, &apex, &err);
        check_error(b.what, e, "find_apex", rc, err);

        // The max-range search sets the elevation itself: a NaN elevation is not an input of it
        if (b.apply != kBad[4].apply)
        {
            BCLIBCFFI_MaxRangeResult mr;
            std::memset(&err, 0, sizeof err);
            rc = do_max_range(e, s, &mr, &err);
            check_error(b.what, e, "find_max_range", rc, err);
        }

        double angle = 0.0;
        std::memset(&err, 0, sizeof err);
        rc = do_zero(e, s, 300.0 * 3.28084, &angle, &err);
        check_error(b.what, e, "find_zero_angle", rc, err);
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Part 3: an inconsistent raw atmosphere (BCLIBCFFI_ShotProps::atmo takes p0 and density_ratio separately): base pressure 0
// with a non-zero density ratio. A vacuum is no air at any altitude, as Python (pressure <= 0 -> density 0); the stored
// density ratio must not apply within 30 ft of the base altitude while 0 applies beyond it.
// ---------------------------------------------------------------------------------------------------------------------

static void test_raw_inconsistent_vacuum()
{
    for (double p0 : {0.0, -5.0})
    {
        const bclibc::BCLIBC_Atmosphere a(59.0, 0.0, p0, 1116.0, 1.0, -60.0);
        for (double alt : {0.0, 10.0, -29.0, 31.0, 500.0, -500.0})
        {
            double rho = -1.0, mach = -1.0;
            a.update_density_factor_and_mach_for_altitude(alt, rho, mach);
            if (rho != 0.0)
                std::fprintf(stderr, "  raw vacuum p0=%g alt=%g: density ratio %g\n", p0, alt, rho);
            CHECK(rho == 0.0);
            CHECK(std::isfinite(mach) && mach > 0.0);
        }
    }
    // Control: a real atmosphere keeps its stored density ratio near the base altitude.
    const bclibc::BCLIBC_Atmosphere air(59.0, 0.0, 1013.25, 1116.0, 1.0, -60.0);
    double rho = 0.0, mach = 0.0;
    air.update_density_factor_and_mach_for_altitude(10.0, rho, mach);
    CHECK(rho == 1.0);
}

int main()
{
    test_raw_inconsistent_vacuum();
    for (const Engine &e : kEngines)
    {
        std::fprintf(stderr, "vacuum: %s\n", e.name);
        test_vacuum_rows(e);
        test_vacuum_apex(e);
        test_vacuum_max_range(e);
        test_vacuum_zero(e);
        test_non_finite(e);
    }

    if (failures != 0)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::fprintf(stderr, "all vacuum checks passed\n");
    return 0;
}
