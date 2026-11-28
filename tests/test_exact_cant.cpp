// test_exact_cant.cpp -- the zero, max-range and error searches solve in the exact canted frame (class C37).
//
// A weapon canted by c about the line of sight (look angle L) and pitched by r in its (canted) sight plane points at
//     d = (cos r, sin r cos c, sin r sin c)         (look frame: x forward along the line of sight, y up, z right)
// which in the world frame (x downrange, y up, z right) is
//     x = cos L cos r - sin L sin r cos c,  y = sin L cos r + cos L sin r cos c,  z = sin r sin c,
//     elevation E = atan2(y, hypot(x, z)),  azimuth Az = atan2(z, x).
// c == 0 reduces exactly to E = L + r, Az = 0. The inverse: sin E = R sin(r + phi) with R = hypot(sin L, cos L cos c),
// phi = atan2(sin L, cos L cos c), r = asin(sin E / R) - phi (an elevation with |sin E / R| > 1 is not reachable).
//
// The solvers' unknown is the elevation E of the vertical plane; for every trial elevation they derive the azimuth of the
// canted barrel from this model (Python reference: ShotProps.set_pitch), so the shot that is solved is the shot that is
// fired with (E, Az(E)). Before, the trial shots kept the azimuth of the caller's shot (0), and the zero fired with the azimuth
// of the canted barrel missed (the lofted zero of a canted rifle by about a kilometre).
//
// The test computes (E, Az) here, from the formulas above, and checks: the zeros (low, lofted, the point entries, the Newton
// entry) hit when fired with them; the error of the solver's own trial (BCLIBCFFI_error_at_distance_shot, which is given only
// E) equals the error of the shot fired with the closed-form azimuth; an elevation that is not reachable is clamped to the
// reachable limit; the lofted zero of the canted shot of the report (cant 10, 300 m) is the shot of the Python reference
// (elevation 79.87 deg, azimuth 80.6 deg).
// `--print` prints the values compared with the Python reference.

#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <limits>

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

static const double kPi = 3.14159265358979323846;
static const double kFt = 3.28084;
static double deg(double rad) { return rad * 180.0 / kPi; }
static double rad(double d) { return d * kPi / 180.0; }

struct G7Point
{
    double mach, cd;
};
static const G7Point kG7[] = {
#include "g7_table.inc"
};
static const int kG7Size = static_cast<int>(sizeof kG7 / sizeof kG7[0]);
static double g_mach[sizeof kG7 / sizeof kG7[0]];
static double g_cd[sizeof kG7 / sizeof kG7[0]];

// The shot of the report: G7 bc 0.31, 175 gr, 0.308 in, 800 m/s, sight height 0.05 m, twist 11, no wind.
static BCLIBCFFI_Shot make_shot(int32_t method, double look_deg, double cant_deg)
{
    for (int i = 0; i < kG7Size; ++i)
    {
        g_mach[i] = kG7[i].mach;
        g_cd[i] = kG7[i].cd;
    }
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = 0.31;
    s.weight_grain = 175.0;
    s.diameter_inch = 0.308;
    s.length_inch = 1.24;
    s.muzzle_velocity_fps = 800.0 * kFt;
    s.sight_height_ft = 0.05 * kFt;
    s.twist_inch = 11.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.humidity = 0.0;
    s.mach_data = g_mach;
    s.cd_data = g_cd;
    s.drag_table_size = kG7Size;
    s.winds = nullptr;
    s.wind_count = 0;
    s.look_angle_rad = rad(look_deg);
    s.cant_angle_rad = rad(cant_deg);
    s.latitude_deg = std::numeric_limits<double>::quiet_NaN();
    s.azimuth_deg = std::numeric_limits<double>::quiet_NaN();
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

static const int32_t kMethods[] = {BCLIBCFFI_INTEGRATION_RK4, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE};
static const char *kNames[] = {"rk4", "dopri"};
static bool is_adaptive(int32_t m) { return m >= BCLIBCFFI_INTEGRATION_CASH_KARP; }
static const double kTol = 1e-6; // adaptive tolerances

// ---- the closed-form model (the formulas of the header comment) ------------------------------------------------------

struct Aim
{
    double elevation, azimuth;
};

// (elevation, azimuth) of a barrel pitched r in a sight plane canted c about the line of sight at look L.
static Aim exact_direction(double look, double cant, double r)
{
    const double cc = std::cos(cant), cs = std::sin(cant);
    const double cp = std::cos(r), sp = std::sin(r);
    const double x = std::cos(look) * cp - std::sin(look) * sp * cc;
    const double y = std::sin(look) * cp + std::cos(look) * sp * cc;
    const double z = sp * cs;
    return Aim{std::atan2(y, std::hypot(x, z)), std::atan2(z, x)};
}

// The pitch r that gives the total elevation E; the elevation is clamped to the reachable limit (as the solver does).
static double pitch_from_elevation(double look, double cant, double elev, bool *reachable = nullptr)
{
    const double a = std::sin(look), b = std::cos(look) * std::cos(cant);
    const double big_r = std::hypot(a, b);
    double ratio = std::sin(elev) / big_r;
    if (reachable)
        *reachable = std::fabs(ratio) <= 1.0 + 1e-12;
    if (std::fabs(ratio) > 1.0)
        ratio = std::copysign(1.0, ratio);
    return std::asin(ratio) - std::atan2(a, b);
}

// The azimuth the barrel has when its elevation is E (the model's Az(E)).
static double azimuth_for_elevation(double look, double cant, double elev)
{
    return exact_direction(look, cant, pitch_from_elevation(look, cant, elev)).azimuth;
}

// ---- solver / fire helpers ------------------------------------------------------------------------------------------

static int32_t zero_angle(const BCLIBCFFI_Shot &shot, double d_m, int lofted, double *angle, BCLIBCFFI_Error *err)
{
    std::memset(err, 0, sizeof *err);
    if (is_adaptive(shot.method))
    {
        BCLIBCFFI_StepStats st{0, 0};
        return BCLIBCFFI_find_zero_angle_lofted_adaptive_shot(&shot, shot.method, kTol, kTol, d_m * kFt, lofted, angle, &st, err);
    }
    return BCLIBCFFI_find_zero_angle_lofted_shot(&shot, d_m * kFt, lofted, angle, err);
}

static int32_t zero_point(const BCLIBCFFI_Shot &shot, double d_m, int lofted, BCLIBCFFI_ZeroPointResult *out,
                          BCLIBCFFI_Error *err)
{
    std::memset(err, 0, sizeof *err);
    std::memset(out, 0, sizeof *out);
    if (is_adaptive(shot.method))
    {
        BCLIBCFFI_StepStats st{0, 0};
        return BCLIBCFFI_find_zero_point_lofted_adaptive_shot(&shot, shot.method, kTol, kTol, d_m * kFt, lofted, out, &st, err);
    }
    return BCLIBCFFI_find_zero_point_lofted_shot(&shot, d_m * kFt, lofted, out, err);
}

static int32_t zero_newton(const BCLIBCFFI_Shot &shot, double d_m, double *angle, BCLIBCFFI_Error *err)
{
    std::memset(err, 0, sizeof *err);
    if (is_adaptive(shot.method))
    {
        BCLIBCFFI_StepStats st{0, 0};
        return BCLIBCFFI_find_zero_angle_newton_adaptive_shot(&shot, shot.method, kTol, kTol, d_m * kFt, angle, &st, err);
    }
    return BCLIBCFFI_find_zero_angle_newton_shot(&shot, d_m * kFt, angle, err);
}

// A shot that is fired for a long flight: the limits that end a shot short are lifted (the zero search does the same).
static BCLIBCFFI_Shot lifted(BCLIBCFFI_Shot shot)
{
    shot.config.cMinimumVelocity = 0.0;
    shot.config.cMaximumDrop = -60000.0;
    shot.config.cMinimumAltitude = -60000.0;
    return shot;
}

// The shot fired at elevation E with azimuth Az, where its x reaches the target: the point of the flight.
static bool fire_at(BCLIBCFFI_Shot shot, double elevation, double azimuth, double d_m, BCLIBCFFI_BaseTrajData *hit)
{
    shot.barrel_elevation_rad = elevation;
    shot.barrel_azimuth_rad = azimuth;
    const double tx = d_m * kFt * std::cos(shot.look_angle_rad);
    BCLIBCFFI_Interception ic;
    std::memset(&ic, 0, sizeof ic);
    BCLIBCFFI_Error err;
    std::memset(&err, 0, sizeof err);
    int32_t rc;
    if (is_adaptive(shot.method))
    {
        BCLIBCFFI_StepStats st{0, 0};
        rc = BCLIBCFFI_integrate_at_adaptive_shot(&shot, shot.method, kTol, kTol, BCLIBCFFI_INTERP_KEY_POS_X, tx, &ic, &st, &err);
    }
    else
        rc = BCLIBCFFI_integrate_at_shot(&shot, BCLIBCFFI_INTERP_KEY_POS_X, tx, &ic, &err);
    if (rc != BCLIBCFFI_OK)
        return false;
    *hit = ic.raw_data;
    return true;
}

// Height (ft) above the target of the shot fired at E with the closed-form azimuth of the canted barrel.
static double miss_fired(const BCLIBCFFI_Shot &shot, double elevation, double d_m)
{
    const double look = shot.look_angle_rad, cant = shot.cant_angle_rad;
    BCLIBCFFI_BaseTrajData hit;
    if (!fire_at(lifted(shot), elevation, azimuth_for_elevation(look, cant, elevation), d_m, &hit))
        return 1e9;
    return hit.py - d_m * kFt * std::sin(look);
}

// The error of the solver's own trial at E: it is given only E.
static double error_trial(const BCLIBCFFI_Shot &shot_in, double elevation, double d_m)
{
    BCLIBCFFI_Shot shot = lifted(shot_in);
    const double tx = d_m * kFt * std::cos(shot.look_angle_rad), ty = d_m * kFt * std::sin(shot.look_angle_rad);
    double f = 1e9;
    BCLIBCFFI_Error err;
    std::memset(&err, 0, sizeof err);
    int32_t rc;
    if (is_adaptive(shot.method))
    {
        BCLIBCFFI_StepStats st{0, 0};
        rc = BCLIBCFFI_error_at_distance_adaptive_shot(&shot, shot.method, kTol, kTol, elevation, tx, ty, &f, nullptr, &st, &err);
    }
    else
        rc = BCLIBCFFI_error_at_distance_shot(&shot, elevation, tx, ty, &f, nullptr, &err);
    return rc == BCLIBCFFI_OK ? f : 1e9;
}

static bool g_print = false;

// ---- tests ----------------------------------------------------------------------------------------------------------

// The model itself: c == 0 is E = L + r, Az = 0, and the inverse undoes it.
static void test_model()
{
    for (double look : {-0.2, 0.0, 0.09, 0.5})
        for (double r : {0.0, 0.01, 0.2, 0.7})
        {
            const Aim a = exact_direction(look, 0.0, r);
            CHECK(std::fabs(a.elevation - (look + r)) < 1e-15);
            CHECK(a.azimuth == 0.0);
            for (double cant : {0.0, rad(10.0), rad(45.0), rad(-30.0)})
            {
                const Aim e = exact_direction(look, cant, r);
                CHECK(std::fabs(pitch_from_elevation(look, cant, e.elevation) - r) < 1e-12);
            }
        }
}

// The low zero fired with the closed-form (E, Az) hits; the same for the point entry and the Newton entry.
static void test_low_zero_hits()
{
    const double kHeightTol = 1e-3; // ft
    for (int mi = 0; mi < 2; ++mi)
        for (double cant : {10.0, 45.0})
            for (double look : {0.0, 5.0})
                for (double d : {300.0, 1000.0})
                {
                    const BCLIBCFFI_Shot shot = make_shot(kMethods[mi], look, cant);
                    BCLIBCFFI_Error err;
                    double e_zero = 0.0;
                    const int32_t rc = zero_angle(shot, d, 0, &e_zero, &err);
                    CHECK(rc == BCLIBCFFI_OK);
                    if (rc != BCLIBCFFI_OK)
                    {
                        std::fprintf(stderr, "  %s cant %g look %g d %g: rc %d (%s)\n", kNames[mi], cant, look, d, (int)rc, err.message);
                        continue;
                    }
                    const double miss = miss_fired(shot, e_zero, d);
                    if (g_print)
                        std::printf("low %s cant %g look %g d %g: E %.9f deg, Az %.9f deg, miss %.3e ft\n", kNames[mi], cant, look, d,
                                    deg(e_zero), deg(azimuth_for_elevation(shot.look_angle_rad, shot.cant_angle_rad, e_zero)), miss);
                    if (!(std::fabs(miss) < kHeightTol))
                        std::fprintf(stderr, "  %s cant %g look %g d %g: fired miss %.6f ft\n", kNames[mi], cant, look, d, miss);
                    CHECK(std::fabs(miss) < kHeightTol);

                    double e_newton = 0.0;
                    if (zero_newton(shot, d, &e_newton, &err) == BCLIBCFFI_OK)
                    {
                        const double miss_n = miss_fired(shot, e_newton, d);
                        if (!(std::fabs(miss_n) < kHeightTol))
                            std::fprintf(stderr, "  %s cant %g look %g d %g: newton fired miss %.6f ft\n", kNames[mi], cant, look, d, miss_n);
                        CHECK(std::fabs(miss_n) < kHeightTol);
                    }
                    else
                        CHECK(false);

                    BCLIBCFFI_ZeroPointResult zp;
                    CHECK(zero_point(shot, d, 0, &zp, &err) == BCLIBCFFI_OK);
                    CHECK(std::fabs(zp.angle_rad - e_zero) < 1e-9);
                    CHECK(std::fabs(zp.point.slant_height_ft) < kHeightTol); // the point of the solve is at the target
                }
}

// The error of the solver's own trial (it is given only E) is the error of the shot fired with (E, closed-form Az); an
// elevation beyond the reach of the cant is the clamped one.
static void test_trial_azimuth_is_the_model()
{
    for (int mi = 0; mi < 2; ++mi)
        for (double cant : {10.0, 45.0, -45.0})
            for (double look : {0.0, 5.0})
            {
                const BCLIBCFFI_Shot shot = make_shot(kMethods[mi], look, cant);
                const double d = 400.0;
                for (double e : {0.01, 0.05, 0.1, 0.3, 0.6})
                {
                    const double trial = error_trial(shot, e, d);
                    const double fired = miss_fired(shot, e, d);
                    // |x - target_x| is zero where the point is interpolated at the target
                    const double tol = 1e-9 * (1.0 + std::fabs(fired));
                    if (!(std::fabs(trial - fired) < tol))
                        std::fprintf(stderr, "  %s cant %g look %g E %g: trial %.12f fired %.12f\n", kNames[mi], cant, look, e, trial, fired);
                    CHECK(std::fabs(trial - fired) < tol);
                }
            }
}

static void test_unreachable_is_clamped()
{
    for (double cant : {45.0, 70.0})
        for (double look : {0.0, 5.0})
        {
            const BCLIBCFFI_Shot shot = make_shot(BCLIBCFFI_INTEGRATION_RK4, look, cant);
            const double big_r = std::hypot(std::sin(rad(look)), std::cos(rad(look)) * std::cos(rad(cant)));
            const double e_max = std::asin(big_r);
            const double e_beyond = std::fmin(e_max + 0.3, 1.5);
            bool reachable = true;
            pitch_from_elevation(rad(look), rad(cant), e_beyond, &reachable);
            CHECK(!reachable);
            // a lofted shot fired this steeply is airborne for a minute: compare at a short target (the error is of the
            // clamped shot either way)
            const double f_beyond = error_trial(shot, e_beyond, 100.0);
            const double f_max = error_trial(shot, e_max, 100.0);
            if (!(std::fabs(f_beyond - f_max) < 1e-6 * (1.0 + std::fabs(f_max))))
                std::fprintf(stderr, "  cant %g look %g: beyond %.9f max %.9f\n", cant, look, f_beyond, f_max);
            CHECK(std::fabs(f_beyond - f_max) < 1e-6 * (1.0 + std::fabs(f_max)));
        }
}

// The lofted zero of the canted shot of the report (cant 10, 300 m): the elevation clamps at the reach of the cant and the
// azimuth is large; the shot hits when fired with the model's azimuth. The values are those of the Python reference.
static void test_lofted_zero_with_cant()
{
    for (int mi = 0; mi < 2; ++mi)
    {
        const BCLIBCFFI_Shot shot = make_shot(kMethods[mi], 0.0, 10.0);
        BCLIBCFFI_Error err;
        double e_zero = 0.0;
        const int32_t rc = zero_angle(shot, 300.0, 1, &e_zero, &err);
        CHECK(rc == BCLIBCFFI_OK);
        if (rc != BCLIBCFFI_OK)
        {
            std::fprintf(stderr, "  %s lofted cant 10: rc %d (%s)\n", kNames[mi], (int)rc, err.message);
            continue;
        }
        const double az = azimuth_for_elevation(0.0, rad(10.0), e_zero);
        const double miss_ft = miss_fired(shot, e_zero, 300.0);
        if (g_print)
            std::printf("lofted %s cant 10 d 300: E %.9f deg, Az %.9f deg, miss %.6f m\n", kNames[mi], deg(e_zero), deg(az), miss_ft / kFt);
        if (!(std::fabs(miss_ft / kFt) < 0.05))
            std::fprintf(stderr, "  %s lofted cant 10: fired miss %.4f m\n", kNames[mi], miss_ft / kFt);
        CHECK(std::fabs(miss_ft / kFt) < 0.05);
        CHECK(std::fabs(deg(e_zero) - 79.87) < 0.05); // the Python reference: 79.87 deg
        CHECK(std::fabs(deg(az) - 80.6) < 0.5);       // and 80.6 deg
        CHECK(deg(az) > 79.0 && deg(az) < 82.0);

        BCLIBCFFI_ZeroPointResult zp;
        CHECK(zero_point(shot, 300.0, 1, &zp, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(zp.angle_rad - e_zero) < 1e-9);
        CHECK(std::fabs(zp.point.slant_height_ft / kFt) < 0.05);
    }
}

// The max range of a canted shot is searched over E; it is the range of the shot fired with (E, Az(E)).
static void test_max_range_with_cant()
{
    for (double cant : {10.0, 45.0})
    {
        const BCLIBCFFI_Shot shot = make_shot(BCLIBCFFI_INTEGRATION_RK4, 0.0, cant);
        BCLIBCFFI_MaxRangeResult mr;
        BCLIBCFFI_Error err;
        std::memset(&mr, 0, sizeof mr);
        std::memset(&err, 0, sizeof err);
        CHECK(BCLIBCFFI_find_max_range_shot(&shot, 0.0, 90.0, &mr, &err) == BCLIBCFFI_OK);
        // the reach of the cant bounds the elevation: the range is that of an elevation that can be fired
        const double e_reach = std::asin(std::cos(rad(cant)));
        CHECK(mr.angle_at_max_rad <= e_reach + 1e-4);
        CHECK(mr.max_range_ft > 0.0);
        // range_for_angle at E, with the shot's own aim, agrees with the solver's range at the maximum
        double r = 0.0;
        CHECK(BCLIBCFFI_range_for_angle_shot(&shot, mr.angle_at_max_rad, &r, &err) == BCLIBCFFI_OK);
        CHECK(std::fabs(r - mr.max_range_ft) < 1e-6 * mr.max_range_ft);
        if (g_print)
            std::printf("maxrange cant %g: %.9f ft at %.9f deg\n", cant, mr.max_range_ft, deg(mr.angle_at_max_rad));
    }
}

int main(int argc, char **argv)
{
    g_print = argc > 1 && std::strcmp(argv[1], "--print") == 0;
    test_model();
    test_low_zero_hits();
    test_trial_azimuth_is_the_model();
    test_unreachable_is_clamped();
    test_lofted_zero_with_cant();
    test_max_range_with_cant();
    if (failures != 0)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::puts("test_exact_cant: OK");
    return 0;
}
