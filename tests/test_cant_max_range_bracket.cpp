// test_cant_max_range_bracket.cpp -- the max-range search of a canted shot brackets only the reachable elevations (class C38).
//
// With a cant the elevation E of the vertical plane that set_aim can reach is |E| <= asin(R), R = hypot(sin L, cos L cos cant)
// (30 deg at cant 60 and a horizontal look). The golden-section search of the maximum range used to bracket (0, 90 deg): its
// first probes (34.4 and 55.6 deg) were both above the limit, set_aim clamped both to the same state, the ranges were equal,
// `yc > yd` was false and the search walked to 90 deg, away from the maximum (about 13 deg for the shot below): a max range of
// ~0 and OutOfRange for a shot whose zero exists. The upper bound is now clipped to asin(R) when cant != 0 (cant == 0 is
// untouched, bit for bit).
// Checks: the close zero (D = 0.1 and 0.2 m, sight height 0.05 m, look 0, cant 60 deg) is found and hits within 1e-3 ft for every
// integration method; for cant 45, 60, 75, 80 the angle of the maximum is <= asin(R) and its range is the maximum of a scan of
// the reachable elevations (range_for_angle with the shot's own aim); cant == 0 is scanned too (its bracket is untouched).
// `--print` prints the values compared with the Python reference.

#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <limits>
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

static bool is_adaptive(int32_t m) { return m >= BCLIBCFFI_INTEGRATION_CASH_KARP; }
static const double kTol = 1e-6; // adaptive tolerances

// ---- the closed-form model (the formulas of the header comment) ------------------------------------------------------

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

// A shot that is fired for a long flight: the limits that end a shot short are lifted (the zero search does the same).
static BCLIBCFFI_Shot lifted(BCLIBCFFI_Shot shot)
{
    shot.config.cMinimumVelocity = 0.0;
    shot.config.cMaximumDrop = -60000.0;
    shot.config.cMinimumAltitude = -60000.0;
    return shot;
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

static const int32_t kMethods[] = {BCLIBCFFI_INTEGRATION_RK4, BCLIBCFFI_INTEGRATION_EULER, BCLIBCFFI_INTEGRATION_VELOCITY_VERLET,
                                   BCLIBCFFI_INTEGRATION_CASH_KARP, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE,
                                   BCLIBCFFI_INTEGRATION_TSITOURAS};
static const char *kNames[] = {"rk4", "euler", "verlet", "cash_karp", "dopri", "tsitouras"};

static int32_t max_range(const BCLIBCFFI_Shot &shot, BCLIBCFFI_MaxRangeResult *mr, BCLIBCFFI_Error *err)
{
    std::memset(mr, 0, sizeof *mr);
    std::memset(err, 0, sizeof *err);
    if (is_adaptive(shot.method))
    {
        BCLIBCFFI_StepStats st{0, 0};
        return BCLIBCFFI_find_max_range_adaptive_shot(&shot, shot.method, kTol, kTol, 0.0, 90.0, mr, &st, err);
    }
    return BCLIBCFFI_find_max_range_shot(&shot, 0.0, 90.0, mr, err);
}

static double range_at(const BCLIBCFFI_Shot &shot_in, double elevation)
{
    // a downhill shot needs the limits that end a shot short lifted (the max-range search lifts them itself)
    const BCLIBCFFI_Shot shot = lifted(shot_in);
    double r = 0.0;
    BCLIBCFFI_Error err;
    std::memset(&err, 0, sizeof err);
    int32_t rc;
    if (is_adaptive(shot.method))
    {
        BCLIBCFFI_StepStats st{0, 0};
        rc = BCLIBCFFI_range_for_angle_adaptive_shot(&shot, shot.method, kTol, kTol, elevation, &r, &st, &err);
    }
    else
        rc = BCLIBCFFI_range_for_angle_shot(&shot, elevation, &r, &err);
    return rc == BCLIBCFFI_OK ? r : -1.0;
}

// The shot of the report with a cant of 60 deg and a close target: its zero exists (about 12.92 deg), the max-range search
// used to fail.
static void test_close_zero_cant60()
{
    for (double d_m : {0.1, 0.2})
        for (size_t k = 0; k < sizeof kMethods / sizeof kMethods[0]; ++k)
        {
            const BCLIBCFFI_Shot shot = make_shot(kMethods[k], 0.0, 60.0);
            double angle = 0.0;
            BCLIBCFFI_Error err;
            const int32_t rc = zero_angle(shot, d_m, 0, &angle, &err);
            if (rc != BCLIBCFFI_OK)
            {
                std::fprintf(stderr, "zero failed: %s D=%g rc=%d %s\n", kNames[k], d_m, rc, err.message);
                CHECK(rc == BCLIBCFFI_OK);
                continue;
            }
            const double miss = error_trial(shot, angle, d_m);
            CHECK(std::fabs(miss) < 1e-3);
            CHECK(d_m != 0.1 || (deg(angle) > 12.5 && deg(angle) < 13.5)); // the lower arc of D = 0.1 m is 12.92 deg
            CHECK(angle <= rad(30.0) + 1e-9); // the reachable limit at cant 60, look 0
            if (g_print)
                std::printf("zero cant 60 D %g %s: %.9f deg miss %.3e ft\n", d_m, kNames[k], deg(angle), miss);
        }
}

// The angle of the maximum is within the reachable elevations and its range is the maximum of a scan of them.
static void test_max_range_scan()
{
    for (double look : {0.0, 10.0, -10.0})
        for (double cant : {0.0, 45.0, 60.0, 75.0, 80.0})
        {
            const BCLIBCFFI_Shot shot = make_shot(BCLIBCFFI_INTEGRATION_RK4, look, cant);
            const double reach = std::asin(std::hypot(std::sin(rad(look)), std::cos(rad(look)) * std::cos(rad(cant))));
            BCLIBCFFI_MaxRangeResult mr;
            BCLIBCFFI_Error err;
            const int32_t rc = max_range(shot, &mr, &err);
            CHECK(rc == BCLIBCFFI_OK);
            if (rc != BCLIBCFFI_OK)
                continue;
            CHECK(mr.angle_at_max_rad <= reach + 1e-9);
            CHECK(mr.angle_at_max_rad >= 0.0);
            double best = 0.0, best_e = 0.0;
            const int n = 300;
            for (int i = 0; i <= n; ++i) // the search brackets [0, limit]: the caller's lower bound is 0
            {
                const double e = reach * i / n;
                const double r = range_at(shot, e);
                if (r > best)
                {
                    best = r;
                    best_e = e;
                }
            }
            // the search tolerance: the golden-section search stops at a bracket of 1e-6 rad; the maximum is flat
            CHECK(mr.max_range_ft >= best * (1.0 - 1e-6));
            CHECK(mr.max_range_ft <= best * (1.0 + 1e-3));
            if (g_print)
                std::printf("maxrange look %g cant %g: %.9f ft at %.9f deg (limit %.6f deg; scan %.9f ft at %.6f deg)\n", look,
                            cant, mr.max_range_ft, deg(mr.angle_at_max_rad), deg(reach), best, deg(best_e));
        }
}

int main(int argc, char **argv)
{
    g_print = argc > 1 && std::strcmp(argv[1], "--print") == 0;
    test_close_zero_cant60();
    test_max_range_scan();
    if (failures)
    {
        std::fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    std::printf("OK\n");
    return 0;
}
