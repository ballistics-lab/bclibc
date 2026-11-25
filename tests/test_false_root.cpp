// test_false_root.cpp -- a lofted zero search must not report a jump of the error function as a root (class C18).
//
// The miss at a target (`height - |range error|`) is not continuous for a lofted shot near the vertical: a shot that falls
// back short of the target and one that carries past it differ by thousands of feet. A bracket of the search that holds such
// a jump has a sign change that is not a zero, and Ridder's method converged on it all the same: the angle tolerance was met,
// the search "succeeded" and returned an angle that does not reach the target. The Ridder exits on the angle tolerance and on
// the collapse of the bracket now return a ZeroFindingError "No continuous lofted zero" when the error at the returned angle
// is more than 0.45 of the smaller error at the ends of the initial bracket (the Python reference, CONTINUOUS_ROOT_RATIO).
//
// The reported case: 175 gr G7 .308, 800 m/s, 5 m/s tail wind, 100 m zero, lofted: the search converged on 88.4362 degrees,
// where the error jumps from +250 ft to -1600 ft. The test checks the six integration methods (fixed step through the
// *_lofted_shot entries, adaptive through the *_lofted_adaptive_shot entries), the point entries, that the low zero of the
// same shot is unchanged, and that lofted zeros that are real roots (several end on the bracket collapse) keep their recorded
// elevations (1e-7 rad: the libm of the platforms differs in the last digits) and still hit.
// `--print-guard` prints the recorded values.

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

static const double kFt = 3.28084;

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

static BCLIBCFFI_Wind wind(double mps, double direction_deg, double until_m)
{
    BCLIBCFFI_Wind w;
    w.velocity_fps = mps * kFt;
    w.direction_from_rad = direction_deg * M_PI / 180.0;
    w.until_distance_ft = until_m * kFt;
    w.max_distance_ft = 1e8;
    return w;
}

static const BCLIBCFFI_Wind kTail[] = {wind(5.0, 180.0, 3000.0)};
static const BCLIBCFFI_Wind kCross[] = {wind(5.0, 90.0, 3000.0)};

// The shot of the report: G7 bc 0.31, 175 gr, 0.308 in, 800 m/s, sight height 0.05 m, twist 11, look 0.
static BCLIBCFFI_Shot make_shot(int32_t method, const BCLIBCFFI_Wind *winds, int n_winds, double cant_deg)
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
    s.winds = winds;
    s.wind_count = n_winds;
    s.look_angle_rad = 0.0;
    s.cant_angle_rad = cant_deg * M_PI / 180.0;
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

static const int32_t kMethods[] = {BCLIBCFFI_INTEGRATION_RK4,       BCLIBCFFI_INTEGRATION_EULER,
                                   BCLIBCFFI_INTEGRATION_VELOCITY_VERLET, BCLIBCFFI_INTEGRATION_CASH_KARP,
                                   BCLIBCFFI_INTEGRATION_DORMAND_PRINCE,  BCLIBCFFI_INTEGRATION_TSITOURAS};
static const char *kNames[] = {"rk4", "euler", "verlet", "rkck", "dopri", "tsitouras"};
static bool is_adaptive(int32_t m) { return m >= BCLIBCFFI_INTEGRATION_CASH_KARP; }

static const double kTol = 1e-6; // adaptive tolerances

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

// The jump: ZeroFindingError "No continuous lofted zero", with the evidence in the error.
static void test_false_root()
{
    for (int i = 0; i < 6; ++i)
    {
        const BCLIBCFFI_Shot shot = make_shot(kMethods[i], kTail, 1, 0.0);
        BCLIBCFFI_Error err;
        double angle = 0.0;
        int32_t rc = zero_angle(shot, 100.0, 1, &angle, &err);
        const bool ok = rc == BCLIBCFFI_ERR_ZERO_FINDING && std::strstr(err.message, "No continuous lofted zero") != nullptr;
        if (!ok)
            std::fprintf(stderr, "  %s angle: rc %d angle %.12f (%s)\n", kNames[i], (int)rc, angle,
                         rc != BCLIBCFFI_OK ? err.message : "no error");
        CHECK(ok);
        if (ok)
        {
            CHECK(std::strstr(err.message, "(a discontinuity, not a root)") != nullptr);
            CHECK(std::strstr(err.message, "f(low)=") != nullptr && std::strstr(err.message, "f(high)=") != nullptr);
            // the elevation the search ended on (near the jump, 88.4 degrees) and its error (thousands of feet off)
            CHECK(std::fabs(err.f64_1 * 180.0 / M_PI - 88.43) < 0.1);
            CHECK(std::fabs(err.f64_0) > 1000.0);
            CHECK(err.i32_0 > 0);
        }

        BCLIBCFFI_ZeroPointResult zp;
        rc = zero_point(shot, 100.0, 1, &zp, &err);
        const bool okp = rc == BCLIBCFFI_ERR_ZERO_FINDING && std::strstr(err.message, "No continuous lofted zero") != nullptr;
        if (!okp)
            std::fprintf(stderr, "  %s point: rc %d (%s)\n", kNames[i], (int)rc, rc != BCLIBCFFI_OK ? err.message : "no error");
        CHECK(okp);
    }
}

// The low zero of the same shot is fine (and the lofted zero of the same shot without wind).
static void test_low_zero_unchanged()
{
    for (int i = 0; i < 6; ++i)
    {
        const BCLIBCFFI_Shot shot = make_shot(kMethods[i], kTail, 1, 0.0);
        BCLIBCFFI_Error err;
        double angle = 0.0;
        const int32_t rc = zero_angle(shot, 100.0, 0, &angle, &err);
        CHECK(rc == BCLIBCFFI_OK);
        CHECK(angle > 0.0 && angle * 180.0 / M_PI < 1.0);
    }
}

struct Legit
{
    const char *name;
    double d_m;
    int wind; // 0 none, 1 tail, 2 cross
    double cant_deg;
};
static const Legit kLegit[] = {
    {"d50", 50.0, 0, 0.0},     {"d100", 100.0, 0, 0.0},    {"d300", 300.0, 0, 0.0},    {"d1000", 1000.0, 0, 0.0},
    {"cross300", 300.0, 2, 0.0}, {"cant300", 300.0, 0, 10.0}, {"tail500", 500.0, 1, 0.0},
};

struct GuardRow
{
    int method, case_index;
    double angle;
};

#include "false_root_guard.inc"

static BCLIBCFFI_Shot legit_shot(int32_t method, const Legit &c)
{
    const BCLIBCFFI_Wind *w = c.wind == 1 ? kTail : c.wind == 2 ? kCross : nullptr;
    return make_shot(method, w, w ? 1 : 0, c.cant_deg);
}

static const double kAngleTol = 1e-7; // rad; the libm of the platforms differs in the last digits

static void print_guard()
{
    std::puts("// Recorded before the C18 check (src/engine.cpp find_zero_angle_ridder), lofted zeros that are real roots.");
    std::puts("// {method index, case index, angle (rad)}");
    std::puts("static const GuardRow kGuard[] = {");
    for (int i = 0; i < 6; ++i)
        for (size_t k = 0; k < sizeof kLegit / sizeof kLegit[0]; ++k)
        {
            const BCLIBCFFI_Shot shot = legit_shot(kMethods[i], kLegit[k]);
            BCLIBCFFI_Error err;
            double angle = 0.0;
            const int32_t rc = zero_angle(shot, kLegit[k].d_m, 1, &angle, &err);
            if (rc == BCLIBCFFI_OK)
                std::printf("    {%d, %zu, %.17g},\n", i, k, angle);
            else
                std::fprintf(stderr, "  %s %s: rc %d\n", kNames[i], kLegit[k].name, (int)rc);
        }
    std::puts("};");
}

// The error of a shot fired at the angle, at the target (height above the line of sight less the range error).
static double error_at(BCLIBCFFI_Shot shot, double angle, double d_m)
{
    // A lofted flight of a minute: lift the limits that end a shot short (the zero search does the same).
    shot.config.cMinimumVelocity = 0.0;
    shot.config.cMaximumDrop = -60000.0;
    shot.config.cMinimumAltitude = -60000.0;
    double f = 1e9;
    BCLIBCFFI_Error err;
    std::memset(&err, 0, sizeof err);
    const double tx = d_m * kFt * std::cos(shot.look_angle_rad), ty = d_m * kFt * std::sin(shot.look_angle_rad);
    int32_t rc;
    if (is_adaptive(shot.method))
    {
        BCLIBCFFI_StepStats st{0, 0};
        rc = BCLIBCFFI_error_at_distance_adaptive_shot(&shot, shot.method, kTol, kTol, angle, tx, ty, &f, nullptr, &st, &err);
    }
    else
        rc = BCLIBCFFI_error_at_distance_shot(&shot, angle, tx, ty, &f, nullptr, &err);
    return rc == BCLIBCFFI_OK ? f : 1e9;
}

static void test_real_roots_unchanged()
{
    for (const GuardRow &g : kGuard)
    {
        const Legit &c = kLegit[g.case_index];
        const BCLIBCFFI_Shot shot = legit_shot(kMethods[g.method], c);
        BCLIBCFFI_Error err;
        double angle = 0.0;
        const int32_t rc = zero_angle(shot, c.d_m, 1, &angle, &err);
        if (rc != BCLIBCFFI_OK)
            std::fprintf(stderr, "  %s %s: rc %d (%s)\n", kNames[g.method], c.name, (int)rc, err.message);
        CHECK(rc == BCLIBCFFI_OK);
        if (rc != BCLIBCFFI_OK)
            continue;
        const bool same = std::fabs(angle - g.angle) <= kAngleTol;
        if (!same)
            std::fprintf(stderr, "  %s %s: %.17g, recorded %.17g\n", kNames[g.method], c.name, angle, g.angle);
        CHECK(same);
        // and it still hits (the cant case is canted through the error function the same way: it is the zero's own error)
        const double miss = error_at(shot, angle, c.d_m);
        if (!(std::fabs(miss) < 0.05))
            std::fprintf(stderr, "  %s %s: miss %.6g ft\n", kNames[g.method], c.name, miss);
        CHECK(std::fabs(miss) < 0.05);
    }
}

int main(int argc, char **argv)
{
    if (argc > 1 && std::strcmp(argv[1], "--print-guard") == 0)
    {
        print_guard();
        return 0;
    }
    test_false_root();
    test_low_zero_unchanged();
    test_real_roots_unchanged();
    if (failures != 0)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::puts("OK: a jump of the error function is not a lofted zero; real roots unchanged");
    return 0;
}
