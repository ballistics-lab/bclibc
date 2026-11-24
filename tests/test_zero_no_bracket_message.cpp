// test_zero_no_bracket_message.cpp -- the diagnostic text of "no zero trajectory in the elevation bracket".
//
// When the Ridder zero search finds no sign change between the two ends of its elevation bracket, the Python reference raises
// ZeroFindingError with "No <low|lofted> zero trajectory in elevation range (A, B deg). Errors at bracket: f(low)=X,
// f(high)=Y": the bracket and the miss at each end tell the user why (a lofted zero of a target too close for any lofted arc,
// say). The C++ core formatted that text but could only log it: BCLIBC_SolverZeroFindingError held a non-owning
// `const char *`, so the error it returned carried a short constant ("No lofted zero trajectory in elevation search bracket")
// and the native engines' users never saw the numbers. The error now owns a fixed buffer for its message.
//
// The reference texts below are the messages of the pure-Python engine (py_ballisticcalc RK4IntegrationEngine.find_zero_angle,
// the same G7-like table, bore, atmosphere and configuration) for a lofted zero at 1 m; a low zero with no bracket is not
// reachable through the C ABI (the maximum-range check rejects every target the low bracket cannot reach), so the low variant
// is checked on the error payload itself. The payload checks also pin down that copying and moving an error keep its text.

#include "bclibc.hpp"
#include "bclibc/ffi/bclibc_ffi.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <utility>

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

static const double kFt = 3.28084;      // feet per metre


// Two drag tables: a G7-like one and a blunter, G1-like one.
static const double kMachG7[] = {0.0, 0.8, 1.2, 3.0};
static const double kCdG7[] = {0.12, 0.13, 0.38, 0.30};
static const double kMachG1[] = {0.0, 0.8, 1.2, 3.0};
static const double kCdG1[] = {0.26, 0.30, 0.55, 0.45};

struct Table
{
    const char *name;
    const double *mach;
    const double *cd;
    double bc;
};
static const Table kTables[] = {{"G7", kMachG7, kCdG7, 0.243}, {"G1", kMachG1, kCdG1, 0.45}};

// The fixed-step methods of shot->method; the adaptive ones are exercised through the *_adaptive_* entry points.
static const int32_t kMethods[] = {BCLIBCFFI_INTEGRATION_RK4, BCLIBCFFI_INTEGRATION_VELOCITY_VERLET};
static const double kSightsM[] = {0.05, 0.1, 0.5};
static const double kVelocitiesMps[] = {300.0, 800.0};
static const double kTargetsM[] = {0.05, 0.1, 0.2, 0.4, 0.6, 0.9};
static const double kLooksDeg[] = {0.0, 10.0, -10.0};

static BCLIBCFFI_Shot make_shot(int32_t method, const Table &t, double sight_m, double velocity_mps, double look_deg,
                                double cant_deg = 0.0, double step_multiplier = 1.0)
{
    BCLIBCFFI_Shot s;
    std::memset(&s, 0, sizeof s);
    s.bc = t.bc;
    s.weight_grain = 175.0;
    s.diameter_inch = 0.3;
    s.length_inch = 1.26;
    s.muzzle_velocity_fps = velocity_mps * kFt;
    s.sight_height_ft = sight_m * kFt;
    s.twist_inch = 10.0;
    s.temp_c = 15.0;
    s.pressure_hpa = 1013.25;
    s.humidity = 0.0;
    s.mach_data = t.mach;
    s.cd_data = t.cd;
    s.drag_table_size = 4;
    s.look_angle_rad = look_deg * M_PI / 180.0;
    s.cant_angle_rad = cant_deg * M_PI / 180.0;
    s.latitude_deg = std::nan("");
    s.azimuth_deg = std::nan("");
    s.config.cStepMultiplier = step_multiplier;
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
    double velocity_mps;
    double look_deg;
    double sight_m;
    double target_m;
    const char *python_message; // the Python engine's text for the same shot, a lofted zero
};

static const Case kCases[] = {
    {300.0, 0.0, 0.05, 1.0,
     "No lofted zero trajectory in elevation range (38.76, 89.90 deg). Errors at bracket: f(low)=2.47, f(high)=1811.52"},
    {800.0, 0.0, 0.05, 1.0,
     "No lofted zero trajectory in elevation range (36.52, 89.90 deg). Errors at bracket: f(low)=2.27, f(high)=1866.91"},
    {300.0, 30.0, 0.05, 1.0,
     "No lofted zero trajectory in elevation range (54.64, 89.90 deg). Errors at bracket: f(low)=2.20, f(high)=1576.01"},
    {300.0, -30.0, 0.05, 1.0,
     "No lofted zero trajectory in elevation range (26.17, 89.90 deg). Errors at bracket: f(low)=2.87, f(high)=1579.29"},
    {800.0, 0.0, 0.5, 3.0,
     "No lofted zero trajectory in elevation range (36.52, 89.90 deg). Errors at bracket: f(low)=5.65, f(high)=5374.60"},
};

static void check_abi_messages()
{
    const Table &t = kTables[0];
    for (const Case &c : kCases)
    {
        const BCLIBCFFI_Shot shot = make_shot(BCLIBCFFI_INTEGRATION_RK4, t, c.sight_m, c.velocity_mps, c.look_deg);
        BCLIBCFFI_Error err;
        std::memset(&err, 0, sizeof err);
        double angle = 0.0;
        const int32_t rc = BCLIBCFFI_find_zero_angle_lofted_shot(&shot, c.target_m * kFt, 1, &angle, &err);
        CHECK(rc == BCLIBCFFI_ERR_ZERO_FINDING);
        const bool same = std::strcmp(err.message, c.python_message) == 0;
        if (!same)
            std::fprintf(stderr, "  v %.0f look %.0f sight %.2f target %.0f m (rc %d)\n    C++:    %s\n    Python: %s\n",
                         c.velocity_mps, c.look_deg, c.sight_m, c.target_m, (int)rc, err.message, c.python_message);
        CHECK(same);
        // The numeric fields keep their meaning: the target height, no iterations, the barrel elevation of the shot.
        CHECK(err.i32_0 == 0);
    }
}

static void check_payload()
{
    using bclibc::BCLIBC_Error;
    using bclibc::BCLIBC_SolverZeroFindingError;

    const char *text = "No low zero trajectory in elevation range (-1.23, 45.67 deg). Errors at bracket: f(low)=1.00, f(high)=2.00";
    BCLIBC_Error original{BCLIBC_SolverZeroFindingError{text, 1.5, 7, 0.25}};
    CHECK(std::strcmp(original.what(), text) == 0);

    // The text lives in the payload: a copy and a move of the error (and of its source going out of scope) keep it, and
    // the copy's text is its own, not a pointer into the original.
    char *scratch = new char[std::strlen(text) + 1];
    std::strcpy(scratch, text);
    BCLIBC_Error from_scratch{BCLIBC_SolverZeroFindingError{scratch, 1.5, 7, 0.25}};
    std::memset(scratch, 'x', std::strlen(text));
    delete[] scratch;
    CHECK(std::strcmp(from_scratch.what(), text) == 0);

    BCLIBC_Error copy = original;
    CHECK(std::strcmp(copy.what(), text) == 0);
    CHECK(copy.what() != original.what());
    BCLIBC_Error moved = std::move(copy);
    CHECK(std::strcmp(moved.what(), text) == 0);
    const auto *p = moved.payload<BCLIBC_SolverZeroFindingError>();
    CHECK(p != nullptr && p->zero_finding_error == 1.5 && p->iterations_count == 7 && p->last_barrel_elevation_rad == 0.25);

    // A text longer than the buffer is cut, not overrun, and stays terminated.
    char longtext[1024];
    std::memset(longtext, 'a', sizeof longtext - 1);
    longtext[sizeof longtext - 1] = '\0';
    BCLIBC_Error cut{BCLIBC_SolverZeroFindingError{longtext, 0.0, 0, 0.0}};
    CHECK(std::strlen(cut.what()) > 100 && std::strlen(cut.what()) < sizeof longtext - 1);

    // No text: the default one.
    BCLIBC_Error none{BCLIBC_SolverZeroFindingError{nullptr, 0.0, 0, 0.0}};
    CHECK(std::strcmp(none.what(), "Zero finding error") == 0);
}

int main()
{
    check_abi_messages();
    check_payload();
    if (failures != 0)
    {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::puts("OK: the no-bracket error carries the diagnostic text");
    return 0;
}
