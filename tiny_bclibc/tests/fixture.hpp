#pragma once
// fixture.hpp — shared test data for bclibc ↔ tbclibc identity tests.
//
// G7 drag table taken from py_ballisticcalc/py_ballisticcalc/drag_tables.py.
// Parameters chosen to match test_trajectory.py parametrized G7 case.

#include <cmath>

// ── G7 drag table ──────────────────────────────────────────────────────────

struct DragPoint { double mach; double cd; };

static const DragPoint kG7Table[] = {
    {0.00, 0.1198}, {0.05, 0.1197}, {0.10, 0.1196}, {0.15, 0.1194},
    {0.20, 0.1193}, {0.25, 0.1194}, {0.30, 0.1194}, {0.35, 0.1194},
    {0.40, 0.1193}, {0.45, 0.1193}, {0.50, 0.1194}, {0.55, 0.1193},
    {0.60, 0.1194}, {0.65, 0.1197}, {0.70, 0.1202}, {0.725,0.1207},
    {0.75, 0.1215}, {0.775,0.1226}, {0.80, 0.1242}, {0.825,0.1266},
    {0.85, 0.1306}, {0.875,0.1368}, {0.90, 0.1464}, {0.925,0.1660},
    {0.95, 0.2054}, {0.975,0.2993}, {1.0,  0.3803}, {1.025,0.4015},
    {1.05, 0.4043}, {1.075,0.4034}, {1.10, 0.4014}, {1.125,0.3987},
    {1.15, 0.3955}, {1.20, 0.3884}, {1.25, 0.3810}, {1.30, 0.3732},
    {1.35, 0.3657}, {1.40, 0.3580}, {1.50, 0.3440}, {1.55, 0.3376},
    {1.60, 0.3315}, {1.65, 0.3260}, {1.70, 0.3209}, {1.75, 0.3160},
    {1.80, 0.3117}, {1.85, 0.3078}, {1.90, 0.3042}, {1.95, 0.3010},
    {2.00, 0.2980}, {2.05, 0.2951}, {2.10, 0.2922}, {2.15, 0.2892},
    {2.20, 0.2864}, {2.25, 0.2835}, {2.30, 0.2807}, {2.35, 0.2779},
    {2.40, 0.2752}, {2.45, 0.2725}, {2.50, 0.2697}, {2.55, 0.2670},
    {2.60, 0.2643}, {2.65, 0.2615}, {2.70, 0.2588}, {2.75, 0.2561},
    {2.80, 0.2533}, {2.85, 0.2506}, {2.90, 0.2479}, {2.95, 0.2451},
    {3.00, 0.2424}, {3.10, 0.2368}, {3.20, 0.2313}, {3.30, 0.2258},
    {3.40, 0.2205}, {3.50, 0.2154}, {3.60, 0.2106}, {3.70, 0.2060},
    {3.80, 0.2017}, {3.90, 0.1975}, {4.00, 0.1935}, {4.20, 0.1861},
    {4.40, 0.1793}, {4.60, 0.1730}, {4.80, 0.1672}, {5.00, 0.1618},
};
static const int kG7TableSize = static_cast<int>(sizeof(kG7Table) / sizeof(kG7Table[0]));

// ── Common RK4 time step ───────────────────────────────────────────────────
//
// bclibc FFI uses 0.0025s * cStepMultiplier for RK4.
// We fix both engines to this value in the test; tbclibc ShotProps.calc_step
// is overridden after tbclibc_build_shot_props() to ensure exact equality.
static constexpr double kCalcStep = 0.0025;

// ── Fixture parameters ─────────────────────────────────────────────────────

// G7_BASIC — .308/168gr G7 bullet, MV=2750fps, standard atmosphere,
//            no wind, no Coriolis, barrel horizontal (elevation=0), look=0.
// Matches the G7 case in py_ballisticcalc/tests/test_trajectory.py.
namespace G7_BASIC {
    static constexpr double BC             = 0.223;
    static constexpr double WEIGHT_GR      = 168.0;
    static constexpr double DIAMETER_IN    = 0.308;
    static constexpr double LENGTH_IN      = 1.282;
    static constexpr double MV_FPS         = 2750.0;
    static constexpr double SIGHT_HT_FT    = 0.0;
    static constexpr double TWIST_IN       = 0.0;   // no spin drift
    static constexpr double TEMP_C         = 15.0;
    static constexpr double PRESSURE_HPA   = 1013.25;
    static constexpr double ALT_FT         = 0.0;
    static constexpr double HUMIDITY       = 0.5;
    static constexpr double LOOK_ANGLE_RAD = 0.0;
    static constexpr double BARREL_EL_RAD  = 0.0;   // horizontal
    static constexpr double BARREL_AZ_RAD  = 0.0;
    static constexpr double CANT_ANGLE_RAD = 0.0;
    static constexpr double LAT_DEG        = std::numeric_limits<double>::quiet_NaN(); // no Coriolis
    static constexpr double AZ_DEG         = std::numeric_limits<double>::quiet_NaN();
}

// G7_WIND — same bullet with a 10mph crosswind (right-to-left, 90°)
namespace G7_WIND {
    using namespace G7_BASIC;
    // Wind: velocity=14.667fps (10mph), direction_from=90° (right-to-left)
    static constexpr double WIND_VEL_FPS  = 14.6667; // 10 mph
    static constexpr double WIND_DIR_RAD  = 1.5707963267948966; // π/2 (90°)
    static constexpr double WIND_UNTIL_FT = 1e9;
    static constexpr double WIND_MAX_FT   = 1e9;
}

// ── Engine config (matching py_ballisticcalc BaseIntegrationEngine defaults) ─

struct EngineConfigValues {
    double cStepMultiplier      = 1.0;
    double cZeroFindingAccuracy = 5e-6;
    double cMinimumVelocity     = 50.0;
    double cMaximumDrop         = -15000.0;
    int    cMaxIterations       = 40;
    double cGravityConstant     = -32.17405;
    double cMinimumAltitude     = -1500.0;
};

static constexpr EngineConfigValues kDefaultConfig{};
