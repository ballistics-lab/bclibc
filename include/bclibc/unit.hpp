#ifndef BCLIBC_UNIT_HPP
#define BCLIBC_UNIT_HPP

/**
 * @file unit.hpp
 * @brief Type-safe, header-only physical unit system for ballistic calculations.
 *
 * Provides compile-time unit safety via `BCLIBC_Dimension<DimTag, Unit>` — a single template
 * that covers all physical dimensions. Values are stored internally in a fixed base unit
 * (e.g. inches for Distance, m/s for Velocity, °F for Temperature) and converted on demand.
 *
 * **Supported dimensions and their base (raw) units:**
 * | Alias         | Base unit   | Unit tags                                                         |
 * |---------------|-------------|-------------------------------------------------------------------|
 * | Angular       | radian      | Radian, Degree, MOA, Mil, MRad, Thousandth, InchesPer100Yd, CmPer100m, OClock |
 * | Distance      | inch        | Inch, Foot, Yard, Mile, NauticalMile, Millimeter, Centimeter, Meter, Kilometer, Line |
 * | Energy        | foot-pound  | FootPound, Joule                                                  |
 * | Pressure      | mmHg        | MmHg, InHg, Bar, hPa, PSI                                        |
 * | Temperature   | °Fahrenheit | Fahrenheit, Celsius, Kelvin, Rankin                               |
 * | Velocity      | m/s         | MPS, KMH, FPS, MPH, KT                                           |
 * | Weight        | grain       | Grain, Ounce, Gram, Pound, Kilogram, Newton                       |
 * | Time          | second      | Second, Minute, Millisecond, Microsecond, Nanosecond, Picosecond  |
 *
 * **Quick start:**
 * @code
 * using namespace bclibc;
 *
 * BCLIBC_Distance<BCLIBC_Meter>      d(100.0);            // 100 m
 * BCLIBC_Distance<BCLIBC_Yard>       dy = d.to<BCLIBC_Yard>();   // → 109.361 yd
 * double               ft = d.to<BCLIBC_Foot>().value(); // → 328.084
 *
 * BCLIBC_Velocity<BCLIBC_FPS>        v(2800.0);
 * double               mps = v.to<BCLIBC_MPS>().value(); // → 853.44
 *
 * BCLIBC_Temperature<BCLIBC_Celsius> t(20.0);
 * double               k = t.to<BCLIBC_Kelvin>().value(); // → 293.15
 *
 * // Arithmetic (same dimension, any units)
 * BCLIBC_Distance<BCLIBC_Meter> sum = d + BCLIBC_Distance<BCLIBC_Yard>(50.0).to<BCLIBC_Meter>();
 * BCLIBC_Distance<BCLIBC_Meter> scaled = d * 2.0;
 *
 * // Runtime conversion
 * double val = BCLIBC_convert(100.0, BCLIBC_Unit::Meter, BCLIBC_Unit::Foot); // 328.084
 * double tmp = BCLIBC_convert_temperature(100.0, BCLIBC_Unit::Celsius, BCLIBC_Unit::Fahrenheit); // 212.0
 * @endcode
 */

#include <type_traits>
#include <cmath>
#include <iostream>
#include <cassert>

namespace bclibc
{

    // ================= MATH CONSTANT =================

    namespace unit_constants
    {
        static constexpr double pi = 3.14159265358979323846;
        static constexpr double tolerance = 1e-12;
        static constexpr double zeroDivTol = 1e-18;
    } // namespace unit_constants

    // ================= UNIT ENUM =================

    /**
     * @brief Enum of all supported unit identifiers.
     *
     * Values match py-ballisticcalc `Unit` enum exactly, enabling round-trip
     * interoperability with the Python library over FFI.
     *
     * @code
     * BCLIBC_Unit u = BCLIBC_Unit::Meter;
     * double f = BCLIBC_unit_factor(u);  // factor for linear conversion
     *
     * // Compile-time enum → type bridge:
     * using T = BCLIBC_unit_from_enum<BCLIBC_Unit::Kilometer>::type; // → Kilometer tag
     * BCLIBC_Distance<T> d(5.0);  // BCLIBC_Distance<BCLIBC_Kilometer>
     * @endcode
     */
    enum class BCLIBC_Unit : int
    {
        // Angular [0–8]
        Radian = 0,
        Degree = 1,
        MOA = 2,
        Mil = 3,
        MRad = 4,
        Thousandth = 5,
        InchesPer100Yd = 6,
        CmPer100m = 7,
        OClock = 8,

        // Distance [10–19]
        Inch = 10,
        Foot = 11,
        Yard = 12,
        Mile = 13,
        NauticalMile = 14,
        Millimeter = 15,
        Centimeter = 16,
        Meter = 17,
        Kilometer = 18,
        Line = 19,

        // Energy [30–31]
        FootPound = 30,
        Joule = 31,

        // Pressure [40–44]
        MmHg = 40,
        InHg = 41,
        Bar = 42,
        hPa = 43,
        PSI = 44,

        // Temperature [50–53]
        Fahrenheit = 50,
        Celsius = 51,
        Kelvin = 52,
        Rankin = 53,

        // Velocity [60–64]
        MPS = 60,
        KMH = 61,
        FPS = 62,
        MPH = 63,
        KT = 64,

        // Weight [70–75]
        Grain = 70,
        Ounce = 71,
        Gram = 72,
        Pound = 73,
        Kilogram = 74,
        Newton = 75,

        // Time [80–85]
        Minute = 80,
        Second = 81,
        Millisecond = 82,
        Microsecond = 83,
        Nanosecond = 84,
        Picosecond = 85,
    };

    /// @brief Compatibility alias for code that previously used BCLIBC_DistanceUnit.
    using BCLIBC_DistanceUnit = BCLIBC_Unit;

    // ================= UNIT TAG BASE =================

    /**
     * @brief CRTP base for linear unit tags.
     *
     * `Derived` must define:
     * - `static constexpr double factor` — scaling factor: raw = value × factor
     * - `static constexpr BCLIBC_Unit id` — enum identifier
     *
     * Provides default `to_raw` / `from_raw` via the factor:
     * @code
     * static constexpr double to_raw(double v)   { return v * Derived::factor; }
     * static constexpr double from_raw(double r) { return r / Derived::factor; }
     * @endcode
     *
     * Temperature tags do **not** inherit `UnitTag` — they define affine
     * `to_raw` / `from_raw` directly (raw unit = °Fahrenheit).
     *
     * Every tag (linear or affine) must expose:
     * - `static constexpr BCLIBC_Unit id`
     * - `static constexpr double to_raw(double v)`
     * - `static constexpr double from_raw(double r)`
     */
    template <typename Derived>
    struct BCLIBC_UnitTag
    {
        static constexpr BCLIBC_Unit id = Derived::id;

        static constexpr double to_raw(double v) { return v * Derived::factor; }
        static constexpr double from_raw(double r) { return r / Derived::factor; }
    };

    // ---- Angular tags (raw = radians) ----

    struct BCLIBC_Radian : BCLIBC_UnitTag<BCLIBC_Radian>
    {
        static constexpr double factor = 1.0;
        static constexpr auto id = BCLIBC_Unit::Radian;
    };
    struct BCLIBC_Degree : BCLIBC_UnitTag<BCLIBC_Degree>
    {
        static constexpr double factor = unit_constants::pi / 180.0;
        static constexpr auto id = BCLIBC_Unit::Degree;
    };
    struct BCLIBC_MOA : BCLIBC_UnitTag<BCLIBC_MOA>
    {
        static constexpr double factor = unit_constants::pi / (60.0 * 180.0);
        static constexpr auto id = BCLIBC_Unit::MOA;
    };
    struct BCLIBC_Mil : BCLIBC_UnitTag<BCLIBC_Mil>
    {
        static constexpr double factor = unit_constants::pi / 3200.0;
        static constexpr auto id = BCLIBC_Unit::Mil;
    };
    struct BCLIBC_MRad : BCLIBC_UnitTag<BCLIBC_MRad>
    {
        static constexpr double factor = 1.0 / 1000.0;
        static constexpr auto id = BCLIBC_Unit::MRad;
    };
    struct BCLIBC_Thousandth : BCLIBC_UnitTag<BCLIBC_Thousandth>
    {
        static constexpr double factor = unit_constants::pi / 3000.0;
        static constexpr auto id = BCLIBC_Unit::Thousandth;
    };
    struct BCLIBC_InchesPer100Yd : BCLIBC_UnitTag<BCLIBC_InchesPer100Yd>
    {
        static constexpr double factor = 1.0 / 3600.0;
        static constexpr auto id = BCLIBC_Unit::InchesPer100Yd;
    };
    struct BCLIBC_CmPer100m : BCLIBC_UnitTag<BCLIBC_CmPer100m>
    {
        static constexpr double factor = 1.0 / 10000.0;
        static constexpr auto id = BCLIBC_Unit::CmPer100m;
    };
    struct BCLIBC_OClock : BCLIBC_UnitTag<BCLIBC_OClock>
    {
        static constexpr double factor = unit_constants::pi / 6.0;
        static constexpr auto id = BCLIBC_Unit::OClock;
    };

    // ---- Distance tags (raw = inches) ----

    struct BCLIBC_Inch : BCLIBC_UnitTag<BCLIBC_Inch>
    {
        static constexpr double factor = 1.0;
        static constexpr auto id = BCLIBC_Unit::Inch;
    };
    struct BCLIBC_Foot : BCLIBC_UnitTag<BCLIBC_Foot>
    {
        static constexpr double factor = 12.0;
        static constexpr auto id = BCLIBC_Unit::Foot;
    };
    struct BCLIBC_Yard : BCLIBC_UnitTag<BCLIBC_Yard>
    {
        static constexpr double factor = 36.0;
        static constexpr auto id = BCLIBC_Unit::Yard;
    };
    struct BCLIBC_Mile : BCLIBC_UnitTag<BCLIBC_Mile>
    {
        static constexpr double factor = 63360.0;
        static constexpr auto id = BCLIBC_Unit::Mile;
    };
    struct BCLIBC_NauticalMile : BCLIBC_UnitTag<BCLIBC_NauticalMile>
    {
        static constexpr double factor = 72913.3858;
        static constexpr auto id = BCLIBC_Unit::NauticalMile;
    };
    struct BCLIBC_Millimeter : BCLIBC_UnitTag<BCLIBC_Millimeter>
    {
        static constexpr double factor = 1.0 / 25.4;
        static constexpr auto id = BCLIBC_Unit::Millimeter;
    };
    struct BCLIBC_Centimeter : BCLIBC_UnitTag<BCLIBC_Centimeter>
    {
        static constexpr double factor = 10.0 / 25.4;
        static constexpr auto id = BCLIBC_Unit::Centimeter;
    };
    struct BCLIBC_Meter : BCLIBC_UnitTag<BCLIBC_Meter>
    {
        static constexpr double factor = 1000.0 / 25.4;
        static constexpr auto id = BCLIBC_Unit::Meter;
    };
    struct BCLIBC_Kilometer : BCLIBC_UnitTag<BCLIBC_Kilometer>
    {
        static constexpr double factor = 1000000.0 / 25.4;
        static constexpr auto id = BCLIBC_Unit::Kilometer;
    };
    struct BCLIBC_Line : BCLIBC_UnitTag<BCLIBC_Line>
    {
        static constexpr double factor = 0.1;
        static constexpr auto id = BCLIBC_Unit::Line;
    };

    // ---- Energy tags (raw = foot-pounds) ----

    struct BCLIBC_FootPound : BCLIBC_UnitTag<BCLIBC_FootPound>
    {
        static constexpr double factor = 1.0;
        static constexpr auto id = BCLIBC_Unit::FootPound;
    };
    struct BCLIBC_Joule : BCLIBC_UnitTag<BCLIBC_Joule>
    {
        static constexpr double factor = 1.0 / 1.3558179483314;
        static constexpr auto id = BCLIBC_Unit::Joule;
    };

    // ---- Pressure tags (raw = mmHg) ----

    struct BCLIBC_MmHg : BCLIBC_UnitTag<BCLIBC_MmHg>
    {
        static constexpr double factor = 1.0;
        static constexpr auto id = BCLIBC_Unit::MmHg;
    };
    struct BCLIBC_InHg : BCLIBC_UnitTag<BCLIBC_InHg>
    {
        static constexpr double factor = 25.4;
        static constexpr auto id = BCLIBC_Unit::InHg;
    };
    struct BCLIBC_Bar : BCLIBC_UnitTag<BCLIBC_Bar>
    {
        static constexpr double factor = 750.061683;
        static constexpr auto id = BCLIBC_Unit::Bar;
    };
    struct BCLIBC_hPa : BCLIBC_UnitTag<BCLIBC_hPa>
    {
        static constexpr double factor = 750.061683 / 1000.0;
        static constexpr auto id = BCLIBC_Unit::hPa;
    };
    struct BCLIBC_PSI : BCLIBC_UnitTag<BCLIBC_PSI>
    {
        static constexpr double factor = 51.714924102396;
        static constexpr auto id = BCLIBC_Unit::PSI;
    };

    // ---- Velocity tags (raw = m/s) ----

    struct BCLIBC_MPS : BCLIBC_UnitTag<BCLIBC_MPS>
    {
        static constexpr double factor = 1.0;
        static constexpr auto id = BCLIBC_Unit::MPS;
    };
    struct BCLIBC_KMH : BCLIBC_UnitTag<BCLIBC_KMH>
    {
        static constexpr double factor = 1.0 / 3.6;
        static constexpr auto id = BCLIBC_Unit::KMH;
    };
    struct BCLIBC_FPS : BCLIBC_UnitTag<BCLIBC_FPS>
    {
        static constexpr double factor = 1.0 / 3.2808399;
        static constexpr auto id = BCLIBC_Unit::FPS;
    };
    struct BCLIBC_MPH : BCLIBC_UnitTag<BCLIBC_MPH>
    {
        static constexpr double factor = 1.0 / 2.23693629;
        static constexpr auto id = BCLIBC_Unit::MPH;
    };
    struct BCLIBC_KT : BCLIBC_UnitTag<BCLIBC_KT>
    {
        static constexpr double factor = 1.0 / 1.94384449;
        static constexpr auto id = BCLIBC_Unit::KT;
    };

    // ---- Weight tags (raw = grains) ----

    struct BCLIBC_Grain : BCLIBC_UnitTag<BCLIBC_Grain>
    {
        static constexpr double factor = 1.0;
        static constexpr auto id = BCLIBC_Unit::Grain;
    };
    struct BCLIBC_Ounce : BCLIBC_UnitTag<BCLIBC_Ounce>
    {
        static constexpr double factor = 437.5;
        static constexpr auto id = BCLIBC_Unit::Ounce;
    };
    struct BCLIBC_Gram : BCLIBC_UnitTag<BCLIBC_Gram>
    {
        static constexpr double factor = 15.4323584;
        static constexpr auto id = BCLIBC_Unit::Gram;
    };
    struct BCLIBC_Pound : BCLIBC_UnitTag<BCLIBC_Pound>
    {
        static constexpr double factor = 7000.0;
        static constexpr auto id = BCLIBC_Unit::Pound;
    };
    struct BCLIBC_Kilogram : BCLIBC_UnitTag<BCLIBC_Kilogram>
    {
        static constexpr double factor = 15432.3584;
        static constexpr auto id = BCLIBC_Unit::Kilogram;
    };
    struct BCLIBC_Newton : BCLIBC_UnitTag<BCLIBC_Newton>
    {
        static constexpr double factor = 1573.662597;
        static constexpr auto id = BCLIBC_Unit::Newton;
    };

    // ---- Time tags (raw = seconds) ----

    struct BCLIBC_Second : BCLIBC_UnitTag<BCLIBC_Second>
    {
        static constexpr double factor = 1.0;
        static constexpr auto id = BCLIBC_Unit::Second;
    };
    struct BCLIBC_Minute : BCLIBC_UnitTag<BCLIBC_Minute>
    {
        static constexpr double factor = 60.0;
        static constexpr auto id = BCLIBC_Unit::Minute;
    };
    struct BCLIBC_Millisecond : BCLIBC_UnitTag<BCLIBC_Millisecond>
    {
        static constexpr double factor = 1.0 / 1000.0;
        static constexpr auto id = BCLIBC_Unit::Millisecond;
    };
    struct BCLIBC_Microsecond : BCLIBC_UnitTag<BCLIBC_Microsecond>
    {
        static constexpr double factor = 1.0 / 1000000.0;
        static constexpr auto id = BCLIBC_Unit::Microsecond;
    };
    struct BCLIBC_Nanosecond : BCLIBC_UnitTag<BCLIBC_Nanosecond>
    {
        static constexpr double factor = 1.0 / 1000000000.0;
        static constexpr auto id = BCLIBC_Unit::Nanosecond;
    };
    struct BCLIBC_Picosecond : BCLIBC_UnitTag<BCLIBC_Picosecond>
    {
        static constexpr double factor = 1.0 / 1000000000000.0;
        static constexpr auto id = BCLIBC_Unit::Picosecond;
    };

    // ---- Temperature tags (raw = °Fahrenheit; affine conversions) ----

    /**
     * @brief Temperature unit tags use affine (offset + scale) conversions.
     *
     * Unlike linear tags, these do **not** inherit `UnitTag` and have no `factor`.
     * `to_raw(v)` converts a value in the tag's unit to °Fahrenheit (raw).
     * `from_raw(r)` converts °Fahrenheit back to the tag's unit.
     *
     * @warning Arithmetic between two `BCLIBC_Temperature<>` objects adds their raw °F values.
     *          This is only physically meaningful for the Fahrenheit and Rankin scales.
     *          For other scales use `to<>()` for conversion only.
     *
     * @code
     * BCLIBC_Temperature<BCLIBC_Celsius> tc(20.0);
     * double k = tc.to<BCLIBC_Kelvin>().value();   // 293.15 K  ✓
     * double f = tc.to<BCLIBC_Fahrenheit>().value(); // 68.0 °F ✓
     * @endcode
     */
    struct BCLIBC_Fahrenheit
    {
        static constexpr BCLIBC_Unit id = BCLIBC_Unit::Fahrenheit;
        static constexpr double to_raw(double v) { return v; }
        static constexpr double from_raw(double r) { return r; }
    };

    /// @copydoc Fahrenheit
    struct BCLIBC_Celsius
    {
        static constexpr BCLIBC_Unit id = BCLIBC_Unit::Celsius;
        static constexpr double to_raw(double v) { return v * 9.0 / 5.0 + 32.0; }
        static constexpr double from_raw(double r) { return (r - 32.0) * 5.0 / 9.0; }
    };

    /// @copydoc Fahrenheit
    struct BCLIBC_Kelvin
    {
        static constexpr BCLIBC_Unit id = BCLIBC_Unit::Kelvin;
        static constexpr double to_raw(double v) { return (v - 273.15) * 9.0 / 5.0 + 32.0; }
        static constexpr double from_raw(double r) { return (r - 32.0) * 5.0 / 9.0 + 273.15; }
    };

    /// @copydoc Fahrenheit
    struct BCLIBC_Rankin
    {
        static constexpr BCLIBC_Unit id = BCLIBC_Unit::Rankin;
        static constexpr double to_raw(double v) { return v - 459.67; }
        static constexpr double from_raw(double r) { return r + 459.67; }
    };

    // ================= DIMENSION PHANTOM TAGS =================

    /// @brief Phantom tags that distinguish dimension types at compile time.
    /// `BCLIBC_Distance<BCLIBC_Meter>` and `BCLIBC_Velocity<BCLIBC_Meter>` are distinct types even though
    /// both use the `BCLIBC_Meter` unit tag.
    struct BCLIBC_AngularDimTag
    {
    };
    struct BCLIBC_DistanceDimTag
    {
    };
    struct BCLIBC_EnergyDimTag
    {
    };
    struct BCLIBC_PressureDimTag
    {
    };
    struct BCLIBC_TemperatureDimTag
    {
    };
    struct BCLIBC_VelocityDimTag
    {
    };
    struct BCLIBC_WeightDimTag
    {
    };
    struct BCLIBC_TimeDimTag
    {
    };

    // ================= UNIFIED DIMENSION =================

    /**
     * @brief Core type-safe measurement class.
     *
     * Stores a value in the dimension's base unit (`_raw`) via `Unit::to_raw`.
     * All arithmetic and comparisons operate on raw values, ensuring correctness
     * across mixed units within the same dimension.
     *
     * `DimTag` is a phantom type that prevents cross-dimension assignment:
     * `BCLIBC_Distance<BCLIBC_Meter>` and `BCLIBC_Velocity<BCLIBC_Meter>` are unrelated types.
     *
     * `Unit` must expose:
     * - `static constexpr BCLIBC_Unit id`
     * - `static constexpr double to_raw(double v)`
     * - `static constexpr double from_raw(double r)`
     *
     * Use the dimension aliases (`BCLIBC_Distance`, `BCLIBC_Velocity`, etc.) instead of
     * instantiating `BCLIBC_Dimension` directly.
     *
     * @tparam DimTag  Phantom tag identifying the physical dimension.
     * @tparam Unit    Unit tag (e.g. `Meter`, `FPS`, `Celsius`).
     *
     * @code
     * using namespace bclibc;
     *
     * // --- Creation ---
     * BCLIBC_Distance<BCLIBC_Meter>  d(100.0);         // 100 m, stored as 3937.0 inches internally
     * BCLIBC_Velocity<BCLIBC_FPS>    v(2800.0);        // 2800 ft/s, stored as 853.44 m/s internally
     * BCLIBC_Weight<BCLIBC_Grain>    w(175.0);
     *
     * // --- Conversion ---
     * BCLIBC_Distance<BCLIBC_Yard>   dy  = d.to<BCLIBC_Yard>();          // 109.361 yd
     * double           ft  = d.to<BCLIBC_Foot>().value();  // 328.084
     * double           mps = v.to<BCLIBC_MPS>().value();   // 853.44
     *
     * // --- value() vs raw() ---
     * d.value();  // 100.0  (in meters, the declared unit)
     * d.raw();    // 3937.0 (always in inches, the BCLIBC_Distance base unit)
     *
     * // --- Arithmetic (mixed units OK within same dimension) ---
     * BCLIBC_Distance<BCLIBC_Meter> sum  = d + BCLIBC_Distance<BCLIBC_Yard>(50.0);  // adds via raw inches
     * BCLIBC_Distance<BCLIBC_Meter> half = d / 2.0;
     * BCLIBC_Distance<BCLIBC_Meter> big  = 3.0 * d;
     * double ratio         = d / BCLIBC_Distance<BCLIBC_Foot>(328.084); // dimensionless
     *
     * // --- Comparison ---
     * BCLIBC_Distance<BCLIBC_Meter>(1.0) == BCLIBC_Distance<BCLIBC_Yard>(1.0936133); // true
     * BCLIBC_Distance<BCLIBC_Meter>(1.0) <  BCLIBC_Distance<BCLIBC_Kilometer>(1.0);  // true
     *
     * // --- Cross-dimension safety (compile error) ---
     * // BCLIBC_Distance<BCLIBC_Meter> x = BCLIBC_Velocity<BCLIBC_MPS>(1.0);  // error: incompatible DimTag
     *
     * // --- As struct fields ---
     * struct BCLIBC_Bullet {
     *     BCLIBC_Weight<BCLIBC_Grain>  weight;
     *     BCLIBC_Velocity<BCLIBC_FPS>  muzzle_velocity;
     * };
     * BCLIBC_Bullet b{ BCLIBC_Weight<BCLIBC_Grain>(175.0), BCLIBC_Velocity<BCLIBC_FPS>(2800.0) };
     * double kg = b.weight.to<BCLIBC_Kilogram>().value(); // 0.01134
     *
     * // --- Generic field accepting any unit ---
     * struct BCLIBC_Target {
     *     BCLIBC_Distance<BCLIBC_Meter> range;
     *     template<typename U>
     *     explicit BCLIBC_Target(BCLIBC_Distance<U> r) : range(r.to<BCLIBC_Meter>()) {}
     * };
     * BCLIBC_Target t(BCLIBC_Distance<BCLIBC_Yard>(500.0));
     * @endcode
     */
    template <typename DimTag, typename Unit>
    class BCLIBC_Dimension
    {
        double _raw;

        struct raw_tag
        {
        };
        constexpr BCLIBC_Dimension(raw_tag, double raw) : _raw(raw) {}

    public:
        /// The unit tag type this instance was constructed with.
        using unit = Unit;

        /// Construct from a value in `Unit`'s scale.
        constexpr explicit BCLIBC_Dimension(double v)
            : _raw(Unit::to_raw(v)) {}

        /// Construct directly from a raw (base-unit) value — bypasses `to_raw`.
        constexpr static BCLIBC_Dimension from_raw(double raw)
        {
            return BCLIBC_Dimension(raw_tag{}, raw);
        }

        /// Raw value in the dimension's base unit (e.g. inches for Distance).
        constexpr double raw() const { return _raw; }

        /// Value in the unit this object was declared with.
        constexpr double value() const { return Unit::from_raw(_raw); }

        /// Convert to a different unit within the same dimension.
        /// @code
        /// BCLIBC_Distance<BCLIBC_Meter> d(100.0);
        /// auto yd = d.to<BCLIBC_Yard>();   // BCLIBC_Distance<BCLIBC_Yard>(109.361)
        /// auto ft = d.to<BCLIBC_Foot>();   // BCLIBC_Distance<BCLIBC_Foot>(328.084)
        /// @endcode
        template <typename OtherUnit>
        constexpr BCLIBC_Dimension<DimTag, OtherUnit> to() const
        {
            return BCLIBC_Dimension<DimTag, OtherUnit>::from_raw(_raw);
        }

        /// Get value in the specified unit without creating a new object.
        /// @code
        /// double meters = d.getIn<BCLIBC_Meter>();   // 100.0
        /// double yards  = d.getIn<BCLIBC_Yard>();    // 109.361
        /// @endcode
        template <typename OtherUnit>
        constexpr double getIn() const { return to<OtherUnit>().value(); }

        // ===== Arithmetic =====

        /// Negate in the same dimension (any units). Result is in `Unit`.
        constexpr BCLIBC_Dimension operator-() const
        {
            return from_raw(-_raw);
        }

        /// Add two measurements of the same dimension (any units). Result is in `Unit`.
        template <typename U2>
        constexpr BCLIBC_Dimension operator+(const BCLIBC_Dimension<DimTag, U2> &o) const
        {
            return from_raw(_raw + o.raw());
        }

        /// Add-assign another measurement (any units).
        template <typename U2>
        constexpr BCLIBC_Dimension &operator+=(const BCLIBC_Dimension<DimTag, U2> &o)
        {
            _raw += o.raw();
            return *this;
        }

        /// Subtract two measurements of the same dimension (any units). Result is in `Unit`.
        template <typename U2>
        constexpr BCLIBC_Dimension operator-(const BCLIBC_Dimension<DimTag, U2> &o) const
        {
            return from_raw(_raw - o.raw());
        }

        /// Subtract-assign another measurement (any units).
        template <typename U2>
        constexpr BCLIBC_Dimension &operator-=(const BCLIBC_Dimension<DimTag, U2> &o)
        {
            _raw -= o.raw();
            return *this;
        }

        /// Scale by a dimensionless scalar.
        constexpr BCLIBC_Dimension operator*(double s) const { return from_raw(_raw * s); }

        /// Scale-assign by a dimensionless scalar.
        constexpr BCLIBC_Dimension &operator*=(double s)
        {
            _raw *= s;
            return *this;
        }

        /// Divide by a dimensionless scalar.
        constexpr BCLIBC_Dimension operator/(double s) const
        {
            assert(std::abs(s) > unit_constants::zeroDivTol && "BCLIBC_Dimension: division by zero scalar!");
            return from_raw(_raw / s);
        }

        /// Divide-assign by a dimensionless scalar.
        constexpr BCLIBC_Dimension &operator/=(double s)
        {
            assert(std::abs(s) > unit_constants::zeroDivTol && "BCLIBC_Dimension: division by zero scalar!");
            _raw /= s;
            return *this;
        }

        /// `scalar * dimension` convenience form.
        friend constexpr BCLIBC_Dimension operator*(double s, const BCLIBC_Dimension &d) { return d * s; }

        /// Divide two measurements of the same dimension — returns a dimensionless ratio.
        /// @code
        /// double r = BCLIBC_Distance<BCLIBC_Meter>(200.0) / BCLIBC_Distance<BCLIBC_Meter>(100.0); // 2.0
        /// @endcode
        template <typename U2>
        constexpr double operator/(const BCLIBC_Dimension<DimTag, U2> &o) const
        {
            assert(std::abs(o.raw()) > unit_constants::zeroDivTol && "Division by zero dimension!");
            return _raw / o.raw();
        }

        // ===== Comparison =====

        /// Equality with raw-unit tolerance.
        template <typename U2>
        bool operator==(const BCLIBC_Dimension<DimTag, U2> &o) const
        {
            return std::abs(_raw - o.raw()) < unit_constants::tolerance;
        }

        template <typename U2>
        bool operator!=(const BCLIBC_Dimension<DimTag, U2> &o) const { return !(*this == o); }

        template <typename U2>
        constexpr bool operator<(const BCLIBC_Dimension<DimTag, U2> &o) const { return _raw < o.raw(); }

        template <typename U2>
        constexpr bool operator>(const BCLIBC_Dimension<DimTag, U2> &o) const { return _raw > o.raw(); }

        template <typename U2>
        constexpr bool operator<=(const BCLIBC_Dimension<DimTag, U2> &o) const { return _raw <= o.raw(); }

        template <typename U2>
        constexpr bool operator>=(const BCLIBC_Dimension<DimTag, U2> &o) const { return _raw >= o.raw(); }

        /// Prints `value()` (in the declared unit).
        friend std::ostream &operator<<(std::ostream &os, const BCLIBC_Dimension &d)
        {
            return os << d.value();
        }
    };

    // ================= TYPE ALIASES =================

    /**
     * @name Dimension aliases
     * Convenience aliases over `BCLIBC_Dimension<DimTag, Unit>`.
     * @{
     * @code
     * BCLIBC_Angular<BCLIBC_Degree>      a(45.0);
     * BCLIBC_Distance<BCLIBC_Meter>      d(100.0);
     * BCLIBC_Energy<BCLIBC_Joule>        e(3500.0);
     * BCLIBC_Pressure<BCLIBC_InHg>       p(29.92);
     * BCLIBC_Temperature<BCLIBC_Celsius> t(15.0);
     * BCLIBC_Velocity<BCLIBC_MPS>        v(900.0);
     * BCLIBC_Weight<BCLIBC_Gram>         w(11.34);
     * BCLIBC_Time<BCLIBC_Millisecond>    tm(500.0);
     * @endcode
     */
    template <typename Unit>
    using BCLIBC_Angular = BCLIBC_Dimension<BCLIBC_AngularDimTag, Unit>;
    template <typename Unit>
    using BCLIBC_Distance = BCLIBC_Dimension<BCLIBC_DistanceDimTag, Unit>;
    template <typename Unit>
    using BCLIBC_Energy = BCLIBC_Dimension<BCLIBC_EnergyDimTag, Unit>;
    template <typename Unit>
    using BCLIBC_Pressure = BCLIBC_Dimension<BCLIBC_PressureDimTag, Unit>;
    template <typename Unit>
    using BCLIBC_Temperature = BCLIBC_Dimension<BCLIBC_TemperatureDimTag, Unit>;
    template <typename Unit>
    using BCLIBC_Velocity = BCLIBC_Dimension<BCLIBC_VelocityDimTag, Unit>;
    template <typename Unit>
    using BCLIBC_Weight = BCLIBC_Dimension<BCLIBC_WeightDimTag, Unit>;
    template <typename Unit>
    using BCLIBC_Time = BCLIBC_Dimension<BCLIBC_TimeDimTag, Unit>;
    /** @} */

    // ================= FACTORY HELPERS =================

    /**
     * @name Factory helpers
     * Alternative to the constructor when the unit must be inferred from context.
     * @{
     * @code
     * auto d = BCLIBC_make_distance<BCLIBC_Yard>(100.0);   // BCLIBC_Distance<BCLIBC_Yard>
     * auto v = BCLIBC_make_velocity<BCLIBC_MPS>(340.0);    // BCLIBC_Velocity<BCLIBC_MPS>
     * auto t = BCLIBC_make_temperature<BCLIBC_Kelvin>(293.15); // BCLIBC_Temperature<BCLIBC_Kelvin>
     * @endcode
     */
    template <typename Unit>
    constexpr BCLIBC_Angular<Unit> BCLIBC_make_angular(double v) { return BCLIBC_Angular<Unit>(v); }
    template <typename Unit>
    constexpr BCLIBC_Distance<Unit> BCLIBC_make_distance(double v) { return BCLIBC_Distance<Unit>(v); }
    template <typename Unit>
    constexpr BCLIBC_Energy<Unit> BCLIBC_make_energy(double v) { return BCLIBC_Energy<Unit>(v); }
    template <typename Unit>
    constexpr BCLIBC_Pressure<Unit> BCLIBC_make_pressure(double v) { return BCLIBC_Pressure<Unit>(v); }
    template <typename Unit>
    constexpr BCLIBC_Temperature<Unit> BCLIBC_make_temperature(double v) { return BCLIBC_Temperature<Unit>(v); }
    template <typename Unit>
    constexpr BCLIBC_Velocity<Unit> BCLIBC_make_velocity(double v) { return BCLIBC_Velocity<Unit>(v); }
    template <typename Unit>
    constexpr BCLIBC_Weight<Unit> BCLIBC_make_weight(double v) { return BCLIBC_Weight<Unit>(v); }
    template <typename Unit>
    constexpr BCLIBC_Time<Unit> BCLIBC_make_time(double v) { return BCLIBC_Time<Unit>(v); }
    /** @} */

    // ================= ENUM ↔ TYPE BRIDGE =================

    /**
     * @brief Compile-time mapping from `BCLIBC_Unit` enum value to a unit tag type.
     *
     * @code
     * using T = BCLIBC_unit_from_enum<BCLIBC_Unit::Meter>::type;   // → Meter
     * BCLIBC_Distance<T> d(42.0);  // BCLIBC_Distance<BCLIBC_Meter>
     *
     * // Useful in generic FFI wrappers:
     * template<BCLIBC_Unit U>
     * auto wrap(double v) { return BCLIBC_Distance<typename BCLIBC_unit_from_enum<U>::type>(v); }
     * @endcode
     */
    template <BCLIBC_Unit U>
    struct BCLIBC_unit_from_enum;

    // Angular
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Radian>
    {
        using type = BCLIBC_Radian;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Degree>
    {
        using type = BCLIBC_Degree;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::MOA>
    {
        using type = BCLIBC_MOA;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Mil>
    {
        using type = BCLIBC_Mil;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::MRad>
    {
        using type = BCLIBC_MRad;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Thousandth>
    {
        using type = BCLIBC_Thousandth;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::InchesPer100Yd>
    {
        using type = BCLIBC_InchesPer100Yd;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::CmPer100m>
    {
        using type = BCLIBC_CmPer100m;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::OClock>
    {
        using type = BCLIBC_OClock;
    };

    // Distance
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Inch>
    {
        using type = BCLIBC_Inch;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Foot>
    {
        using type = BCLIBC_Foot;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Yard>
    {
        using type = BCLIBC_Yard;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Mile>
    {
        using type = BCLIBC_Mile;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::NauticalMile>
    {
        using type = BCLIBC_NauticalMile;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Millimeter>
    {
        using type = BCLIBC_Millimeter;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Centimeter>
    {
        using type = BCLIBC_Centimeter;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Meter>
    {
        using type = BCLIBC_Meter;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Kilometer>
    {
        using type = BCLIBC_Kilometer;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Line>
    {
        using type = BCLIBC_Line;
    };

    // Energy
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::FootPound>
    {
        using type = BCLIBC_FootPound;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Joule>
    {
        using type = BCLIBC_Joule;
    };

    // Pressure
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::MmHg>
    {
        using type = BCLIBC_MmHg;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::InHg>
    {
        using type = BCLIBC_InHg;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Bar>
    {
        using type = BCLIBC_Bar;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::hPa>
    {
        using type = BCLIBC_hPa;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::PSI>
    {
        using type = BCLIBC_PSI;
    };

    // Temperature
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Fahrenheit>
    {
        using type = BCLIBC_Fahrenheit;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Celsius>
    {
        using type = BCLIBC_Celsius;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Kelvin>
    {
        using type = BCLIBC_Kelvin;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Rankin>
    {
        using type = BCLIBC_Rankin;
    };

    // Velocity
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::MPS>
    {
        using type = BCLIBC_MPS;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::KMH>
    {
        using type = BCLIBC_KMH;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::FPS>
    {
        using type = BCLIBC_FPS;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::MPH>
    {
        using type = BCLIBC_MPH;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::KT>
    {
        using type = BCLIBC_KT;
    };

    // Weight
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Grain>
    {
        using type = BCLIBC_Grain;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Ounce>
    {
        using type = BCLIBC_Ounce;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Gram>
    {
        using type = BCLIBC_Gram;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Pound>
    {
        using type = BCLIBC_Pound;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Kilogram>
    {
        using type = BCLIBC_Kilogram;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Newton>
    {
        using type = BCLIBC_Newton;
    };

    // Time
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Minute>
    {
        using type = BCLIBC_Minute;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Second>
    {
        using type = BCLIBC_Second;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Millisecond>
    {
        using type = BCLIBC_Millisecond;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Microsecond>
    {
        using type = BCLIBC_Microsecond;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Nanosecond>
    {
        using type = BCLIBC_Nanosecond;
    };
    template <>
    struct BCLIBC_unit_from_enum<BCLIBC_Unit::Picosecond>
    {
        using type = BCLIBC_Picosecond;
    };

    // ================= RUNTIME CONVERSION =================

    /**
     * @brief Returns the linear scaling `factor` for a given unit (raw = value × factor).
     *
     * Temperature units are not linear — returns `0.0` for them.
     * Use `BCLIBC_convert_temperature()` for temperature at runtime.
     *
     * @code
     * double f = BCLIBC_unit_factor(BCLIBC_Unit::Meter);  // 1000/25.4 ≈ 39.37
     * double f2 = BCLIBC_unit_factor(BCLIBC_Unit::Celsius); // 0.0 — not linear
     * @endcode
     */
    inline double BCLIBC_unit_factor(BCLIBC_Unit u)
    {
        switch (u)
        {
        case BCLIBC_Unit::Radian:
            return BCLIBC_Radian::factor;
        case BCLIBC_Unit::Degree:
            return BCLIBC_Degree::factor;
        case BCLIBC_Unit::MOA:
            return BCLIBC_MOA::factor;
        case BCLIBC_Unit::Mil:
            return BCLIBC_Mil::factor;
        case BCLIBC_Unit::MRad:
            return BCLIBC_MRad::factor;
        case BCLIBC_Unit::Thousandth:
            return BCLIBC_Thousandth::factor;
        case BCLIBC_Unit::InchesPer100Yd:
            return BCLIBC_InchesPer100Yd::factor;
        case BCLIBC_Unit::CmPer100m:
            return BCLIBC_CmPer100m::factor;
        case BCLIBC_Unit::OClock:
            return BCLIBC_OClock::factor;

        case BCLIBC_Unit::Inch:
            return BCLIBC_Inch::factor;
        case BCLIBC_Unit::Foot:
            return BCLIBC_Foot::factor;
        case BCLIBC_Unit::Yard:
            return BCLIBC_Yard::factor;
        case BCLIBC_Unit::Mile:
            return BCLIBC_Mile::factor;
        case BCLIBC_Unit::NauticalMile:
            return BCLIBC_NauticalMile::factor;
        case BCLIBC_Unit::Millimeter:
            return BCLIBC_Millimeter::factor;
        case BCLIBC_Unit::Centimeter:
            return BCLIBC_Centimeter::factor;
        case BCLIBC_Unit::Meter:
            return BCLIBC_Meter::factor;
        case BCLIBC_Unit::Kilometer:
            return BCLIBC_Kilometer::factor;
        case BCLIBC_Unit::Line:
            return BCLIBC_Line::factor;

        case BCLIBC_Unit::FootPound:
            return BCLIBC_FootPound::factor;
        case BCLIBC_Unit::Joule:
            return BCLIBC_Joule::factor;

        case BCLIBC_Unit::MmHg:
            return BCLIBC_MmHg::factor;
        case BCLIBC_Unit::InHg:
            return BCLIBC_InHg::factor;
        case BCLIBC_Unit::Bar:
            return BCLIBC_Bar::factor;
        case BCLIBC_Unit::hPa:
            return BCLIBC_hPa::factor;
        case BCLIBC_Unit::PSI:
            return BCLIBC_PSI::factor;

        case BCLIBC_Unit::MPS:
            return BCLIBC_MPS::factor;
        case BCLIBC_Unit::KMH:
            return BCLIBC_KMH::factor;
        case BCLIBC_Unit::FPS:
            return BCLIBC_FPS::factor;
        case BCLIBC_Unit::MPH:
            return BCLIBC_MPH::factor;
        case BCLIBC_Unit::KT:
            return BCLIBC_KT::factor;

        case BCLIBC_Unit::Grain:
            return BCLIBC_Grain::factor;
        case BCLIBC_Unit::Ounce:
            return BCLIBC_Ounce::factor;
        case BCLIBC_Unit::Gram:
            return BCLIBC_Gram::factor;
        case BCLIBC_Unit::Pound:
            return BCLIBC_Pound::factor;
        case BCLIBC_Unit::Kilogram:
            return BCLIBC_Kilogram::factor;
        case BCLIBC_Unit::Newton:
            return BCLIBC_Newton::factor;

        case BCLIBC_Unit::Minute:
            return BCLIBC_Minute::factor;
        case BCLIBC_Unit::Second:
            return BCLIBC_Second::factor;
        case BCLIBC_Unit::Millisecond:
            return BCLIBC_Millisecond::factor;
        case BCLIBC_Unit::Microsecond:
            return BCLIBC_Microsecond::factor;
        case BCLIBC_Unit::Nanosecond:
            return BCLIBC_Nanosecond::factor;
        case BCLIBC_Unit::Picosecond:
            return BCLIBC_Picosecond::factor;

        default:
            return 0.0;
        }
    }

    /**
     * @brief Returns dimension ID
     * 0: Angular, 1: Distance, 3: Energy, 4: Pressure, 5: Temperature, 6: Velocity, 7: Weight, 8: Time.
     */
    inline int BCLIBC_get_dimension_id(BCLIBC_Unit u)
    {
        return static_cast<int>(u) / 10;
    }

    /**
     * @brief Runtime conversion for **BCLIBC_Temperature** (affine — offset + scale).
     *
     * @code
     * double f = BCLIBC_convert_temperature(100.0, BCLIBC_Unit::Celsius,    BCLIBC_Unit::Fahrenheit); // 212.0
     * double k = BCLIBC_convert_temperature(100.0, BCLIBC_Unit::Celsius,    BCLIBC_Unit::Kelvin);     // 373.15
     * double c = BCLIBC_convert_temperature(98.6,  BCLIBC_Unit::Fahrenheit, BCLIBC_Unit::Celsius);    // 37.0
     * @endcode
     */
    inline double BCLIBC_convert_temperature(double v, BCLIBC_Unit from, BCLIBC_Unit to)
    {
        double raw_f;
        switch (from)
        {
        case BCLIBC_Unit::Fahrenheit:
            raw_f = BCLIBC_Fahrenheit::to_raw(v);
            break;
        case BCLIBC_Unit::Celsius:
            raw_f = BCLIBC_Celsius::to_raw(v);
            break;
        case BCLIBC_Unit::Kelvin:
            raw_f = BCLIBC_Kelvin::to_raw(v);
            break;
        case BCLIBC_Unit::Rankin:
            raw_f = BCLIBC_Rankin::to_raw(v);
            break;
        default:
            raw_f = v;
            break;
        }
        switch (to)
        {
        case BCLIBC_Unit::Fahrenheit:
            return BCLIBC_Fahrenheit::from_raw(raw_f);
        case BCLIBC_Unit::Celsius:
            return BCLIBC_Celsius::from_raw(raw_f);
        case BCLIBC_Unit::Kelvin:
            return BCLIBC_Kelvin::from_raw(raw_f);
        case BCLIBC_Unit::Rankin:
            return BCLIBC_Rankin::from_raw(raw_f);
        default:
            return raw_f;
        }
    }

    /**
     * @brief Runtime conversion for all dimensions.
     *
     * Equivalent to the compile-time `d.to<BCLIBC_OtherUnit>().value()` but for
     * units known only at runtime (e.g. from user config or FFI).
     *
     * @code
     * double ft  = BCLIBC_convert(100.0, BCLIBC_Unit::Meter, BCLIBC_Unit::Foot);  // 328.084
     * double moa = BCLIBC_convert(1.0,   BCLIBC_Unit::Mil,   BCLIBC_Unit::MOA);   // 3.438
     * double fps = BCLIBC_convert(900.0, BCLIBC_Unit::MPS,   BCLIBC_Unit::FPS);   // 2952.76
     * @endcode
     */
    inline double BCLIBC_convert(double v, BCLIBC_Unit from, BCLIBC_Unit to)
    {
        if (BCLIBC_get_dimension_id(from) != BCLIBC_get_dimension_id(to))
        {
            assert(from == to && "Units must belong to the same dimension!");
            return NAN;
        }

        int dim = BCLIBC_get_dimension_id(from);
        switch (dim)
        {
        case 5:
            return BCLIBC_convert_temperature(v, from, to);
        default:
        {
            double to_factor = BCLIBC_unit_factor(to);
            if (std::abs(to_factor) < unit_constants::zeroDivTol)
            {
                assert(false && "Unit factor cannot be zero!");
                return NAN;
            }
            return v * BCLIBC_unit_factor(from) / to_factor;
        }
        }
    }

} // namespace bclibc

#endif // BCLIBC_UNIT_HPP
