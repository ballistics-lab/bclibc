#ifndef BCLIBC_UNIT_HPP
#define BCLIBC_UNIT_HPP

/**
 * @file unit.hpp
 * @brief Type-safe, header-only physical unit system for ballistic calculations.
 *
 * Provides compile-time unit safety via `Dimension<DimTag, Unit>` — a single template
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
 * Distance<Meter>      d(100.0);            // 100 m
 * Distance<Yard>       dy = d.to<Yard>();   // → 109.361 yd
 * double               ft = d.to<Foot>().value(); // → 328.084
 *
 * Velocity<FPS>        v(2800.0);
 * double               mps = v.to<MPS>().value(); // → 853.44
 *
 * Temperature<Celsius> t(20.0);
 * double               k = t.to<Kelvin>().value(); // → 293.15
 *
 * // Arithmetic (same dimension, any units)
 * Distance<Meter> sum = d + Distance<Yard>(50.0).to<Meter>();
 * Distance<Meter> scaled = d * 2.0;
 *
 * // Runtime conversion
 * double val = convert_linear(100.0, BCLIBC_Unit::Meter, BCLIBC_Unit::Foot); // 328.084
 * double tmp = convert_temperature(100.0, BCLIBC_Unit::Celsius, BCLIBC_Unit::Fahrenheit); // 212.0
 * @endcode
 */

#include <type_traits>
#include <cmath>
#include <iostream>

namespace bclibc
{

// ================= MATH CONSTANT =================

namespace detail
{
    static constexpr double pi = 3.14159265358979323846;
} // namespace detail

// ================= UNIT ENUM =================

/**
 * @brief Enum of all supported unit identifiers.
 *
 * Values match py-ballisticcalc `Unit` enum exactly, enabling round-trip
 * interoperability with the Python library over FFI.
 *
 * @code
 * BCLIBC_Unit u = BCLIBC_Unit::Meter;
 * double f = unit_factor(u);  // factor for linear conversion
 *
 * // Compile-time enum → type bridge:
 * using T = unit_from_enum<BCLIBC_Unit::Kilometer>::type; // → Kilometer tag
 * Distance<T> d(5.0);  // Distance<Kilometer>
 * @endcode
 */
enum class BCLIBC_Unit : int
{
    // Angular [0–8]
    Radian         = 0,
    Degree         = 1,
    MOA            = 2,
    Mil            = 3,
    MRad           = 4,
    Thousandth     = 5,
    InchesPer100Yd = 6,
    CmPer100m      = 7,
    OClock         = 8,

    // Distance [10–19]
    Inch           = 10,
    Foot           = 11,
    Yard           = 12,
    Mile           = 13,
    NauticalMile   = 14,
    Millimeter     = 15,
    Centimeter     = 16,
    Meter          = 17,
    Kilometer      = 18,
    Line           = 19,

    // Energy [30–31]
    FootPound      = 30,
    Joule          = 31,

    // Pressure [40–44]
    MmHg           = 40,
    InHg           = 41,
    Bar            = 42,
    hPa            = 43,
    PSI            = 44,

    // Temperature [50–53]
    Fahrenheit     = 50,
    Celsius        = 51,
    Kelvin         = 52,
    Rankin         = 53,

    // Velocity [60–64]
    MPS            = 60,
    KMH            = 61,
    FPS            = 62,
    MPH            = 63,
    KT             = 64,

    // Weight [70–75]
    Grain          = 70,
    Ounce          = 71,
    Gram           = 72,
    Pound          = 73,
    Kilogram       = 74,
    Newton         = 75,

    // Time [80–85]
    Minute         = 80,
    Second         = 81,
    Millisecond    = 82,
    Microsecond    = 83,
    Nanosecond     = 84,
    Picosecond     = 85,
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
template<typename Derived>
struct UnitTag
{
    static constexpr BCLIBC_Unit id = Derived::id;

    static constexpr double to_raw(double v)   { return v * Derived::factor; }
    static constexpr double from_raw(double r) { return r / Derived::factor; }
};

// ---- Angular tags (raw = radians) ----

struct Radian         : UnitTag<Radian>         { static constexpr double factor = 1.0;                               static constexpr auto id = BCLIBC_Unit::Radian; };
struct Degree         : UnitTag<Degree>         { static constexpr double factor = detail::pi / 180.0;                static constexpr auto id = BCLIBC_Unit::Degree; };
struct MOA            : UnitTag<MOA>            { static constexpr double factor = detail::pi / (60.0 * 180.0);       static constexpr auto id = BCLIBC_Unit::MOA; };
struct Mil            : UnitTag<Mil>            { static constexpr double factor = detail::pi / 3200.0;               static constexpr auto id = BCLIBC_Unit::Mil; };
struct MRad           : UnitTag<MRad>           { static constexpr double factor = 1.0 / 1000.0;                      static constexpr auto id = BCLIBC_Unit::MRad; };
struct Thousandth     : UnitTag<Thousandth>     { static constexpr double factor = detail::pi / 3000.0;               static constexpr auto id = BCLIBC_Unit::Thousandth; };
struct InchesPer100Yd : UnitTag<InchesPer100Yd> { static constexpr double factor = 1.0 / 3600.0;                      static constexpr auto id = BCLIBC_Unit::InchesPer100Yd; };
struct CmPer100m      : UnitTag<CmPer100m>      { static constexpr double factor = 1.0 / 10000.0;                     static constexpr auto id = BCLIBC_Unit::CmPer100m; };
struct OClock         : UnitTag<OClock>         { static constexpr double factor = detail::pi / 6.0;                  static constexpr auto id = BCLIBC_Unit::OClock; };

// ---- Distance tags (raw = inches) ----

struct Inch           : UnitTag<Inch>           { static constexpr double factor = 1.0;                               static constexpr auto id = BCLIBC_Unit::Inch; };
struct Foot           : UnitTag<Foot>           { static constexpr double factor = 12.0;                              static constexpr auto id = BCLIBC_Unit::Foot; };
struct Yard           : UnitTag<Yard>           { static constexpr double factor = 36.0;                              static constexpr auto id = BCLIBC_Unit::Yard; };
struct Mile           : UnitTag<Mile>           { static constexpr double factor = 63360.0;                           static constexpr auto id = BCLIBC_Unit::Mile; };
struct NauticalMile   : UnitTag<NauticalMile>   { static constexpr double factor = 72913.3858;                        static constexpr auto id = BCLIBC_Unit::NauticalMile; };
struct Millimeter     : UnitTag<Millimeter>     { static constexpr double factor = 1.0 / 25.4;                        static constexpr auto id = BCLIBC_Unit::Millimeter; };
struct Centimeter     : UnitTag<Centimeter>     { static constexpr double factor = 10.0 / 25.4;                       static constexpr auto id = BCLIBC_Unit::Centimeter; };
struct Meter          : UnitTag<Meter>          { static constexpr double factor = 1000.0 / 25.4;                     static constexpr auto id = BCLIBC_Unit::Meter; };
struct Kilometer      : UnitTag<Kilometer>      { static constexpr double factor = 1000000.0 / 25.4;                  static constexpr auto id = BCLIBC_Unit::Kilometer; };
struct Line           : UnitTag<Line>           { static constexpr double factor = 0.1;                               static constexpr auto id = BCLIBC_Unit::Line; };

// ---- Energy tags (raw = foot-pounds) ----

struct FootPound      : UnitTag<FootPound>      { static constexpr double factor = 1.0;                               static constexpr auto id = BCLIBC_Unit::FootPound; };
struct Joule          : UnitTag<Joule>          { static constexpr double factor = 1.0 / 1.3558179483314;             static constexpr auto id = BCLIBC_Unit::Joule; };

// ---- Pressure tags (raw = mmHg) ----

struct MmHg           : UnitTag<MmHg>           { static constexpr double factor = 1.0;                               static constexpr auto id = BCLIBC_Unit::MmHg; };
struct InHg           : UnitTag<InHg>           { static constexpr double factor = 25.4;                              static constexpr auto id = BCLIBC_Unit::InHg; };
struct Bar            : UnitTag<Bar>            { static constexpr double factor = 750.061683;                        static constexpr auto id = BCLIBC_Unit::Bar; };
struct hPa            : UnitTag<hPa>            { static constexpr double factor = 750.061683 / 1000.0;               static constexpr auto id = BCLIBC_Unit::hPa; };
struct PSI            : UnitTag<PSI>            { static constexpr double factor = 51.714924102396;                   static constexpr auto id = BCLIBC_Unit::PSI; };

// ---- Velocity tags (raw = m/s) ----

struct MPS            : UnitTag<MPS>            { static constexpr double factor = 1.0;                               static constexpr auto id = BCLIBC_Unit::MPS; };
struct KMH            : UnitTag<KMH>            { static constexpr double factor = 1.0 / 3.6;                         static constexpr auto id = BCLIBC_Unit::KMH; };
struct FPS            : UnitTag<FPS>            { static constexpr double factor = 1.0 / 3.2808399;                   static constexpr auto id = BCLIBC_Unit::FPS; };
struct MPH            : UnitTag<MPH>            { static constexpr double factor = 1.0 / 2.23693629;                  static constexpr auto id = BCLIBC_Unit::MPH; };
struct KT             : UnitTag<KT>             { static constexpr double factor = 1.0 / 1.94384449;                  static constexpr auto id = BCLIBC_Unit::KT; };

// ---- Weight tags (raw = grains) ----

struct Grain          : UnitTag<Grain>          { static constexpr double factor = 1.0;                               static constexpr auto id = BCLIBC_Unit::Grain; };
struct Ounce          : UnitTag<Ounce>          { static constexpr double factor = 437.5;                             static constexpr auto id = BCLIBC_Unit::Ounce; };
struct Gram           : UnitTag<Gram>           { static constexpr double factor = 15.4323584;                        static constexpr auto id = BCLIBC_Unit::Gram; };
struct Pound          : UnitTag<Pound>          { static constexpr double factor = 7000.0;                            static constexpr auto id = BCLIBC_Unit::Pound; };
struct Kilogram       : UnitTag<Kilogram>       { static constexpr double factor = 15432.3584;                        static constexpr auto id = BCLIBC_Unit::Kilogram; };
struct Newton         : UnitTag<Newton>         { static constexpr double factor = 1573.662597;                       static constexpr auto id = BCLIBC_Unit::Newton; };

// ---- Time tags (raw = seconds) ----

struct Second         : UnitTag<Second>         { static constexpr double factor = 1.0;                               static constexpr auto id = BCLIBC_Unit::Second; };
struct Minute         : UnitTag<Minute>         { static constexpr double factor = 60.0;                              static constexpr auto id = BCLIBC_Unit::Minute; };
struct Millisecond    : UnitTag<Millisecond>    { static constexpr double factor = 1.0 / 1000.0;                      static constexpr auto id = BCLIBC_Unit::Millisecond; };
struct Microsecond    : UnitTag<Microsecond>    { static constexpr double factor = 1.0 / 1000000.0;                   static constexpr auto id = BCLIBC_Unit::Microsecond; };
struct Nanosecond     : UnitTag<Nanosecond>     { static constexpr double factor = 1.0 / 1000000000.0;                static constexpr auto id = BCLIBC_Unit::Nanosecond; };
struct Picosecond     : UnitTag<Picosecond>     { static constexpr double factor = 1.0 / 1000000000000.0;             static constexpr auto id = BCLIBC_Unit::Picosecond; };

// ---- Temperature tags (raw = °Fahrenheit; affine conversions) ----

/**
 * @brief Temperature unit tags use affine (offset + scale) conversions.
 *
 * Unlike linear tags, these do **not** inherit `UnitTag` and have no `factor`.
 * `to_raw(v)` converts a value in the tag's unit to °Fahrenheit (raw).
 * `from_raw(r)` converts °Fahrenheit back to the tag's unit.
 *
 * @warning Arithmetic between two `Temperature<>` objects adds their raw °F values.
 *          This is only physically meaningful for the Fahrenheit and Rankin scales.
 *          For other scales use `to<>()` for conversion only.
 *
 * @code
 * Temperature<Celsius> tc(20.0);
 * double k = tc.to<Kelvin>().value();   // 293.15 K  ✓
 * double f = tc.to<Fahrenheit>().value(); // 68.0 °F ✓
 * @endcode
 */
struct Fahrenheit
{
    static constexpr BCLIBC_Unit id = BCLIBC_Unit::Fahrenheit;
    static constexpr double to_raw(double v)   { return v; }
    static constexpr double from_raw(double r) { return r; }
};

/// @copydoc Fahrenheit
struct Celsius
{
    static constexpr BCLIBC_Unit id = BCLIBC_Unit::Celsius;
    static constexpr double to_raw(double v)   { return v * 9.0 / 5.0 + 32.0; }
    static constexpr double from_raw(double r) { return (r - 32.0) * 5.0 / 9.0; }
};

/// @copydoc Fahrenheit
struct Kelvin
{
    static constexpr BCLIBC_Unit id = BCLIBC_Unit::Kelvin;
    static constexpr double to_raw(double v)   { return (v - 273.15) * 9.0 / 5.0 + 32.0; }
    static constexpr double from_raw(double r) { return (r - 32.0) * 5.0 / 9.0 + 273.15; }
};

/// @copydoc Fahrenheit
struct Rankin
{
    static constexpr BCLIBC_Unit id = BCLIBC_Unit::Rankin;
    static constexpr double to_raw(double v)   { return v - 459.67; }
    static constexpr double from_raw(double r) { return r + 459.67; }
};

// ================= DIMENSION PHANTOM TAGS =================

/// @brief Phantom tags that distinguish dimension types at compile time.
/// `Distance<Meter>` and `Velocity<Meter>` are distinct types even though
/// both use the `Meter` unit tag.
struct AngularDimTag     {};
struct DistanceDimTag    {};
struct EnergyDimTag      {};
struct PressureDimTag    {};
struct TemperatureDimTag {};
struct VelocityDimTag    {};
struct WeightDimTag      {};
struct TimeDimTag        {};

// ================= UNIFIED DIMENSION =================

/**
 * @brief Core type-safe measurement class.
 *
 * Stores a value in the dimension's base unit (`_raw`) via `Unit::to_raw`.
 * All arithmetic and comparisons operate on raw values, ensuring correctness
 * across mixed units within the same dimension.
 *
 * `DimTag` is a phantom type that prevents cross-dimension assignment:
 * `Distance<Meter>` and `Velocity<Meter>` are unrelated types.
 *
 * `Unit` must expose:
 * - `static constexpr BCLIBC_Unit id`
 * - `static constexpr double to_raw(double v)`
 * - `static constexpr double from_raw(double r)`
 *
 * Use the dimension aliases (`Distance`, `Velocity`, etc.) instead of
 * instantiating `Dimension` directly.
 *
 * @tparam DimTag  Phantom tag identifying the physical dimension.
 * @tparam Unit    Unit tag (e.g. `Meter`, `FPS`, `Celsius`).
 *
 * @code
 * using namespace bclibc;
 *
 * // --- Creation ---
 * Distance<Meter>  d(100.0);         // 100 m, stored as 3937.0 inches internally
 * Velocity<FPS>    v(2800.0);        // 2800 ft/s, stored as 853.44 m/s internally
 * Weight<Grain>    w(175.0);
 *
 * // --- Conversion ---
 * Distance<Yard>   dy  = d.to<Yard>();          // 109.361 yd
 * double           ft  = d.to<Foot>().value();  // 328.084
 * double           mps = v.to<MPS>().value();   // 853.44
 *
 * // --- value() vs raw() ---
 * d.value();  // 100.0  (in meters, the declared unit)
 * d.raw();    // 3937.0 (always in inches, the Distance base unit)
 *
 * // --- Arithmetic (mixed units OK within same dimension) ---
 * Distance<Meter> sum  = d + Distance<Yard>(50.0);  // adds via raw inches
 * Distance<Meter> half = d / 2.0;
 * Distance<Meter> big  = 3.0 * d;
 * double ratio         = d / Distance<Foot>(328.084); // dimensionless
 *
 * // --- Comparison ---
 * Distance<Meter>(1.0) == Distance<Yard>(1.0936133); // true
 * Distance<Meter>(1.0) <  Distance<Kilometer>(1.0);  // true
 *
 * // --- Cross-dimension safety (compile error) ---
 * // Distance<Meter> x = Velocity<MPS>(1.0);  // error: incompatible DimTag
 *
 * // --- As struct fields ---
 * struct Bullet {
 *     Weight<Grain>  weight;
 *     Velocity<FPS>  muzzle_velocity;
 * };
 * Bullet b{ Weight<Grain>(175.0), Velocity<FPS>(2800.0) };
 * double kg = b.weight.to<Kilogram>().value(); // 0.01134
 *
 * // --- Generic field accepting any unit ---
 * struct Target {
 *     Distance<Meter> range;
 *     template<typename U>
 *     explicit Target(Distance<U> r) : range(r.to<Meter>()) {}
 * };
 * Target t(Distance<Yard>(500.0));
 * @endcode
 */
template<typename DimTag, typename Unit>
class Dimension
{
    double _raw;

    struct raw_tag {};
    constexpr Dimension(raw_tag, double raw) : _raw(raw) {}

public:
    /// The unit tag type this instance was constructed with.
    using unit = Unit;

    /// Construct from a value in `Unit`'s scale.
    constexpr explicit Dimension(double v)
        : _raw(Unit::to_raw(v)) {}

    /// Construct directly from a raw (base-unit) value — bypasses `to_raw`.
    constexpr static Dimension from_raw(double raw)
    {
        return Dimension(raw_tag{}, raw);
    }

    /// Raw value in the dimension's base unit (e.g. inches for Distance).
    constexpr double raw()   const { return _raw; }

    /// Value in the unit this object was declared with.
    constexpr double value() const { return Unit::from_raw(_raw); }

    /// Convert to a different unit within the same dimension.
    /// @code
    /// Distance<Meter> d(100.0);
    /// auto yd = d.to<Yard>();   // Distance<Yard>(109.361)
    /// auto ft = d.to<Foot>();   // Distance<Foot>(328.084)
    /// @endcode
    template<typename OtherUnit>
    constexpr Dimension<DimTag, OtherUnit> to() const
    {
        return Dimension<DimTag, OtherUnit>::from_raw(_raw);
    }

    // ===== Arithmetic =====

    /// Add two measurements of the same dimension (any units). Result is in `Unit`.
    template<typename U2>
    constexpr Dimension operator+(const Dimension<DimTag, U2>& o) const
    {
        return from_raw(_raw + o.raw());
    }

    /// Subtract two measurements of the same dimension (any units). Result is in `Unit`.
    template<typename U2>
    constexpr Dimension operator-(const Dimension<DimTag, U2>& o) const
    {
        return from_raw(_raw - o.raw());
    }

    /// Scale by a dimensionless scalar.
    constexpr Dimension operator*(double s) const { return from_raw(_raw * s); }
    /// Divide by a dimensionless scalar.
    constexpr Dimension operator/(double s) const { return from_raw(_raw / s); }

    /// `scalar * dimension` convenience form.
    friend constexpr Dimension operator*(double s, const Dimension& d) { return d * s; }

    /// Divide two measurements of the same dimension — returns a dimensionless ratio.
    /// @code
    /// double r = Distance<Meter>(200.0) / Distance<Meter>(100.0); // 2.0
    /// @endcode
    template<typename U2>
    constexpr double operator/(const Dimension<DimTag, U2>& o) const
    {
        return _raw / o.raw();
    }

    // ===== Comparison =====

    /// Equality with 1e-12 raw-unit tolerance.
    template<typename U2>
    bool operator==(const Dimension<DimTag, U2>& o) const
    {
        return std::abs(_raw - o.raw()) < 1e-12;
    }

    template<typename U2>
    bool operator!=(const Dimension<DimTag, U2>& o) const { return !(*this == o); }

    template<typename U2>
    constexpr bool operator< (const Dimension<DimTag, U2>& o) const { return _raw <  o.raw(); }

    template<typename U2>
    constexpr bool operator> (const Dimension<DimTag, U2>& o) const { return _raw >  o.raw(); }

    template<typename U2>
    constexpr bool operator<=(const Dimension<DimTag, U2>& o) const { return _raw <= o.raw(); }

    template<typename U2>
    constexpr bool operator>=(const Dimension<DimTag, U2>& o) const { return _raw >= o.raw(); }

    /// Prints `value()` (in the declared unit).
    friend std::ostream& operator<<(std::ostream& os, const Dimension& d)
    {
        return os << d.value();
    }
};

// ================= TYPE ALIASES =================

/**
 * @name Dimension aliases
 * Convenience aliases over `Dimension<DimTag, Unit>`.
 * @{
 * @code
 * Angular<Degree>      a(45.0);
 * Distance<Meter>      d(100.0);
 * Energy<Joule>        e(3500.0);
 * Pressure<InHg>       p(29.92);
 * Temperature<Celsius> t(15.0);
 * Velocity<MPS>        v(900.0);
 * Weight<Gram>         w(11.34);
 * Time<Millisecond>    tm(500.0);
 * @endcode
 */
template<typename Unit> using Angular     = Dimension<AngularDimTag,     Unit>;
template<typename Unit> using Distance    = Dimension<DistanceDimTag,    Unit>;
template<typename Unit> using Energy      = Dimension<EnergyDimTag,      Unit>;
template<typename Unit> using Pressure    = Dimension<PressureDimTag,    Unit>;
template<typename Unit> using Temperature = Dimension<TemperatureDimTag, Unit>;
template<typename Unit> using Velocity    = Dimension<VelocityDimTag,    Unit>;
template<typename Unit> using Weight      = Dimension<WeightDimTag,      Unit>;
template<typename Unit> using Time        = Dimension<TimeDimTag,        Unit>;
/** @} */

// ================= FACTORY HELPERS =================

/**
 * @name Factory helpers
 * Alternative to the constructor when the unit must be inferred from context.
 * @{
 * @code
 * auto d = make_distance<Yard>(100.0);   // Distance<Yard>
 * auto v = make_velocity<MPS>(340.0);    // Velocity<MPS>
 * auto t = make_temperature<Kelvin>(293.15); // Temperature<Kelvin>
 * @endcode
 */
template<typename Unit> constexpr Angular<Unit>     make_angular(double v)     { return Angular<Unit>(v); }
template<typename Unit> constexpr Distance<Unit>    make_distance(double v)    { return Distance<Unit>(v); }
template<typename Unit> constexpr Energy<Unit>      make_energy(double v)      { return Energy<Unit>(v); }
template<typename Unit> constexpr Pressure<Unit>    make_pressure(double v)    { return Pressure<Unit>(v); }
template<typename Unit> constexpr Temperature<Unit> make_temperature(double v) { return Temperature<Unit>(v); }
template<typename Unit> constexpr Velocity<Unit>    make_velocity(double v)    { return Velocity<Unit>(v); }
template<typename Unit> constexpr Weight<Unit>      make_weight(double v)      { return Weight<Unit>(v); }
template<typename Unit> constexpr Time<Unit>        make_time(double v)        { return Time<Unit>(v); }
/** @} */

// ================= ENUM ↔ TYPE BRIDGE =================

/**
 * @brief Compile-time mapping from `BCLIBC_Unit` enum value to a unit tag type.
 *
 * @code
 * using T = unit_from_enum<BCLIBC_Unit::Meter>::type;   // → Meter
 * Distance<T> d(42.0);  // Distance<Meter>
 *
 * // Useful in generic FFI wrappers:
 * template<BCLIBC_Unit U>
 * auto wrap(double v) { return Distance<typename unit_from_enum<U>::type>(v); }
 * @endcode
 */
template<BCLIBC_Unit U>
struct unit_from_enum;

// Angular
template<> struct unit_from_enum<BCLIBC_Unit::Radian>         { using type = Radian; };
template<> struct unit_from_enum<BCLIBC_Unit::Degree>         { using type = Degree; };
template<> struct unit_from_enum<BCLIBC_Unit::MOA>            { using type = MOA; };
template<> struct unit_from_enum<BCLIBC_Unit::Mil>            { using type = Mil; };
template<> struct unit_from_enum<BCLIBC_Unit::MRad>           { using type = MRad; };
template<> struct unit_from_enum<BCLIBC_Unit::Thousandth>     { using type = Thousandth; };
template<> struct unit_from_enum<BCLIBC_Unit::InchesPer100Yd> { using type = InchesPer100Yd; };
template<> struct unit_from_enum<BCLIBC_Unit::CmPer100m>      { using type = CmPer100m; };
template<> struct unit_from_enum<BCLIBC_Unit::OClock>         { using type = OClock; };

// Distance
template<> struct unit_from_enum<BCLIBC_Unit::Inch>           { using type = Inch; };
template<> struct unit_from_enum<BCLIBC_Unit::Foot>           { using type = Foot; };
template<> struct unit_from_enum<BCLIBC_Unit::Yard>           { using type = Yard; };
template<> struct unit_from_enum<BCLIBC_Unit::Mile>           { using type = Mile; };
template<> struct unit_from_enum<BCLIBC_Unit::NauticalMile>   { using type = NauticalMile; };
template<> struct unit_from_enum<BCLIBC_Unit::Millimeter>     { using type = Millimeter; };
template<> struct unit_from_enum<BCLIBC_Unit::Centimeter>     { using type = Centimeter; };
template<> struct unit_from_enum<BCLIBC_Unit::Meter>          { using type = Meter; };
template<> struct unit_from_enum<BCLIBC_Unit::Kilometer>      { using type = Kilometer; };
template<> struct unit_from_enum<BCLIBC_Unit::Line>           { using type = Line; };

// Energy
template<> struct unit_from_enum<BCLIBC_Unit::FootPound>      { using type = FootPound; };
template<> struct unit_from_enum<BCLIBC_Unit::Joule>          { using type = Joule; };

// Pressure
template<> struct unit_from_enum<BCLIBC_Unit::MmHg>           { using type = MmHg; };
template<> struct unit_from_enum<BCLIBC_Unit::InHg>           { using type = InHg; };
template<> struct unit_from_enum<BCLIBC_Unit::Bar>            { using type = Bar; };
template<> struct unit_from_enum<BCLIBC_Unit::hPa>            { using type = hPa; };
template<> struct unit_from_enum<BCLIBC_Unit::PSI>            { using type = PSI; };

// Temperature
template<> struct unit_from_enum<BCLIBC_Unit::Fahrenheit>     { using type = Fahrenheit; };
template<> struct unit_from_enum<BCLIBC_Unit::Celsius>        { using type = Celsius; };
template<> struct unit_from_enum<BCLIBC_Unit::Kelvin>         { using type = Kelvin; };
template<> struct unit_from_enum<BCLIBC_Unit::Rankin>         { using type = Rankin; };

// Velocity
template<> struct unit_from_enum<BCLIBC_Unit::MPS>            { using type = MPS; };
template<> struct unit_from_enum<BCLIBC_Unit::KMH>            { using type = KMH; };
template<> struct unit_from_enum<BCLIBC_Unit::FPS>            { using type = FPS; };
template<> struct unit_from_enum<BCLIBC_Unit::MPH>            { using type = MPH; };
template<> struct unit_from_enum<BCLIBC_Unit::KT>             { using type = KT; };

// Weight
template<> struct unit_from_enum<BCLIBC_Unit::Grain>          { using type = Grain; };
template<> struct unit_from_enum<BCLIBC_Unit::Ounce>          { using type = Ounce; };
template<> struct unit_from_enum<BCLIBC_Unit::Gram>           { using type = Gram; };
template<> struct unit_from_enum<BCLIBC_Unit::Pound>          { using type = Pound; };
template<> struct unit_from_enum<BCLIBC_Unit::Kilogram>       { using type = Kilogram; };
template<> struct unit_from_enum<BCLIBC_Unit::Newton>         { using type = Newton; };

// Time
template<> struct unit_from_enum<BCLIBC_Unit::Minute>         { using type = Minute; };
template<> struct unit_from_enum<BCLIBC_Unit::Second>         { using type = Second; };
template<> struct unit_from_enum<BCLIBC_Unit::Millisecond>    { using type = Millisecond; };
template<> struct unit_from_enum<BCLIBC_Unit::Microsecond>    { using type = Microsecond; };
template<> struct unit_from_enum<BCLIBC_Unit::Nanosecond>     { using type = Nanosecond; };
template<> struct unit_from_enum<BCLIBC_Unit::Picosecond>     { using type = Picosecond; };

// ================= RUNTIME CONVERSION =================

/**
 * @brief Returns the linear scaling `factor` for a given unit (raw = value × factor).
 *
 * Temperature units are not linear — returns `0.0` for them.
 * Use `convert_temperature()` for temperature at runtime.
 *
 * @code
 * double f = unit_factor(BCLIBC_Unit::Meter);  // 1000/25.4 ≈ 39.37
 * double f2 = unit_factor(BCLIBC_Unit::Celsius); // 0.0 — not linear
 * @endcode
 */
inline double unit_factor(BCLIBC_Unit u)
{
    switch (u)
    {
    case BCLIBC_Unit::Radian:          return Radian::factor;
    case BCLIBC_Unit::Degree:          return Degree::factor;
    case BCLIBC_Unit::MOA:             return MOA::factor;
    case BCLIBC_Unit::Mil:             return Mil::factor;
    case BCLIBC_Unit::MRad:            return MRad::factor;
    case BCLIBC_Unit::Thousandth:      return Thousandth::factor;
    case BCLIBC_Unit::InchesPer100Yd:  return InchesPer100Yd::factor;
    case BCLIBC_Unit::CmPer100m:       return CmPer100m::factor;
    case BCLIBC_Unit::OClock:          return OClock::factor;

    case BCLIBC_Unit::Inch:            return Inch::factor;
    case BCLIBC_Unit::Foot:            return Foot::factor;
    case BCLIBC_Unit::Yard:            return Yard::factor;
    case BCLIBC_Unit::Mile:            return Mile::factor;
    case BCLIBC_Unit::NauticalMile:    return NauticalMile::factor;
    case BCLIBC_Unit::Millimeter:      return Millimeter::factor;
    case BCLIBC_Unit::Centimeter:      return Centimeter::factor;
    case BCLIBC_Unit::Meter:           return Meter::factor;
    case BCLIBC_Unit::Kilometer:       return Kilometer::factor;
    case BCLIBC_Unit::Line:            return Line::factor;

    case BCLIBC_Unit::FootPound:       return FootPound::factor;
    case BCLIBC_Unit::Joule:           return Joule::factor;

    case BCLIBC_Unit::MmHg:            return MmHg::factor;
    case BCLIBC_Unit::InHg:            return InHg::factor;
    case BCLIBC_Unit::Bar:             return Bar::factor;
    case BCLIBC_Unit::hPa:             return hPa::factor;
    case BCLIBC_Unit::PSI:             return PSI::factor;

    case BCLIBC_Unit::MPS:             return MPS::factor;
    case BCLIBC_Unit::KMH:             return KMH::factor;
    case BCLIBC_Unit::FPS:             return FPS::factor;
    case BCLIBC_Unit::MPH:             return MPH::factor;
    case BCLIBC_Unit::KT:              return KT::factor;

    case BCLIBC_Unit::Grain:           return Grain::factor;
    case BCLIBC_Unit::Ounce:           return Ounce::factor;
    case BCLIBC_Unit::Gram:            return Gram::factor;
    case BCLIBC_Unit::Pound:           return Pound::factor;
    case BCLIBC_Unit::Kilogram:        return Kilogram::factor;
    case BCLIBC_Unit::Newton:          return Newton::factor;

    case BCLIBC_Unit::Minute:          return Minute::factor;
    case BCLIBC_Unit::Second:          return Second::factor;
    case BCLIBC_Unit::Millisecond:     return Millisecond::factor;
    case BCLIBC_Unit::Microsecond:     return Microsecond::factor;
    case BCLIBC_Unit::Nanosecond:      return Nanosecond::factor;
    case BCLIBC_Unit::Picosecond:      return Picosecond::factor;

    default: return 0.0;
    }
}

/**
 * @brief Runtime conversion for all **linear** dimensions.
 *
 * Equivalent to the compile-time `d.to<OtherUnit>().value()` but for
 * units known only at runtime (e.g. from user config or FFI).
 *
 * Do **not** use for Temperature — call `convert_temperature()` instead.
 *
 * @code
 * double ft  = convert_linear(100.0, BCLIBC_Unit::Meter, BCLIBC_Unit::Foot);  // 328.084
 * double moa = convert_linear(1.0,   BCLIBC_Unit::Mil,   BCLIBC_Unit::MOA);   // 3.438
 * double fps = convert_linear(900.0, BCLIBC_Unit::MPS,   BCLIBC_Unit::FPS);   // 2952.76
 * @endcode
 */
inline double convert_linear(double v, BCLIBC_Unit from, BCLIBC_Unit to)
{
    return v * unit_factor(from) / unit_factor(to);
}

/**
 * @brief Runtime conversion for **Temperature** (affine — offset + scale).
 *
 * Cannot use `convert_linear()` for temperature because conversions have
 * an additive offset (e.g. 0 °C ≠ 0 °F).
 *
 * @code
 * double f = convert_temperature(100.0, BCLIBC_Unit::Celsius,    BCLIBC_Unit::Fahrenheit); // 212.0
 * double k = convert_temperature(100.0, BCLIBC_Unit::Celsius,    BCLIBC_Unit::Kelvin);     // 373.15
 * double c = convert_temperature(98.6,  BCLIBC_Unit::Fahrenheit, BCLIBC_Unit::Celsius);    // 37.0
 * @endcode
 */
inline double convert_temperature(double v, BCLIBC_Unit from, BCLIBC_Unit to)
{
    double raw_f;
    switch (from)
    {
    case BCLIBC_Unit::Fahrenheit: raw_f = Fahrenheit::to_raw(v); break;
    case BCLIBC_Unit::Celsius:    raw_f = Celsius::to_raw(v);    break;
    case BCLIBC_Unit::Kelvin:     raw_f = Kelvin::to_raw(v);     break;
    case BCLIBC_Unit::Rankin:     raw_f = Rankin::to_raw(v);     break;
    default: raw_f = v; break;
    }
    switch (to)
    {
    case BCLIBC_Unit::Fahrenheit: return Fahrenheit::from_raw(raw_f);
    case BCLIBC_Unit::Celsius:    return Celsius::from_raw(raw_f);
    case BCLIBC_Unit::Kelvin:     return Kelvin::from_raw(raw_f);
    case BCLIBC_Unit::Rankin:     return Rankin::from_raw(raw_f);
    default: return raw_f;
    }
}

} // namespace bclibc

#endif // BCLIBC_UNIT_HPP
