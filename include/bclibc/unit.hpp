#ifndef BCLIBC_UNIT_HPP
#define BCLIBC_UNIT_HPP

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

// Compatibility alias
using BCLIBC_DistanceUnit = BCLIBC_Unit;

// ================= UNIT TAG BASE =================
//
// Linear units inherit UnitTag<Derived> and define `factor`.
// Default to_raw / from_raw implement the linear mapping:
//   raw = value * factor
//   value = raw / factor
//
// Temperature tags do NOT inherit UnitTag — they override to_raw / from_raw
// with affine conversions (raw unit = Fahrenheit).
//
// Every tag must expose:
//   static constexpr BCLIBC_Unit id
//   static constexpr double to_raw(double v)
//   static constexpr double from_raw(double r)

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

// ---- Temperature tags (raw = Fahrenheit; affine conversions) ----
// Note: arithmetic between two Temperature<> instances sums raw °F values,
// which is meaningful only as a coincidence — use for conversion, not math.

struct Fahrenheit
{
    static constexpr BCLIBC_Unit id = BCLIBC_Unit::Fahrenheit;
    static constexpr double to_raw(double v)   { return v; }
    static constexpr double from_raw(double r) { return r; }
};

struct Celsius
{
    static constexpr BCLIBC_Unit id = BCLIBC_Unit::Celsius;
    static constexpr double to_raw(double v)   { return v * 9.0 / 5.0 + 32.0; }
    static constexpr double from_raw(double r) { return (r - 32.0) * 5.0 / 9.0; }
};

struct Kelvin
{
    static constexpr BCLIBC_Unit id = BCLIBC_Unit::Kelvin;
    static constexpr double to_raw(double v)   { return (v - 273.15) * 9.0 / 5.0 + 32.0; }
    static constexpr double from_raw(double r) { return (r - 32.0) * 5.0 / 9.0 + 273.15; }
};

struct Rankin
{
    static constexpr BCLIBC_Unit id = BCLIBC_Unit::Rankin;
    static constexpr double to_raw(double v)   { return v - 459.67; }
    static constexpr double from_raw(double r) { return r + 459.67; }
};

// ================= DIMENSION PHANTOM TAGS =================

struct AngularDimTag     {};
struct DistanceDimTag    {};
struct EnergyDimTag      {};
struct PressureDimTag    {};
struct TemperatureDimTag {};
struct VelocityDimTag    {};
struct WeightDimTag      {};
struct TimeDimTag        {};

// ================= UNIFIED DIMENSION =================
//
// _raw is stored in the dimension's base unit via Unit::to_raw.
// Conversion between units goes through _raw — no intermediate step needed.
//
// Unit requirements:
//   static constexpr BCLIBC_Unit id
//   static constexpr double to_raw(double v)
//   static constexpr double from_raw(double r)

template<typename DimTag, typename Unit>
class Dimension
{
    double _raw;

    struct raw_tag {};
    constexpr Dimension(raw_tag, double raw) : _raw(raw) {}

public:
    using unit = Unit;

    constexpr explicit Dimension(double v)
        : _raw(Unit::to_raw(v)) {}

    constexpr static Dimension from_raw(double raw)
    {
        return Dimension(raw_tag{}, raw);
    }

    constexpr double raw()   const { return _raw; }
    constexpr double value() const { return Unit::from_raw(_raw); }

    template<typename OtherUnit>
    constexpr Dimension<DimTag, OtherUnit> to() const
    {
        return Dimension<DimTag, OtherUnit>::from_raw(_raw);
    }

    // ===== Arithmetic =====

    template<typename U2>
    constexpr Dimension operator+(const Dimension<DimTag, U2>& o) const
    {
        return from_raw(_raw + o.raw());
    }

    template<typename U2>
    constexpr Dimension operator-(const Dimension<DimTag, U2>& o) const
    {
        return from_raw(_raw - o.raw());
    }

    constexpr Dimension operator*(double s) const { return from_raw(_raw * s); }
    constexpr Dimension operator/(double s) const { return from_raw(_raw / s); }

    friend constexpr Dimension operator*(double s, const Dimension& d) { return d * s; }

    template<typename U2>
    constexpr double operator/(const Dimension<DimTag, U2>& o) const
    {
        return _raw / o.raw();
    }

    // ===== Comparison =====

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

    friend std::ostream& operator<<(std::ostream& os, const Dimension& d)
    {
        return os << d.value();
    }
};

// ================= TYPE ALIASES =================

template<typename Unit> using Angular     = Dimension<AngularDimTag,     Unit>;
template<typename Unit> using Distance    = Dimension<DistanceDimTag,    Unit>;
template<typename Unit> using Energy      = Dimension<EnergyDimTag,      Unit>;
template<typename Unit> using Pressure    = Dimension<PressureDimTag,    Unit>;
template<typename Unit> using Temperature = Dimension<TemperatureDimTag, Unit>;
template<typename Unit> using Velocity    = Dimension<VelocityDimTag,    Unit>;
template<typename Unit> using Weight      = Dimension<WeightDimTag,      Unit>;
template<typename Unit> using Time        = Dimension<TimeDimTag,        Unit>;

// ================= FACTORY HELPERS =================

template<typename Unit> constexpr Angular<Unit>     make_angular(double v)     { return Angular<Unit>(v); }
template<typename Unit> constexpr Distance<Unit>    make_distance(double v)    { return Distance<Unit>(v); }
template<typename Unit> constexpr Energy<Unit>      make_energy(double v)      { return Energy<Unit>(v); }
template<typename Unit> constexpr Pressure<Unit>    make_pressure(double v)    { return Pressure<Unit>(v); }
template<typename Unit> constexpr Temperature<Unit> make_temperature(double v) { return Temperature<Unit>(v); }
template<typename Unit> constexpr Velocity<Unit>    make_velocity(double v)    { return Velocity<Unit>(v); }
template<typename Unit> constexpr Weight<Unit>      make_weight(double v)      { return Weight<Unit>(v); }
template<typename Unit> constexpr Time<Unit>        make_time(double v)        { return Time<Unit>(v); }

// ================= ENUM ↔ TYPE BRIDGE =================

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

// Returns the linear `factor` for non-temperature units (raw = value * factor).
// Returns 0.0 for temperature units — use convert_temperature() instead.
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

// Runtime conversion for all linear dimensions (Angular, Distance, Energy, Pressure, Velocity, Weight, Time).
inline double convert_linear(double v, BCLIBC_Unit from, BCLIBC_Unit to)
{
    return v * unit_factor(from) / unit_factor(to);
}

// Runtime conversion for Temperature (affine — not interchangeable with convert_linear).
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
