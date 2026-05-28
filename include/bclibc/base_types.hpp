#ifndef BCLIBC_BASE_TYPES_HPP
#define BCLIBC_BASE_TYPES_HPP

#include <cstddef>
#include <limits>
#include <vector>
#include "v3d.hpp"

namespace bclibc
{
    // Constants for unit conversions and atmospheric calculations

    /**
     * @brief Conversion factor from degrees Fahrenheit to degrees Rankine.
     */
    extern const double BCLIBC_cDegreesFtoR;
    /**
     * @brief Conversion factor from degrees Celsius to Kelvin.
     */
    extern const double BCLIBC_cDegreesCtoK;
    /**
     * @brief Constant for speed of sound calculation in Imperial units (fps).
     *
     * (Approx. $\sqrt{\gamma R}$)
     */
    extern const double BCLIBC_cSpeedOfSoundImperial;
    /**
     * @brief Constant for speed of sound calculation in Metric units.
     *
     * (Approx. $\sqrt{\gamma R}$)
     */
    extern const double BCLIBC_cSpeedOfSoundMetric;
    /**
     * @brief Standard lapse rate in Kelvin per foot in the troposphere.
     */
    extern const double BCLIBC_cLapseRateKperFoot;
    /**
     * @brief Standard lapse rate in Imperial units (degrees per foot).
     */
    extern const double BCLIBC_cLapseRateImperial;
    /**
     * @brief Exponent used in the barometric formula for pressure calculation.
     *
     * (Approx. $g / (L \cdot R)$)
     */
    extern const double BCLIBC_cPressureExponent;
    /**
     * @brief Lowest allowed temperature in Fahrenheit for atmospheric model.
     */
    extern const double BCLIBC_cLowestTempF;
    /**
     * @brief Conversion factor from meters to feet.
     */
    extern const double BCLIBC_mToFeet;

    /**
     * @brief Maximum distance in feet for a wind segment (used as a sentinel value).
     */
    extern const double BCLIBC_cMaxWindDistanceFeet;
    /**
     * @brief Earth's angular velocity in radians per second.
     */
    extern const double BCLIBC_cEarthAngularVelocityRadS;
    /**
     * @brief ICAO standard sea-level air density (kg/m³). Used to normalise CIPM-2007 density to a ratio.
     */
    extern const double BCLIBC_cStandardDensityMetric;

    enum class BCLIBC_TerminationReason
    {
        // Solver specific flags (always include RANGE_ERROR)
        NO_TERMINATE,
        TARGET_RANGE_REACHED,
        MINIMUM_VELOCITY_REACHED,
        MAXIMUM_DROP_REACHED,
        MINIMUM_ALTITUDE_REACHED,
        // Special flag to terminate integration via handler's request
        HANDLER_REQUESTED_STOP,
    };

    enum BCLIBC_TrajFlag
    {
        BCLIBC_TRAJ_FLAG_NONE = 0,
        BCLIBC_TRAJ_FLAG_ZERO_UP = 1,
        BCLIBC_TRAJ_FLAG_ZERO_DOWN = 2,
        BCLIBC_TRAJ_FLAG_ZERO = BCLIBC_TRAJ_FLAG_ZERO_UP | BCLIBC_TRAJ_FLAG_ZERO_DOWN,
        BCLIBC_TRAJ_FLAG_MACH = 4,
        BCLIBC_TRAJ_FLAG_RANGE = 8,
        BCLIBC_TRAJ_FLAG_APEX = 16,
        BCLIBC_TRAJ_FLAG_ALL = BCLIBC_TRAJ_FLAG_RANGE | BCLIBC_TRAJ_FLAG_ZERO_UP | BCLIBC_TRAJ_FLAG_ZERO_DOWN | BCLIBC_TRAJ_FLAG_MACH | BCLIBC_TRAJ_FLAG_APEX,
        BCLIBC_TRAJ_FLAG_MRT = 32
    };

    struct BCLIBC_Config
    {
    public:
        double cStepMultiplier;
        double cZeroFindingAccuracy;
        double cMinimumVelocity;
        double cMaximumDrop;
        int cMaxIterations;
        double cGravityConstant;
        double cMinimumAltitude;

        BCLIBC_Config() = default;
        BCLIBC_Config(
            double cStepMultiplier,
            double cZeroFindingAccuracy,
            double cMinimumVelocity,
            double cMaximumDrop,
            int cMaxIterations,
            double cGravityConstant,
            double cMinimumAltitude);
    };

    struct BCLIBC_CurvePoint
    {
    public:
        double a;
        double b;
        double c;
        double d; // PCHIP cubic constant term for segment (y at left knot)

        BCLIBC_CurvePoint() = default;
        BCLIBC_CurvePoint(
            double a,
            double b,
            double c,
            double d);
    };

    using BCLIBC_Curve = std::vector<BCLIBC_CurvePoint>;
    using BCLIBC_MachList = std::vector<double>;

    struct BCLIBC_Atmosphere
    {
    public:
        double _t0;
        double _a0;
        double _p0;
        double _mach;
        double density_ratio;
        double cLowestTempC;

        BCLIBC_Atmosphere() = default;
        BCLIBC_Atmosphere(
            double _t0,
            double _a0,
            double _p0,
            double _mach,
            double density_ratio,
            double cLowestTempC);

        /**
         * @brief Factory: construct BCLIBC_Atmosphere from user-facing conditions using CIPM-2007.
         *
         * Computes air density via the CIPM-2007 moist-air equation, normalises to
         * BCLIBC_cStandardDensityMetric (1.2250 kg/m³), and derives Mach 1 in fps.
         * Mirrors Python's Atmo.calculate_air_density() — the authoritative reference.
         *
         * @param t_c       Dry-bulb temperature in degrees Celsius.
         * @param p_hpa     Atmospheric pressure in hectopascals (hPa).
         * @param alt_ft    Base altitude in feet (stored as _a0; used by altitude update).
         * @param humidity  Relative humidity: fraction [0..1] or percent [0..100]. Default 0.
         */
        static BCLIBC_Atmosphere from_conditions(
            double t_c,
            double p_hpa,
            double alt_ft,
            double humidity = 0.0);

        /**
         * @brief Updates the density ratio and speed of sound (Mach 1) for a given altitude.
         *
         * This function calculates the new atmospheric pressure, temperature, and resulting
         * density ratio and speed of sound (Mach 1) at a given altitude using the
         * Standard Atmosphere model for the troposphere, adjusted for base conditions ($\text{atmo\_ptr->_t0, atmo\_ptr->_p0, atmo\_ptr->_a0}$).
         *
         * The barometric formula is used for pressure, and the lapse rate for temperature.
         *
         * @param altitude The new altitude in feet.
         * @param density_ratio_ptr Pointer to store the calculated density ratio ($\rho / \rho_{\text{std}}$).
         * @param mach_ptr Pointer to store the calculated speed of sound (Mach 1) in feet per second (fps).
         */
        void update_density_factor_and_mach_for_altitude(
            double altitude,
            double &density_ratio_out,
            double &mach_out) const;
    };

    struct BCLIBC_Coriolis
    {
    public:
        double sin_lat;
        double cos_lat;
        double sin_az;
        double cos_az;
        double range_east;
        double range_north;
        double cross_east;
        double cross_north;
        int flat_fire_only;
        double muzzle_velocity_fps;

        BCLIBC_Coriolis() = default;
        BCLIBC_Coriolis(
            double sin_lat,
            double cos_lat,
            double sin_az,
            double cos_az,
            double range_east,
            double range_north,
            double cross_east,
            double cross_north,
            int flat_fire_only,
            double muzzle_velocity_fps);

        /**
         * @brief Factory: construct BCLIBC_Coriolis from geographic inputs.
         *
         * Mirrors the Python reference implementation `Coriolis.create()`.
         *
         * - lat_deg == NaN  → all fields zero, flat_fire_only=1 (no Coriolis effect)
         * - az_deg  == NaN  → flat-fire only (sin/cos lat computed, azimuth zeroed)
         * - both provided   → full 3D Coriolis (all trig fields computed)
         *
         * @param lat_deg            Geographic latitude in degrees (-90…+90). NaN disables Coriolis.
         * @param muzzle_velocity_fps Muzzle velocity in feet per second.
         * @param az_deg             Shot azimuth in degrees (0=North, 90=East). Default NaN → flat-fire only.
         */
        static BCLIBC_Coriolis from_lat_az(
            double lat_deg,
            double muzzle_velocity_fps,
            double az_deg = std::numeric_limits<double>::quiet_NaN());

        void flat_fire_offsets(
            double time,
            double distance_ft,
            double drop_ft,
            double &delta_y,
            double &delta_z) const;

        BCLIBC_V3dT adjust_range(
            double time, const BCLIBC_V3dT &range_vector) const;

        /**
         * @brief Calculate Coriolis acceleration in local coordinates (range, up, crossrange).
         *
         * Transforms the projectile's ground velocity (local coordinates) to the
         * Earth-North-Up (ENU) coordinate system, calculates the Coriolis acceleration
         * in ENU, and then transforms the acceleration back to local coordinates.
         *
         * Coriolis acceleration formula in ENU:
         * - $\mathbf{a}_{\text{coriolis}} = -2 \cdot \mathbf{\omega}_{\text{earth}} \times \mathbf{v}_{\text{ENU}}$
         *
         * @param velocity_vector Pointer to the projectile's ground velocity vector (local coordinates: x=range, y=up, z=crossrange).
         * @param accel_out Pointer to store the calculated Coriolis acceleration vector (local coordinates).
         */
        void coriolis_acceleration_local(
            const BCLIBC_V3dT &velocity_vector,
            BCLIBC_V3dT &accel_out) const;
    };

    struct BCLIBC_Wind
    {
    public:
        double velocity;
        double direction_from;
        double until_distance;
        double MAX_DISTANCE_FEET;

        BCLIBC_Wind() = default;
        BCLIBC_Wind(double velocity,
                    double direction_from,
                    double until_distance,
                    double MAX_DISTANCE_FEET);

        /**
         * @brief Converts a BCLIBC_Wind structure to a BCLIBC_V3dT vector.
         *
         * The wind vector components are calculated assuming a standard coordinate system
         * where x is positive downrange and z is positive across-range (windage).
         * Wind direction is 'from' the specified direction (e.g., $0^\circ$ is tailwind, $90^\circ$ is wind from the right).
         *
         * @return A BCLIBC_V3dT structure representing the wind velocity vector (x=downrange, y=vertical, z=crossrange).
         */
        BCLIBC_V3dT as_V3dT() const;
    };

    struct BCLIBC_WindSock
    {
    public:
        std::vector<BCLIBC_Wind> winds;
        size_t current;
        double next_range;
        BCLIBC_V3dT last_vector_cache;

        /**
         * @brief Default constructor for BCLIBC_WindSock.
         *
         * Initializes state variables to their defaults and calculates the initial cache.
         */
        BCLIBC_WindSock();

        BCLIBC_WindSock(std::vector<BCLIBC_Wind> winds_vec);

        void push(const BCLIBC_Wind &wind);

        /**
         * @brief Updates the internal wind vector cache and next range threshold.
         *
         * Fetches the data for the wind segment at `ws->current`, converts it to a vector,
         * and updates `ws->last_vector_cache` and `ws->next_range`.
         * If `ws->current` is out of bounds, the cache is set to a zero vector and the next range to `BCLIBC_cMaxWindDistanceFeet`.
         */
        void update_cache();

        /**
         * @brief Returns the wind vector for the currently active wind segment.
         *
         * The vector is pre-calculated and stored in the cache.
         *
         * @return The current wind velocity vector (BCLIBC_V3dT). Returns a zero vector if the pointer is NULL.
         */
        BCLIBC_V3dT current_vector() const;

        /**
         * @brief Gets the current wind vector, updating to the next segment if necessary.
         *
         * Compares the given `next_range_param` (the current range in the simulation)
         * against the threshold for the current wind segment (`ws->next_range`).
         * If the threshold is met or exceeded, it advances to the next wind segment
         * and updates the cache.
         *
         * @param ws Pointer to the BCLIBC_WindSock structure.
         * @param next_range_param The current range (distance from muzzle) of the projectile.
         * @return The wind velocity vector (BCLIBC_V3dT) for the current or next applicable segment. Returns a zero vector if the pointer is NULL or an update fails.
         */
        BCLIBC_V3dT vector_for_range(double next_range_param);
    };

    struct BCLIBC_ShotProps
    {
        double bc;
        double look_angle;
        double twist;
        double length;
        double diameter;
        double weight;
        double barrel_elevation;
        double barrel_azimuth;
        double sight_height;
        double cant_cosine;
        double cant_sine;
        double alt0;
        double calc_step;
        double muzzle_velocity;
        double stability_coefficient;
        BCLIBC_Curve curve;
        BCLIBC_MachList mach_list;
        BCLIBC_Atmosphere atmo;
        BCLIBC_Coriolis coriolis;
        BCLIBC_WindSock wind_sock;
        BCLIBC_TrajFlag filter_flags;

        BCLIBC_ShotProps() = default;

        BCLIBC_ShotProps(
            double bc,
            double look_angle,
            double twist,
            double length,
            double diameter,
            double weight,
            double barrel_elevation,
            double barrel_azimuth,
            double sight_height,
            double cant_cosine,
            double cant_sine,
            double alt0,
            double calc_step,
            double muzzle_velocity,
            double stability_coefficient,
            BCLIBC_Curve curve,
            BCLIBC_MachList mach_list,
            BCLIBC_Atmosphere atmo,
            BCLIBC_Coriolis coriolis,
            BCLIBC_WindSock wind_sock,
            BCLIBC_TrajFlag filter_flags);

        ~BCLIBC_ShotProps();

        /**
         * @brief Updates the Miller stability coefficient ($S_g$) for the projectile.
         *
         * Calculates the Miller stability coefficient based on bullet dimensions, weight,
         * muzzle velocity, and atmospheric conditions ($\text{temperature, pressure}$).
         * The result is stored in `this->stability_coefficient`.
         *
         * Formula components:
         * - $\text{sd}$ (Stability Divisor)
         * - $\text{fv}$ (Velocity Factor)
         * - $\text{ftp}$ (Temperature/Pressure Factor)
         * - $S_g = \text{sd} \cdot \text{fv} \cdot \text{ftp}$
         *
         */
        void update_stability_coefficient();

        /**
         * @brief Litz spin-drift approximation
         *
         * Calculates the lateral displacement (windage) due to spin drift using
         * Litz's approximation formula. This formula provides an estimate based on
         * the stability coefficient and time of flight.
         *
         * Formula used (converted to feet):
         * $\text{Spin Drift (ft)} = \text{sign} \cdot \frac{1.25 \cdot (S_g + 1.2) \cdot \text{time}^{1.83}}{12.0}$
         * where $S_g$ is the stability coefficient.
         *
         * @param time Time of flight in seconds.
         * @return Windage due to spin drift, in feet. Returns 0.0 if twist or stability_coefficient is zero.
         */
        double spin_drift(double time) const;

        /**
         * @brief Computes the scaled drag force coefficient ($C_d$) for a projectile at a given Mach number.
         *
         * This function calculates the drag coefficient using a cubic spline interpolation
         * (via `calculate_by_curve_and_mach_list`) and scales it by a constant factor and the
         * bullet's ballistic coefficient (BC). The result is $\frac{C_d}{\text{BC} \cdot \text{scale\_factor}}$.
         *
         * The constant $2.08551\text{e-}04$ is a combination of standard air density,
         * cross-sectional area conversion, and mass conversion factors.
         *
         * Formula used:
         * $\text{Scaled } C_d = \frac{C_d(\text{Mach}) \cdot 2.08551\text{e-}04}{\text{BC}}$
         *
         * @param mach Mach number at which to evaluate the drag.
         * @return Drag coefficient $C_d$ scaled by $\text{BC}$ and conversion factors, in units suitable for the trajectory calculation.
         */
        double drag_by_mach(double mach) const;
        size_t size() const;
    };

    /**
     * @brief Universal PCHIP core calculation.
     * Takes vectors X and Y, returns BCLIBC_Curve.
     */
    BCLIBC_Curve build_pchip_curve_from_arrays(const std::vector<double> &x, const std::vector<double> &y);

    /**
     * @brief User-facing shot descriptor — all fields in natural units, no pre-computation.
     *
     * Wrappers (Python/Cython, Dart FFI, WASM) fill this struct from their domain objects
     * and call to_shot_props() to obtain the engine-ready BCLIBC_ShotProps.  All physics
     * conversions (cant trig, CIPM-2007 atmosphere, Coriolis trig, PCHIP drag curve) happen
     * once, inside to_shot_props(), eliminating per-wrapper duplication.
     *
     * Lifetime note: mach_data, cd_data, and winds are non-owning pointers.  The caller must
     * keep the backing arrays alive until to_shot_props() returns.
     */
    struct BCLIBC_Shot
    {
        // ammo
        double bc;
        double weight_grain;
        double diameter_inch;
        double length_inch;
        double muzzle_velocity_fps;
        double stability_coefficient;   ///< 0.0 → computed by engine on first step

        // drag table — raw Mach/CD pairs; PCHIP built inside to_shot_props()
        const double* mach_data;
        const double* cd_data;
        int           drag_table_size;

        // weapon geometry
        double sight_height_ft;
        double twist_inch;             ///< positive = right-hand twist, negative = left-hand

        // atmosphere (user-facing inputs, not pre-computed)
        double temp_c;       ///< dry-bulb temperature in °C
        double pressure_hpa; ///< atmospheric pressure in hPa; 0 → vacuum (zero drag)
        double altitude_ft;  ///< base altitude in feet (used for density altitude adjustment)
        double humidity;     ///< relative humidity: fraction [0..1] or percent [0..100]

        // winds — non-owning array; wind_count == 0 → no wind
        const BCLIBC_Wind* winds;
        int                wind_count;

        // aiming (all in radians)
        double look_angle_rad;
        double barrel_elevation_rad;
        double barrel_azimuth_rad;
        double cant_angle_rad;   ///< to_shot_props() computes cant_cosine/cant_sine internally

        // Coriolis inputs (degrees; NaN disables)
        double latitude_deg;   ///< geographic latitude [-90..+90]; NaN → no Coriolis effect
        double azimuth_deg;    ///< shot azimuth [0..360); NaN → flat-fire drift only

        double calc_step;      ///< integration step in feet

        /**
         * @brief Assemble engine-ready BCLIBC_ShotProps.
         *
         * Performs all physics/unit conversions:
         *   - cant_cosine / cant_sine  from cant_angle_rad
         *   - BCLIBC_Atmosphere        via BCLIBC_Atmosphere::from_conditions()  (CIPM-2007)
         *   - BCLIBC_Coriolis          via BCLIBC_Coriolis::from_lat_az()
         *   - BCLIBC_Curve / MachList  via build_pchip_curve_from_arrays()
         *   - BCLIBC_WindSock          from the winds array
         */
        BCLIBC_ShotProps to_shot_props() const;
    };

    /**
     * @brief Calculates the angular correction needed to hit a target.
     *
     * Computes the angle (in radians) to correct a shot based on the linear offset
     * at a given distance using the arc tangent function ($\arctan(\text{offset}/\text{distance})$).
     *
     * @param distance The distance to the target (or the point of offset).
     * @param offset The linear offset (e.g., vertical drop or windage).
     * @return The correction angle in radians. Returns 0.0 if distance is zero (to avoid division by zero).
     */
    double BCLIBC_getCorrection(double distance, double offset);

    /**
     * @brief Calculates the kinetic energy of the projectile.
     *
     * Uses the formula: $\text{Energy (ft-lbs)} = \frac{\text{Weight (grains)} \cdot \text{Velocity (fps)}^2}{450400}$.
     *
     * @param bulletWeight Bullet weight in grains.
     * @param velocity Projectile velocity in feet per second (fps).
     * @return Kinetic energy in foot-pounds (ft-lbs).
     */
    double BCLIBC_calculateEnergy(double bulletWeight, double velocity);

    /**
     * @brief Calculates the Optimum Game Weight (OGW) factor.
     *
     * OGW is a metric that attempts to combine kinetic energy and momentum into a single number.
     * Formula used: $\text{OGW} = \text{Weight (grains)}^2 \cdot \text{Velocity (fps)}^3 \cdot 1.5\text{e-}12$.
     *
     * @param bulletWeight Bullet weight in grains.
     * @param velocity Projectile velocity in feet per second (fps).
     * @return The Optimum Game Weight (OGW) factor.
     */
    double BCLIBC_calculateOgw(double bulletWeight, double velocity);

}; // namespace bclibc

#endif // BCLIBC_BASE_TYPES_HPP
