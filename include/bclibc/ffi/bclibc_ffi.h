/**
 * bclibc_ffi.h
 *
 * Thin C interface over the bclibc ballistics engine.
 * Designed to be consumed by Dart FFI via ffigen.
 *
 * Mirrors the WASM bindings API surface (findApex, findMaxRange,
 * findZeroAngle, integrate, integrateAt) with flat C structs.
 */

#ifndef BCLIBC_FFI_H
#define BCLIBC_FFI_H

#include <stdint.h>
#include <stddef.h>

// ============================================================================
// Symbol visibility
//
// A binding generator (cffi, autopxd2, SWIG, ...) that parses this header itself, without a C preprocessor in
// front of it, defines BCLIBC_FFI_PARSE_ONLY first: BCLIBC_API then expands to nothing, and the header holds
// only declarations the generators understand. The library itself is never built with it.
// ============================================================================
#if defined(BCLIBC_FFI_PARSE_ONLY) && defined(BCLIBC_FFI_EXPORT)
#error "BCLIBC_FFI_PARSE_ONLY empties BCLIBC_API, so a library built with it exports nothing on Windows: do not define it with BCLIBC_FFI_EXPORT"
#endif
#if defined(BCLIBC_FFI_PARSE_ONLY)
#define BCLIBC_API
#elif defined(_WIN32)
#ifdef BCLIBC_FFI_EXPORT
#define BCLIBC_API __declspec(dllexport)
#else
#define BCLIBC_API __declspec(dllimport)
#endif
#else
#define BCLIBC_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Returns the library version string.
     */
    BCLIBC_API const char *BCLIBCFFI_get_version();

    // ============================================================================
    // Error codes
    // ============================================================================

    typedef enum BCLIBCFFI_Status
    {
        BCLIBCFFI_OK = 0,
        BCLIBCFFI_ERR_SOLVER_RUNTIME = 1,
        BCLIBCFFI_ERR_OUT_OF_RANGE = 2,
        BCLIBCFFI_ERR_ZERO_FINDING = 3,
        BCLIBCFFI_ERR_INTERCEPTION = 4,
        BCLIBCFFI_ERR_GENERIC = 5,
        /* The kinds of BCLIBC_Error that do not belong to a solver; before they all came back as BCLIBCFFI_ERR_GENERIC.
         * A caller that tells the codes apart by `switch` with `default:` for BCLIBCFFI_ERR_GENERIC has to handle them. */
        BCLIBCFFI_ERR_DOMAIN = 6,           /**< BCLIBC_DomainError: inputs the operation cannot work with (a degenerate interpolation segment, too few points). */
        BCLIBCFFI_ERR_INVALID_ARGUMENT = 7, /**< BCLIBC_InvalidArgumentError: an argument is not acceptable (a drag table of fewer than 2 points, an unknown key). */
        BCLIBCFFI_ERR_OUT_OF_BOUNDS = 8,    /**< BCLIBC_OutOfRangeError (not a solver's): an index or a value outside the data (a key beyond the trajectory, an index outside [1, n-2]). */
        BCLIBCFFI_ERR_LOGIC = 9,            /**< BCLIBC_LogicError: a defect inside the library. */
        BCLIBCFFI_ERR_RUNTIME = 10,         /**< BCLIBC_RuntimeError: any other failure at run time (out of memory in BCLIBCFFI_base_traj_buf_append). */
    } BCLIBCFFI_Status;

    /**
     * Error output struct – filled by every function on failure.
     *
     * What f64_0, f64_1, f64_2 and i32_0 hold depends on `code`; the fields a code does not use are 0:
     *
     * | code                              | f64_0                    | f64_1                      | f64_2           | i32_0            |
     * |-----------------------------------|--------------------------|----------------------------|-----------------|------------------|
     * | BCLIBCFFI_ERR_OUT_OF_RANGE        | requested_distance_ft    | max_range_ft               | look_angle_rad  |                  |
     * | BCLIBCFFI_ERR_ZERO_FINDING        | zero_finding_error       | last_barrel_elevation_rad  |                 | iterations_count |
     * | BCLIBCFFI_ERR_OUT_OF_BOUNDS       | requested value or index | minimum allowed            | maximum allowed |                  |
     * | BCLIBCFFI_ERR_DOMAIN              | lhs (first value)        | rhs (second value)         |                 |                  |
     * | the other codes                   |                          |                            |                 |                  |
     */
    typedef struct BCLIBCFFI_Error
    {
        int32_t code;      /**< BCLIBCFFI_Status */
        char message[512]; /**< Null-terminated error message */
        /* Extra fields for typed errors: see the table above */
        double f64_0;
        double f64_1;
        double f64_2;
        int32_t i32_0;
    } BCLIBCFFI_Error;

    // ============================================================================
    // Enums
    // ============================================================================

    typedef enum BCLIBCFFI_TrajFlag
    {
        BCLIBCFFI_TRAJ_FLAG_NONE = 0,
        BCLIBCFFI_TRAJ_FLAG_ZERO_UP = 1,
        BCLIBCFFI_TRAJ_FLAG_ZERO_DOWN = 2,
        BCLIBCFFI_TRAJ_FLAG_ZERO = 3,
        BCLIBCFFI_TRAJ_FLAG_MACH = 4,
        BCLIBCFFI_TRAJ_FLAG_RANGE = 8,
        BCLIBCFFI_TRAJ_FLAG_APEX = 16,
        BCLIBCFFI_TRAJ_FLAG_ALL = 31,
        BCLIBCFFI_TRAJ_FLAG_MRT = 32,
    } BCLIBCFFI_TrajFlag;

    typedef enum BCLIBCFFI_TerminationReason
    {
        BCLIBCFFI_TERM_NO_TERMINATE = 0,
        BCLIBCFFI_TERM_TARGET_RANGE_REACHED = 1,
        BCLIBCFFI_TERM_MINIMUM_VELOCITY_REACHED = 2,
        BCLIBCFFI_TERM_MAXIMUM_DROP_REACHED = 3,
        BCLIBCFFI_TERM_MINIMUM_ALTITUDE_REACHED = 4,
        BCLIBCFFI_TERM_HANDLER_REQUESTED_STOP = 5,
    } BCLIBCFFI_TerminationReason;

    /** Interpolation key for BCLIBC_BaseTrajData fields. */
    typedef enum BCLIBCFFI_BaseTrajInterpKey
    {
        BCLIBCFFI_INTERP_KEY_TIME = 0,
        BCLIBCFFI_INTERP_KEY_MACH = 1,
        BCLIBCFFI_INTERP_KEY_POS_X = 2,
        BCLIBCFFI_INTERP_KEY_POS_Y = 3,
        BCLIBCFFI_INTERP_KEY_POS_Z = 4,
        BCLIBCFFI_INTERP_KEY_VEL_X = 5,
        BCLIBCFFI_INTERP_KEY_VEL_Y = 6,
        BCLIBCFFI_INTERP_KEY_VEL_Z = 7,
    } BCLIBCFFI_BaseTrajInterpKey;

    typedef enum BCLIBCFFI_IntegrationMethod
    {
        BCLIBCFFI_INTEGRATION_RK4 = 0,
        BCLIBCFFI_INTEGRATION_EULER = 1,
        BCLIBCFFI_INTEGRATION_VELOCITY_VERLET = 2,
        /** EXPERIMENTAL -- see bclibc/cash_karp.hpp's doc comment before
         * relying on this for anything accuracy-sensitive: known event-
         * interpolation regressions under this method's sparse/irregular
         * step spacing (see project issue tracker). */
        BCLIBCFFI_INTEGRATION_CASH_KARP = 3,
        BCLIBCFFI_INTEGRATION_DORMAND_PRINCE = 4,
        /** Adaptive Tsitouras 5(4) ("Tsit5") -- same FSAL shape and step-size
         * controller as BCLIBCFFI_INTEGRATION_DORMAND_PRINCE, see
         * bclibc/tsitouras.hpp's doc comment. */
        BCLIBCFFI_INTEGRATION_TSITOURAS = 5,
    } BCLIBCFFI_IntegrationMethod;

    // ============================================================================
    // Input structs
    // ============================================================================

    typedef struct BCLIBCFFI_Config
    {
        double cStepMultiplier;
        double cZeroFindingAccuracy;
        double cMinimumVelocity;
        double cMaximumDrop;
        int32_t cMaxIterations;
        double cGravityConstant;
        double cMinimumAltitude;
    } BCLIBCFFI_Config;

    typedef struct BCLIBCFFI_Atmosphere
    {
        double t0;            /**< Temperature at base altitude (°F) */
        double a0;            /**< Base altitude (ft) */
        double p0;            /**< Pressure at base altitude (hPa) */
        double mach;          /**< Speed of sound (fps) */
        double density_ratio; /**< Air density / standard density */
        double cLowestTempC;  /**< Lowest allowed temperature (°C) */
    } BCLIBCFFI_Atmosphere;

    typedef struct BCLIBCFFI_Coriolis
    {
        double sin_lat;
        double cos_lat;
        double sin_az;
        double cos_az;
        double range_east;
        double range_north;
        double cross_east;
        double cross_north;
        int32_t flat_fire_only; /**< Non-zero = flat-fire approximation */
        double muzzle_velocity_fps;
    } BCLIBCFFI_Coriolis;

    typedef struct BCLIBCFFI_Wind
    {
        double velocity_fps;
        double direction_from_rad;
        double until_distance_ft;
        double max_distance_ft; /**< Sentinel for last segment (use BCLIBC_cMaxWindDistanceFeet) */
    } BCLIBCFFI_Wind;

    typedef struct BCLIBCFFI_DragPoint
    {
        double Mach;
        double CD;
    } BCLIBCFFI_DragPoint;

    /** Flat shot properties passed to every engine function. */
    typedef struct BCLIBCFFI_ShotProps
    {
        double bc;
        double look_angle_rad;
        double twist_inch;
        double length_inch;
        double diameter_inch;
        double weight_grain;
        double barrel_elevation_rad;
        double barrel_azimuth_rad;
        double sight_height_ft;
        double cant_angle_rad;
        double alt0_ft;
        double muzzle_velocity_fps;

        BCLIBCFFI_Atmosphere atmo;
        BCLIBCFFI_Coriolis coriolis;
        BCLIBCFFI_Config config;

        BCLIBCFFI_IntegrationMethod method;

        /** Drag table – pointer must remain valid for the duration of the call. */
        const BCLIBCFFI_DragPoint *drag_table;
        int32_t drag_table_count;

        /** Wind list – pointer must remain valid for the duration of the call. */
        const BCLIBCFFI_Wind *winds;
        int32_t wind_count;
    } BCLIBCFFI_ShotProps;

    /**
     * User-facing shot descriptor in natural units.
     *
     * Preferred input for BCLIBCFFI_*_shot() functions.  All physics conversions
     * (CIPM-2007 atmosphere density, Coriolis trig, PCHIP drag curve, cant sin/cos)
     * are performed inside C++ by BCLIBC_Shot::to_shot_props().
     *
     * latitude_deg / azimuth_deg: pass NaN to disable Coriolis / flat-fire-only.
     * pressure_hpa == 0          : vacuum (zero drag).
     *
     * All pointers must remain valid for the duration of the call.
     * Drag table is passed as two parallel arrays (mach_data / cd_data) matching
     * the BCLIBC_Shot layout — no interleaved struct needed.
     */
    typedef struct BCLIBCFFI_Shot
    {
        double bc;
        double weight_grain;
        double diameter_inch;
        double length_inch;
        double muzzle_velocity_fps;

        double sight_height_ft;
        double twist_inch;

        /* Atmosphere – raw user-facing units, not pre-computed */
        double temp_c;
        double pressure_hpa; /**< 0 = vacuum */
        double altitude_ft;
        double humidity; /**< 0.0 – 1.0 */

        /** Parallel Mach / CD arrays – must remain valid for the duration of the call. */
        const double *mach_data;
        const double *cd_data;
        int32_t drag_table_size;

        /** Wind list – must remain valid for the duration of the call. */
        const BCLIBCFFI_Wind *winds;
        int32_t wind_count;

        /* Aiming */
        double look_angle_rad;
        double barrel_elevation_rad;
        double barrel_azimuth_rad;
        double cant_angle_rad;

        /* Coriolis – raw geographic degrees; NaN disables Coriolis / enables flat-fire-only */
        double latitude_deg;
        double azimuth_deg;

        BCLIBCFFI_Config config;
        BCLIBCFFI_IntegrationMethod method;
    } BCLIBCFFI_Shot;

    typedef struct BCLIBCFFI_TrajectoryRequest
    {
        double range_limit_ft;
        double range_step_ft;
        double time_step;
        int32_t filter_flags; /**< BCLIBCFFI_TrajFlag bitmask */
    } BCLIBCFFI_TrajectoryRequest;

    // ============================================================================
    // Output structs
    // ============================================================================

    typedef struct BCLIBCFFI_BaseTrajData
    {
        double time;
        double px; /**< Position x (downrange, ft) */
        double py; /**< Position y (height, ft) */
        double pz; /**< Position z (windage, ft) */
        double vx; /**< Velocity x (fps) */
        double vy; /**< Velocity y (fps) */
        double vz; /**< Velocity z (fps) */
        double mach;
    } BCLIBCFFI_BaseTrajData;

    /**
     * One record of a trajectory: 15 doubles and an int32 flag. Its size is what the compiler makes of that:
     * 128 bytes where a double is aligned to 8 (x86-64, aarch64, armv7l, Windows, ppc64le, s390x, riscv64) and 124 on
     * 32-bit x86 Linux (the i386 ABI aligns a double to 4, so there is no tail padding). A binding that reads the
     * struct as bytes takes the size from BCLIBCFFI_get_layout() (the value after the TrajectoryData block starts), or
     * checks its own against it when the library is loaded, and does not count it itself.
     */
    typedef struct BCLIBCFFI_TrajectoryData
    {
        double time;
        double distance_ft;
        double velocity_fps;
        double mach;
        double height_ft;
        double slant_height_ft;
        double drop_angle_rad;
        double windage_ft;
        double windage_angle_rad;
        double slant_distance_ft;
        double angle_rad;
        double density_ratio;
        double drag;
        double energy_ft_lb;
        double ogw_lb;
        int32_t flag; /**< BCLIBCFFI_TrajFlag */
    } BCLIBCFFI_TrajectoryData;

    typedef struct BCLIBCFFI_MaxRangeResult
    {
        double max_range_ft;
        double angle_at_max_rad;
    } BCLIBCFFI_MaxRangeResult;

    /**
     * @brief Result of a lower-arc zero solve.
     *
     * `point` is the terminal RANGE point evaluated by the successful
     * zero-finding iteration. It is not obtained by a separate final
     * trajectory integration.
     */
    typedef struct BCLIBCFFI_ZeroPointResult
    {
        double angle_rad;               /**< Solved barrel elevation (radians). */
        BCLIBCFFI_TrajectoryData point; /**< Terminal RANGE point of the solve. */
    } BCLIBCFFI_ZeroPointResult;

    typedef struct BCLIBCFFI_Interception
    {
        BCLIBCFFI_BaseTrajData raw_data;
        BCLIBCFFI_TrajectoryData full_data;
    } BCLIBCFFI_Interception;

    // ============================================================================
    // Core functions
    // ============================================================================

    /**
     * Find the apex (highest point) of the trajectory.
     * @return BCLIBCFFI_OK on success, error code otherwise (fills *err).
     */
    BCLIBC_API int32_t BCLIBCFFI_find_apex(
        const BCLIBCFFI_ShotProps *props,
        BCLIBCFFI_TrajectoryData *out,
        BCLIBCFFI_Error *err);

    /**
     * Find the maximum range and corresponding angle.
     * @param low_angle_deg   Lower search bound (degrees).
     * @param high_angle_deg  Upper search bound (degrees).
     */
    BCLIBC_API int32_t BCLIBCFFI_find_max_range(
        const BCLIBCFFI_ShotProps *props,
        double low_angle_deg,
        double high_angle_deg,
        BCLIBCFFI_MaxRangeResult *out,
        BCLIBCFFI_Error *err);

    /**
     * Find the barrel elevation angle to zero at the given distance.
     * @param distance_ft     Slant distance to target (ft).
     * @param out_angle_rad   Output: barrel elevation (radians).
     */
    BCLIBC_API int32_t BCLIBCFFI_find_zero_angle(
        const BCLIBCFFI_ShotProps *props,
        double distance_ft,
        double *out_angle_rad,
        BCLIBCFFI_Error *err);

    /**
     * @brief Find the lower zero and its terminal trajectory point.
     *
     * Uses the Newton solver with Ridder's fallback. The returned point is
     * captured from the successful solve, so no final trajectory integration
     * is performed.
     *
     * @param props        Pre-computed shot properties.
     * @param distance_ft  Slant distance to the target (ft).
     * @param out          Output zero angle and terminal RANGE point.
     * @param err          Error details when the return value is not OK.
     * @return BCLIBCFFI_OK on success; an error status otherwise.
     */
    BCLIBC_API int32_t BCLIBCFFI_find_zero_point(
        const BCLIBCFFI_ShotProps *props,
        double distance_ft,
        BCLIBCFFI_ZeroPointResult *out,
        BCLIBCFFI_Error *err);

    /**
     * Integrate trajectory and return filtered records.
     *
     * On success *out_records points to a heap-allocated BCLIBCFFI_TrajectoryData array
     * of length *out_count.  Call BCLIBCFFI_free_trajectory() to release it.
     */
    BCLIBC_API int32_t BCLIBCFFI_integrate(
        const BCLIBCFFI_ShotProps *props,
        const BCLIBCFFI_TrajectoryRequest *request,
        BCLIBCFFI_TrajectoryData **out_records,
        int32_t *out_count,
        int32_t *out_reason, /**< BCLIBCFFI_TerminationReason */
        BCLIBCFFI_Error *err);

    /** Free a trajectory array allocated by BCLIBCFFI_integrate(). */
    BCLIBC_API void BCLIBCFFI_free_trajectory(BCLIBCFFI_TrajectoryData *records);

    /**
     * Integrate and interpolate the single point where a key field reaches
     * the target value.
     * @param key          BCLIBCFFI_BaseTrajInterpKey
     * @param target_value Value the key field must reach.
     */
    BCLIBC_API int32_t BCLIBCFFI_integrate_at(
        const BCLIBCFFI_ShotProps *props,
        int32_t key,
        double target_value,
        BCLIBCFFI_Interception *out,
        BCLIBCFFI_Error *err);

    // ============================================================================
    // Core functions (BCLIBCFFI_Shot – preferred, all physics conversion in C++)
    // ============================================================================

    /**
     * Find the apex of the trajectory.
     * All physics/unit conversion is performed inside C++ via BCLIBCFFI_Shot::to_shot_props().
     */
    BCLIBC_API int32_t BCLIBCFFI_find_apex_shot(
        const BCLIBCFFI_Shot *shot,
        BCLIBCFFI_TrajectoryData *out,
        BCLIBCFFI_Error *err);

    BCLIBC_API int32_t BCLIBCFFI_find_max_range_shot(
        const BCLIBCFFI_Shot *shot,
        double low_angle_deg,
        double high_angle_deg,
        BCLIBCFFI_MaxRangeResult *out,
        BCLIBCFFI_Error *err);

    BCLIBC_API int32_t BCLIBCFFI_find_zero_angle_shot(
        const BCLIBCFFI_Shot *shot,
        double distance_ft,
        double *out_angle_rad,
        BCLIBCFFI_Error *err);

    /**
     * @brief Find the lower zero for a user-facing shot and its terminal point.
     *
     * Converts `shot` to engine properties, then uses the Newton solver with
     * Ridder's fallback. The returned point is captured from the successful
     * solve, so no final trajectory integration is performed.
     *
     * @param shot         User-facing shot descriptor.
     * @param distance_ft  Slant distance to the target (ft).
     * @param out          Output zero angle and terminal RANGE point.
     * @param err          Error details when the return value is not OK.
     * @return BCLIBCFFI_OK on success; an error status otherwise.
     */
    BCLIBC_API int32_t BCLIBCFFI_find_zero_point_shot(
        const BCLIBCFFI_Shot *shot,
        double distance_ft,
        BCLIBCFFI_ZeroPointResult *out,
        BCLIBCFFI_Error *err);

    BCLIBC_API int32_t BCLIBCFFI_integrate_shot(
        const BCLIBCFFI_Shot *shot,
        const BCLIBCFFI_TrajectoryRequest *request,
        BCLIBCFFI_TrajectoryData **out_records,
        int32_t *out_count,
        int32_t *out_reason, /**< BCLIBCFFI_TerminationReason */
        BCLIBCFFI_Error *err);

    BCLIBC_API int32_t BCLIBCFFI_integrate_at_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t key,
        double target_value,
        BCLIBCFFI_Interception *out,
        BCLIBCFFI_Error *err);

    // ============================================================================
    // Utility functions
    // ============================================================================

    /** Angular correction (radians) to hit target at distance with given offset. */
    BCLIBC_API double BCLIBCFFI_get_correction(double distance_ft, double offset_ft);

    /** Kinetic energy (ft-lb) from bullet weight (grains) and velocity (fps). */
    BCLIBC_API double BCLIBCFFI_calculate_energy(double bullet_weight_grain, double velocity_fps);

    /** Optimal Game Weight from bullet weight (grains) and velocity (fps). */
    BCLIBC_API double BCLIBCFFI_calculate_ogw(double bullet_weight_grain, double velocity_fps);

    // ============================================================================
    // Interpolation between trajectory points
    // ============================================================================

    /** Cubic Hermite polynomial through (xk, yk) and (xk1, yk1) with slopes mk and mk1, at x. */
    BCLIBC_API double BCLIBCFFI_hermite(double x, double xk, double xk1, double yk, double yk1, double mk, double mk1);

    /** PCHIP interpolation at x through three points (x0, y0), (x1, y1), (x2, y2). */
    BCLIBC_API double BCLIBCFFI_interpolate_3pt(double x, double x0, double x1, double x2, double y0, double y1, double y2);

    /**
     * Linear interpolation at x between (x0, y0) and (x1, y1).
     * @return BCLIBCFFI_OK, or BCLIBCFFI_ERR_GENERIC (fills *err) when x0 == x1.
     */
    BCLIBC_API int32_t BCLIBCFFI_interpolate_2pt(double x, double x0, double y0, double x1, double y1, double *out, BCLIBCFFI_Error *err);

    /** BCLIBC_TrajectoryData_InterpKey: the field of a trajectory point that is the independent variable. */
    typedef enum BCLIBCFFI_TrajectoryInterpKey
    {
        BCLIBCFFI_TRAJ_KEY_TIME = 0,
        BCLIBCFFI_TRAJ_KEY_DISTANCE = 1,
        BCLIBCFFI_TRAJ_KEY_VELOCITY = 2,
        BCLIBCFFI_TRAJ_KEY_MACH = 3,
        BCLIBCFFI_TRAJ_KEY_HEIGHT = 4,
        BCLIBCFFI_TRAJ_KEY_SLANT_HEIGHT = 5,
        BCLIBCFFI_TRAJ_KEY_DROP_ANGLE = 6,
        BCLIBCFFI_TRAJ_KEY_WINDAGE = 7,
        BCLIBCFFI_TRAJ_KEY_WINDAGE_ANGLE = 8,
        BCLIBCFFI_TRAJ_KEY_SLANT_DISTANCE = 9,
        BCLIBCFFI_TRAJ_KEY_ANGLE = 10,
        BCLIBCFFI_TRAJ_KEY_DENSITY_RATIO = 11,
        BCLIBCFFI_TRAJ_KEY_DRAG = 12,
        BCLIBCFFI_TRAJ_KEY_ENERGY = 13,
        BCLIBCFFI_TRAJ_KEY_OGW = 14,
        BCLIBCFFI_TRAJ_KEY_FLAG = 15,
    } BCLIBCFFI_TrajectoryInterpKey;

    typedef enum BCLIBCFFI_InterpMethod
    {
        BCLIBCFFI_INTERP_PCHIP = 0,
        BCLIBCFFI_INTERP_LINEAR = 1,
    } BCLIBCFFI_InterpMethod;

    /**
     * The point of a trajectory where the field `key` has the value `value`, interpolated from the three points
     * around it (BCLIBC_TrajectoryData::interpolate): every field is interpolated, the key field is set to `value`
     * and `flag` (BCLIBCFFI_TrajFlag) is the flag of the result. A trajectory record carries no velocity of its own, so
     * there is no step to build a Hermite on: this stays a 3-point interpolation (PCHIP or linear), as the fallback of the
     * step-based queries (BCLIBCFFI_dense_get_at, BCLIBCFFI_base_traj_interpolate).
     * @param key     BCLIBCFFI_TrajectoryInterpKey
     * @param method  BCLIBCFFI_InterpMethod
     * @return BCLIBCFFI_OK, or an error code (fills *err), for example when two of the points have the same key.
     */
    BCLIBC_API int32_t BCLIBCFFI_interpolate_trajectory_data(
        int32_t key,
        double value,
        const BCLIBCFFI_TrajectoryData *t0,
        const BCLIBCFFI_TrajectoryData *t1,
        const BCLIBCFFI_TrajectoryData *t2,
        int32_t flag,
        int32_t method,
        BCLIBCFFI_TrajectoryData *out,
        BCLIBCFFI_Error *err);

    // ============================================================================
    // Adaptive integrators (Cash-Karp, Dormand-Prince, Tsitouras)
    //
    // Each call builds its own integrator, so tolerances and step statistics are per call and calls
    // are independent (no shared or thread-local state). `method` must be one of
    // BCLIBCFFI_INTEGRATION_CASH_KARP, BCLIBCFFI_INTEGRATION_DORMAND_PRINCE or
    // BCLIBCFFI_INTEGRATION_TSITOURAS; any other value returns BCLIBCFFI_ERR_GENERIC.
    // `shot->method` is ignored for these calls. Tolerances must be finite and positive (relative)
    // or finite and non-negative (absolute); otherwise BCLIBCFFI_ERR_GENERIC is returned.
    // ============================================================================

    /** Accepted and rejected step counts of the most recent call that reports them: an adaptive call, or
     *  BCLIBCFFI_integrate_dense_shot_stats() (fixed-step, rejected is 0). */
    typedef struct BCLIBCFFI_StepStats
    {
        int32_t accepted; /**< Accepted steps. */
        int32_t rejected; /**< Rejected steps. */
    } BCLIBCFFI_StepStats;

    /** One accepted step of the adaptive integrator (dense output). Same layout as BCLIBCFFI_BaseTrajData. */
    typedef BCLIBCFFI_BaseTrajData BCLIBCFFI_DenseStep;

    /**
     * @brief Integrate with per-call tolerances, optionally returning the accepted-step stream.
     *
     * On success *out_records (and *out_dense when want_dense != 0) point to heap arrays of
     * *out_count (and *out_dense_count) elements. Release them with BCLIBCFFI_free_trajectory() and
     * BCLIBCFFI_free_base_trajectory(). *out_dense is NULL when want_dense == 0.
     *
     * @param out_stats Optional. Filled even when the integration fails (step counts so far).
     */
    BCLIBC_API int32_t BCLIBCFFI_integrate_adaptive_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t method,
        double relative_tolerance,
        double absolute_tolerance,
        const BCLIBCFFI_TrajectoryRequest *request,
        int32_t want_dense,
        BCLIBCFFI_TrajectoryData **out_records,
        int32_t *out_count,
        BCLIBCFFI_DenseStep **out_dense,
        int32_t *out_dense_count,
        int32_t *out_reason, /**< BCLIBCFFI_TerminationReason */
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err);

    /** Free a dense-step array allocated by BCLIBCFFI_integrate_adaptive_shot(). */
    BCLIBC_API void BCLIBCFFI_free_base_trajectory(BCLIBCFFI_DenseStep *steps);

    /** Barrel elevation that hits the sight line at distance_ft, with per-call tolerances. */
    BCLIBC_API int32_t BCLIBCFFI_find_zero_angle_adaptive_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t method,
        double relative_tolerance,
        double absolute_tolerance,
        double distance_ft,
        double *out_angle_rad,
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err);

    /** Lower zero and its terminal RANGE point, with per-call tolerances. */
    BCLIBC_API int32_t BCLIBCFFI_find_zero_point_adaptive_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t method,
        double relative_tolerance,
        double absolute_tolerance,
        double distance_ft,
        BCLIBCFFI_ZeroPointResult *out,
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err);

    /** Maximum range and its launch angle within [low_angle_deg, high_angle_deg], with per-call tolerances. */
    BCLIBC_API int32_t BCLIBCFFI_find_max_range_adaptive_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t method,
        double relative_tolerance,
        double absolute_tolerance,
        double low_angle_deg,
        double high_angle_deg,
        BCLIBCFFI_MaxRangeResult *out,
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err);

    /** Apex point, with per-call tolerances. */
    BCLIBC_API int32_t BCLIBCFFI_find_apex_adaptive_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t method,
        double relative_tolerance,
        double absolute_tolerance,
        BCLIBCFFI_TrajectoryData *out,
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err);

    /**
     * @brief Integrate with a fixed-step method (shot->method: RK4, Euler or Velocity Verlet) and optional dense output.
     *
     * Same outputs as BCLIBCFFI_integrate_adaptive_shot(). Adaptive methods are rejected with BCLIBCFFI_ERR_GENERIC
     * (use BCLIBCFFI_integrate_adaptive_shot for them).
     */
    BCLIBC_API int32_t BCLIBCFFI_integrate_dense_shot(
        const BCLIBCFFI_Shot *shot,
        const BCLIBCFFI_TrajectoryRequest *request,
        int32_t want_dense,
        BCLIBCFFI_TrajectoryData **out_records,
        int32_t *out_count,
        BCLIBCFFI_DenseStep **out_dense,
        int32_t *out_dense_count,
        int32_t *out_reason, /**< BCLIBCFFI_TerminationReason */
        BCLIBCFFI_Error *err);

    /**
     * @brief BCLIBCFFI_integrate_dense_shot() that also reports its step count in out_stats (when given).
     *
     * out_stats->accepted is the number of integration steps of the call (the engine's integration_step_count, as the
     * exts engine reports it); out_stats->rejected is 0, a fixed-step method rejects no step. Filled even when the
     * integration fails, as for the adaptive entry points.
     */
    BCLIBC_API int32_t BCLIBCFFI_integrate_dense_shot_stats(
        const BCLIBCFFI_Shot *shot,
        const BCLIBCFFI_TrajectoryRequest *request,
        int32_t want_dense,
        BCLIBCFFI_TrajectoryData **out_records,
        int32_t *out_count,
        BCLIBCFFI_DenseStep **out_dense,
        int32_t *out_dense_count,
        int32_t *out_reason, /**< BCLIBCFFI_TerminationReason */
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err);

    /**
     * @brief Barrel elevation for a zero at distance_ft; lofted != 0 selects the higher trajectory (the lofted
     * solver), lofted == 0 the lower one. Fixed-step method (shot->method).
     */
    BCLIBC_API int32_t BCLIBCFFI_find_zero_angle_lofted_shot(
        const BCLIBCFFI_Shot *shot,
        double distance_ft,
        int32_t lofted,
        double *out_angle_rad,
        BCLIBCFFI_Error *err);

    /**
     * @brief Zero angle and terminal point for a zero at distance_ft; lofted selects the higher trajectory.
     * Fixed-step method (shot->method).
     */
    BCLIBC_API int32_t BCLIBCFFI_find_zero_point_lofted_shot(
        const BCLIBCFFI_Shot *shot,
        double distance_ft,
        int32_t lofted,
        BCLIBCFFI_ZeroPointResult *out,
        BCLIBCFFI_Error *err);

    /** BCLIBCFFI_find_zero_angle_lofted_shot() with per-call tolerances and step statistics (adaptive method). */
    BCLIBC_API int32_t BCLIBCFFI_find_zero_angle_lofted_adaptive_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t method,
        double relative_tolerance,
        double absolute_tolerance,
        double distance_ft,
        int32_t lofted,
        double *out_angle_rad,
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err);

    /** BCLIBCFFI_find_zero_point_lofted_shot() with per-call tolerances and step statistics (adaptive method). */
    BCLIBC_API int32_t BCLIBCFFI_find_zero_point_lofted_adaptive_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t method,
        double relative_tolerance,
        double absolute_tolerance,
        double distance_ft,
        int32_t lofted,
        BCLIBCFFI_ZeroPointResult *out,
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err);

    /**
     * @brief Integrate until key reaches target_value (interpolated raw and full point), with per-call tolerances and
     * step statistics (adaptive method). The fixed-step variant is BCLIBCFFI_integrate_at_shot().
     */
    BCLIBC_API int32_t BCLIBCFFI_integrate_at_adaptive_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t method,
        double relative_tolerance,
        double absolute_tolerance,
        int32_t key, /**< BCLIBCFFI_BaseTrajInterpKey */
        double target_value,
        BCLIBCFFI_Interception *out,
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err);

    // ============================================================================
    // Solver building blocks (error_at_distance, range_for_angle, Newton zero, zero-init)
    //
    // The pieces the zero and max-range solvers are made of, for callers that run their own search.
    // Each has a fixed-step form (integrates with shot->method, adaptive methods at their default tolerances)
    // and an `_adaptive_` form with per-call tolerances and step statistics (same rules as above).
    // ============================================================================

    /** Outcome of BCLIBCFFI_zero_init_shot(). */
    typedef enum BCLIBCFFI_ZeroInitStatus
    {
        BCLIBCFFI_ZERO_INIT_CONTINUE = 0, /**< A search is needed. */
        BCLIBCFFI_ZERO_INIT_DONE = 1,     /**< The answer is already in look_angle_rad (very close or vertical shot). */
    } BCLIBCFFI_ZeroInitStatus;

    /** Starting data of a zero search (see BCLIBCFFI_zero_init_shot). */
    typedef struct BCLIBCFFI_ZeroInit
    {
        int32_t status;          /**< BCLIBCFFI_ZeroInitStatus */
        int32_t reserved;        /**< Padding, always 0. */
        double look_angle_rad;   /**< Look angle; when status is DONE for a close shot (closer than 2 * max(sight height, calc_step)), the barrel angle to use: the straight line from the bore up to the target, atan2(target_y - start_height_ft, target_x). */
        double slant_range_ft;   /**< Slant distance to the target. */
        double target_x_ft;      /**< Horizontal distance to the target. */
        double target_y_ft;      /**< Target height. */
        double start_height_ft;  /**< Muzzle height relative to the sight line (negative sight height, cant applied). */
    } BCLIBCFFI_ZeroInit;

    /**
     * @brief Vertical error of a shot fired at angle_rad, measured where x reaches target_x_ft:
     * (hit.y - target_y_ft) - |hit.x - target_x_ft|.
     *
     * When the trajectory ends before target_x_ft (minimum velocity, maximum drop or minimum altitude) the error is
     * measured at its last point instead: it is still finite and signed (a shot that falls short is negative), which
     * is what a zero search needs to bracket. Fails with BCLIBCFFI_ERR_SOLVER_RUNTIME only when nothing was
     * integrated.
     *
     * The shot's limits (config) apply as given. The zero solvers widen cMaximumDrop and cMinimumAltitude and drop
     * cMinimumVelocity for their own searches; to reproduce one of their evaluations, do the same.
     *
     * @param out_hit Optional. The interpolated point at target_x_ft, or the last point of a trajectory that ended
     * before it.
     */
    BCLIBC_API int32_t BCLIBCFFI_error_at_distance_shot(
        const BCLIBCFFI_Shot *shot,
        double angle_rad,
        double target_x_ft,
        double target_y_ft,
        double *out_error_ft,
        BCLIBCFFI_BaseTrajData *out_hit,
        BCLIBCFFI_Error *err);

    /** BCLIBCFFI_error_at_distance_shot() with per-call tolerances and step statistics. */
    BCLIBC_API int32_t BCLIBCFFI_error_at_distance_adaptive_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t method,
        double relative_tolerance,
        double absolute_tolerance,
        double angle_rad,
        double target_x_ft,
        double target_y_ft,
        double *out_error_ft,
        BCLIBCFFI_BaseTrajData *out_hit,
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err);

    /**
     * @brief Slant distance where a shot fired at angle_rad crosses the line of sight (downward) again.
     * *out_slant_range_ft is 0.0 when it never does.
     */
    BCLIBC_API int32_t BCLIBCFFI_range_for_angle_shot(
        const BCLIBCFFI_Shot *shot,
        double angle_rad,
        double *out_slant_range_ft,
        BCLIBCFFI_Error *err);

    /** BCLIBCFFI_range_for_angle_shot() with per-call tolerances and step statistics. */
    BCLIBC_API int32_t BCLIBCFFI_range_for_angle_adaptive_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t method,
        double relative_tolerance,
        double absolute_tolerance,
        double angle_rad,
        double *out_slant_range_ft,
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err);

    /**
     * @brief Zero angle for distance_ft by Newton's method only (BCLIBCFFI_find_zero_angle_shot() adds the Ridder
     * fallback). Fails with BCLIBCFFI_ERR_ZERO_FINDING when it does not converge.
     */
    BCLIBC_API int32_t BCLIBCFFI_find_zero_angle_newton_shot(
        const BCLIBCFFI_Shot *shot,
        double distance_ft,
        double *out_angle_rad,
        BCLIBCFFI_Error *err);

    /** BCLIBCFFI_find_zero_angle_newton_shot() with per-call tolerances and step statistics. */
    BCLIBC_API int32_t BCLIBCFFI_find_zero_angle_newton_adaptive_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t method,
        double relative_tolerance,
        double absolute_tolerance,
        double distance_ft,
        double *out_angle_rad,
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err);

    /**
     * @brief Starting data of a zero search for distance_ft. Fails with BCLIBCFFI_ERR_OUT_OF_RANGE for a vertical
     * shot whose apex is below the target.
     */
    BCLIBC_API int32_t BCLIBCFFI_zero_init_shot(
        const BCLIBCFFI_Shot *shot,
        double distance_ft,
        BCLIBCFFI_ZeroInit *out,
        BCLIBCFFI_Error *err);

    /** BCLIBCFFI_zero_init_shot() with per-call tolerances and step statistics. */
    BCLIBC_API int32_t BCLIBCFFI_zero_init_adaptive_shot(
        const BCLIBCFFI_Shot *shot,
        int32_t method,
        double relative_tolerance,
        double absolute_tolerance,
        double distance_ft,
        BCLIBCFFI_ZeroInit *out,
        BCLIBCFFI_StepStats *out_stats,
        BCLIBCFFI_Error *err);

    // ============================================================================
    // Working with dense output
    //
    // The accepted-step stream from the integrate calls above is a plain array of BCLIBCFFI_DenseStep. These
    // calls interpolate in it and turn a step into a full trajectory record, so callers need no Hermite, PCHIP or
    // shot-physics code of their own. `steps` must be time-ordered, as the integrate calls return it.
    // ============================================================================

    /**
     * @brief Interpolated step where `key` reaches `value`: the cubic Hermite of the stream step that holds the value
     * (positions and velocities at its two ends; the velocity of the point is the derivative of the position polynomial,
     * Mach is linear), the exact step when one matches. This is the convention of every step-based query of the library
     * (the rows of the trajectory filter, find_apex, integrate_at). The 3-point PCHIP of the raw steps is only a fallback
     * for a value that no step brackets (a key that is not monotone over the three steps around the bracket).
     *
     * @param start_from_time Only steps with time >= this are searched; 0 or negative disables it (ignored for TIME).
     * Fails with BCLIBCFFI_ERR_INTERCEPTION when the value is not inside the stream.
     */
    BCLIBC_API int32_t BCLIBCFFI_dense_get_at(
        const BCLIBCFFI_DenseStep *steps,
        int32_t count,
        int32_t key, /**< BCLIBCFFI_BaseTrajInterpKey */
        double value,
        double start_from_time,
        BCLIBCFFI_DenseStep *out,
        BCLIBCFFI_Error *err);

    /**
     * Interpolated step where the slant height (height above the line of sight at look_angle_rad) reaches `value`: the
     * Hermite of the stream step that holds it (see BCLIBCFFI_dense_get_at), the 3-point PCHIP only as the fallback.
     */
    BCLIBC_API int32_t BCLIBCFFI_dense_get_at_slant_height(
        const BCLIBCFFI_DenseStep *steps,
        int32_t count,
        double look_angle_rad,
        double value,
        BCLIBCFFI_DenseStep *out,
        BCLIBCFFI_Error *err);

    /**
     * @brief The full trajectory record (distance, drop, windage, energy, ...) of one dense step, using the shot's
     * atmosphere, drag and Coriolis data.
     *
     * @param flag BCLIBCFFI_TrajFlag value stored in the record.
     */
    BCLIBC_API int32_t BCLIBCFFI_trajectory_data_from_step_shot(
        const BCLIBCFFI_Shot *shot,
        const BCLIBCFFI_DenseStep *step,
        int32_t flag,
        BCLIBCFFI_TrajectoryData *out,
        BCLIBCFFI_Error *err);

    // ============================================================================
    // Growable buffer of base trajectory points and interpolation over points
    //
    // What BCLIBC_BaseTrajSeq and BCLIBC_BaseTrajData::interpolate / BCLIBC_BaseTrajSeq::interpolate_at give C++ code,
    // so that a binding does not rewrite them: the results are the C++ ones (same code). `key` is
    // BCLIBCFFI_BaseTrajInterpKey; an unknown key fails with BCLIBCFFI_ERR_INVALID_ARGUMENT. Indices are int64_t (the
    // counts of the older calls are int32_t, and stay so).
    //
    // The memory of a BCLIBCFFI_BaseTrajBuf belongs to the library: it is allocated by BCLIBCFFI_base_traj_buf_append
    // and released by BCLIBCFFI_base_traj_buf_free, both in this library (a module with another C runtime, on Windows,
    // must not free it itself). The arrays from the integrate calls are released with BCLIBCFFI_free_base_trajectory
    // instead, and the two are not interchangeable.
    // ============================================================================

    /** A growing array of points. A zero-initialised one (data NULL, count 0, capacity 0) is empty and ready to use. */
    typedef struct BCLIBCFFI_BaseTrajBuf
    {
        BCLIBCFFI_BaseTrajData *data;
        int32_t count;    /**< Points in use. */
        int32_t capacity; /**< Points that fit without growing. */
    } BCLIBCFFI_BaseTrajBuf;

    /** Releases the memory; the buffer is empty again and can be reused. NULL is ignored. */
    BCLIBC_API void BCLIBCFFI_base_traj_buf_free(BCLIBCFFI_BaseTrajBuf *buf);

    /**
     * Appends a copy of *item; the capacity doubles when the buffer is full.
     * @return BCLIBCFFI_OK; BCLIBCFFI_ERR_INVALID_ARGUMENT for a NULL buf or item; BCLIBCFFI_ERR_RUNTIME when the memory
     * cannot be had or the count would exceed INT32_MAX (the buffer is left as it was).
     */
    BCLIBC_API int32_t BCLIBCFFI_base_traj_buf_append(
        BCLIBCFFI_BaseTrajBuf *buf,
        const BCLIBCFFI_BaseTrajData *item,
        BCLIBCFFI_Error *err);

    /**
     * The point where `key` is `value`, from three consecutive points (BCLIBC_BaseTrajData::interpolate): the points are
     * taken as two integration steps, (p0, p1) and (p1, p2), and the value is located on the cubic Hermite of the one that
     * brackets it (positions and velocities at its ends; the velocity of the result is the derivative of the position
     * polynomial, Mach is linear), as in every step-based query of the library. Only when neither step brackets the value
     * is the 3-point PCHIP used, interpolating every field. The key field of the result is set to `value`.
     * @return BCLIBCFFI_OK, BCLIBCFFI_ERR_DOMAIN when two of the points have the same key value (f64_0, f64_1 are
     * the first two key values), BCLIBCFFI_ERR_INVALID_ARGUMENT for a NULL point or an unknown key.
     */
    BCLIBC_API int32_t BCLIBCFFI_base_traj_interpolate(
        int32_t key,
        double value,
        const BCLIBCFFI_BaseTrajData *p0,
        const BCLIBCFFI_BaseTrajData *p1,
        const BCLIBCFFI_BaseTrajData *p2,
        BCLIBCFFI_BaseTrajData *out,
        BCLIBCFFI_Error *err);

    /**
     * The same through the points idx-1, idx, idx+1 of `steps` (BCLIBC_BaseTrajSeq::interpolate_at): Hermite of the step
     * (idx-1, idx) or (idx, idx+1) that holds the value, the 3-point PCHIP as the fallback. A negative idx counts from the
     * end; the valid range is [1, count-2].
     * @return BCLIBCFFI_OK, BCLIBCFFI_ERR_OUT_OF_BOUNDS for an idx outside it (f64_0 = idx after the correction of a
     * negative one, f64_1 = 1, f64_2 = count-2), BCLIBCFFI_ERR_DOMAIN for equal key values.
     */
    BCLIBC_API int32_t BCLIBCFFI_base_traj_interpolate_at(
        const BCLIBCFFI_BaseTrajData *steps,
        int32_t count,
        int64_t idx,
        int32_t key,
        double value,
        BCLIBCFFI_BaseTrajData *out,
        BCLIBCFFI_Error *err);

    // ============================================================================
    // ABI layout introspection (WASM/JS interop only — unused by dart:ffi,
    // which derives struct layout at ffigen-generation time instead).
    // ============================================================================

    /**
     * Fills `out` with byte sizes/offsets for every BCLIBCFFI_Shot-family
     * struct field consumed by a hand-written (non-Embind) JS/wasm binding,
     * computed via sizeof()/offsetof() by whichever compiler builds this
     * library — so a JS-side binding never has to hardcode/guess struct
     * layout. See bclibc_ffi.cpp for the fixed field order this fills: the
     * structs in the order they were added to the ABI, each as its size and the
     * offsets of its fields, the newest at the end (BCLIBCFFI_StepStats,
     * BCLIBCFFI_ZeroInit, BCLIBCFFI_BaseTrajBuf). A struct added later is appended, so no offset moves.
     * The size of BCLIBCFFI_TrajectoryData in it is the one to use: it is 128 or 124 bytes by the platform.
     *
     * @param out      Caller-allocated buffer of at least BCLIBCFFI_LAYOUT_FIELD_COUNT int32s.
     * @param out_len  Capacity of `out`, in int32 elements.
     * @return Number of int32s written (== BCLIBCFFI_LAYOUT_FIELD_COUNT), or
     *         -1 if out_len is too small.
     */
    BCLIBC_API int32_t BCLIBCFFI_get_layout(int32_t *out, int32_t out_len);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // BCLIBC_FFI_H
