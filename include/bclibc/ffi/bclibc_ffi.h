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
// ============================================================================
#ifdef _WIN32
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
    } BCLIBCFFI_Status;

    /** Error output struct – filled by every function on failure. */
    typedef struct BCLIBCFFI_Error
    {
        int32_t code;      /**< BCLIBCFFI_Status */
        char message[512]; /**< Null-terminated error message */
        /* Extra fields for typed errors */
        double f64_0;  /**< OutOfRange: requested_distance_ft / ZeroFinding: zero_finding_error */
        double f64_1;  /**< OutOfRange: max_range_ft          / ZeroFinding: last_barrel_elevation_rad */
        double f64_2;  /**< OutOfRange: look_angle_rad */
        int32_t i32_0; /**< ZeroFinding: iterations_count */
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
        double humidity;     /**< 0.0 – 1.0 */

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

#ifdef __cplusplus
} // extern "C"
#endif

#endif // BCLIBC_FFI_H
