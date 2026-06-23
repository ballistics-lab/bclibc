#ifndef TINY_BCLIBC_BASE_TYPES_H
#define TINY_BCLIBC_BASE_TYPES_H

#include <stdint.h>
#include <math.h>
#include "platform.h"
#include "v3d.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ── Constants ────────────────────────────────────────────────────── */
#define TINY_BCLIBC_EARTH_ANG_VEL_RADS REAL_C(7.2921159e-5)
#define TINY_BCLIBC_STD_DENSITY_METRIC REAL_C(1.2250)
#define TINY_BCLIBC_DEG_C_TO_K REAL_C(273.15)
#define TINY_BCLIBC_SPEED_SOUND_IMPERIAL REAL_C(49.0223)
#define TINY_BCLIBC_SPEED_SOUND_METRIC REAL_C(20.0467)
#define TINY_BCLIBC_LAPSE_RATE_K_FT REAL_C(-0.0019812)
#define TINY_BCLIBC_PRESSURE_EXPONENT REAL_C(5.255876)
#define TINY_BCLIBC_M_TO_FEET REAL_C(3.280839895)
#define TINY_BCLIBC_MAX_WIND_DIST_FT REAL_C(1e8)
#define TINY_BCLIBC_GRAVITY_IMPERIAL REAL_C(32.17405)
#define TINY_BCLIBC_DEG_TO_RAD REAL_C(0.017453292519943295)
#define TINY_BCLIBC_MAX_INTEGRATION_RANGE REAL_C(3e6)

    /* ── Error codes ─────────────────────────────────────────────────── */
    typedef enum TINY_BCLIBC_Status
    {
        TINY_BCLIBC_OK = 0,
        TINY_BCLIBC_ERR_RUNTIME = 1,
        TINY_BCLIBC_ERR_OUT_OF_RANGE = 2,
        TINY_BCLIBC_ERR_ZERO_FINDING = 3,
        TINY_BCLIBC_ERR_INTERCEPTION = 4,
        TINY_BCLIBC_ERR_INVALID_ARG = 5,
        TINY_BCLIBC_ERR_BUF_TOO_SMALL = 6,
    } TINY_BCLIBC_Status;

    /* ── Trajectory flags ───────────────────────────────────────────── */
    typedef enum TINY_BCLIBC_TrajFlag
    {
        TINY_BCLIBC_TRAJ_FLAG_NONE = 0,
        TINY_BCLIBC_TRAJ_FLAG_ZERO_UP = 1,
        TINY_BCLIBC_TRAJ_FLAG_ZERO_DOWN = 2,
        TINY_BCLIBC_TRAJ_FLAG_ZERO = 3,
        TINY_BCLIBC_TRAJ_FLAG_MACH = 4,
        TINY_BCLIBC_TRAJ_FLAG_RANGE = 8,
        TINY_BCLIBC_TRAJ_FLAG_APEX = 16,
        TINY_BCLIBC_TRAJ_FLAG_ALL = 31,
        TINY_BCLIBC_TRAJ_FLAG_MRT = 32,
    } TINY_BCLIBC_TrajFlag;

    /* ── Termination reasons ─────────────────────────────────────────────── */
    typedef enum TINY_BCLIBC_TerminationReason
    {
        TINY_BCLIBC_TERM_NO_TERMINATE = 0,
        TINY_BCLIBC_TERM_TARGET_RANGE_REACHED = 1,
        TINY_BCLIBC_TERM_MIN_VELOCITY_REACHED = 2,
        TINY_BCLIBC_TERM_MAX_DROP_REACHED = 3,
        TINY_BCLIBC_TERM_MIN_ALTITUDE_REACHED = 4,
        TINY_BCLIBC_TERM_HANDLER_STOP = 5,
    } TINY_BCLIBC_TerminationReason;

    /* ── Config ────────────────────────────────────────────────── */
    typedef struct TINY_BCLIBC_Config
    {
        real_t cStepMultiplier;
        real_t cZeroFindingAccuracy;
        real_t cMinimumVelocity;
        real_t cMaximumDrop;
        int32_t cMaxIterations;
        real_t cGravityConstant;
        real_t cMinimumAltitude;
    } TINY_BCLIBC_Config;

    TINY_BCLIBC_INLINE_FUNC TINY_BCLIBC_Config TINY_BCLIBC_Config_default(void)
    {
        TINY_BCLIBC_Config c;
        c.cStepMultiplier = REAL_C(0.5);
        c.cZeroFindingAccuracy = REAL_C(0.001);
        c.cMinimumVelocity = REAL_C(50.0);
        c.cMaximumDrop = REAL_C(-15000.0);
        c.cMaxIterations = 50;
        c.cGravityConstant = -TINY_BCLIBC_GRAVITY_IMPERIAL;
        c.cMinimumAltitude = REAL_C(-1500.0);
        return c;
    }

    /* ── PCHIP-segment ───────────────────────────────────────────────── */
    typedef struct TINY_BCLIBC_CurvePoint
    {
        real_t a, b, c, d;
    } TINY_BCLIBC_CurvePoint;

    /* ── Atmosphere ───────────────────────────────────────────────────── */
    typedef struct TINY_BCLIBC_Atmosphere
    {
        real_t t0;   /* basic temp (°C)     */
        real_t a0;   /* basic altitude (ft)          */
        real_t p0;   /* basic pressure (hPa)          */
        real_t mach; /* speed of sound (fps)       */
        real_t density_ratio;
        real_t cLowestTempC;
    } TINY_BCLIBC_Atmosphere;

    /* Atmosphere processing — CIPM-2007 */
    TINY_BCLIBC_INLINE_FUNC TINY_BCLIBC_Atmosphere
    TINY_BCLIBC_Atmosphere_from_conditions(real_t t_c, real_t p_hpa, real_t alt_ft, real_t humidity)
    {
        const real_t kLowestTempC = (REAL_C(-130.0) - REAL_C(32.0)) * REAL_C(5.0) / REAL_C(9.0);
        const real_t T_K = t_c + TINY_BCLIBC_DEG_C_TO_K;
        const real_t mach_fps = TINY_BCLIBC_SQRT(T_K * (REAL_C(9.0) / REAL_C(5.0))) * TINY_BCLIBC_SPEED_SOUND_IMPERIAL;

        TINY_BCLIBC_Atmosphere a;
        a.t0 = t_c;
        a.a0 = alt_ft;
        a.p0 = p_hpa;
        a.mach = mach_fps;
        a.cLowestTempC = kLowestTempC;

        if (p_hpa <= REAL_C(0.0))
        {
            a.density_ratio = REAL_C(0.0);
            return a;
        }

        const real_t p = p_hpa * REAL_C(100.0);
        real_t rh = (humidity > REAL_C(1.0)) ? humidity / REAL_C(100.0) : humidity;
        if (rh < REAL_C(0.0))
            rh = REAL_C(0.0);
        if (rh > REAL_C(1.0))
            rh = REAL_C(1.0);

        /* saturated vapour pressure (CIPM-2007 A1.1) */
        const real_t p_sv = TINY_BCLIBC_EXP(
            REAL_C(1.2378847e-5) * T_K * T_K - REAL_C(1.9121316e-2) * T_K + REAL_C(33.93711047) - REAL_C(6.3431645e3) / T_K);

        const real_t f = REAL_C(1.00062) + REAL_C(3.14e-8) * p + REAL_C(5.6e-7) * t_c * t_c;
        const real_t x_v = (rh * f * p_sv) / p;

        const real_t Z = REAL_C(1.0) - (p / T_K) * (REAL_C(1.58123e-6) + REAL_C(-2.9331e-8) * t_c + REAL_C(1.1043e-10) * t_c * t_c + (REAL_C(5.707e-6) + REAL_C(-2.051e-8) * t_c) * x_v + (REAL_C(1.9898e-4) + REAL_C(-2.376e-6) * t_c) * x_v * x_v) + (p / T_K) * (p / T_K) * (REAL_C(1.83e-11) + REAL_C(-0.765e-8) * x_v * x_v);

        const real_t Ma = REAL_C(28.96546e-3);
        const real_t Mv = REAL_C(18.01528e-3);
        const real_t R = REAL_C(8.314472);
        const real_t density = (p * Ma) / (Z * R * T_K) * (REAL_C(1.0) - x_v * (REAL_C(1.0) - Mv / Ma));
        a.density_ratio = density / TINY_BCLIBC_STD_DENSITY_METRIC;
        return a;
    }

    TINY_BCLIBC_INLINE_FUNC void
    TINY_BCLIBC_Atmosphere_update_density_mach(const TINY_BCLIBC_Atmosphere *a, real_t altitude,
                                               real_t *density_ratio_out, real_t *mach_out)
    {
        const real_t alt_diff = altitude - a->a0;
        if (TINY_BCLIBC_FABS(alt_diff) < REAL_C(30.0))
        {
            *density_ratio_out = a->density_ratio;
            *mach_out = a->mach;
            return;
        }
        real_t celsius = alt_diff * TINY_BCLIBC_LAPSE_RATE_K_FT + a->t0;
        const real_t min_temp = -TINY_BCLIBC_DEG_C_TO_K;
        if (celsius < min_temp)
            celsius = min_temp;
        else if (celsius < a->cLowestTempC)
            celsius = a->cLowestTempC;

        const real_t kelvin = celsius + TINY_BCLIBC_DEG_C_TO_K;
        const real_t base_kelvin = a->t0 + TINY_BCLIBC_DEG_C_TO_K;
        const real_t pressure = a->p0 * TINY_BCLIBC_POW(
                                            REAL_C(1.0) + TINY_BCLIBC_LAPSE_RATE_K_FT * alt_diff / base_kelvin,
                                            TINY_BCLIBC_PRESSURE_EXPONENT);
        const real_t density_delta = (base_kelvin * pressure) / (a->p0 * kelvin);
        *density_ratio_out = a->density_ratio * density_delta;
        *mach_out = TINY_BCLIBC_SQRT(kelvin) * TINY_BCLIBC_SPEED_SOUND_METRIC * TINY_BCLIBC_M_TO_FEET;
    }

    /* ── Coriolis ────────────────────────────────────────────────────── */
    typedef struct TINY_BCLIBC_Coriolis
    {
        real_t sin_lat, cos_lat, sin_az, cos_az;
        real_t range_east, range_north;
        real_t cross_east, cross_north;
        int32_t flat_fire_only;
        real_t muzzle_velocity_fps;
    } TINY_BCLIBC_Coriolis;

    TINY_BCLIBC_INLINE_FUNC TINY_BCLIBC_Coriolis
    TINY_BCLIBC_Coriolis_from_lat_az(real_t lat_deg, real_t muzzle_velocity_fps, real_t az_deg)
    {
        TINY_BCLIBC_Coriolis c;
        c.muzzle_velocity_fps = muzzle_velocity_fps;

        /* NaN lat → no Coriolis */
        if (lat_deg != lat_deg)
        { /* isnan */
            c.sin_lat = c.cos_lat = c.sin_az = c.cos_az = REAL_C(0.0);
            c.range_east = c.range_north = c.cross_east = c.cross_north = REAL_C(0.0);
            c.flat_fire_only = 1;
            return c;
        }
        const real_t lat_rad = lat_deg * TINY_BCLIBC_DEG_TO_RAD;
        c.sin_lat = TINY_BCLIBC_SIN(lat_rad);
        c.cos_lat = TINY_BCLIBC_COS(lat_rad);

        /* NaN az → flat-fire only */
        if (az_deg != az_deg)
        {
            c.sin_az = c.cos_az = REAL_C(0.0);
            c.range_east = c.range_north = c.cross_east = c.cross_north = REAL_C(0.0);
            c.flat_fire_only = 1;
            return c;
        }
        const real_t az_rad = az_deg * TINY_BCLIBC_DEG_TO_RAD;
        c.sin_az = TINY_BCLIBC_SIN(az_rad);
        c.cos_az = TINY_BCLIBC_COS(az_rad);
        c.range_east = c.sin_az;
        c.range_north = c.cos_az;
        c.cross_east = c.cos_az;
        c.cross_north = -c.sin_az;
        c.flat_fire_only = 0;
        return c;
    }

    TINY_BCLIBC_INLINE_FUNC void
    TINY_BCLIBC_Coriolis_flat_fire_offsets(const TINY_BCLIBC_Coriolis *c, real_t time,
                                           real_t distance_ft, real_t drop_ft,
                                           real_t *delta_y, real_t *delta_z)
    {
        if (!c->flat_fire_only)
        {
            *delta_y = *delta_z = REAL_C(0.0);
            return;
        }
        real_t horizontal = TINY_BCLIBC_EARTH_ANG_VEL_RADS * distance_ft * c->sin_lat * time;
        real_t vertical = REAL_C(0.0);
        if (c->sin_az != REAL_C(0.0))
        {
            real_t vf = -REAL_C(2.0) * TINY_BCLIBC_EARTH_ANG_VEL_RADS * c->muzzle_velocity_fps * c->cos_lat * c->sin_az;
            vertical = drop_ft * (vf / TINY_BCLIBC_GRAVITY_IMPERIAL);
        }
        *delta_y = vertical;
        *delta_z = horizontal;
    }

    TINY_BCLIBC_INLINE_FUNC TINY_BCLIBC_V3dT
    TINY_BCLIBC_Coriolis_adjust_range(const TINY_BCLIBC_Coriolis *c, real_t time, TINY_BCLIBC_V3dT range_vector)
    {
        if (!c->flat_fire_only)
            return range_vector;
        real_t dy, dz;
        TINY_BCLIBC_Coriolis_flat_fire_offsets(c, time, range_vector.x, range_vector.y, &dy, &dz);
        if (dy == REAL_C(0.0) && dz == REAL_C(0.0))
            return range_vector;
        return TINY_BCLIBC_V3dT_make(range_vector.x, range_vector.y + dy, range_vector.z + dz);
    }

    TINY_BCLIBC_INLINE_FUNC void
    TINY_BCLIBC_Coriolis_acceleration_local(const TINY_BCLIBC_Coriolis *c, TINY_BCLIBC_V3dT vel, TINY_BCLIBC_V3dT *out)
    {
        if (c->flat_fire_only)
        {
            *out = TINY_BCLIBC_V3dT_make(REAL_C(0.0), REAL_C(0.0), REAL_C(0.0));
            return;
        }
        const real_t vx = vel.x, vy = vel.y, vz = vel.z;
        const real_t vel_east = vx * c->range_east + vz * c->cross_east;
        const real_t vel_north = vx * c->range_north + vz * c->cross_north;
        const real_t vel_up = vy;
        const real_t factor = -REAL_C(2.0) * TINY_BCLIBC_EARTH_ANG_VEL_RADS;
        const real_t ae = factor * (c->cos_lat * vel_up - c->sin_lat * vel_north);
        const real_t an = factor * c->sin_lat * vel_east;
        const real_t au = factor * (-c->cos_lat * vel_east);
        out->x = ae * c->range_east + an * c->range_north;
        out->y = au;
        out->z = ae * c->cross_east + an * c->cross_north;
    }

    /* ── Wind / WindSock ────────────────────────────────────────────── */
    typedef struct TINY_BCLIBC_Wind
    {
        real_t velocity_fps;
        real_t direction_from_rad;
        real_t until_distance_ft;
        real_t max_distance_ft; /* sentinel: TINY_BCLIBC_MAX_WIND_DIST_FT */
    } TINY_BCLIBC_Wind;

    TINY_BCLIBC_INLINE_FUNC TINY_BCLIBC_V3dT TINY_BCLIBC_Wind_as_v3d(const TINY_BCLIBC_Wind *w)
    {
        return TINY_BCLIBC_V3dT_make(
            w->velocity_fps * TINY_BCLIBC_COS(w->direction_from_rad),
            REAL_C(0.0),
            w->velocity_fps * TINY_BCLIBC_SIN(w->direction_from_rad));
    }

    typedef struct TINY_BCLIBC_WindSock
    {
        const TINY_BCLIBC_Wind *winds;
        int32_t count;
        int32_t current_idx;
        real_t next_range;
        TINY_BCLIBC_V3dT last_vector;
    } TINY_BCLIBC_WindSock;

    TINY_BCLIBC_INLINE_FUNC void TINY_BCLIBC_WindSock_update_cache(TINY_BCLIBC_WindSock *ws)
    {
        if (ws->current_idx < ws->count)
        {
            ws->last_vector = TINY_BCLIBC_Wind_as_v3d(&ws->winds[ws->current_idx]);
            ws->next_range = ws->winds[ws->current_idx].until_distance_ft;
        }
        else
        {
            ws->last_vector = TINY_BCLIBC_V3dT_make(REAL_C(0.0), REAL_C(0.0), REAL_C(0.0));
            ws->next_range = TINY_BCLIBC_MAX_WIND_DIST_FT;
        }
    }

    TINY_BCLIBC_INLINE_FUNC TINY_BCLIBC_WindSock
    TINY_BCLIBC_WindSock_make(const TINY_BCLIBC_Wind *winds, int32_t count)
    {
        TINY_BCLIBC_WindSock ws;
        ws.winds = winds;
        ws.count = count;
        ws.current_idx = 0;
        ws.next_range = TINY_BCLIBC_MAX_WIND_DIST_FT;
        ws.last_vector = TINY_BCLIBC_V3dT_make(REAL_C(0.0), REAL_C(0.0), REAL_C(0.0));
        TINY_BCLIBC_WindSock_update_cache(&ws);
        return ws;
    }

    TINY_BCLIBC_INLINE_FUNC TINY_BCLIBC_V3dT
    TINY_BCLIBC_WindSock_vector_for_range(TINY_BCLIBC_WindSock *ws, real_t range)
    {
        if (range >= ws->next_range)
        {
            ws->current_idx++;
            TINY_BCLIBC_WindSock_update_cache(ws);
        }
        return ws->last_vector;
    }

    /* ── ShotProps ───────────────────────────────────────────────────── */
    typedef struct TINY_BCLIBC_ShotProps
    {
        real_t bc;
        real_t muzzle_velocity;
        real_t weight;
        real_t diameter;
        real_t length;
        real_t stability_coefficient;
        real_t sight_height;
        real_t twist;
        real_t barrel_elevation;
        real_t barrel_azimuth;
        real_t look_angle;
        real_t cant_cosine;
        real_t cant_sine;
        real_t alt0;
        real_t calc_step;

        TINY_BCLIBC_Atmosphere atmo;
        TINY_BCLIBC_Coriolis coriolis;
        TINY_BCLIBC_WindSock wind_sock;
        TINY_BCLIBC_Config cfg;

        const TINY_BCLIBC_CurvePoint *curve; /* caller-owned PCHIP buffer */
        const real_t *mach_list;             /* caller-owned Mach array   */
        int32_t curve_count;
    } TINY_BCLIBC_ShotProps;

    /* Spin-drift (Litz) */
    TINY_BCLIBC_INLINE_FUNC real_t TINY_BCLIBC_ShotProps_spin_drift(const TINY_BCLIBC_ShotProps *p, real_t time)
    {
        if (p->twist == REAL_C(0.0) || p->stability_coefficient == REAL_C(0.0))
            return REAL_C(0.0);
        real_t sign = (p->twist > REAL_C(0.0)) ? REAL_C(1.0) : REAL_C(-1.0);
        return sign * (REAL_C(1.25) * (p->stability_coefficient + REAL_C(1.2)) * TINY_BCLIBC_POW(time, REAL_C(1.83))) / REAL_C(12.0);
    }

    /* drag_by_mach — Horner evaluation on caller-owned PCHIP curve */
    TINY_BCLIBC_INLINE_FUNC real_t TINY_BCLIBC_ShotProps_drag_by_mach(const TINY_BCLIBC_ShotProps *p, real_t mach)
    {
        const int32_t nm1 = p->curve_count - 1;
        if (nm1 < 1 || !p->curve || !p->mach_list)
            return REAL_C(0.0);
        const real_t *xs = p->mach_list;
        int32_t i;
        if (mach <= xs[0])
        {
            i = 0;
        }
        else if (mach >= xs[p->curve_count - 1])
        {
            i = nm1 - 1;
        }
        else if (p->curve_count <= 15)
        {
            i = 0;
            while (i < nm1 - 1 && xs[i + 1] < mach)
                i++;
        }
        else
        {
            int32_t lo = 0, hi = p->curve_count - 1;
            while (lo < hi)
            {
                int32_t mid = lo + ((hi - lo) >> 1);
                if (xs[mid] < mach)
                    lo = mid + 1;
                else
                    hi = mid;
            }
            i = lo - 1;
            if (i < 0)
                i = 0;
            if (i > nm1 - 1)
                i = nm1 - 1;
        }
        const TINY_BCLIBC_CurvePoint seg = p->curve[i];
        const real_t dx = mach - xs[i];
        const real_t cd = seg.d + dx * (seg.c + dx * (seg.b + dx * seg.a));
        return cd * REAL_C(2.08551e-04) / p->bc;
    }

    /* Miller stability coefficient */
    TINY_BCLIBC_INLINE_FUNC real_t TINY_BCLIBC_ShotProps_calc_stability(const TINY_BCLIBC_ShotProps *p)
    {
        if (p->twist == REAL_C(0.0) || p->length == REAL_C(0.0) ||
            p->diameter == REAL_C(0.0) || p->atmo.p0 == REAL_C(0.0))
            return REAL_C(0.0);
        real_t twist_rate = TINY_BCLIBC_FABS(p->twist) / p->diameter;
        real_t len = p->length / p->diameter;
        real_t denom1 = twist_rate * twist_rate;
        real_t denom2 = p->diameter * p->diameter * p->diameter;
        real_t denom3 = len;
        real_t denom4 = REAL_C(1.0) + len * len;
        if (denom1 == REAL_C(0.0) || denom2 == REAL_C(0.0) || denom3 == REAL_C(0.0) || denom4 == REAL_C(0.0))
            return REAL_C(0.0);
        real_t sd = REAL_C(30.0) * p->weight / (denom1 * denom2 * denom3 * denom4);
        real_t fv = TINY_BCLIBC_POW(p->muzzle_velocity / REAL_C(2800.0), REAL_C(1.0) / REAL_C(3.0));
        real_t ft = (p->atmo.t0 * REAL_C(9.0) / REAL_C(5.0)) + REAL_C(32.0);
        real_t pt = p->atmo.p0 / REAL_C(33.863881565591);
        if (pt == REAL_C(0.0))
            return REAL_C(0.0);
        real_t ftp = ((ft + REAL_C(460.0)) / (REAL_C(59.0) + REAL_C(460.0))) * (REAL_C(29.92) / pt);
        return sd * fv * ftp;
    }

    /* ── User-facing Shot ────────────────────────────────────────────── */
    typedef struct TINY_BCLIBC_Shot
    {
        real_t bc;
        real_t weight_grain;
        real_t diameter_inch;
        real_t length_inch;
        real_t muzzle_velocity_fps;
        real_t sight_height_ft;
        real_t twist_inch;
        real_t temp_c;
        real_t pressure_hpa;
        real_t altitude_ft;
        real_t humidity;
        const real_t *mach_data;
        const real_t *cd_data;
        int32_t drag_table_size;
        const TINY_BCLIBC_Wind *winds;
        int32_t wind_count;
        real_t look_angle_rad;
        real_t barrel_elevation_rad;
        real_t barrel_azimuth_rad;
        real_t cant_angle_rad;
        real_t latitude_deg;
        real_t azimuth_deg;
        TINY_BCLIBC_Config config;
    } TINY_BCLIBC_Shot;

    /* ── PCHIP builder ───────────────────────────────────────────────── */
    TINY_BCLIBC_INLINE_FUNC void
    tiny_bclibc__build_pchip(const real_t *x, const real_t *y, int32_t n, TINY_BCLIBC_CurvePoint *out)
    {
        if (n < 2)
            return;
        int32_t nm1 = n - 1;
        /* out[i].b = h[i], out[i].c = d[i] (temp), out[i].a = m[i] (slope) */
        int32_t i;
        for (i = 0; i < nm1; i++)
        {
            out[i].b = x[i + 1] - x[i];              /* h[i] */
            out[i].c = (y[i + 1] - y[i]) / out[i].b; /* d[i] */
        }
        /* slopes m[0..n-1] — stored in out[i].a for i<nm1, last m in a temp */
        real_t m_last;
        if (n == 2)
        {
            out[0].a = out[0].c; /* m[0] */
            m_last = out[0].c;   /* m[1] */
        }
        else
        {
            /* interior slopes */
            for (i = 1; i < nm1; i++)
            {
                real_t di_m1 = out[i - 1].c;
                real_t di = out[i].c;
                if (di_m1 * di <= REAL_C(0.0))
                {
                    out[i].a = REAL_C(0.0);
                }
                else
                {
                    real_t w1 = REAL_C(2.0) * out[i].b + out[i - 1].b;
                    real_t w2 = out[i].b + REAL_C(2.0) * out[i - 1].b;
                    out[i].a = (w1 + w2) / (w1 / di_m1 + w2 / di);
                }
            }
            /* endpoint slopes */
            {
                real_t h0 = out[0].b, h1 = out[1].b;
                real_t d0 = out[0].c, d1 = out[1].c;
                real_t hs = h0 + h1;
                real_t r = ((REAL_C(2.0) * h0 + h1) * d0 - h0 * d1) / hs;
                if (r * d0 <= REAL_C(0.0))
                    r = REAL_C(0.0);
                else if (TINY_BCLIBC_FABS(r) > REAL_C(3.0) * TINY_BCLIBC_FABS(d0))
                    r = REAL_C(3.0) * d0;
                out[0].a = r;
            }
            {
                int32_t k = nm1 - 1;
                real_t hk = out[k].b, hkm1 = out[k - 1].b;
                real_t dk = out[k].c, dkm1 = out[k - 1].c;
                real_t hs = hk + hkm1;
                real_t r = ((REAL_C(2.0) * hk + hkm1) * dk - hk * dkm1) / hs;
                if (r * dk <= REAL_C(0.0))
                    r = REAL_C(0.0);
                else if (TINY_BCLIBC_FABS(r) > REAL_C(3.0) * TINY_BCLIBC_FABS(dk))
                    r = REAL_C(3.0) * dk;
                m_last = r;
            }
        }
        /* Horner coefficients */
        for (i = 0; i < nm1; i++)
        {
            real_t H = out[i].b;  /* h[i]   */
            real_t mi = out[i].a; /* m[i]   */
            real_t mip1 = (i + 1 < nm1) ? out[i + 1].a : m_last;
            real_t A = (y[i + 1] - y[i] - mi * H) / (H * H);
            real_t B = (mip1 - mi) / H;
            out[i].a = (B - REAL_C(2.0) * A) / H;
            out[i].b = REAL_C(3.0) * A - B;
            out[i].c = mi;
            out[i].d = y[i];
        }
    }

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* TINY_BCLIBC_BASE_TYPES_H */
