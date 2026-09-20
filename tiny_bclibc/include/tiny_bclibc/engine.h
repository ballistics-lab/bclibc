#ifndef TINY_BCLIBC_ENGINE_H
#define TINY_BCLIBC_ENGINE_H

#include <stdint.h>
#include <string.h>
#include "platform.h"
#include "base_types.h"
#include "traj_data.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* ════════════════════════════════════════════════════════════════════
     *  Errors buffer
     * ════════════════════════════════════════════════════════════════════ */

#if defined(TINY_BCLIBC_BUILD_SHARED)
    extern TINY_BCLIBC_THREAD_LOCAL char tiny_bclibc__s_error[512];

    TINY_BCLIBC_FUNC const char *tiny_bclibc_last_error(void);

    static inline void tiny_bclibc__set_error(const char *msg)
    {
        size_t i = 0;
        while (msg[i] && i < 511)
        {
            tiny_bclibc__s_error[i] = msg[i];
            i++;
        }
        tiny_bclibc__s_error[i] = '\0';
    }

#elif defined(TINY_BCLIBC_NO_ERR_BUF)
/* natmod / no-BSS: no writable error buffer — caller uses error codes */
static inline const char *tiny_bclibc_last_error(void) { return "tiny_bclibc error"; }
static inline void tiny_bclibc__set_error(const char *msg) { (void)msg; }

#else
/* Header-only: static local → per-TU buffer */
TINY_BCLIBC_INLINE_FUNC const char *tiny_bclibc_last_error(void)
{
    static TINY_BCLIBC_THREAD_LOCAL char buf[512];
    return buf;
}

static inline void tiny_bclibc__set_error(const char *msg)
{
    char *buf = (char *)tiny_bclibc_last_error();
    size_t i = 0;
    while (msg[i] && i < 511)
    {
        buf[i] = msg[i];
        i++;
    }
    buf[i] = '\0';
}
#endif /* TINY_BCLIBC_BUILD_SHARED */

    /* ════════════════════════════════════════════════════════════════════
     *  Utils (always inline)
     * ════════════════════════════════════════════════════════════════════ */

    TINY_BCLIBC_INLINE_FUNC real_t tiny_bclibc_get_correction(real_t distance_ft, real_t offset_ft)
    {
        if (distance_ft == REAL_C(0.0))
            return REAL_C(0.0);
        return TINY_BCLIBC_ATAN2(offset_ft, distance_ft);
    }

    TINY_BCLIBC_INLINE_FUNC real_t tiny_bclibc_calculate_energy(real_t weight_grain, real_t velocity_fps)
    {
        return weight_grain * velocity_fps * velocity_fps / REAL_C(450400.0);
    }

    TINY_BCLIBC_INLINE_FUNC real_t tiny_bclibc_calculate_ogw(real_t weight_grain, real_t velocity_fps)
    {
        return weight_grain * weight_grain * velocity_fps * velocity_fps * velocity_fps * REAL_C(1.5e-12);
    }

    static inline real_t tiny_bclibc__fmax(real_t a, real_t b)
    {
        return (a > b) ? a : b;
    }

    /* ════════════════════════════════════════════════════════════════════
     *  Internal derivative helpers
     * ════════════════════════════════════════════════════════════════════ */

    /* Projectile acceleration: a = gravity+coriolis − km*|v_rel|*v_rel */
    static inline void tiny_bclibc__calc_dvdt(
        TINY_BCLIBC_V3dT v_rel,
        TINY_BCLIBC_V3dT gravity_plus_coriolis,
        real_t km, real_t v_mag,
        TINY_BCLIBC_V3dT *acc)
    {
        acc->x = gravity_plus_coriolis.x - km * v_mag * v_rel.x;
        acc->y = gravity_plus_coriolis.y - km * v_mag * v_rel.y;
        acc->z = gravity_plus_coriolis.z - km * v_mag * v_rel.z;
    }

    /* ── Context for on_step callback (used only by integrate_raw emulation) ── */
    typedef int32_t (*tiny_bclibc__OnStep)(const TINY_BCLIBC_BaseTrajData *pt, void *ctx);

    /* ── Streaming callback ── */
    typedef int32_t (*tiny_bclibc_StreamCb)(const TINY_BCLIBC_TrajectoryData *pt, void *ctx);

    /* ── Raw per-step callback (public) ── */
    typedef int32_t (*tiny_bclibc_RawStepCb)(const TINY_BCLIBC_BaseTrajData *pt, void *ctx);

    /* ── Interval callback — receives one accepted integration step (start, end) ── */
    typedef int32_t (*tiny_bclibc__OnInterval)(const TINY_BCLIBC_BaseTrajData *start,
                                               const TINY_BCLIBC_BaseTrajData *end,
                                               void *ctx);

    /* ── Stop control ── */
    typedef struct tiny_bclibc__StopCtrl
    {
        real_t range_limit_ft;
        real_t min_velocity_fps;
        real_t max_drop_ft;
        real_t min_altitude_ft;
        real_t initial_altitude_ft;
        int32_t step_count;
        int32_t *reason_out;
    } tiny_bclibc__StopCtrl;

    static inline void tiny_bclibc__stop_ctrl_init(tiny_bclibc__StopCtrl *sc,
                                                   const TINY_BCLIBC_ShotProps *props,
                                                   real_t range_limit_ft,
                                                   real_t min_vel, real_t max_drop, real_t min_alt,
                                                   int32_t *reason_out)
    {
        sc->range_limit_ft = range_limit_ft;
        sc->min_velocity_fps = min_vel;
        sc->max_drop_ft = -TINY_BCLIBC_FABS(max_drop) + ((-props->cant_cosine * props->sight_height) < REAL_C(0.0)
                                                             ? -props->cant_cosine * props->sight_height
                                                             : REAL_C(0.0));
        sc->min_altitude_ft = min_alt;
        sc->initial_altitude_ft = props->alt0;
        sc->step_count = 0;
        sc->reason_out = reason_out;
    }

    static inline void tiny_bclibc__stop_ctrl_check(tiny_bclibc__StopCtrl *sc,
                                                    const TINY_BCLIBC_BaseTrajData *pt)
    {
        if (*sc->reason_out != TINY_BCLIBC_TERM_NO_TERMINATE)
            return;
        sc->step_count++;
        if (sc->step_count >= 3 && pt->px > sc->range_limit_ft)
        {
            *sc->reason_out = TINY_BCLIBC_TERM_TARGET_RANGE_REACHED;
            return;
        }
        real_t vel = TINY_BCLIBC_SQRT(pt->vx * pt->vx + pt->vy * pt->vy + pt->vz * pt->vz);
        if (vel < sc->min_velocity_fps)
        {
            *sc->reason_out = TINY_BCLIBC_TERM_MIN_VELOCITY_REACHED;
            return;
        }
        if (pt->py < sc->max_drop_ft)
        {
            *sc->reason_out = TINY_BCLIBC_TERM_MAX_DROP_REACHED;
            return;
        }
        if (pt->vy <= REAL_C(0.0))
        {
            real_t alt = sc->initial_altitude_ft + pt->py;
            if (alt < sc->min_altitude_ft)
                *sc->reason_out = TINY_BCLIBC_TERM_MIN_ALTITUDE_REACHED;
        }
    }

    /* ════════════════════════════════════════════════════════════════════
     *  Hermite reconstruction helpers (interval-based)
     * ════════════════════════════════════════════════════════════════════ */

    static inline int32_t tiny_bclibc__hermite_at_time(
        const TINY_BCLIBC_BaseTrajData *a, const TINY_BCLIBC_BaseTrajData *b,
        real_t t, TINY_BCLIBC_BaseTrajData *out)
    {
        real_t dt = b->time - a->time;
        if (dt <= REAL_C(0.0))
            return 0;
        real_t u = (t - a->time) / dt;
        out->time = t;
        out->px = tiny_bclibc_hermite(t, a->time, b->time, a->px, b->px, a->vx, b->vx);
        out->py = tiny_bclibc_hermite(t, a->time, b->time, a->py, b->py, a->vy, b->vy);
        out->pz = tiny_bclibc_hermite(t, a->time, b->time, a->pz, b->pz, a->vz, b->vz);
        out->vx = tiny_bclibc_hermite_derivative(t, a->time, b->time, a->px, b->px, a->vx, b->vx);
        out->vy = tiny_bclibc_hermite_derivative(t, a->time, b->time, a->py, b->py, a->vy, b->vy);
        out->vz = tiny_bclibc_hermite_derivative(t, a->time, b->time, a->pz, b->pz, a->vz, b->vz);
        out->mach = a->mach + u * (b->mach - a->mach);
        return 1;
    }

    static inline int32_t tiny_bclibc__hermite_at_x(
        const TINY_BCLIBC_BaseTrajData *a, const TINY_BCLIBC_BaseTrajData *b,
        real_t target_x, TINY_BCLIBC_BaseTrajData *out)
    {
        if (!((a->px <= target_x && target_x <= b->px) ||
              (b->px <= target_x && target_x <= a->px)))
            return 0;

        real_t lo = a->time, hi = b->time;
        int32_t increasing = (b->px >= a->px);
        int32_t i;
        for (i = 0; i < 40; i++)
        {
            real_t mid = REAL_C(0.5) * (lo + hi);
            TINY_BCLIBC_BaseTrajData sample;
            if (!tiny_bclibc__hermite_at_time(a, b, mid, &sample))
                return 0;
            if ((sample.px < target_x) == increasing)
                lo = mid;
            else
                hi = mid;
        }
        if (!tiny_bclibc__hermite_at_time(a, b, REAL_C(0.5) * (lo + hi), out))
            return 0;
        out->px = target_x;
        return 1;
    }

    typedef real_t (*tiny_bclibc__ValueFn)(const TINY_BCLIBC_BaseTrajData *pt, const void *aux);

    static inline int32_t tiny_bclibc__hermite_at_value(
        const TINY_BCLIBC_BaseTrajData *a, const TINY_BCLIBC_BaseTrajData *b,
        tiny_bclibc__ValueFn value, const void *aux, real_t target, TINY_BCLIBC_BaseTrajData *out)
    {
        real_t lo = a->time, hi = b->time;
        real_t flo = value(a, aux) - target;
        real_t fhi = value(b, aux) - target;
        if (flo == REAL_C(0.0))
        {
            *out = *a;
            return 1;
        }
        if (fhi == REAL_C(0.0))
        {
            *out = *b;
            return 1;
        }
        if ((flo < REAL_C(0.0)) == (fhi < REAL_C(0.0)))
            return 0;

        int32_t i;
        for (i = 0; i < 40; i++)
        {
            real_t mid = REAL_C(0.5) * (lo + hi);
            TINY_BCLIBC_BaseTrajData sample;
            if (!tiny_bclibc__hermite_at_time(a, b, mid, &sample))
                return 0;
            real_t fmid = value(&sample, aux) - target;
            if ((flo < REAL_C(0.0)) != (fmid < REAL_C(0.0)))
                hi = mid;
            else
            {
                lo = mid;
                flo = fmid;
            }
        }
        return tiny_bclibc__hermite_at_time(a, b, REAL_C(0.5) * (lo + hi), out);
    }

    static inline real_t tiny_bclibc__value_vy(const TINY_BCLIBC_BaseTrajData *pt, const void *aux)
    {
        (void)aux;
        return pt->vy;
    }

    static inline real_t tiny_bclibc__value_mach(const TINY_BCLIBC_BaseTrajData *pt, const void *aux)
    {
        (void)aux;
        return pt->mach;
    }

    /* Value-by-key: generic key extractor for integrate_at */
    typedef struct tiny_bclibc__AtValueAux
    {
        int32_t key;
    } tiny_bclibc__AtValueAux;

    static inline real_t tiny_bclibc__value_by_key(const TINY_BCLIBC_BaseTrajData *pt, const void *aux_)
    {
        const tiny_bclibc__AtValueAux *aux = (const tiny_bclibc__AtValueAux *)aux_;
        return TINY_BCLIBC_BaseTrajData_get(pt, aux->key);
    }

    typedef struct tiny_bclibc__SlantAux
    {
        real_t la_cos, la_sin;
    } tiny_bclibc__SlantAux;

    static inline real_t tiny_bclibc__value_slant(const TINY_BCLIBC_BaseTrajData *pt, const void *aux_)
    {
        const tiny_bclibc__SlantAux *aux = (const tiny_bclibc__SlantAux *)aux_;
        return pt->py * aux->la_cos - pt->px * aux->la_sin;
    }

    /* ════════════════════════════════════════════════════════════════════
     *  Cash-Karp adaptive RK45 (Numerical Recipes `rkck`)
     * ════════════════════════════════════════════════════════════════════ */

    typedef struct tiny_bclibc__CkDeriv
    {
        TINY_BCLIBC_V3dT dvr;
        TINY_BCLIBC_V3dT dp;
    } tiny_bclibc__CkDeriv;

    static inline tiny_bclibc__CkDeriv tiny_bclibc__ck_deriv(
        const TINY_BCLIBC_ShotProps *props,
        TINY_BCLIBC_V3dT wind,
        TINY_BCLIBC_V3dT gpc,
        TINY_BCLIBC_V3dT vr,
        TINY_BCLIBC_V3dT pos)
    {
        real_t density_ratio, mach_fps;
        TINY_BCLIBC_Atmosphere_update_density_mach(&props->atmo,
                                                   props->alt0 + pos.y, &density_ratio, &mach_fps);
        real_t inv_mach = (mach_fps != REAL_C(0.0)) ? (REAL_C(1.0) / mach_fps) : REAL_C(1.0);
        real_t speed = TINY_BCLIBC_V3dT_mag(vr);
        real_t mach = speed * inv_mach;
        real_t km = density_ratio * TINY_BCLIBC_ShotProps_drag_by_mach(props, mach);

        tiny_bclibc__CkDeriv d;
        tiny_bclibc__calc_dvdt(vr, gpc, km, speed, &d.dvr);
        d.dp = TINY_BCLIBC_V3dT_add(vr, wind);
        return d;
    }

    /* ── Cash-Karp adaptive RK45 loop ── */
    TINY_BCLIBC_INTERNAL int32_t tiny_bclibc__run_cashkarp(
        const TINY_BCLIBC_ShotProps *props,
        const TINY_BCLIBC_TrajectoryRequest *req,
        tiny_bclibc__OnStep on_first,
        tiny_bclibc__OnInterval on_interval,
        void *ctx,
        int32_t *out_reason)
    {
        const real_t base_dt = props->calc_step;
        if (base_dt <= REAL_C(0.0))
        {
            tiny_bclibc__set_error("calc_step must be > 0");
            *out_reason = TINY_BCLIBC_TERM_NO_TERMINATE;
            return TINY_BCLIBC_ERR_INVALID_ARG;
        }
        const real_t min_dt = base_dt / REAL_C(64.0);
        const real_t max_dt = base_dt * REAL_C(64.0);
        real_t dt = base_dt;

        real_t range_limit = req ? req->range_limit_ft : TINY_BCLIBC_MAX_INTEGRATION_RANGE;

        tiny_bclibc__StopCtrl sc;
        tiny_bclibc__stop_ctrl_init(&sc, props, range_limit,
                                    props->cfg.cMinimumVelocity,
                                    props->cfg.cMaximumDrop, props->cfg.cMinimumAltitude, out_reason);

        TINY_BCLIBC_WindSock ws = props->wind_sock;
        TINY_BCLIBC_V3dT gravity = TINY_BCLIBC_V3dT_make(REAL_C(0.0), props->cfg.cGravityConstant, REAL_C(0.0));
        TINY_BCLIBC_V3dT wind = ws.last_vector;

        real_t muzzle = props->muzzle_velocity;
        real_t cos_el = TINY_BCLIBC_COS(props->barrel_elevation);
        TINY_BCLIBC_V3dT dir = TINY_BCLIBC_V3dT_make(
            cos_el * TINY_BCLIBC_COS(props->barrel_azimuth),
            TINY_BCLIBC_SIN(props->barrel_elevation),
            cos_el * TINY_BCLIBC_SIN(props->barrel_azimuth));

        TINY_BCLIBC_V3dT pos = TINY_BCLIBC_V3dT_make(
            REAL_C(0.0),
            -props->cant_cosine * props->sight_height,
            -props->cant_sine * props->sight_height);
        TINY_BCLIBC_V3dT vel = TINY_BCLIBC_V3dT_scale(dir, muzzle);
        TINY_BCLIBC_V3dT vr = TINY_BCLIBC_V3dT_sub(vel, wind);

        real_t time = REAL_C(0.0);
        *out_reason = TINY_BCLIBC_TERM_NO_TERMINATE;

        TINY_BCLIBC_BaseTrajData step_start;
        {
            real_t density_ratio, mach_fps;
            TINY_BCLIBC_Atmosphere_update_density_mach(&props->atmo,
                                                       props->alt0 + pos.y, &density_ratio, &mach_fps);
            real_t inv_mach = (mach_fps != REAL_C(0.0)) ? (REAL_C(1.0) / mach_fps) : REAL_C(1.0);
            step_start.time = time;
            step_start.px = pos.x;
            step_start.py = pos.y;
            step_start.pz = pos.z;
            step_start.vx = vel.x;
            step_start.vy = vel.y;
            step_start.vz = vel.z;
            step_start.mach = TINY_BCLIBC_V3dT_mag(vr) * inv_mach;
            if (on_first && on_first(&step_start, ctx) != 0)
            {
                *out_reason = TINY_BCLIBC_TERM_HANDLER_STOP;
                return TINY_BCLIBC_OK;
            }
        }

        while (*out_reason == TINY_BCLIBC_TERM_NO_TERMINATE)
        {
            if (pos.x >= ws.next_range)
                wind = TINY_BCLIBC_WindSock_vector_for_range(&ws, pos.x);

            TINY_BCLIBC_V3dT gpc = gravity;
            if (!props->coriolis.flat_fire_only)
            {
                TINY_BCLIBC_V3dT ca;
                TINY_BCLIBC_Coriolis_acceleration_local(&props->coriolis, vel, &ca);
                gpc.x += ca.x;
                gpc.y += ca.y;
                gpc.z += ca.z;
            }

            TINY_BCLIBC_V3dT vr_next = vr, pos_next = pos;
            real_t dt_used = dt;
            int32_t attempt;
            for (attempt = 0; attempt < 24; attempt++)
            {
                tiny_bclibc__CkDeriv k1 = tiny_bclibc__ck_deriv(props, wind, gpc, vr, pos);

                TINY_BCLIBC_V3dT vr2 = vr, p2 = pos;
                TINY_BCLIBC_V3dT_fma(&vr2, k1.dvr, dt * REAL_C(0.2));
                TINY_BCLIBC_V3dT_fma(&p2, k1.dp, dt * REAL_C(0.2));
                tiny_bclibc__CkDeriv k2 = tiny_bclibc__ck_deriv(props, wind, gpc, vr2, p2);

                TINY_BCLIBC_V3dT vr3 = vr, p3 = pos;
                TINY_BCLIBC_V3dT_fma(&vr3, k1.dvr, dt * (REAL_C(3.0) / REAL_C(40.0)));
                TINY_BCLIBC_V3dT_fma(&vr3, k2.dvr, dt * (REAL_C(9.0) / REAL_C(40.0)));
                TINY_BCLIBC_V3dT_fma(&p3, k1.dp, dt * (REAL_C(3.0) / REAL_C(40.0)));
                TINY_BCLIBC_V3dT_fma(&p3, k2.dp, dt * (REAL_C(9.0) / REAL_C(40.0)));
                tiny_bclibc__CkDeriv k3 = tiny_bclibc__ck_deriv(props, wind, gpc, vr3, p3);

                TINY_BCLIBC_V3dT vr4 = vr, p4 = pos;
                TINY_BCLIBC_V3dT_fma(&vr4, k1.dvr, dt * (REAL_C(3.0) / REAL_C(10.0)));
                TINY_BCLIBC_V3dT_fma(&vr4, k2.dvr, dt * (REAL_C(-9.0) / REAL_C(10.0)));
                TINY_BCLIBC_V3dT_fma(&vr4, k3.dvr, dt * (REAL_C(6.0) / REAL_C(5.0)));
                TINY_BCLIBC_V3dT_fma(&p4, k1.dp, dt * (REAL_C(3.0) / REAL_C(10.0)));
                TINY_BCLIBC_V3dT_fma(&p4, k2.dp, dt * (REAL_C(-9.0) / REAL_C(10.0)));
                TINY_BCLIBC_V3dT_fma(&p4, k3.dp, dt * (REAL_C(6.0) / REAL_C(5.0)));
                tiny_bclibc__CkDeriv k4 = tiny_bclibc__ck_deriv(props, wind, gpc, vr4, p4);

                TINY_BCLIBC_V3dT vr5 = vr, p5 = pos;
                TINY_BCLIBC_V3dT_fma(&vr5, k1.dvr, dt * (REAL_C(-11.0) / REAL_C(54.0)));
                TINY_BCLIBC_V3dT_fma(&vr5, k2.dvr, dt * REAL_C(2.5));
                TINY_BCLIBC_V3dT_fma(&vr5, k3.dvr, dt * (REAL_C(-70.0) / REAL_C(27.0)));
                TINY_BCLIBC_V3dT_fma(&vr5, k4.dvr, dt * (REAL_C(35.0) / REAL_C(27.0)));
                TINY_BCLIBC_V3dT_fma(&p5, k1.dp, dt * (REAL_C(-11.0) / REAL_C(54.0)));
                TINY_BCLIBC_V3dT_fma(&p5, k2.dp, dt * REAL_C(2.5));
                TINY_BCLIBC_V3dT_fma(&p5, k3.dp, dt * (REAL_C(-70.0) / REAL_C(27.0)));
                TINY_BCLIBC_V3dT_fma(&p5, k4.dp, dt * (REAL_C(35.0) / REAL_C(27.0)));
                tiny_bclibc__CkDeriv k5 = tiny_bclibc__ck_deriv(props, wind, gpc, vr5, p5);

                TINY_BCLIBC_V3dT vr6 = vr, p6 = pos;
                TINY_BCLIBC_V3dT_fma(&vr6, k1.dvr, dt * (REAL_C(1631.0) / REAL_C(55296.0)));
                TINY_BCLIBC_V3dT_fma(&vr6, k2.dvr, dt * (REAL_C(175.0) / REAL_C(512.0)));
                TINY_BCLIBC_V3dT_fma(&vr6, k3.dvr, dt * (REAL_C(575.0) / REAL_C(13824.0)));
                TINY_BCLIBC_V3dT_fma(&vr6, k4.dvr, dt * (REAL_C(44275.0) / REAL_C(110592.0)));
                TINY_BCLIBC_V3dT_fma(&vr6, k5.dvr, dt * (REAL_C(253.0) / REAL_C(4096.0)));
                TINY_BCLIBC_V3dT_fma(&p6, k1.dp, dt * (REAL_C(1631.0) / REAL_C(55296.0)));
                TINY_BCLIBC_V3dT_fma(&p6, k2.dp, dt * (REAL_C(175.0) / REAL_C(512.0)));
                TINY_BCLIBC_V3dT_fma(&p6, k3.dp, dt * (REAL_C(575.0) / REAL_C(13824.0)));
                TINY_BCLIBC_V3dT_fma(&p6, k4.dp, dt * (REAL_C(44275.0) / REAL_C(110592.0)));
                TINY_BCLIBC_V3dT_fma(&p6, k5.dp, dt * (REAL_C(253.0) / REAL_C(4096.0)));
                tiny_bclibc__CkDeriv k6 = tiny_bclibc__ck_deriv(props, wind, gpc, vr6, p6);

                vr_next = vr;
                pos_next = pos;
                TINY_BCLIBC_V3dT_fma(&vr_next, k1.dvr, dt * (REAL_C(37.0) / REAL_C(378.0)));
                TINY_BCLIBC_V3dT_fma(&vr_next, k3.dvr, dt * (REAL_C(250.0) / REAL_C(621.0)));
                TINY_BCLIBC_V3dT_fma(&vr_next, k4.dvr, dt * (REAL_C(125.0) / REAL_C(594.0)));
                TINY_BCLIBC_V3dT_fma(&vr_next, k6.dvr, dt * (REAL_C(512.0) / REAL_C(1771.0)));
                TINY_BCLIBC_V3dT_fma(&pos_next, k1.dp, dt * (REAL_C(37.0) / REAL_C(378.0)));
                TINY_BCLIBC_V3dT_fma(&pos_next, k3.dp, dt * (REAL_C(250.0) / REAL_C(621.0)));
                TINY_BCLIBC_V3dT_fma(&pos_next, k4.dp, dt * (REAL_C(125.0) / REAL_C(594.0)));
                TINY_BCLIBC_V3dT_fma(&pos_next, k6.dp, dt * (REAL_C(512.0) / REAL_C(1771.0)));

                TINY_BCLIBC_V3dT err_v = TINY_BCLIBC_V3dT_make(REAL_C(0.0), REAL_C(0.0), REAL_C(0.0));
                TINY_BCLIBC_V3dT err_p = TINY_BCLIBC_V3dT_make(REAL_C(0.0), REAL_C(0.0), REAL_C(0.0));
                const real_t D1 = REAL_C(37.0) / REAL_C(378.0) - REAL_C(2825.0) / REAL_C(27648.0);
                const real_t D3 = REAL_C(250.0) / REAL_C(621.0) - REAL_C(18575.0) / REAL_C(48384.0);
                const real_t D4 = REAL_C(125.0) / REAL_C(594.0) - REAL_C(13525.0) / REAL_C(55296.0);
                const real_t D5 = REAL_C(0.0) - REAL_C(277.0) / REAL_C(14336.0);
                const real_t D6 = REAL_C(512.0) / REAL_C(1771.0) - REAL_C(0.25);
                TINY_BCLIBC_V3dT_fma(&err_v, k1.dvr, D1);
                TINY_BCLIBC_V3dT_fma(&err_v, k3.dvr, D3);
                TINY_BCLIBC_V3dT_fma(&err_v, k4.dvr, D4);
                TINY_BCLIBC_V3dT_fma(&err_v, k5.dvr, D5);
                TINY_BCLIBC_V3dT_fma(&err_v, k6.dvr, D6);
                TINY_BCLIBC_V3dT_scale_assign(&err_v, dt);
                TINY_BCLIBC_V3dT_fma(&err_p, k1.dp, D1);
                TINY_BCLIBC_V3dT_fma(&err_p, k3.dp, D3);
                TINY_BCLIBC_V3dT_fma(&err_p, k4.dp, D4);
                TINY_BCLIBC_V3dT_fma(&err_p, k5.dp, D5);
                TINY_BCLIBC_V3dT_fma(&err_p, k6.dp, D6);
                TINY_BCLIBC_V3dT_scale_assign(&err_p, dt);

                const real_t atol = REAL_C(1e-6);
                const real_t rtol = REAL_C(1e-6);
                const real_t svx = atol + rtol * ((TINY_BCLIBC_FABS(vr.x) > TINY_BCLIBC_FABS(vr_next.x)) ? TINY_BCLIBC_FABS(vr.x) : TINY_BCLIBC_FABS(vr_next.x));
                const real_t svy = atol + rtol * ((TINY_BCLIBC_FABS(vr.y) > TINY_BCLIBC_FABS(vr_next.y)) ? TINY_BCLIBC_FABS(vr.y) : TINY_BCLIBC_FABS(vr_next.y));
                const real_t svz = atol + rtol * ((TINY_BCLIBC_FABS(vr.z) > TINY_BCLIBC_FABS(vr_next.z)) ? TINY_BCLIBC_FABS(vr.z) : TINY_BCLIBC_FABS(vr_next.z));
                const real_t spx = atol + rtol * ((TINY_BCLIBC_FABS(pos.x) > TINY_BCLIBC_FABS(pos_next.x)) ? TINY_BCLIBC_FABS(pos.x) : TINY_BCLIBC_FABS(pos_next.x));
                const real_t spy = atol + rtol * ((TINY_BCLIBC_FABS(pos.y) > TINY_BCLIBC_FABS(pos_next.y)) ? TINY_BCLIBC_FABS(pos.y) : TINY_BCLIBC_FABS(pos_next.y));
                const real_t spz = atol + rtol * ((TINY_BCLIBC_FABS(pos.z) > TINY_BCLIBC_FABS(pos_next.z)) ? TINY_BCLIBC_FABS(pos.z) : TINY_BCLIBC_FABS(pos_next.z));
                const real_t evx = err_v.x / svx, evy = err_v.y / svy, evz = err_v.z / svz;
                const real_t epx = err_p.x / spx, epy = err_p.y / spy, epz = err_p.z / spz;
                real_t err_norm = TINY_BCLIBC_SQRT(
                    (evx * evx + evy * evy + evz * evz + epx * epx + epy * epy + epz * epz) /
                    REAL_C(6.0));

                if (err_norm <= REAL_C(1.0) || dt <= min_dt * REAL_C(1.0001))
                {
                    dt_used = dt;
                    real_t grow = (err_norm > REAL_C(1.89e-4))
                                      ? REAL_C(0.9) * TINY_BCLIBC_POW(err_norm, REAL_C(-0.20))
                                      : REAL_C(5.0);
                    grow = (grow < REAL_C(1.0)) ? REAL_C(1.0) : ((grow > REAL_C(5.0)) ? REAL_C(5.0) : grow);
                    dt = dt * grow;
                    if (dt > max_dt)
                        dt = max_dt;
                    break;
                }
                real_t shrink = REAL_C(0.9) * TINY_BCLIBC_POW(err_norm, REAL_C(-0.25));
                shrink = (shrink < REAL_C(0.1)) ? REAL_C(0.1) : ((shrink > REAL_C(0.9)) ? REAL_C(0.9) : shrink);
                dt = dt * shrink;
                if (dt < min_dt)
                    dt = min_dt;
            }

            time += dt_used;
            vr = vr_next;
            pos = pos_next;
            vel = TINY_BCLIBC_V3dT_add(vr, wind);

            TINY_BCLIBC_BaseTrajData step_end;
            {
                real_t density_ratio, mach_fps;
                TINY_BCLIBC_Atmosphere_update_density_mach(&props->atmo,
                                                           props->alt0 + pos.y, &density_ratio, &mach_fps);
                real_t inv_mach = (mach_fps != REAL_C(0.0)) ? (REAL_C(1.0) / mach_fps) : REAL_C(1.0);
                step_end.time = time;
                step_end.px = pos.x;
                step_end.py = pos.y;
                step_end.pz = pos.z;
                step_end.vx = vel.x;
                step_end.vy = vel.y;
                step_end.vz = vel.z;
                step_end.mach = TINY_BCLIBC_V3dT_mag(vr) * inv_mach;
            }

            if (on_interval(&step_start, &step_end, ctx) != 0)
            {
                *out_reason = TINY_BCLIBC_TERM_HANDLER_STOP;
                break;
            }
            step_start = step_end;
            tiny_bclibc__stop_ctrl_check(&sc, &step_end);
        }
        return TINY_BCLIBC_OK;
    }

    /* ════════════════════════════════════════════════════════════════════
     *  tiny_bclibc_build_shot_props
     * ════════════════════════════════════════════════════════════════════ */

    TINY_BCLIBC_FUNC int32_t tiny_bclibc_build_shot_props(
        const TINY_BCLIBC_Shot *shot,
        TINY_BCLIBC_CurvePoint *curve_buf,
        TINY_BCLIBC_ShotProps *out)
    {
        if (!shot || !curve_buf || !out || shot->drag_table_size < 2)
        {
            tiny_bclibc__set_error("tiny_bclibc_build_shot_props: invalid argument");
            return TINY_BCLIBC_ERR_INVALID_ARG;
        }

        tiny_bclibc__build_pchip(shot->mach_data, shot->cd_data, shot->drag_table_size, curve_buf);

        out->bc = shot->bc;
        out->muzzle_velocity = shot->muzzle_velocity_fps;
        out->weight = shot->weight_grain;
        out->diameter = shot->diameter_inch;
        out->length = shot->length_inch;
        out->sight_height = shot->sight_height_ft;
        out->twist = shot->twist_inch;
        out->barrel_elevation = shot->barrel_elevation_rad;
        out->barrel_azimuth = shot->barrel_azimuth_rad;
        out->look_angle = shot->look_angle_rad;
        out->cant_cosine = TINY_BCLIBC_COS(shot->cant_angle_rad);
        out->cant_sine = TINY_BCLIBC_SIN(shot->cant_angle_rad);
        out->alt0 = shot->altitude_ft;
        out->calc_step = REAL_C(0.0025) * shot->config.cStepMultiplier;
        out->cfg = shot->config;
        out->curve = curve_buf;
        out->mach_list = shot->mach_data;
        out->curve_count = shot->drag_table_size;

        out->atmo = TINY_BCLIBC_Atmosphere_from_conditions(shot->temp_c, shot->pressure_hpa,
                                                           shot->altitude_ft, shot->humidity);
        out->coriolis = TINY_BCLIBC_Coriolis_from_lat_az(shot->latitude_deg,
                                                         shot->muzzle_velocity_fps,
                                                         shot->azimuth_deg);
        out->wind_sock = TINY_BCLIBC_WindSock_make(shot->winds, shot->wind_count);

        out->stability_coefficient = TINY_BCLIBC_ShotProps_calc_stability(out);
        return TINY_BCLIBC_OK;
    }

    /* ════════════════════════════════════════════════════════════════════
     *  Interval-based filter (Cash-Karp consumer) — UNCHANGED
     * ════════════════════════════════════════════════════════════════════ */

    typedef struct tiny_bclibc__IntegrateCtx
    {
        const TINY_BCLIBC_ShotProps *props;
        const TINY_BCLIBC_TrajectoryRequest *req;
        TINY_BCLIBC_TrajectoryData *buf;
        int32_t capacity;
        int32_t written;
        int32_t total;
        int32_t range_step_index;
        real_t time_of_last;
        int32_t active_flags;
        tiny_bclibc_StreamCb stream_cb;
        void *stream_ctx;
        int32_t stream_stop;
        TINY_BCLIBC_BaseTrajData last_raw;
    } tiny_bclibc__IntegrateCtx;

    static inline void tiny_bclibc__integrate_emit(tiny_bclibc__IntegrateCtx *c,
                                                   const TINY_BCLIBC_BaseTrajData *pt,
                                                   int32_t flag)
    {
        c->total++;
        if (c->stream_cb)
        {
            TINY_BCLIBC_TrajectoryData full;
            TINY_BCLIBC_TrajectoryData_from_props(c->props, pt, flag, &full);
            if (c->stream_cb(&full, c->stream_ctx) != 0)
                c->stream_stop = TINY_BCLIBC_TERM_HANDLER_STOP;
        }
        else if (c->buf && c->written < c->capacity)
        {
            TINY_BCLIBC_TrajectoryData_from_props(c->props, pt, flag, &c->buf[c->written]);
            c->written++;
        }
    }

    static inline int32_t tiny_bclibc__integrate_on_first(const TINY_BCLIBC_BaseTrajData *pt, void *ctx_)
    {
        tiny_bclibc__IntegrateCtx *c = (tiny_bclibc__IntegrateCtx *)ctx_;
        const TINY_BCLIBC_TrajectoryRequest *req = c->req;
        c->last_raw = *pt;

        int32_t ff = req->filter_flags;
        if ((ff & TINY_BCLIBC_TRAJ_FLAG_ZERO_UP) && pt->py >= REAL_C(0.0))
            ff &= ~TINY_BCLIBC_TRAJ_FLAG_ZERO_UP;
        if ((ff & TINY_BCLIBC_TRAJ_FLAG_ZERO) && pt->py < REAL_C(0.0))
        {
            real_t look_el = TINY_BCLIBC_ATAN2(pt->vy, pt->vx);
            if (look_el <= c->props->look_angle)
                ff &= ~(TINY_BCLIBC_TRAJ_FLAG_ZERO | TINY_BCLIBC_TRAJ_FLAG_MRT);
        }
        c->active_flags = ff;
        if (req->range_step_ft > REAL_C(0.0) || req->time_step > REAL_C(0.0))
            tiny_bclibc__integrate_emit(c, pt, TINY_BCLIBC_TRAJ_FLAG_RANGE);
        return 0;
    }

    static inline int32_t tiny_bclibc__integrate_on_interval(
        const TINY_BCLIBC_BaseTrajData *start, const TINY_BCLIBC_BaseTrajData *end, void *ctx_)
    {
        tiny_bclibc__IntegrateCtx *c = (tiny_bclibc__IntegrateCtx *)ctx_;
        const TINY_BCLIBC_TrajectoryRequest *req = c->req;
        c->last_raw = *end;

        if (req->range_step_ft > REAL_C(0.0))
        {
            while ((real_t)(c->range_step_index + 1) * req->range_step_ft <= end->px + REAL_C(1e-9))
            {
                real_t rd = (real_t)(c->range_step_index + 1) * req->range_step_ft;
                if (rd > req->range_limit_ft + REAL_C(1e-9))
                    break;
                TINY_BCLIBC_BaseTrajData r;
                if (TINY_BCLIBC_FABS(rd - end->px) < REAL_C(1e-9))
                    r = *end;
                else if (!tiny_bclibc__hermite_at_x(start, end, rd, &r))
                    break;
                c->range_step_index++;
                tiny_bclibc__integrate_emit(c, &r, TINY_BCLIBC_TRAJ_FLAG_RANGE);
                c->time_of_last = r.time;
            }
        }

        if (req->time_step > REAL_C(0.0))
        {
            while (c->time_of_last + req->time_step <= end->time + REAL_C(1e-9))
            {
                real_t rt = c->time_of_last + req->time_step;
                TINY_BCLIBC_BaseTrajData r;
                if (!tiny_bclibc__hermite_at_time(start, end, rt, &r))
                    break;
                c->time_of_last = rt;
                tiny_bclibc__integrate_emit(c, &r, TINY_BCLIBC_TRAJ_FLAG_RANGE);
            }
        }

        if ((c->active_flags & TINY_BCLIBC_TRAJ_FLAG_APEX) &&
            start->vy > REAL_C(0.0) && end->vy <= REAL_C(0.0))
        {
            TINY_BCLIBC_BaseTrajData r;
            if (tiny_bclibc__hermite_at_value(start, end, tiny_bclibc__value_vy, NULL, REAL_C(0.0), &r))
                tiny_bclibc__integrate_emit(c, &r, TINY_BCLIBC_TRAJ_FLAG_APEX);
            c->active_flags &= ~TINY_BCLIBC_TRAJ_FLAG_APEX;
        }

        if ((c->active_flags & TINY_BCLIBC_TRAJ_FLAG_MACH) &&
            start->mach > REAL_C(1.0) && end->mach < REAL_C(1.0))
        {
            TINY_BCLIBC_BaseTrajData r;
            if (tiny_bclibc__hermite_at_value(start, end, tiny_bclibc__value_mach, NULL, REAL_C(1.0), &r))
                tiny_bclibc__integrate_emit(c, &r, TINY_BCLIBC_TRAJ_FLAG_MACH);
            c->active_flags &= ~TINY_BCLIBC_TRAJ_FLAG_MACH;
        }

        if (c->active_flags & TINY_BCLIBC_TRAJ_FLAG_ZERO)
        {
            tiny_bclibc__SlantAux aux;
            aux.la_cos = TINY_BCLIBC_COS(c->props->look_angle);
            aux.la_sin = TINY_BCLIBC_SIN(c->props->look_angle);
            real_t start_slant = start->py * aux.la_cos - start->px * aux.la_sin;
            real_t end_slant = end->py * aux.la_cos - end->px * aux.la_sin;
            int32_t cross_flag = 0;
            if (c->active_flags & TINY_BCLIBC_TRAJ_FLAG_ZERO_UP)
            {
                if (start_slant < REAL_C(0.0) && end_slant > REAL_C(0.0))
                    cross_flag = TINY_BCLIBC_TRAJ_FLAG_ZERO_UP;
            }
            else if (c->active_flags & TINY_BCLIBC_TRAJ_FLAG_ZERO_DOWN)
            {
                if (start_slant > REAL_C(0.0) && end_slant < REAL_C(0.0))
                    cross_flag = TINY_BCLIBC_TRAJ_FLAG_ZERO_DOWN;
            }
            if (cross_flag)
            {
                TINY_BCLIBC_BaseTrajData r;
                if (tiny_bclibc__hermite_at_value(start, end, tiny_bclibc__value_slant, &aux, REAL_C(0.0), &r))
                {
                    tiny_bclibc__integrate_emit(c, &r, cross_flag);
                    c->active_flags &= ~cross_flag;
                }
            }
        }

        return c->stream_stop;
    }

    /* ════════════════════════════════════════════════════════════════════
     *  Public API
     * ════════════════════════════════════════════════════════════════════ */

    TINY_BCLIBC_FUNC int32_t tiny_bclibc_integrate(
        const TINY_BCLIBC_ShotProps *props,
        const TINY_BCLIBC_TrajectoryRequest *req,
        TINY_BCLIBC_TrajectoryData *out_buf,
        int32_t buf_capacity,
        int32_t *out_written,
        int32_t *out_total,
        int32_t *out_reason)
    {
        if (!props || !req || !out_written || !out_total || !out_reason)
        {
            tiny_bclibc__set_error("tiny_bclibc_integrate: NULL argument");
            return TINY_BCLIBC_ERR_INVALID_ARG;
        }

        tiny_bclibc__IntegrateCtx ctx;
        memset(&ctx, 0, sizeof(ctx));
        ctx.props = props;
        ctx.req = req;
        ctx.buf = out_buf;
        ctx.capacity = buf_capacity;

        int32_t reason = TINY_BCLIBC_TERM_NO_TERMINATE;
        int32_t rc = tiny_bclibc__run_cashkarp(props, req,
                                               tiny_bclibc__integrate_on_first,
                                               tiny_bclibc__integrate_on_interval,
                                               &ctx, &reason);

        *out_written = ctx.written;
        *out_total = ctx.total;
        *out_reason = reason;

        if (rc != TINY_BCLIBC_OK)
            return rc;
        if (out_buf && ctx.total > buf_capacity && buf_capacity > 0)
            return TINY_BCLIBC_ERR_BUF_TOO_SMALL;
        return TINY_BCLIBC_OK;
    }

    TINY_BCLIBC_FUNC int32_t tiny_bclibc_integrate_stream(
        const TINY_BCLIBC_ShotProps *props,
        const TINY_BCLIBC_TrajectoryRequest *req,
        tiny_bclibc_StreamCb cb,
        void *cb_ctx,
        int32_t *out_total,
        int32_t *out_reason,
        TINY_BCLIBC_BaseTrajData *out_final_raw)
    {
        if (!props || !req || !cb || !out_total || !out_reason)
        {
            tiny_bclibc__set_error("tiny_bclibc_integrate_stream: NULL argument");
            return TINY_BCLIBC_ERR_INVALID_ARG;
        }

        tiny_bclibc__IntegrateCtx ctx;
        memset(&ctx, 0, sizeof(ctx));
        ctx.props = props;
        ctx.req = req;
        ctx.stream_cb = cb;
        ctx.stream_ctx = cb_ctx;

        int32_t reason = TINY_BCLIBC_TERM_NO_TERMINATE;
        int32_t rc = tiny_bclibc__run_cashkarp(props, req,
                                               tiny_bclibc__integrate_on_first,
                                               tiny_bclibc__integrate_on_interval,
                                               &ctx, &reason);

        *out_total = ctx.total;
        *out_reason = reason;
        if (out_final_raw)
            *out_final_raw = ctx.last_raw;
        return rc;
    }

    /* ── integrate_raw — now Cash-Karp based ──────────────────────────────
     *  Emits the *start* of every accepted Cash-Karp interval, plus the
     *  terminal point after the loop.  The callback signature is unchanged:
     *  it still receives one TINY_BCLIBC_BaseTrajData per call.  What changed
     *  is the spacing of those calls — no longer uniform, no longer one per
     *  fixed RK4 step, but one per accepted adaptive step.  Consumers that
     *  assumed a fixed uniform dt must be updated; consumers that just
     *  re-filter or re-sample the stream are unaffected. */

    typedef struct tiny_bclibc__RawCkCtx
    {
        tiny_bclibc_RawStepCb cb;
        void *cb_ctx;
        int32_t stopped;
        TINY_BCLIBC_BaseTrajData last;
        int32_t has_last;
    } tiny_bclibc__RawCkCtx;

    static inline int32_t tiny_bclibc__raw_ck_on_first(const TINY_BCLIBC_BaseTrajData *pt, void *ctx_)
    {
        tiny_bclibc__RawCkCtx *c = (tiny_bclibc__RawCkCtx *)ctx_;
        c->last = *pt;
        c->has_last = 1;
        int32_t r = c->cb(pt, c->cb_ctx);
        if (r != 0)
        {
            c->stopped = 1;
            return 1;
        }
        return 0;
    }

    static inline int32_t tiny_bclibc__raw_ck_on_interval(
        const TINY_BCLIBC_BaseTrajData *start,
        const TINY_BCLIBC_BaseTrajData *end,
        void *ctx_)
    {
        tiny_bclibc__RawCkCtx *c = (tiny_bclibc__RawCkCtx *)ctx_;
        (void)start;
        c->last = *end;
        c->has_last = 1;
        int32_t r = c->cb(end, c->cb_ctx);
        if (r != 0)
        {
            c->stopped = 1;
            return 1;
        }
        return 0;
    }

    TINY_BCLIBC_FUNC int32_t tiny_bclibc_integrate_raw(
        const TINY_BCLIBC_ShotProps *props,
        real_t range_limit_ft,
        tiny_bclibc_RawStepCb cb,
        void *cb_ctx,
        int32_t *out_reason)
    {
        if (!props || !cb || !out_reason)
        {
            tiny_bclibc__set_error("tiny_bclibc_integrate_raw: NULL argument");
            return TINY_BCLIBC_ERR_INVALID_ARG;
        }

        TINY_BCLIBC_TrajectoryRequest req;
        req.range_limit_ft = range_limit_ft;
        req.range_step_ft = REAL_C(0.0);
        req.time_step = REAL_C(0.0);
        req.filter_flags = TINY_BCLIBC_TRAJ_FLAG_NONE;

        tiny_bclibc__RawCkCtx ctx;
        memset(&ctx, 0, sizeof(ctx));
        ctx.cb = cb;
        ctx.cb_ctx = cb_ctx;

        int32_t reason = TINY_BCLIBC_TERM_NO_TERMINATE;
        int32_t rc = tiny_bclibc__run_cashkarp(props, &req,
                                               tiny_bclibc__raw_ck_on_first,
                                               tiny_bclibc__raw_ck_on_interval,
                                               &ctx, &reason);
        *out_reason = reason;
        return rc;
    }

    /* ── sizeof helpers ── */
    TINY_BCLIBC_FUNC int32_t tiny_bclibc_sizeof_shot_props(void)
    {
        return (int32_t)sizeof(TINY_BCLIBC_ShotProps);
    }

    TINY_BCLIBC_FUNC int32_t tiny_bclibc_sizeof_curve_point(void)
    {
        return (int32_t)sizeof(TINY_BCLIBC_CurvePoint);
    }

    /* ════════════════════════════════════════════════════════════════════
     *  integrate_at — now Cash-Karp based
     * ════════════════════════════════════════════════════════════════════ */

    typedef struct tiny_bclibc__AtCtx
    {
        int32_t key;
        real_t target;
        /* 3-point sliding window for PCHIP interpolation, mirroring bclibc's
         * BCLIBC_SinglePointHandler and the pre-Cash-Karp RK4 at_on_step path.
         * A 2-point Hermite using endpoint tangents measurably shifts the
         * interpolated crossing (esp. near apex where vy is flat) relative to
         * bclibc's own 3-point scheme, by up to ~1.7e-2 ft at 3500 ft range. */
        TINY_BCLIBC_BaseTrajData win[3];
        int32_t n;
        int32_t found;
        TINY_BCLIBC_BaseTrajData result;
    } tiny_bclibc__AtCtx;

    static inline int32_t tiny_bclibc__at_ck_on_first(const TINY_BCLIBC_BaseTrajData *pt, void *ctx_)
    {
        tiny_bclibc__AtCtx *c = (tiny_bclibc__AtCtx *)ctx_;
        /* Seed the 3-point window with the initial sample. */
        c->win[0] = *pt;
        c->n = 1;

        /* Only an exact hit at the very first sample can short-circuit here. */
        real_t v = TINY_BCLIBC_BaseTrajData_get(pt, c->key);
        if (v == c->target)
        {
            c->result = *pt;
            c->found = 1;
            return 1;
        }
        return 0;
    }

    static inline int32_t tiny_bclibc__at_ck_on_interval(
        const TINY_BCLIBC_BaseTrajData *start,
        const TINY_BCLIBC_BaseTrajData *end,
        void *ctx_)
    {
        tiny_bclibc__AtCtx *c = (tiny_bclibc__AtCtx *)ctx_;
        if (c->found)
            return 1;

        /* Mirror BCLIBC_SinglePointHandler::handle_step in bclibc:
         * - For POS_X, use the exact 2-point Hermite reconstruction
         *   (tiny_bclibc__hermite_at_x), NOT 3-point PCHIP.  This is the path
         *   bclibc itself takes for POS_X queries, and it is what zero-finding
         *   depends on: error_at_distance() issues a POS_X query per Newton
         *   iteration, and any deviation from hermite_at_x shifts the root.
         * - For every other key (VEL_Y, MACH, ...), accumulate a 3-point window
         *   and use 3-point PCHIP (TINY_BCLIBC_BaseTrajData_interpolate), the
         *   same path bclibc's handle() takes. */
        if (c->key == TINY_BCLIBC_KEY_POS_X)
        {
            if (!tiny_bclibc__hermite_at_x(start, end, c->target, &c->result))
                return 0;
            c->found = 1;
            return 1;
        }

        /* Non-POS_X: 3-point sliding window + PCHIP. */
        if (c->n < 3)
        {
            c->win[c->n++] = *end;
        }
        else
        {
            c->win[0] = c->win[1];
            c->win[1] = c->win[2];
            c->win[2] = *end;
        }

        if (c->n < 3)
            return 0;

        real_t v1 = TINY_BCLIBC_BaseTrajData_get(&c->win[1], c->key);
        real_t v2 = TINY_BCLIBC_BaseTrajData_get(&c->win[2], c->key);
        int32_t crossed = ((v1 <= c->target && c->target <= v2) ||
                           (v2 <= c->target && c->target <= v1));
        if (!crossed)
            return 0;

        TINY_BCLIBC_BaseTrajData_interpolate(c->key, c->target,
                                             &c->win[0], &c->win[1], &c->win[2], &c->result);
        c->found = 1;
        return 1;
    }

    TINY_BCLIBC_FUNC int32_t tiny_bclibc_integrate_at(
        const TINY_BCLIBC_ShotProps *props,
        int32_t key,
        real_t target_value,
        TINY_BCLIBC_BaseTrajData *out_raw,
        TINY_BCLIBC_TrajectoryData *out_full)
    {
        if (!props || !out_raw)
        {
            tiny_bclibc__set_error("tiny_bclibc_integrate_at: NULL argument");
            return TINY_BCLIBC_ERR_INVALID_ARG;
        }
        tiny_bclibc__AtCtx ctx;
        memset(&ctx, 0, sizeof(ctx));
        ctx.key = key;
        ctx.target = target_value;

        TINY_BCLIBC_TrajectoryRequest req;
        req.range_limit_ft = TINY_BCLIBC_MAX_INTEGRATION_RANGE;
        req.range_step_ft = REAL_C(0.0);
        req.time_step = REAL_C(0.0);
        req.filter_flags = TINY_BCLIBC_TRAJ_FLAG_NONE;

        int32_t reason;
        tiny_bclibc__run_cashkarp(props, &req,
                                  tiny_bclibc__at_ck_on_first,
                                  tiny_bclibc__at_ck_on_interval,
                                  &ctx, &reason);

        if (!ctx.found)
        {
            tiny_bclibc__set_error("tiny_bclibc_integrate_at: intercept not found");
            return TINY_BCLIBC_ERR_INTERCEPTION;
        }
        *out_raw = ctx.result;
        if (out_full)
            TINY_BCLIBC_TrajectoryData_from_props(props, out_raw, TINY_BCLIBC_TRAJ_FLAG_NONE, out_full);
        return TINY_BCLIBC_OK;
    }

    /* ── find_apex ── */
    TINY_BCLIBC_FUNC int32_t tiny_bclibc_find_apex(const TINY_BCLIBC_ShotProps *props,
                                                   TINY_BCLIBC_TrajectoryData *out)
    {
        if (!props || !out)
        {
            tiny_bclibc__set_error("tiny_bclibc_find_apex: NULL argument");
            return TINY_BCLIBC_ERR_INVALID_ARG;
        }
        TINY_BCLIBC_BaseTrajData raw;
        int32_t rc = tiny_bclibc_integrate_at(props, TINY_BCLIBC_KEY_VEL_Y, REAL_C(0.0), &raw, out);
        if (rc != TINY_BCLIBC_OK)
        {
            tiny_bclibc__set_error("tiny_bclibc_find_apex: apex not found");
            return TINY_BCLIBC_ERR_RUNTIME;
        }
        out->flag = TINY_BCLIBC_TRAJ_FLAG_APEX;
        return TINY_BCLIBC_OK;
    }

    /* ── error_at_distance ── */
    static inline real_t tiny_bclibc__error_at_distance(
        TINY_BCLIBC_ShotProps *props_mut,
        real_t angle_rad,
        real_t target_x_ft,
        real_t target_y_ft)
    {
        props_mut->barrel_elevation = angle_rad;
        TINY_BCLIBC_BaseTrajData raw;
        TINY_BCLIBC_TrajectoryData full;
        int32_t rc = tiny_bclibc_integrate_at(props_mut, TINY_BCLIBC_KEY_POS_X, target_x_ft, &raw, &full);
        if (rc != TINY_BCLIBC_OK)
            return REAL_C(1e9);
        if (raw.time == REAL_C(0.0))
            return REAL_C(1e9);
        return (raw.py - target_y_ft) - TINY_BCLIBC_FABS(raw.px - target_x_ft);
    }

    /* ── range_for_angle — now Cash-Karp based ──────────────────────────── */

    typedef struct tiny_bclibc__ZeroCrossCtx
    {
        real_t la_cos, la_sin;
        int32_t found;
        real_t slant_dist;
    } tiny_bclibc__ZeroCrossCtx;

    static inline int32_t tiny_bclibc__zero_cross_ck_on_first(
        const TINY_BCLIBC_BaseTrajData *pt, void *ctx_)
    {
        (void)pt;
        (void)ctx_;
        return 0;
    }

    static inline int32_t tiny_bclibc__zero_cross_ck_on_interval(
        const TINY_BCLIBC_BaseTrajData *start,
        const TINY_BCLIBC_BaseTrajData *end,
        void *ctx_)
    {
        tiny_bclibc__ZeroCrossCtx *c = (tiny_bclibc__ZeroCrossCtx *)ctx_;
        if (c->found)
            return 1;

        tiny_bclibc__SlantAux aux;
        aux.la_cos = c->la_cos;
        aux.la_sin = c->la_sin;

        real_t h_prev = start->py * c->la_cos - start->px * c->la_sin;
        real_t h_curr = end->py * c->la_cos - end->px * c->la_sin;

        if (h_prev > REAL_C(0.0) && h_curr <= REAL_C(0.0))
        {
            TINY_BCLIBC_BaseTrajData r;
            if (tiny_bclibc__hermite_at_value(start, end,
                                              tiny_bclibc__value_slant, &aux,
                                              REAL_C(0.0), &r))
            {
                c->slant_dist = r.px * c->la_cos + r.py * c->la_sin;
                c->found = 1;
                return 1;
            }
        }
        return 0;
    }

    static inline real_t tiny_bclibc__range_for_angle(TINY_BCLIBC_ShotProps *props_mut, real_t angle_rad)
    {
        props_mut->barrel_elevation = angle_rad;
        tiny_bclibc__ZeroCrossCtx ctx;
        memset(&ctx, 0, sizeof(ctx));
        ctx.la_cos = TINY_BCLIBC_COS(props_mut->look_angle);
        ctx.la_sin = TINY_BCLIBC_SIN(props_mut->look_angle);

        TINY_BCLIBC_TrajectoryRequest req;
        req.range_limit_ft = TINY_BCLIBC_MAX_INTEGRATION_RANGE;
        req.range_step_ft = REAL_C(0.0);
        req.time_step = REAL_C(0.0);
        req.filter_flags = TINY_BCLIBC_TRAJ_FLAG_NONE;

        int32_t reason;
        tiny_bclibc__run_cashkarp(props_mut, &req,
                                  tiny_bclibc__zero_cross_ck_on_first,
                                  tiny_bclibc__zero_cross_ck_on_interval,
                                  &ctx, &reason);
        return ctx.found ? ctx.slant_dist : REAL_C(0.0);
    }

    /* ── find_zero_angle_ridders — UNCHANGED ── */
    TINY_BCLIBC_INTERNAL int32_t tiny_bclibc__find_zero_angle_ridders(
        const TINY_BCLIBC_ShotProps *props,
        real_t distance_ft,
        real_t *out_angle_rad)
    {
        if (!props || !out_angle_rad)
            return TINY_BCLIBC_ERR_INVALID_ARG;

        TINY_BCLIBC_ShotProps p = *props;
        /* Match BCLIBC_BaseEngine::find_zero_angle_ridder in engine.cpp: zero
         * out minimum velocity. ... */
        p.cfg.cMinimumVelocity = REAL_C(0.0);

        real_t la = p.look_angle;
        real_t ca = TINY_BCLIBC_COS(la), sa = TINY_BCLIBC_SIN(la);
        real_t tx = distance_ft * ca;
        real_t ty = distance_ft * sa;
        real_t sh = -p.cant_cosine * p.sight_height;
        /* Mirror BCLIBC_BaseEngine::init_zero_calculation:
         * - ZERO_ERR_FT is ALLOWED_ZERO_ERROR_FEET (1e-2 ft), not 0.5 ft.
         * - "very close shot" cutoff is 2 * max(|sh|, cStepMultiplier). */
        const real_t ZERO_ERR_FT = REAL_C(1e-2);

        if (TINY_BCLIBC_FABS(distance_ft) < ZERO_ERR_FT)
        {
            *out_angle_rad = la;
            return TINY_BCLIBC_OK;
        }
        if (TINY_BCLIBC_FABS(distance_ft) < REAL_C(2.0) * tiny_bclibc__fmax(TINY_BCLIBC_FABS(sh), p.cfg.cStepMultiplier))
        {
            *out_angle_rad = TINY_BCLIBC_ATAN2(ty + sh, tx);
            return TINY_BCLIBC_OK;
        }

        real_t inv_phi = REAL_C(0.6180339887498949);
        real_t inv_phi_sq = REAL_C(0.38196601125010515);
        real_t a = REAL_C(0.0), b = REAL_C(1.5707963267948966);
        real_t h = b - a;
        real_t c2 = a + inv_phi_sq * h;
        real_t d = a + inv_phi * h;
        real_t yc, yd;
#ifdef TINY_BCLIBC_FAST_ZERO_FIND
        const real_t GSS_H_TOL = REAL_C(1e-2);
#else
    const real_t GSS_H_TOL = REAL_C(1e-5);
#endif

#ifdef TINY_BCLIBC_FAST_ZERO_FIND
        const real_t gss_step_save = p.calc_step;
        p.calc_step *= REAL_C(8.0);
#endif

        yc = tiny_bclibc__range_for_angle(&p, c2);
        yd = tiny_bclibc__range_for_angle(&p, d);
        for (int32_t i = 0; i < 100; i++)
        {
            if (h < GSS_H_TOL)
                break;
            if (yc > yd)
            {
                b = d;
                d = c2;
                yd = yc;
                h = b - a;
                c2 = a + inv_phi_sq * h;
                yc = tiny_bclibc__range_for_angle(&p, c2);
            }
            else
            {
                a = c2;
                c2 = d;
                yc = yd;
                h = b - a;
                d = a + inv_phi * h;
                yd = tiny_bclibc__range_for_angle(&p, d);
            }
        }
        real_t angle_at_max = (a + b) / REAL_C(2.0);
        real_t max_range = tiny_bclibc__range_for_angle(&p, angle_at_max);

#ifdef TINY_BCLIBC_FAST_ZERO_FIND
        p.calc_step = gss_step_save;
#endif

        if (distance_ft > max_range)
        {
            tiny_bclibc__set_error("tiny_bclibc_find_zero_angle: out of range");
            return TINY_BCLIBC_ERR_OUT_OF_RANGE;
        }
        if (TINY_BCLIBC_FABS(distance_ft - max_range) < ZERO_ERR_FT)
        {
            p.barrel_elevation = angle_at_max;
            *out_angle_rad = angle_at_max;
            return TINY_BCLIBC_OK;
        }

        real_t low_angle = la;
        real_t high_angle = angle_at_max;

        real_t f_low = tiny_bclibc__error_at_distance(&p, low_angle, tx, ty);
        if (f_low > REAL_C(1e8) && TINY_BCLIBC_FABS(low_angle - la) < REAL_C(1e-9))
        {
            low_angle += REAL_C(1e-3);
            f_low = tiny_bclibc__error_at_distance(&p, low_angle, tx, ty);
        }
        real_t f_high = tiny_bclibc__error_at_distance(&p, high_angle, tx, ty);

        if (f_low * f_high >= REAL_C(0.0))
        {
            tiny_bclibc__set_error("tiny_bclibc_find_zero_angle: no bracket");
            return TINY_BCLIBC_ERR_ZERO_FINDING;
        }

#ifdef TINY_BCLIBC_FAST_ZERO_FIND
        const real_t acc = REAL_C(0.01);
#else
    const real_t acc = REAL_C(0.001);
#endif
        const real_t angle_tol = REAL_C(1e-5);
        real_t mid_angle, f_mid, s, next_angle, f_next;
        int32_t converged = 0;
        int32_t max_iter = 50;

        for (int32_t i = 0; i < max_iter; i++)
        {
            mid_angle = (low_angle + high_angle) / REAL_C(2.0);
            f_mid = tiny_bclibc__error_at_distance(&p, mid_angle, tx, ty);
            if (TINY_BCLIBC_FABS(f_mid) < acc)
            {
                converged = 1;
                p.barrel_elevation = mid_angle;
                *out_angle_rad = mid_angle;
                return TINY_BCLIBC_OK;
            }

            real_t inner = f_mid * f_mid - f_low * f_high;
            if (inner <= REAL_C(0.0))
                break;
            s = TINY_BCLIBC_SQRT(inner);
            if (s == REAL_C(0.0))
                break;

            real_t sign = (f_low > f_high) ? REAL_C(1.0) : REAL_C(-1.0);
            next_angle = mid_angle + (mid_angle - low_angle) * (sign * f_mid / s);
            f_next = tiny_bclibc__error_at_distance(&p, next_angle, tx, ty);
            if (TINY_BCLIBC_FABS(f_next) < acc)
            {
                converged = 1;
                p.barrel_elevation = next_angle;
                *out_angle_rad = next_angle;
                return TINY_BCLIBC_OK;
            }

            if (TINY_BCLIBC_FABS(next_angle - mid_angle) < angle_tol)
            {
                converged = 1;
                p.barrel_elevation = next_angle;
                *out_angle_rad = next_angle;
                return TINY_BCLIBC_OK;
            }

            if (f_mid * f_next < REAL_C(0.0))
            {
                low_angle = mid_angle;
                f_low = f_mid;
                high_angle = next_angle;
                f_high = f_next;
            }
            else if (f_low * f_next < REAL_C(0.0))
            {
                high_angle = next_angle;
                f_high = f_next;
            }
            else
            {
                low_angle = next_angle;
                f_low = f_next;
            }

            if (TINY_BCLIBC_FABS(high_angle - low_angle) < angle_tol)
            {
                converged = 1;
                real_t res = (low_angle + high_angle) / REAL_C(2.0);
                p.barrel_elevation = res;
                *out_angle_rad = res;
                return TINY_BCLIBC_OK;
            }
        }

        if (!converged)
        {
            if (TINY_BCLIBC_FABS(high_angle - low_angle) < REAL_C(10.0) * acc)
            {
                *out_angle_rad = (low_angle + high_angle) / REAL_C(2.0);
                p.barrel_elevation = *out_angle_rad;
                return TINY_BCLIBC_OK;
            }
            tiny_bclibc__set_error("tiny_bclibc_find_zero_angle: not converged");
            return TINY_BCLIBC_ERR_ZERO_FINDING;
        }
        *out_angle_rad = (low_angle + high_angle) / REAL_C(2.0);
        return TINY_BCLIBC_OK;
    }

    /* ── zero_angle_newton — UNCHANGED (uses integrate_at, now Cash-Karp) ── */
    TINY_BCLIBC_INTERNAL int32_t tiny_bclibc__zero_angle_newton(
        TINY_BCLIBC_ShotProps *p,
        real_t slant_range_ft,
        real_t target_x_ft,
        real_t look_angle_rad,
        real_t *out_angle_rad)
    {
        real_t ca = TINY_BCLIBC_COS(look_angle_rad);
        real_t sa = TINY_BCLIBC_SIN(look_angle_rad);

#if defined(TINY_BCLIBC_SINGLE_PRECISION)
        const real_t cZeroFindingAccuracy = REAL_C(1e-3);
#else
    const real_t cZeroFindingAccuracy = REAL_C(5e-6);
#endif
        const real_t ALLOWED_ZERO_ERROR_FT = REAL_C(1e-2);
        const int32_t cMaxIterations = 40;

        int32_t iterations = 0;
        real_t range_error_ft = REAL_C(9e9);
        real_t prev_range_error_ft = REAL_C(9e9);
        real_t height_error_ft = cZeroFindingAccuracy * REAL_C(2.0);
        real_t prev_height_error_ft = REAL_C(9e9);
        real_t damping_factor = REAL_C(1.0);
        const real_t damping_rate = REAL_C(0.7);
        real_t last_correction = REAL_C(0.0);

        while (iterations < cMaxIterations)
        {
            TINY_BCLIBC_BaseTrajData raw;
            TINY_BCLIBC_TrajectoryData full;
            int32_t rc = tiny_bclibc_integrate_at(p, TINY_BCLIBC_KEY_POS_X, target_x_ft, &raw, &full);
            if (rc != TINY_BCLIBC_OK || raw.time == REAL_C(0.0))
                return TINY_BCLIBC_ERR_ZERO_FINDING;

            if (REAL_C(2.0) * raw.px < target_x_ft &&
                p->barrel_elevation == REAL_C(0.0) && look_angle_rad < REAL_C(1.5))
            {
                p->barrel_elevation = REAL_C(0.01);
                iterations++;
                continue;
            }

            real_t height_diff_ft = raw.py * ca - raw.px * sa;
            real_t look_dist_ft = raw.px * ca + raw.py * sa;
            range_error_ft = TINY_BCLIBC_FABS(look_dist_ft - slant_range_ft);
            height_error_ft = TINY_BCLIBC_FABS(height_diff_ft);

            real_t traj_angle = TINY_BCLIBC_ATAN2(raw.vy, raw.vx);
            real_t d_el = p->barrel_elevation - look_angle_rad;
            real_t d_tr = traj_angle - look_angle_rad;
            real_t cos_el = TINY_BCLIBC_COS(d_el), cos_tr = TINY_BCLIBC_COS(d_tr);
            real_t denom_prod = cos_el * cos_tr;
            real_t sensitivity = (TINY_BCLIBC_FABS(denom_prod) > REAL_C(1e-12))
                                     ? (TINY_BCLIBC_SIN(d_el) * TINY_BCLIBC_SIN(d_tr)) / denom_prod
                                     : REAL_C(0.0);
            real_t denominator = (sensitivity < REAL_C(-0.5))
                                     ? look_dist_ft
                                     : look_dist_ft * (REAL_C(1.0) + sensitivity);
            if (TINY_BCLIBC_FABS(denominator) <= REAL_C(1e-9))
                return TINY_BCLIBC_ERR_ZERO_FINDING;

            real_t correction = -height_diff_ft / denominator;

            if (range_error_ft > ALLOWED_ZERO_ERROR_FT)
            {
                if (range_error_ft > prev_range_error_ft - REAL_C(1e-6))
                    return TINY_BCLIBC_ERR_ZERO_FINDING;
            }
            else if (height_error_ft > TINY_BCLIBC_FABS(prev_height_error_ft))
            {
                damping_factor *= damping_rate;
                if (damping_factor < REAL_C(0.3))
                    return TINY_BCLIBC_ERR_ZERO_FINDING;
                p->barrel_elevation -= last_correction;
                correction = last_correction;
            }
            else if (damping_factor < REAL_C(1.0))
            {
                damping_factor = REAL_C(1.0);
            }

            prev_range_error_ft = range_error_ft;
            prev_height_error_ft = height_error_ft;

            if (height_error_ft <= cZeroFindingAccuracy && range_error_ft <= ALLOWED_ZERO_ERROR_FT)
            {
                *out_angle_rad = p->barrel_elevation;
                return TINY_BCLIBC_OK;
            }

            real_t applied = correction * damping_factor;
            p->barrel_elevation += applied;
            last_correction = applied;
            iterations++;
        }

        if (height_error_ft <= cZeroFindingAccuracy && range_error_ft <= ALLOWED_ZERO_ERROR_FT)
        {
            *out_angle_rad = p->barrel_elevation;
            return TINY_BCLIBC_OK;
        }
        return TINY_BCLIBC_ERR_ZERO_FINDING;
    }

    /* ── find_zero_angle — UNCHANGED ── */
    TINY_BCLIBC_FUNC int32_t tiny_bclibc_find_zero_angle(
        const TINY_BCLIBC_ShotProps *props,
        real_t distance_ft,
        real_t *out_angle_rad)
    {
        if (!props || !out_angle_rad)
        {
            tiny_bclibc__set_error("tiny_bclibc_find_zero_angle: NULL argument");
            return TINY_BCLIBC_ERR_INVALID_ARG;
        }

        real_t la = props->look_angle;
        real_t ca = TINY_BCLIBC_COS(la), sa = TINY_BCLIBC_SIN(la);
        real_t tx = distance_ft * ca;
        real_t ty = distance_ft * sa;
        real_t sh = -props->cant_cosine * props->sight_height;
        const real_t ZERO_ERR_FT = REAL_C(1e-2);

        if (TINY_BCLIBC_FABS(distance_ft) < ZERO_ERR_FT)
        {
            *out_angle_rad = la;
            return TINY_BCLIBC_OK;
        }
        if (TINY_BCLIBC_FABS(distance_ft) < REAL_C(2.0) * tiny_bclibc__fmax(TINY_BCLIBC_FABS(sh), props->cfg.cStepMultiplier))
        {
            *out_angle_rad = TINY_BCLIBC_ATAN2(ty + sh, tx);
            return TINY_BCLIBC_OK;
        }
        (void)ty;

        TINY_BCLIBC_ShotProps p_newton = *props;
        if (tiny_bclibc__zero_angle_newton(&p_newton, distance_ft, tx, la, out_angle_rad) == TINY_BCLIBC_OK)
            return TINY_BCLIBC_OK;

        tiny_bclibc__set_error("tiny_bclibc_find_zero_angle: Newton failed, using Ridder's fallback");
        return tiny_bclibc__find_zero_angle_ridders(props, distance_ft, out_angle_rad);
    }

    /* ── find_zero_point — UNCHANGED ── */
    TINY_BCLIBC_FUNC int32_t tiny_bclibc_find_zero_point(
        const TINY_BCLIBC_ShotProps *props,
        real_t distance_ft,
        TINY_BCLIBC_ZeroPointResult *out)
    {
        if (!props || !out)
        {
            tiny_bclibc__set_error("tiny_bclibc_find_zero_point: NULL argument");
            return TINY_BCLIBC_ERR_INVALID_ARG;
        }

        real_t angle_rad;
        int32_t rc = tiny_bclibc_find_zero_angle(props, distance_ft, &angle_rad);
        if (rc != TINY_BCLIBC_OK)
            return rc;

        TINY_BCLIBC_ShotProps solved_props = *props;
        solved_props.barrel_elevation = angle_rad;
        {
            real_t target_x_ft = distance_ft * TINY_BCLIBC_COS(props->look_angle);
            TINY_BCLIBC_BaseTrajData raw;
            rc = tiny_bclibc_integrate_at(&solved_props, TINY_BCLIBC_KEY_POS_X,
                                          target_x_ft, &raw, &out->point);
        }
        if (rc != TINY_BCLIBC_OK)
        {
            tiny_bclibc__set_error("tiny_bclibc_find_zero_point: target intercept not found");
            return rc;
        }
        out->point.flag = TINY_BCLIBC_TRAJ_FLAG_RANGE;
        out->angle_rad = angle_rad;
        return TINY_BCLIBC_OK;
    }

    /* ── find_max_range — UNCHANGED (uses range_for_angle, now Cash-Karp) ── */
    TINY_BCLIBC_FUNC int32_t tiny_bclibc_find_max_range(
        const TINY_BCLIBC_ShotProps *props,
        real_t low_deg,
        real_t high_deg,
        real_t *out_range_ft,
        real_t *out_angle_rad)
    {
        if (!props || !out_range_ft || !out_angle_rad)
        {
            tiny_bclibc__set_error("tiny_bclibc_find_max_range: NULL argument");
            return TINY_BCLIBC_ERR_INVALID_ARG;
        }
        TINY_BCLIBC_ShotProps p = *props;

        real_t inv_phi = REAL_C(0.6180339887498949);
        real_t inv_phi_sq = REAL_C(0.38196601125010515);
        real_t a = low_deg * TINY_BCLIBC_DEG_TO_RAD;
        real_t b = high_deg * TINY_BCLIBC_DEG_TO_RAD;
        real_t h = b - a;
        real_t c2 = a + inv_phi_sq * h;
        real_t d = a + inv_phi * h;
        real_t yc = tiny_bclibc__range_for_angle(&p, c2);
        real_t yd = tiny_bclibc__range_for_angle(&p, d);

        for (int32_t i = 0; i < 100; i++)
        {
            if (h < REAL_C(1e-5))
                break;
            if (yc > yd)
            {
                b = d;
                d = c2;
                yd = yc;
                h = b - a;
                c2 = a + inv_phi_sq * h;
                yc = tiny_bclibc__range_for_angle(&p, c2);
            }
            else
            {
                a = c2;
                c2 = d;
                yc = yd;
                h = b - a;
                d = a + inv_phi * h;
                yd = tiny_bclibc__range_for_angle(&p, d);
            }
        }
        *out_angle_rad = (a + b) / REAL_C(2.0);
        *out_range_ft = tiny_bclibc__range_for_angle(&p, *out_angle_rad);
        return TINY_BCLIBC_OK;
    }

/* ── tiny_bclibc_impl.c entry point ── */
#ifdef TINY_BCLIBC_BUILD_SHARED
    TINY_BCLIBC_THREAD_LOCAL char tiny_bclibc__s_error[512];

    TINY_BCLIBC_FUNC const char *tiny_bclibc_last_error(void)
    {
        return tiny_bclibc__s_error;
    }
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* TINY_BCLIBC_ENGINE_H */