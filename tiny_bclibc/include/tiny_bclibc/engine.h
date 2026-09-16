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
    /* In compiled-mode — one true thread-local global in tiny_bclibc_impl.c */
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

    /* ════════════════════════════════════════════════════════════════════
     *  Internal RK4 functions (always static inline)
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

    /* ── Context for on_step callback ───────────────────────────────── */
    typedef int32_t (*tiny_bclibc__OnStep)(const TINY_BCLIBC_BaseTrajData *pt, void *ctx);

    /* ── Streaming callback — returns 0 to continue, TINY_BCLIBC_TERM_HANDLER_STOP to stop ── */
    typedef int32_t (*tiny_bclibc_StreamCb)(const TINY_BCLIBC_TrajectoryData *pt, void *ctx);

    /* ── Raw per-step callback (public) — same signature as the internal on_step callback,
     *  so it can be handed straight through to tiny_bclibc__run_rk4 by tiny_bclibc_integrate_raw.
     *  Receives every raw RK4 step (no C-side interpolation/filtering applied).
     *  Return 0 to continue, non-zero to request early stop. ── */
    typedef int32_t (*tiny_bclibc_RawStepCb)(const TINY_BCLIBC_BaseTrajData *pt, void *ctx);

    /* ── Interval callback — receives one accepted integration step (start, end).
     *  Return 0 to continue, non-zero to request early stop. Used by adaptive
     *  integrators (Cash-Karp) whose accepted steps are sparse and irregularly
     *  spaced, so downstream event/row reconstruction cannot assume dense,
     *  near-uniform raw sampling the way tiny_bclibc__OnStep's per-raw-point
     *  contract does -- see tiny_bclibc__hermite_at_x/_at_value below. ── */
    typedef int32_t (*tiny_bclibc__OnInterval)(const TINY_BCLIBC_BaseTrajData *start,
                                               const TINY_BCLIBC_BaseTrajData *end,
                                               void *ctx);

    /* ── Stop control ────────────────────────────────────────────────── */
    typedef struct tiny_bclibc__StopCtrl
    {
        real_t range_limit_ft;
        real_t min_velocity_fps;
        real_t max_drop_ft; /* signed and corrected for sight_height */
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
     *  Exact-derivative 2-point Hermite reconstruction within one accepted
     *  interval (start, end) -- used by the adaptive (Cash-Karp) filter to
     *  recover RANGE/time-step rows and APEX/MACH/ZERO events without
     *  assuming dense, near-uniform raw sampling. Position uses
     *  tiny_bclibc_hermite() with the *exact* endpoint velocities as slopes;
     *  velocity is the derivative of that same cubic
     *  (tiny_bclibc_hermite_derivative()), not a separately-interpolated
     *  series, so position and velocity stay mutually consistent. Mirrors
     *  bclibc's src/traj_filter.cpp hermite_at_time/_at_x/_at_value
     *  (project issue #350).
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

    /* Bisects on time for the sample within [a,b] whose px equals target_x.
     * Returns 0 if target_x isn't bracketed by a->px/b->px. */
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
        /* The query axis is assigned the exact target, not re-derived from the converged
         * Hermite sample -- mirrors TINY_BCLIBC_BaseTrajData_interpolate's own convention
         * (`out->px = (key == KEY_POS_X) ? key_val : ...`). At double precision the converged
         * sample.px already matches target_x to within solver residual either way, but at
         * single precision one float32 ULP near a large position (e.g. ~0.004 ft at 36000 ft)
         * is *larger* than that residual, so re-deriving it measurably misses an exact RANGE
         * step target instead of hitting it bit-for-bit. */
        out->px = target_x;
        return 1;
    }

    /* aux lets the value function close over extra state (e.g. look-angle
     * trig) without a global -- see tiny_bclibc__value_slant below. */
    typedef real_t (*tiny_bclibc__ValueFn)(const TINY_BCLIBC_BaseTrajData *pt, const void *aux);

    /* Bisects on time for the sample within [a,b] where value(sample)==target.
     * Returns 0 if target isn't bracketed by value(a)/value(b). */
    static inline int32_t tiny_bclibc__hermite_at_value(
        const TINY_BCLIBC_BaseTrajData *a, const TINY_BCLIBC_BaseTrajData *b,
        tiny_bclibc__ValueFn value, const void *aux, real_t target, TINY_BCLIBC_BaseTrajData *out)
    {
        real_t lo = a->time, hi = b->time;
        real_t flo = value(a, aux) - target;
        real_t fhi = value(b, aux) - target;
        if (flo == REAL_C(0.0)) { *out = *a; return 1; }
        if (fhi == REAL_C(0.0)) { *out = *b; return 1; }
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
        TINY_BCLIBC_V3dT dvr; /* d(v_rel)/dt */
        TINY_BCLIBC_V3dT dp;  /* d(pos)/dt   */
    } tiny_bclibc__CkDeriv;

    /* Per-stage derivative: the drag coefficient AND the atmosphere sample are
     * both re-evaluated fresh from this stage's own intermediate
     * velocity/altitude, unlike tiny_bclibc__run_rk4's once-per-step
     * memoization. This is required, not optional: an earlier variant that
     * froze km once per step (mirroring RK4's own trick, extended to 6
     * stages) produced real, tolerance-independent accuracy failures,
     * because the embedded 4th/5th-order error estimator cannot see model
     * error from a stale drag/atmosphere sample -- only discretization error
     * of the frozen sub-problem (project issue #350). */
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

    /* ── Cash-Karp adaptive RK45 loop ────────────────────────────────────
     * Adaptive-step alternative to tiny_bclibc__run_rk4: grows the step
     * during smooth flight (up to 64x the configured base step) and shrinks
     * it (down to base/64) whenever the embedded error estimate exceeds
     * tolerance, retrying the same attempted step. Produces far fewer, much
     * less uniformly spaced raw points than run_rk4's fixed step -- callers
     * MUST reconstruct RANGE/time-step rows and APEX/MACH/ZERO events via
     * the 2-point exact-derivative Hermite helpers above
     * (tiny_bclibc__hermite_at_x/_at_value), not the 3-point
     * finite-difference PCHIP window run_rk4's consumers use: that scheme's
     * slopes are estimated from neighboring raw-point spacing and
     * measurably mispredict crossings once points are sparse/irregular
     * (project issue #350). See tiny_bclibc__integrate_on_interval below
     * for the consumer that does this correctly. ── */
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
            step_start.px = pos.x; step_start.py = pos.y; step_start.pz = pos.z;
            step_start.vx = vel.x; step_start.vy = vel.y; step_start.vz = vel.z;
            step_start.mach = TINY_BCLIBC_V3dT_mag(vr) * inv_mach;
            if (on_first(&step_start, ctx) != 0)
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

                /* 5th-order solution */
                vr_next = vr; pos_next = pos;
                TINY_BCLIBC_V3dT_fma(&vr_next, k1.dvr, dt * (REAL_C(37.0) / REAL_C(378.0)));
                TINY_BCLIBC_V3dT_fma(&vr_next, k3.dvr, dt * (REAL_C(250.0) / REAL_C(621.0)));
                TINY_BCLIBC_V3dT_fma(&vr_next, k4.dvr, dt * (REAL_C(125.0) / REAL_C(594.0)));
                TINY_BCLIBC_V3dT_fma(&vr_next, k6.dvr, dt * (REAL_C(512.0) / REAL_C(1771.0)));
                TINY_BCLIBC_V3dT_fma(&pos_next, k1.dp, dt * (REAL_C(37.0) / REAL_C(378.0)));
                TINY_BCLIBC_V3dT_fma(&pos_next, k3.dp, dt * (REAL_C(250.0) / REAL_C(621.0)));
                TINY_BCLIBC_V3dT_fma(&pos_next, k4.dp, dt * (REAL_C(125.0) / REAL_C(594.0)));
                TINY_BCLIBC_V3dT_fma(&pos_next, k6.dp, dt * (REAL_C(512.0) / REAL_C(1771.0)));

                /* Error estimate: 5th-order minus embedded 4th-order coefficients. */
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

                real_t scale_v = REAL_C(1e-3) + REAL_C(1e-6) * TINY_BCLIBC_V3dT_mag(vr_next);
                real_t scale_p = REAL_C(1e-4) + REAL_C(1e-6) * TINY_BCLIBC_V3dT_mag(pos_next);
                real_t err_norm_v = TINY_BCLIBC_V3dT_mag(err_v) / scale_v;
                real_t err_norm_p = TINY_BCLIBC_V3dT_mag(err_p) / scale_p;
                real_t err_norm = (err_norm_v > err_norm_p) ? err_norm_v : err_norm_p;

                if (err_norm <= REAL_C(1.0) || dt <= min_dt * REAL_C(1.0001))
                {
                    dt_used = dt; /* the step actually just integrated, before growing dt for next time */
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
                step_end.px = pos.x; step_end.py = pos.y; step_end.pz = pos.z;
                step_end.vx = vel.x; step_end.vy = vel.y; step_end.vz = vel.z;
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

    /* ── Main RK4 loop ───────────────────────────────────────────────── */
    TINY_BCLIBC_INTERNAL int32_t tiny_bclibc__run_rk4(
        const TINY_BCLIBC_ShotProps *props,
        const TINY_BCLIBC_TrajectoryRequest *req,
        tiny_bclibc__OnStep on_step,
        void *ctx,
        int32_t *out_reason)
    {
        const real_t dt = props->calc_step;
        if (dt <= REAL_C(0.0))
        {
            tiny_bclibc__set_error("calc_step must be > 0");
            *out_reason = TINY_BCLIBC_TERM_NO_TERMINATE;
            return TINY_BCLIBC_ERR_INVALID_ARG;
        }

        real_t range_limit = req ? req->range_limit_ft : TINY_BCLIBC_MAX_INTEGRATION_RANGE;

        tiny_bclibc__StopCtrl sc;
        tiny_bclibc__stop_ctrl_init(&sc, props, range_limit,
                                    props->cfg.cMinimumVelocity,
                                    props->cfg.cMaximumDrop, props->cfg.cMinimumAltitude, out_reason);

        TINY_BCLIBC_WindSock ws = props->wind_sock; /* local copy */

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
        TINY_BCLIBC_V3dT vel = TINY_BCLIBC_V3dT_make(dir.x * muzzle, dir.y * muzzle, dir.z * muzzle);

        real_t time = REAL_C(0.0);
        *out_reason = TINY_BCLIBC_TERM_NO_TERMINATE;

        while (*out_reason == TINY_BCLIBC_TERM_NO_TERMINATE)
        {
            /* update wind */
            if (pos.x >= ws.next_range)
                wind = TINY_BCLIBC_WindSock_vector_for_range(&ws, pos.x);

            /* atmosphere */
            real_t density_ratio, mach_fps;
            TINY_BCLIBC_Atmosphere_update_density_mach(&props->atmo,
                                                       props->alt0 + pos.y, &density_ratio, &mach_fps);

            /* current point */
            real_t inv_mach = (mach_fps != REAL_C(0.0)) ? (REAL_C(1.0) / mach_fps) : REAL_C(1.0);
            TINY_BCLIBC_V3dT v_rel = TINY_BCLIBC_V3dT_make(vel.x - wind.x, vel.y - wind.y, vel.z - wind.z);
            real_t rel_speed = TINY_BCLIBC_SQRT(v_rel.x * v_rel.x + v_rel.y * v_rel.y + v_rel.z * v_rel.z);
            real_t cur_mach = rel_speed * inv_mach;

            TINY_BCLIBC_BaseTrajData pt;
            pt.time = time;
            pt.px = pos.x;
            pt.py = pos.y;
            pt.pz = pos.z;
            pt.vx = vel.x;
            pt.vy = vel.y;
            pt.vz = vel.z;
            pt.mach = cur_mach;

            tiny_bclibc__stop_ctrl_check(&sc, &pt);
            if (*out_reason != TINY_BCLIBC_TERM_NO_TERMINATE)
                break;

            int32_t cb = on_step(&pt, ctx);
            if (cb != 0)
            {
                *out_reason = TINY_BCLIBC_TERM_HANDLER_STOP;
                break;
            }

            /* km = density_ratio * drag_by_mach */
            real_t km = density_ratio * TINY_BCLIBC_ShotProps_drag_by_mach(props, cur_mach);

            /* Coriolis */
            TINY_BCLIBC_V3dT gpc = gravity;
            if (!props->coriolis.flat_fire_only)
            {
                TINY_BCLIBC_V3dT ca;
                TINY_BCLIBC_Coriolis_acceleration_local(&props->coriolis, vel, &ca);
                gpc.x += ca.x;
                gpc.y += ca.y;
                gpc.z += ca.z;
            }

            /* RK4 */
            const real_t dt_half = REAL_C(0.5) * dt;
            const real_t dt_sixth = dt / REAL_C(6.0);

            TINY_BCLIBC_V3dT k1v, k2v, k3v, k4v;
            TINY_BCLIBC_V3dT k1p, k2p, k3p, k4p;
            TINY_BCLIBC_V3dT vt, pt2;

            tiny_bclibc__calc_dvdt(v_rel, gpc, km, rel_speed, &k1v);
            k1p = vel;

            vt = TINY_BCLIBC_V3dT_make(v_rel.x + k1v.x * dt_half, v_rel.y + k1v.y * dt_half, v_rel.z + k1v.z * dt_half);
            {
                real_t sp = TINY_BCLIBC_SQRT(vt.x * vt.x + vt.y * vt.y + vt.z * vt.z);
                tiny_bclibc__calc_dvdt(vt, gpc, km, sp, &k2v);
            }
            pt2 = TINY_BCLIBC_V3dT_make(vel.x + k1v.x * dt_half, vel.y + k1v.y * dt_half, vel.z + k1v.z * dt_half);
            k2p = pt2;

            vt = TINY_BCLIBC_V3dT_make(v_rel.x + k2v.x * dt_half, v_rel.y + k2v.y * dt_half, v_rel.z + k2v.z * dt_half);
            {
                real_t sp = TINY_BCLIBC_SQRT(vt.x * vt.x + vt.y * vt.y + vt.z * vt.z);
                tiny_bclibc__calc_dvdt(vt, gpc, km, sp, &k3v);
            }
            pt2 = TINY_BCLIBC_V3dT_make(vel.x + k2v.x * dt_half, vel.y + k2v.y * dt_half, vel.z + k2v.z * dt_half);
            k3p = pt2;

            vt = TINY_BCLIBC_V3dT_make(v_rel.x + k3v.x * dt, v_rel.y + k3v.y * dt, v_rel.z + k3v.z * dt);
            {
                real_t sp = TINY_BCLIBC_SQRT(vt.x * vt.x + vt.y * vt.y + vt.z * vt.z);
                tiny_bclibc__calc_dvdt(vt, gpc, km, sp, &k4v);
            }
            pt2 = TINY_BCLIBC_V3dT_make(vel.x + k3v.x * dt, vel.y + k3v.y * dt, vel.z + k3v.z * dt);
            k4p = pt2;

            vel.x += (k1v.x + REAL_C(2.0) * k2v.x + REAL_C(2.0) * k3v.x + k4v.x) * dt_sixth;
            vel.y += (k1v.y + REAL_C(2.0) * k2v.y + REAL_C(2.0) * k3v.y + k4v.y) * dt_sixth;
            vel.z += (k1v.z + REAL_C(2.0) * k2v.z + REAL_C(2.0) * k3v.z + k4v.z) * dt_sixth;

            pos.x += (k1p.x + REAL_C(2.0) * k2p.x + REAL_C(2.0) * k3p.x + k4p.x) * dt_sixth;
            pos.y += (k1p.y + REAL_C(2.0) * k2p.y + REAL_C(2.0) * k3p.y + k4p.y) * dt_sixth;
            pos.z += (k1p.z + REAL_C(2.0) * k2p.z + REAL_C(2.0) * k3p.z + k4p.z) * dt_sixth;

            time += dt;
        }

        /* final point */
        {
            real_t density_ratio, mach_fps;
            TINY_BCLIBC_Atmosphere_update_density_mach(&props->atmo,
                                                       props->alt0 + pos.y, &density_ratio, &mach_fps);
            real_t inv_mach = (mach_fps != REAL_C(0.0)) ? REAL_C(1.0) / mach_fps : REAL_C(1.0);
            TINY_BCLIBC_V3dT v_rel = TINY_BCLIBC_V3dT_make(vel.x - wind.x, vel.y - wind.y, vel.z - wind.z);
            real_t rel_speed = TINY_BCLIBC_SQRT(v_rel.x * v_rel.x + v_rel.y * v_rel.y + v_rel.z * v_rel.z);
            TINY_BCLIBC_BaseTrajData fin;
            fin.time = time;
            fin.px = pos.x;
            fin.py = pos.y;
            fin.pz = pos.z;
            fin.vx = vel.x;
            fin.vy = vel.y;
            fin.vz = vel.z;
            fin.mach = rel_speed * inv_mach;
            on_step(&fin, ctx);
        }
        return TINY_BCLIBC_OK;
    }

    /* ════════════════════════════════════════════════════════════════════
     *  tiny_bclibc_build_shot_props
     * ════════════════════════════════════════════════════════════════════ */

    TINY_BCLIBC_FUNC int32_t tiny_bclibc_build_shot_props(
        const TINY_BCLIBC_Shot *shot,
        TINY_BCLIBC_CurvePoint *curve_buf, /* caller-allocated, >= drag_table_size */
        TINY_BCLIBC_ShotProps *out)
    {
        if (!shot || !curve_buf || !out || shot->drag_table_size < 2)
        {
            tiny_bclibc__set_error("tiny_bclibc_build_shot_props: invalid argument");
            return TINY_BCLIBC_ERR_INVALID_ARG;
        }

        /* PCHIP curve */
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
     *  tiny_bclibc_integrate  — interval-based filter (Cash-Karp consumer)
     *
     *  Reconstructs RANGE/time-step rows and APEX/MACH/ZERO events from
     *  each accepted (start, end) interval via the 2-point exact-derivative
     *  Hermite helpers above, instead of a 3-point finite-difference window
     *  over raw points -- see tiny_bclibc__run_cashkarp's doc comment for
     *  why that distinction matters once raw samples are sparse/irregular.
     * ════════════════════════════════════════════════════════════════════ */

    typedef struct tiny_bclibc__IntegrateCtx
    {
        const TINY_BCLIBC_ShotProps *props;
        const TINY_BCLIBC_TrajectoryRequest *req;
        TINY_BCLIBC_TrajectoryData *buf;
        int32_t capacity;
        int32_t written;
        int32_t total;
        /* Step index rather than an accumulated `next_range_dist += range_step_ft`: each
         * target is computed fresh as range_step_ft * index, so per-target rounding does not
         * compound across iterations the way repeated float32 addition would (confirmed to
         * matter: a 10-step accumulation measurably drifts a range_limit_ft target by ~1e-4 ft
         * in single precision). */
        int32_t range_step_index;
        real_t time_of_last;
        /* filter state */
        int32_t active_flags;
        /* streaming - if specified, emit calls cb instead of writing to buf */
        tiny_bclibc_StreamCb stream_cb;
        void *stream_ctx;
        int32_t stream_stop;
        /* last raw point seen (set unconditionally, whether or not it was emitted) --
         * lets a caller reconstruct the exact terminal state of an incomplete trajectory
         * even when it didn't happen to land on a requested range/time step. */
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
        /* if the start is above zero, we do not search ZERO_UP */
        if ((ff & TINY_BCLIBC_TRAJ_FLAG_ZERO_UP) && pt->py >= REAL_C(0.0))
            ff &= ~TINY_BCLIBC_TRAJ_FLAG_ZERO_UP;
        if ((ff & TINY_BCLIBC_TRAJ_FLAG_ZERO) && pt->py < REAL_C(0.0))
        {
            real_t look_el = TINY_BCLIBC_ATAN2(pt->vy, pt->vx);
            if (look_el <= c->props->look_angle)
                ff &= ~(TINY_BCLIBC_TRAJ_FLAG_ZERO | TINY_BCLIBC_TRAJ_FLAG_MRT);
        }
        c->active_flags = ff;
        /* the first point always comes out */
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

        /* ── range steps ── */
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

        /* ── time steps ── */
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

        /* ── apex ── */
        if ((c->active_flags & TINY_BCLIBC_TRAJ_FLAG_APEX) &&
            start->vy > REAL_C(0.0) && end->vy <= REAL_C(0.0))
        {
            TINY_BCLIBC_BaseTrajData r;
            if (tiny_bclibc__hermite_at_value(start, end, tiny_bclibc__value_vy, NULL, REAL_C(0.0), &r))
                tiny_bclibc__integrate_emit(c, &r, TINY_BCLIBC_TRAJ_FLAG_APEX);
            c->active_flags &= ~TINY_BCLIBC_TRAJ_FLAG_APEX;
        }

        /* ── mach crossing ──
         * .mach is a Mach *ratio* (relative_speed / speed_of_sound), not a speed --
         * the crossing condition is simply "ratio dropped below 1", mirrored below by
         * bisecting that same ratio field to the value 1.0. Strict `>` on the start side
         * (not `>=`) matters: a muzzle velocity exactly at Mach 1 must NOT count as an
         * already-crossed state on the very first interval (it would otherwise bisect to
         * a root at essentially t=0, merging a MACH flag onto the muzzle row) -- mirrors
         * base_engine.py's TrajectoryDataFilter.record_step. */
        if ((c->active_flags & TINY_BCLIBC_TRAJ_FLAG_MACH) &&
            start->mach > REAL_C(1.0) && end->mach < REAL_C(1.0))
        {
            TINY_BCLIBC_BaseTrajData r;
            if (tiny_bclibc__hermite_at_value(start, end, tiny_bclibc__value_mach, NULL, REAL_C(1.0), &r))
                tiny_bclibc__integrate_emit(c, &r, TINY_BCLIBC_TRAJ_FLAG_MACH);
            c->active_flags &= ~TINY_BCLIBC_TRAJ_FLAG_MACH;
        }

        /* ── zero crossings ──
         * Bisect directly on slant height (py*cos(look_angle) - px*sin(look_angle), the
         * signed distance orthogonal to the sight line), exact regardless of look angle.
         * Strict `<`/`>` on BOTH sides (not `>=`/`<=`) -- a start exactly on the sight line
         * must not count as an already-crossed state on the interval that starts there
         * (same reasoning as the Mach-ratio gate above); mirrors base_engine.py's
         * TrajectoryDataFilter.record_step. */
        if (c->active_flags & TINY_BCLIBC_TRAJ_FLAG_ZERO)
        {
            tiny_bclibc__SlantAux aux;
            aux.la_cos = TINY_BCLIBC_COS(c->props->look_angle);
            aux.la_sin = TINY_BCLIBC_SIN(c->props->look_angle);
            real_t start_slant = start->py * aux.la_cos - start->px * aux.la_sin;
            real_t end_slant = end->py * aux.la_cos - end->px * aux.la_sin;
            int32_t cross_flag = 0;
            /* Branch on which bit is still live, not on (bit && condition) together --
             * ZERO_DOWN must never be considered while still waiting for ZERO_UP. */
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

        /* Modified copy of ShotProps with config for break */
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

    /* ── integrate_stream ────────────────────────────────────────────── */

    TINY_BCLIBC_FUNC int32_t tiny_bclibc_integrate_stream(
        const TINY_BCLIBC_ShotProps *props,
        const TINY_BCLIBC_TrajectoryRequest *req,
        tiny_bclibc_StreamCb cb,
        void *cb_ctx,
        int32_t *out_total,
        int32_t *out_reason,
        TINY_BCLIBC_BaseTrajData *out_final_raw /* optional (may be NULL) */)
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

    /* ── integrate_raw ────────────────────────────────────────────────
     * Streams every raw RK4 step (time, position, velocity, mach — no C-side
     * interpolation, filtering, or derived-field computation) to cb. Intended for
     * external drivers (e.g. ctypes bindings) that want to reuse a host-language
     * trajectory filter / zero-finding / apex / max-range implementation while
     * delegating only the numerically-sensitive RK4 stepping to tiny_bclibc — so the
     * host can exercise tiny_bclibc's compiled precision (float or double) as the
     * physics core of its own engine.
     *
     * Stop conditions (min velocity, max drop, min altitude) are taken from
     * props->cfg — see tiny_bclibc_build_shot_props, which populates props->cfg
     * from TINY_BCLIBC_Shot::config. range_limit_ft is passed explicitly since
     * callers typically vary it per call (e.g. zero-finding probes vs full
     * trajectories) independent of the shot's own config. */
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

        int32_t reason = TINY_BCLIBC_TERM_NO_TERMINATE;
        int32_t rc = tiny_bclibc__run_rk4(props, &req, cb, cb_ctx, &reason);
        *out_reason = reason;
        return rc;
    }

    /* ── sizeof helpers — let external (e.g. ctypes) callers validate their opaque
     *  buffer sizes / struct mirrors against the actual compiled layout. ── */
    TINY_BCLIBC_FUNC int32_t tiny_bclibc_sizeof_shot_props(void)
    {
        return (int32_t)sizeof(TINY_BCLIBC_ShotProps);
    }

    TINY_BCLIBC_FUNC int32_t tiny_bclibc_sizeof_curve_point(void)
    {
        return (int32_t)sizeof(TINY_BCLIBC_CurvePoint);
    }

    /* ── integrate_at ─────────────────────────────────────────────────── */

    typedef struct tiny_bclibc__AtCtx
    {
        int32_t key;
        real_t target;
        TINY_BCLIBC_BaseTrajData win[3];
        int32_t n;
        int32_t found;
        TINY_BCLIBC_BaseTrajData result;
    } tiny_bclibc__AtCtx;

    static inline int32_t tiny_bclibc__at_on_step(const TINY_BCLIBC_BaseTrajData *pt, void *ctx_)
    {
        tiny_bclibc__AtCtx *c = (tiny_bclibc__AtCtx *)ctx_;
        if (c->found)
            return 1; /* early return */

        if (c->n < 3)
        {
            c->win[c->n++] = *pt;
        }
        else
        {
            c->win[0] = c->win[1];
            c->win[1] = c->win[2];
            c->win[2] = *pt;
        }

        if (c->n < 3)
            return 0;

        real_t v1 = TINY_BCLIBC_BaseTrajData_get(&c->win[1], c->key);
        real_t v2 = TINY_BCLIBC_BaseTrajData_get(&c->win[2], c->key);
        int32_t crossed = ((v1 <= c->target && c->target <= v2) ||
                           (v2 <= c->target && c->target <= v1));
        if (crossed)
        {
            TINY_BCLIBC_BaseTrajData_interpolate(c->key, c->target,
                                                 &c->win[0], &c->win[1], &c->win[2], &c->result);
            c->found = 1;
            return 1;
        }
        return 0;
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
        tiny_bclibc__run_rk4(props, &req, tiny_bclibc__at_on_step, &ctx, &reason);

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

    /* ── find_apex ────────────────────────────────────────────────────── */

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

    /* ── error_at_distance (internal) ───────────────────────────────── */
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

    /* ── range_for_angle (internal) ─────────────────────────────────── */
    typedef struct tiny_bclibc__ZeroCrossCtx
    {
        real_t la_cos, la_sin;
        int32_t found;
        real_t slant_dist;
        TINY_BCLIBC_BaseTrajData prev;
        int32_t has_prev;
    } tiny_bclibc__ZeroCrossCtx;

    static inline int32_t tiny_bclibc__zero_cross_on_step(const TINY_BCLIBC_BaseTrajData *pt, void *ctx_)
    {
        tiny_bclibc__ZeroCrossCtx *c = (tiny_bclibc__ZeroCrossCtx *)ctx_;
        if (c->found)
            return 1;
        if (!c->has_prev)
        {
            c->prev = *pt;
            c->has_prev = 1;
            return 0;
        }

        real_t h_prev = c->prev.py * c->la_cos - c->prev.px * c->la_sin;
        real_t h_curr = pt->py * c->la_cos - pt->px * c->la_sin;

        if (h_prev > REAL_C(0.0) && h_curr <= REAL_C(0.0))
        {
            real_t denom = h_prev - h_curr;
            real_t t = (denom == REAL_C(0.0)) ? REAL_C(1.0)
                                              : TINY_BCLIBC_FABS(h_prev) / denom;
            if (t < REAL_C(0.0))
                t = REAL_C(0.0);
            if (t > REAL_C(1.0))
                t = REAL_C(1.0);
            real_t ix = c->prev.px + t * (pt->px - c->prev.px);
            real_t iy = c->prev.py + t * (pt->py - c->prev.py);
            c->slant_dist = ix * c->la_cos + iy * c->la_sin;
            c->found = 1;
            return 1;
        }
        c->prev = *pt;
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
        tiny_bclibc__run_rk4(props_mut, &req, tiny_bclibc__zero_cross_on_step, &ctx, &reason);
        return ctx.found ? ctx.slant_dist : REAL_C(0.0);
    }

    /* ── find_zero_angle_ridders (fallback: GSS + Ridder's) ─────────── */

    TINY_BCLIBC_INTERNAL int32_t tiny_bclibc__find_zero_angle_ridders(
        const TINY_BCLIBC_ShotProps *props,
        real_t distance_ft,
        real_t *out_angle_rad)
    {
        if (!props || !out_angle_rad)
            return TINY_BCLIBC_ERR_INVALID_ARG;

        /* Mutable copy for barrel_elevation change */
        TINY_BCLIBC_ShotProps p = *props;

        real_t la = p.look_angle;
        real_t ca = TINY_BCLIBC_COS(la), sa = TINY_BCLIBC_SIN(la);
        real_t tx = distance_ft * ca;
        real_t ty = distance_ft * sa;
        real_t sh = -p.cant_cosine * p.sight_height;
        const real_t ZERO_ERR_FT = REAL_C(0.5);

        if (TINY_BCLIBC_FABS(distance_ft) < ZERO_ERR_FT)
        {
            *out_angle_rad = la;
            return TINY_BCLIBC_OK;
        }
        if (TINY_BCLIBC_FABS(distance_ft) < REAL_C(2.0) * TINY_BCLIBC_FABS(sh))
        {
            *out_angle_rad = TINY_BCLIBC_ATAN2(ty + sh, tx);
            return TINY_BCLIBC_OK;
        }

        /* Find max range to resolve bracket */
        real_t inv_phi = REAL_C(0.6180339887498949);
        real_t inv_phi_sq = REAL_C(0.38196601125010515);
        real_t a = REAL_C(0.0), b = REAL_C(1.5707963267948966); /* 0..90 deg */
        real_t h = b - a;
        real_t c2 = a + inv_phi_sq * h;
        real_t d = a + inv_phi * h;
        real_t yc, yd;
        /* For single precision the bracket only needs to be tight enough to
         * enclose the zero angle — Ridder's method provides the final accuracy.
         * 1e-2 rad (~0.57 deg) requires ~13 GSS iterations vs ~25 for 1e-5,
         * halving the number of expensive range_for_angle calls. */
#ifdef TINY_BCLIBC_FAST_ZERO_FIND
        const real_t GSS_H_TOL = REAL_C(1e-2);
#else
    const real_t GSS_H_TOL = REAL_C(1e-5);
#endif

        /* FAST_ZERO_FIND: coarser step for GSS bracket search only.
         * range_for_angle just needs to rank angles by range — fine accuracy
         * is not required.  Ridder's below restores the original step. */
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

        /* Ridder's height-error convergence in feet.  float has ~7 significant
         * digits so 0.01 ft (3 mm) is the practical accuracy floor for single
         * precision; double uses 0.001 ft. */
#ifdef TINY_BCLIBC_FAST_ZERO_FIND
        const real_t acc = REAL_C(0.01);
#else
    const real_t acc = REAL_C(0.001);
#endif
        /* Angle bracket convergence in radians — independent of acc (which is in
         * feet).  1e-5 rad ≈ 0.0006° gives sub-milliradian accuracy on any
         * target distance; kept constant across FAST/normal modes. */
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

    /* ── zero_angle_newton (primary: damped Newton-like iteration) ────── */
    /*
     * Matches BCLIBC_BaseEngine::zero_angle() in src/engine.cpp.
     * Integrates only to target_x_ft each iteration — fast convergence
     * (~3-5 iterations) without a full trajectory to max range.
     */
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

            /* Degenerate: bullet barely moves — nudge elevation */
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

    /* ── find_zero_angle (public: Newton primary + Ridder's fallback) ─── */
    /*
     * Matches BCLIBC_BaseEngine::zero_angle_with_fallback() in src/engine.cpp.
     */
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
        const real_t ZERO_ERR_FT = REAL_C(0.5);

        if (TINY_BCLIBC_FABS(distance_ft) < ZERO_ERR_FT)
        {
            *out_angle_rad = la;
            return TINY_BCLIBC_OK;
        }
        if (TINY_BCLIBC_FABS(distance_ft) < REAL_C(2.0) * TINY_BCLIBC_FABS(sh))
        {
            *out_angle_rad = TINY_BCLIBC_ATAN2(ty + sh, tx);
            return TINY_BCLIBC_OK;
        }
        (void)ty;

        /* Primary: Newton-like damped iteration */
        TINY_BCLIBC_ShotProps p_newton = *props;
        if (tiny_bclibc__zero_angle_newton(&p_newton, distance_ft, tx, la, out_angle_rad) == TINY_BCLIBC_OK)
            return TINY_BCLIBC_OK;

        /* Fallback: GSS + Ridder's (guaranteed bracket method) */
        tiny_bclibc__set_error("tiny_bclibc_find_zero_angle: Newton failed, using Ridder's fallback");
        return tiny_bclibc__find_zero_angle_ridders(props, distance_ft, out_angle_rad);
    }

    /* ── find_max_range ───────────────────────────────────────────────── */

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

/* ── tiny_bclibc_impl.c entry point ──────────────────────────────────── */
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
