#ifndef TINY_BCLIBC_TRAJ_DATA_H
#define TINY_BCLIBC_TRAJ_DATA_H

#include <stdint.h>
#include "platform.h"
#include "base_types.h"
#include "interp.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* ── Base trajectory point data ───────────────────────────────────
     *  9 × real_t = 72 B (double) / 36 B (float)
     */
    typedef struct TINY_BCLIBC_BaseTrajData
    {
        real_t time;
        real_t px, py, pz; /* position  (ft) */
        real_t vx, vy, vz; /* velocity  (fps)*/
        real_t mach;
    } TINY_BCLIBC_BaseTrajData;

    /* ── Interpolation key for BaseTrajData ─────────────────────────── */
    typedef enum TINY_BCLIBC_InterpKey
    {
        TINY_BCLIBC_KEY_TIME = 0,
        TINY_BCLIBC_KEY_MACH = 1,
        TINY_BCLIBC_KEY_POS_X = 2,
        TINY_BCLIBC_KEY_POS_Y = 3,
        TINY_BCLIBC_KEY_POS_Z = 4,
        TINY_BCLIBC_KEY_VEL_X = 5,
        TINY_BCLIBC_KEY_VEL_Y = 6,
        TINY_BCLIBC_KEY_VEL_Z = 7,
    } TINY_BCLIBC_InterpKey;

    TINY_BCLIBC_INLINE_FUNC real_t TINY_BCLIBC_BaseTrajData_get(const TINY_BCLIBC_BaseTrajData *p, int32_t key)
    {
        switch (key)
        {
        case TINY_BCLIBC_KEY_TIME:
            return p->time;
        case TINY_BCLIBC_KEY_MACH:
            return p->mach;
        case TINY_BCLIBC_KEY_POS_X:
            return p->px;
        case TINY_BCLIBC_KEY_POS_Y:
            return p->py;
        case TINY_BCLIBC_KEY_POS_Z:
            return p->pz;
        case TINY_BCLIBC_KEY_VEL_X:
            return p->vx;
        case TINY_BCLIBC_KEY_VEL_Y:
            return p->vy;
        case TINY_BCLIBC_KEY_VEL_Z:
            return p->vz;
        default:
            return REAL_C(0.0);
        }
    }

    /* 3-point PCHIP interpolation of all BaseTrajData fields */
    TINY_BCLIBC_INLINE_FUNC void
    TINY_BCLIBC_BaseTrajData_interpolate(int32_t key, real_t key_val,
                                         const TINY_BCLIBC_BaseTrajData *p0,
                                         const TINY_BCLIBC_BaseTrajData *p1,
                                         const TINY_BCLIBC_BaseTrajData *p2,
                                         TINY_BCLIBC_BaseTrajData *out)
    {
        real_t x0 = TINY_BCLIBC_BaseTrajData_get(p0, key);
        real_t x1 = TINY_BCLIBC_BaseTrajData_get(p1, key);
        real_t x2 = TINY_BCLIBC_BaseTrajData_get(p2, key);

#define TINY_BCLIBC__INTERP_FIELD(field, k) \
    out->field = (key == (k)) ? key_val     \
                              : tiny_bclibc_interpolate3pt(key_val, x0, x1, x2, p0->field, p1->field, p2->field)

        TINY_BCLIBC__INTERP_FIELD(time, TINY_BCLIBC_KEY_TIME);
        TINY_BCLIBC__INTERP_FIELD(px, TINY_BCLIBC_KEY_POS_X);
        TINY_BCLIBC__INTERP_FIELD(py, TINY_BCLIBC_KEY_POS_Y);
        TINY_BCLIBC__INTERP_FIELD(pz, TINY_BCLIBC_KEY_POS_Z);
        TINY_BCLIBC__INTERP_FIELD(vx, TINY_BCLIBC_KEY_VEL_X);
        TINY_BCLIBC__INTERP_FIELD(vy, TINY_BCLIBC_KEY_VEL_Y);
        TINY_BCLIBC__INTERP_FIELD(vz, TINY_BCLIBC_KEY_VEL_Z);
        TINY_BCLIBC__INTERP_FIELD(mach, TINY_BCLIBC_KEY_MACH);
#undef TINY_BCLIBC__INTERP_FIELD
    }

    /* ── Full trajectory point data ──────────────────────────────────
     *  15 × real_t + int32 = 124 B (double) / 64 B (float)
     */
    typedef struct TINY_BCLIBC_TrajectoryData
    {
        real_t time;
        real_t distance_ft;
        real_t velocity_fps;
        real_t mach;
        real_t height_ft;
        real_t slant_height_ft;
        real_t drop_angle_rad;
        real_t windage_ft;
        real_t windage_angle_rad;
        real_t slant_distance_ft;
        real_t angle_rad;
        real_t density_ratio;
        real_t drag;
        real_t energy_ft_lb;
        real_t ogw_lb;
        int32_t flag;
    } TINY_BCLIBC_TrajectoryData;

    /* ── Integration request ────────────────────────────────────────── */
    typedef struct TINY_BCLIBC_TrajectoryRequest
    {
        real_t range_limit_ft;
        real_t range_step_ft;
        real_t time_step;
        int32_t filter_flags;
    } TINY_BCLIBC_TrajectoryRequest;

    /* ── TrajectoryData factory from BaseTrajData + ShotProps ───────── */
    TINY_BCLIBC_INLINE_FUNC void
    TINY_BCLIBC_TrajectoryData_from_props(const TINY_BCLIBC_ShotProps *props,
                                          const TINY_BCLIBC_BaseTrajData *base,
                                          int32_t flag,
                                          TINY_BCLIBC_TrajectoryData *out)
    {
        TINY_BCLIBC_V3dT range_v = TINY_BCLIBC_V3dT_make(base->px, base->py, base->pz);
        TINY_BCLIBC_V3dT vel_v = TINY_BCLIBC_V3dT_make(base->vx, base->vy, base->vz);

        TINY_BCLIBC_V3dT adj = TINY_BCLIBC_Coriolis_adjust_range(&props->coriolis, base->time, range_v);
        real_t spin_drift = TINY_BCLIBC_ShotProps_spin_drift(props, base->time);
        real_t velocity = TINY_BCLIBC_V3dT_mag(vel_v);

        real_t density_ratio, mach_fps;
        TINY_BCLIBC_Atmosphere_update_density_mach(&props->atmo,
                                                   props->alt0 + base->py, &density_ratio, &mach_fps);

        real_t traj_angle = TINY_BCLIBC_ATAN2(vel_v.y, vel_v.x);
        real_t la_cos = TINY_BCLIBC_COS(props->look_angle);
        real_t la_sin = TINY_BCLIBC_SIN(props->look_angle);

        out->time = base->time;
        out->flag = flag;
        out->windage_ft = adj.z + spin_drift;
        out->distance_ft = adj.x;
        out->velocity_fps = velocity;
        out->mach = (mach_fps != REAL_C(0.0)) ? velocity / mach_fps : REAL_C(0.0);
        out->height_ft = adj.y;
        out->slant_height_ft = adj.y * la_cos - adj.x * la_sin;
        out->drop_angle_rad = (adj.x != REAL_C(0.0))
                                  ? TINY_BCLIBC_ATAN2(adj.y, adj.x) - props->look_angle
                                  : REAL_C(0.0);
        out->windage_angle_rad = (adj.x != REAL_C(0.0))
                                     ? TINY_BCLIBC_ATAN2(out->windage_ft, adj.x)
                                     : REAL_C(0.0);
        out->slant_distance_ft = adj.x * la_cos + adj.y * la_sin;
        out->angle_rad = traj_angle;
        out->density_ratio = density_ratio;
        out->drag = TINY_BCLIBC_ShotProps_drag_by_mach(props, out->mach);
        out->energy_ft_lb = props->weight * velocity * velocity / REAL_C(450400.0);
        out->ogw_lb = props->weight * props->weight * velocity * velocity * velocity * REAL_C(1.5e-12);
    }

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* TINY_BCLIBC_TRAJ_DATA_H */
