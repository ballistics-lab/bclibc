/**
 * bclibc_mp.c
 *
 * Shared Python binding layer for bclibc, supporting two build modes:
 *
 *   BCLIBC_BUILD_NATMOD  (defined by natmod/Makefile)
 *     → MicroPython native .mpy module via dynruntime.h + mpy_init()
 *
 *   (flag absent)
 *     → Static firmware module for MicroPython or CircuitPython firmware
 *       builds via standard py/obj.h headers + MP_REGISTER_MODULE
 *
 * Python API
 * ----------
 *
 *   import bclibc
 *
 *   bclibc.version()                              -> str
 *   bclibc.integrate(shot, request)               -> (list[tuple], int)
 *   bclibc.find_zero_angle(shot, distance_ft)     -> float
 *   bclibc.find_apex(shot)                        -> tuple
 *   bclibc.find_max_range(shot, lo_deg, hi_deg)   -> (float, float)
 *   bclibc.integrate_at(shot, key, target)        -> (tuple, tuple)
 *   bclibc.get_correction(distance_ft, offset_ft) -> float
 *   bclibc.calculate_energy(weight_gr, vel_fps)   -> float
 *   bclibc.calculate_ogw(weight_gr, vel_fps)      -> float
 *
 * Trajectory tuple layout (16 elements, use T_* index constants):
 *   0  time             1  distance_ft       2  velocity_fps
 *   3  mach             4  height_ft         5  slant_height_ft
 *   6  drop_angle_rad   7  windage_ft        8  windage_angle_rad
 *   9  slant_distance_ft 10 angle_rad        11 density_ratio
 *   12 drag             13 energy_ft_lb      14 ogw_lb
 *   15 flag
 *
 * Raw-trajectory tuple layout (8 elements):
 *   0  time  1  px  2  py  3  pz  4  vx  5  vy  6  vz  7  mach
 *
 * shot dict keys (all optional – sensible defaults are used):
 *   bc, weight_grain, diameter_inch, length_inch, muzzle_velocity_fps,
 *   sight_height_ft, twist_inch, temp_c, pressure_hpa, altitude_ft,
 *   humidity, look_angle_rad, barrel_elevation_rad, barrel_azimuth_rad,
 *   cant_angle_rad, latitude_deg (NaN → no Coriolis), azimuth_deg,
 *   method (0=RK4 / 1=Euler), config (sub-dict, see BCLIBCFFI_Config),
 *   mach_data (list/tuple of floats), cd_data (list/tuple of floats),
 *   winds (list of dicts: velocity_fps, direction_from_rad,
 *                         until_distance_ft, max_distance_ft).
 */

#ifdef BCLIBC_BUILD_NATMOD
/* natmod: dynruntime.h provides the full API via mp_fun_table pointers */
#include "py/dynruntime.h"
#else
/* firmware: standard MicroPython / CircuitPython C API */
#include "py/obj.h"
#include "py/runtime.h"
#include "py/misc.h"
#include "py/nlr.h"
#include "py/objlist.h"
#include "py/objtuple.h"
#endif

#include "bclibc/ffi/bclibc_ffi.h"
#include <string.h>

/* ============================================================================
 * NaN helper
 * ========================================================================= */

static inline double _bclibc_nan(void) {
#ifdef __GNUC__
    return __builtin_nan("");
#else
    /* Portable trick: 0/0 for doubles triggers UB in C, but most compilers
     * produce NaN here.  The GCC path covers every toolchain used for MCU
     * cross-compilation (arm-none-eabi, xtensa-esp, riscv32). */
    volatile double z = 0.0;
    return z / z;
#endif
}

/* ============================================================================
 * Dict / subscriptable accessor helpers
 *
 * All use NLR to catch KeyError so missing keys return the caller's default.
 * ========================================================================= */

static mp_float_t _safe_float(mp_obj_t obj, const char *key, mp_float_t def) {
    nlr_buf_t nlr;
    mp_float_t result = def;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t k = mp_obj_new_str(key, (size_t)strlen(key));
        mp_obj_t v = mp_obj_subscr(obj, k, MP_OBJ_SENTINEL);
        if (v != mp_const_none) {
            result = mp_obj_get_float(v);
        }
        nlr_pop();
    }
    /* If exception fired, nlr_buf_t already popped – just return def. */
    return result;
}

static mp_int_t _safe_int(mp_obj_t obj, const char *key, mp_int_t def) {
    nlr_buf_t nlr;
    mp_int_t result = def;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t k = mp_obj_new_str(key, (size_t)strlen(key));
        mp_obj_t v = mp_obj_subscr(obj, k, MP_OBJ_SENTINEL);
        if (v != mp_const_none) {
            result = mp_obj_get_int(v);
        }
        nlr_pop();
    }
    return result;
}

static mp_obj_t _safe_obj(mp_obj_t obj, const char *key, mp_obj_t def) {
    nlr_buf_t nlr;
    mp_obj_t result = def;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t k = mp_obj_new_str(key, (size_t)strlen(key));
        mp_obj_t v = mp_obj_subscr(obj, k, MP_OBJ_SENTINEL);
        if (v != MP_OBJ_NULL) {
            result = v;
        }
        nlr_pop();
    }
    return result;
}

/* Convenience shorthands */
#define SF(d, k, def)  ((double)_safe_float((d), (k), (mp_float_t)(def)))
#define SI(d, k, def)  ((int32_t)_safe_int((d), (k), (mp_int_t)(def)))
#define SO(d, k, def)  _safe_obj((d), (k), (def))

/* ============================================================================
 * BCLIBCFFI_Config with defaults
 * ========================================================================= */

static void fill_config_defaults(BCLIBCFFI_Config *c) {
    c->cStepMultiplier      = 1.0;
    c->cZeroFindingAccuracy = 0.001;
    c->cMinimumVelocity     = 50.0;
    c->cMaximumDrop         = 15000.0;
    c->cMaxIterations       = 100;
    c->cGravityConstant     = -32.17405;
    c->cMinimumAltitude     = -1000.0;
}

static void fill_config(mp_obj_t cfg, BCLIBCFFI_Config *c) {
    fill_config_defaults(c);
    if (cfg == mp_const_none) {
        return;
    }
    c->cStepMultiplier      = SF(cfg, "cStepMultiplier",      c->cStepMultiplier);
    c->cZeroFindingAccuracy = SF(cfg, "cZeroFindingAccuracy", c->cZeroFindingAccuracy);
    c->cMinimumVelocity     = SF(cfg, "cMinimumVelocity",     c->cMinimumVelocity);
    c->cMaximumDrop         = SF(cfg, "cMaximumDrop",         c->cMaximumDrop);
    c->cMaxIterations       = SI(cfg, "cMaxIterations",       (mp_int_t)c->cMaxIterations);
    c->cGravityConstant     = SF(cfg, "cGravityConstant",     c->cGravityConstant);
    c->cMinimumAltitude     = SF(cfg, "cMinimumAltitude",     c->cMinimumAltitude);
}

/* ============================================================================
 * Shot holder: owns heap-allocated drag-table and wind arrays
 * ========================================================================= */

typedef struct {
    BCLIBCFFI_Shot  shot;
    double         *mach_buf;
    double         *cd_buf;
    BCLIBCFFI_Wind *wind_buf;
} ShotHolder;

static void shot_holder_free(ShotHolder *h) {
    if (h->mach_buf) { m_free(h->mach_buf); h->mach_buf = NULL; }
    if (h->cd_buf)   { m_free(h->cd_buf);   h->cd_buf   = NULL; }
    if (h->wind_buf) { m_free(h->wind_buf); h->wind_buf = NULL; }
}

/* Parse a Python shot dict into a ShotHolder.
 * Raises a Python exception on any conversion error (missing required arrays,
 * type mismatches, etc.).  The caller must call shot_holder_free() after use,
 * even if this function throws. */
static void parse_shot(mp_obj_t d, ShotHolder *h) {
    h->mach_buf = NULL;
    h->cd_buf   = NULL;
    h->wind_buf = NULL;

    BCLIBCFFI_Shot *s = &h->shot;

    /* Scalar fields */
    s->bc                   = SF(d, "bc",                   0.5);
    s->weight_grain         = SF(d, "weight_grain",         168.0);
    s->diameter_inch        = SF(d, "diameter_inch",        0.308);
    s->length_inch          = SF(d, "length_inch",          1.2);
    s->muzzle_velocity_fps  = SF(d, "muzzle_velocity_fps",  2650.0);
    s->sight_height_ft      = SF(d, "sight_height_ft",      0.15);
    s->twist_inch           = SF(d, "twist_inch",           11.0);
    s->temp_c               = SF(d, "temp_c",               15.0);
    s->pressure_hpa         = SF(d, "pressure_hpa",         1013.25);
    s->altitude_ft          = SF(d, "altitude_ft",          0.0);
    s->humidity             = SF(d, "humidity",             0.5);
    s->look_angle_rad       = SF(d, "look_angle_rad",       0.0);
    s->barrel_elevation_rad = SF(d, "barrel_elevation_rad", 0.0);
    s->barrel_azimuth_rad   = SF(d, "barrel_azimuth_rad",   0.0);
    s->cant_angle_rad       = SF(d, "cant_angle_rad",       0.0);
    /* NaN disables Coriolis / enables flat-fire-only */
    s->latitude_deg         = SF(d, "latitude_deg",  _bclibc_nan());
    s->azimuth_deg          = SF(d, "azimuth_deg",   _bclibc_nan());
    s->method = (BCLIBCFFI_IntegrationMethod)SI(d, "method", (mp_int_t)BCLIBCFFI_INTEGRATION_RK4);

    /* Config sub-dict */
    fill_config(SO(d, "config", mp_const_none), &s->config);

    /* Drag table – required arrays */
    mp_obj_t mach_list = mp_obj_subscr(d, mp_obj_new_str("mach_data", 9), MP_OBJ_SENTINEL);
    mp_obj_t cd_list   = mp_obj_subscr(d, mp_obj_new_str("cd_data",   7), MP_OBJ_SENTINEL);
    size_t n = (size_t)mp_obj_get_int(mp_obj_len(mach_list));

    h->mach_buf = (double *)m_malloc(n * sizeof(double));
    h->cd_buf   = (double *)m_malloc(n * sizeof(double));

    for (size_t i = 0; i < n; i++) {
        mp_obj_t idx = mp_obj_new_int((mp_int_t)i);
        h->mach_buf[i] = (double)mp_obj_get_float(
            mp_obj_subscr(mach_list, idx, MP_OBJ_SENTINEL));
        h->cd_buf[i] = (double)mp_obj_get_float(
            mp_obj_subscr(cd_list, idx, MP_OBJ_SENTINEL));
    }

    s->mach_data       = h->mach_buf;
    s->cd_data         = h->cd_buf;
    s->drag_table_size = (int32_t)n;

    /* Winds – optional list */
    mp_obj_t empty_tuple = mp_obj_new_tuple(0, NULL);
    mp_obj_t winds_obj   = SO(d, "winds", empty_tuple);
    size_t nw = (size_t)mp_obj_get_int(mp_obj_len(winds_obj));

    if (nw > 0) {
        h->wind_buf = (BCLIBCFFI_Wind *)m_malloc(nw * sizeof(BCLIBCFFI_Wind));
        for (size_t i = 0; i < nw; i++) {
            mp_obj_t w = mp_obj_subscr(winds_obj, mp_obj_new_int((mp_int_t)i), MP_OBJ_SENTINEL);
            h->wind_buf[i].velocity_fps       = SF(w, "velocity_fps",       0.0);
            h->wind_buf[i].direction_from_rad = SF(w, "direction_from_rad", 0.0);
            h->wind_buf[i].until_distance_ft  = SF(w, "until_distance_ft",  1e16);
            h->wind_buf[i].max_distance_ft    = SF(w, "max_distance_ft",    1e16);
        }
    }

    s->winds      = h->wind_buf;
    s->wind_count = (int32_t)nw;
}

/* ============================================================================
 * Output helpers
 * ========================================================================= */

/* TrajectoryData → 16-element Python tuple */
static mp_obj_t traj_to_tuple(const BCLIBCFFI_TrajectoryData *t) {
    mp_obj_t items[16] = {
        mp_obj_new_float((mp_float_t)t->time),
        mp_obj_new_float((mp_float_t)t->distance_ft),
        mp_obj_new_float((mp_float_t)t->velocity_fps),
        mp_obj_new_float((mp_float_t)t->mach),
        mp_obj_new_float((mp_float_t)t->height_ft),
        mp_obj_new_float((mp_float_t)t->slant_height_ft),
        mp_obj_new_float((mp_float_t)t->drop_angle_rad),
        mp_obj_new_float((mp_float_t)t->windage_ft),
        mp_obj_new_float((mp_float_t)t->windage_angle_rad),
        mp_obj_new_float((mp_float_t)t->slant_distance_ft),
        mp_obj_new_float((mp_float_t)t->angle_rad),
        mp_obj_new_float((mp_float_t)t->density_ratio),
        mp_obj_new_float((mp_float_t)t->drag),
        mp_obj_new_float((mp_float_t)t->energy_ft_lb),
        mp_obj_new_float((mp_float_t)t->ogw_lb),
        mp_obj_new_int((mp_int_t)t->flag),
    };
    return mp_obj_new_tuple(16, items);
}

/* BaseTrajData → 8-element Python tuple */
static mp_obj_t base_traj_to_tuple(const BCLIBCFFI_BaseTrajData *t) {
    mp_obj_t items[8] = {
        mp_obj_new_float((mp_float_t)t->time),
        mp_obj_new_float((mp_float_t)t->px),
        mp_obj_new_float((mp_float_t)t->py),
        mp_obj_new_float((mp_float_t)t->pz),
        mp_obj_new_float((mp_float_t)t->vx),
        mp_obj_new_float((mp_float_t)t->vy),
        mp_obj_new_float((mp_float_t)t->vz),
        mp_obj_new_float((mp_float_t)t->mach),
    };
    return mp_obj_new_tuple(8, items);
}

/* Raise RuntimeError from a BCLIBCFFI_Error */
static NORETURN void raise_bclibc_error(const BCLIBCFFI_Error *err) {
    mp_raise_msg(&mp_type_RuntimeError,
                 mp_obj_new_str(err->message, (size_t)strlen(err->message)));
}

/* ============================================================================
 * Module functions
 * ========================================================================= */

/* version() -> str */
STATIC mp_obj_t mp_bclibc_version(void) {
    const char *v = BCLIBCFFI_get_version();
    return mp_obj_new_str(v, (size_t)strlen(v));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_bclibc_version_obj, mp_bclibc_version);

/* find_zero_angle(shot, distance_ft) -> float */
STATIC mp_obj_t mp_bclibc_find_zero_angle(mp_obj_t shot_arg, mp_obj_t dist_arg) {
    ShotHolder h;
    parse_shot(shot_arg, &h);

    double dist_ft   = (double)mp_obj_get_float(dist_arg);
    double out_angle = 0.0;
    BCLIBCFFI_Error err;
    int32_t rc = BCLIBCFFI_find_zero_angle_shot(&h.shot, dist_ft, &out_angle, &err);
    shot_holder_free(&h);

    if (rc != BCLIBCFFI_OK) {
        raise_bclibc_error(&err);
    }
    return mp_obj_new_float((mp_float_t)out_angle);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_bclibc_find_zero_angle_obj, mp_bclibc_find_zero_angle);

/* find_apex(shot) -> traj_tuple */
STATIC mp_obj_t mp_bclibc_find_apex(mp_obj_t shot_arg) {
    ShotHolder h;
    parse_shot(shot_arg, &h);

    BCLIBCFFI_TrajectoryData out;
    BCLIBCFFI_Error err;
    int32_t rc = BCLIBCFFI_find_apex_shot(&h.shot, &out, &err);
    shot_holder_free(&h);

    if (rc != BCLIBCFFI_OK) {
        raise_bclibc_error(&err);
    }
    return traj_to_tuple(&out);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_bclibc_find_apex_obj, mp_bclibc_find_apex);

/* find_max_range(shot, low_angle_deg, high_angle_deg) -> (max_range_ft, angle_at_max_rad) */
STATIC mp_obj_t mp_bclibc_find_max_range(mp_obj_t shot_arg, mp_obj_t low_arg, mp_obj_t high_arg) {
    ShotHolder h;
    parse_shot(shot_arg, &h);

    double low  = (double)mp_obj_get_float(low_arg);
    double high = (double)mp_obj_get_float(high_arg);
    BCLIBCFFI_MaxRangeResult out;
    BCLIBCFFI_Error err;
    int32_t rc = BCLIBCFFI_find_max_range_shot(&h.shot, low, high, &out, &err);
    shot_holder_free(&h);

    if (rc != BCLIBCFFI_OK) {
        raise_bclibc_error(&err);
    }

    mp_obj_t items[2] = {
        mp_obj_new_float((mp_float_t)out.max_range_ft),
        mp_obj_new_float((mp_float_t)out.angle_at_max_rad),
    };
    return mp_obj_new_tuple(2, items);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(mp_bclibc_find_max_range_obj, mp_bclibc_find_max_range);

/* integrate(shot, request) -> (list[traj_tuple], termination_reason) */
STATIC mp_obj_t mp_bclibc_integrate(mp_obj_t shot_arg, mp_obj_t req_arg) {
    ShotHolder h;
    parse_shot(shot_arg, &h);

    BCLIBCFFI_TrajectoryRequest req;
    req.range_limit_ft = SF(req_arg, "range_limit_ft", 3000.0);
    req.range_step_ft  = SF(req_arg, "range_step_ft",  100.0);
    req.time_step      = SF(req_arg, "time_step",       0.0);
    req.filter_flags   = SI(req_arg, "filter_flags",   (mp_int_t)BCLIBCFFI_TRAJ_FLAG_RANGE);

    BCLIBCFFI_TrajectoryData *records = NULL;
    int32_t count  = 0;
    int32_t reason = 0;
    BCLIBCFFI_Error err;

    int32_t rc = BCLIBCFFI_integrate_shot(&h.shot, &req, &records, &count, &reason, &err);
    shot_holder_free(&h);

    if (rc != BCLIBCFFI_OK) {
        raise_bclibc_error(&err);
    }

    /* Convert to Python list of tuples before freeing the C array */
    mp_obj_t list = mp_obj_new_list(0, NULL);
    for (int32_t i = 0; i < count; i++) {
        mp_obj_list_append(list, traj_to_tuple(&records[i]));
    }
    BCLIBCFFI_free_trajectory(records);

    mp_obj_t result[2] = { list, mp_obj_new_int((mp_int_t)reason) };
    return mp_obj_new_tuple(2, result);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_bclibc_integrate_obj, mp_bclibc_integrate);

/* integrate_at(shot, key, target_value) -> (raw_tuple, full_tuple) */
STATIC mp_obj_t mp_bclibc_integrate_at(mp_obj_t shot_arg, mp_obj_t key_arg, mp_obj_t target_arg) {
    ShotHolder h;
    parse_shot(shot_arg, &h);

    int32_t key    = (int32_t)mp_obj_get_int(key_arg);
    double  target = (double)mp_obj_get_float(target_arg);
    BCLIBCFFI_Interception out;
    BCLIBCFFI_Error err;
    int32_t rc = BCLIBCFFI_integrate_at_shot(&h.shot, key, target, &out, &err);
    shot_holder_free(&h);

    if (rc != BCLIBCFFI_OK) {
        raise_bclibc_error(&err);
    }

    mp_obj_t result[2] = {
        base_traj_to_tuple(&out.raw_data),
        traj_to_tuple(&out.full_data),
    };
    return mp_obj_new_tuple(2, result);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(mp_bclibc_integrate_at_obj, mp_bclibc_integrate_at);

/* get_correction(distance_ft, offset_ft) -> float */
STATIC mp_obj_t mp_bclibc_get_correction(mp_obj_t dist_arg, mp_obj_t offset_arg) {
    double dist   = (double)mp_obj_get_float(dist_arg);
    double offset = (double)mp_obj_get_float(offset_arg);
    return mp_obj_new_float((mp_float_t)BCLIBCFFI_get_correction(dist, offset));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_bclibc_get_correction_obj, mp_bclibc_get_correction);

/* calculate_energy(weight_grain, velocity_fps) -> float */
STATIC mp_obj_t mp_bclibc_calculate_energy(mp_obj_t wt_arg, mp_obj_t vel_arg) {
    return mp_obj_new_float((mp_float_t)BCLIBCFFI_calculate_energy(
        (double)mp_obj_get_float(wt_arg),
        (double)mp_obj_get_float(vel_arg)));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_bclibc_calculate_energy_obj, mp_bclibc_calculate_energy);

/* calculate_ogw(weight_grain, velocity_fps) -> float */
STATIC mp_obj_t mp_bclibc_calculate_ogw(mp_obj_t wt_arg, mp_obj_t vel_arg) {
    return mp_obj_new_float((mp_float_t)BCLIBCFFI_calculate_ogw(
        (double)mp_obj_get_float(wt_arg),
        (double)mp_obj_get_float(vel_arg)));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_bclibc_calculate_ogw_obj, mp_bclibc_calculate_ogw);

/* ============================================================================
 * Module entry point
 * ========================================================================= */

mp_obj_t mpy_init(mp_obj_fun_bc_t *self, size_t n_args, size_t n_kw, mp_obj_t *args) {
    MP_DYNRUNTIME_INIT_ENTRY

    /* --- Functions --- */
    mp_store_global(MP_QSTR_version,          MP_OBJ_FROM_PTR(&mp_bclibc_version_obj));
    mp_store_global(MP_QSTR_integrate,        MP_OBJ_FROM_PTR(&mp_bclibc_integrate_obj));
    mp_store_global(MP_QSTR_find_zero_angle,  MP_OBJ_FROM_PTR(&mp_bclibc_find_zero_angle_obj));
    mp_store_global(MP_QSTR_find_apex,        MP_OBJ_FROM_PTR(&mp_bclibc_find_apex_obj));
    mp_store_global(MP_QSTR_find_max_range,   MP_OBJ_FROM_PTR(&mp_bclibc_find_max_range_obj));
    mp_store_global(MP_QSTR_integrate_at,     MP_OBJ_FROM_PTR(&mp_bclibc_integrate_at_obj));
    mp_store_global(MP_QSTR_get_correction,   MP_OBJ_FROM_PTR(&mp_bclibc_get_correction_obj));
    mp_store_global(MP_QSTR_calculate_energy, MP_OBJ_FROM_PTR(&mp_bclibc_calculate_energy_obj));
    mp_store_global(MP_QSTR_calculate_ogw,    MP_OBJ_FROM_PTR(&mp_bclibc_calculate_ogw_obj));

    /* --- TrajFlag constants --- */
    mp_store_global(MP_QSTR_TRAJ_FLAG_NONE,       MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_TRAJ_FLAG_NONE));
    mp_store_global(MP_QSTR_TRAJ_FLAG_ZERO_UP,    MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_TRAJ_FLAG_ZERO_UP));
    mp_store_global(MP_QSTR_TRAJ_FLAG_ZERO_DOWN,  MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_TRAJ_FLAG_ZERO_DOWN));
    mp_store_global(MP_QSTR_TRAJ_FLAG_ZERO,       MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_TRAJ_FLAG_ZERO));
    mp_store_global(MP_QSTR_TRAJ_FLAG_MACH,       MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_TRAJ_FLAG_MACH));
    mp_store_global(MP_QSTR_TRAJ_FLAG_RANGE,      MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_TRAJ_FLAG_RANGE));
    mp_store_global(MP_QSTR_TRAJ_FLAG_APEX,       MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_TRAJ_FLAG_APEX));
    mp_store_global(MP_QSTR_TRAJ_FLAG_ALL,        MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_TRAJ_FLAG_ALL));
    mp_store_global(MP_QSTR_TRAJ_FLAG_MRT,        MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_TRAJ_FLAG_MRT));

    /* --- Integration method constants --- */
    mp_store_global(MP_QSTR_METHOD_RK4,           MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_INTEGRATION_RK4));
    mp_store_global(MP_QSTR_METHOD_EULER,         MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_INTEGRATION_EULER));

    /* --- Interp key constants --- */
    mp_store_global(MP_QSTR_INTERP_TIME,          MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_INTERP_KEY_TIME));
    mp_store_global(MP_QSTR_INTERP_MACH,          MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_INTERP_KEY_MACH));
    mp_store_global(MP_QSTR_INTERP_POS_X,         MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_INTERP_KEY_POS_X));
    mp_store_global(MP_QSTR_INTERP_POS_Y,         MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_INTERP_KEY_POS_Y));
    mp_store_global(MP_QSTR_INTERP_POS_Z,         MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_INTERP_KEY_POS_Z));
    mp_store_global(MP_QSTR_INTERP_VEL_X,         MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_INTERP_KEY_VEL_X));
    mp_store_global(MP_QSTR_INTERP_VEL_Y,         MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_INTERP_KEY_VEL_Y));
    mp_store_global(MP_QSTR_INTERP_VEL_Z,         MP_OBJ_NEW_SMALL_INT(BCLIBCFFI_INTERP_KEY_VEL_Z));

    /* --- Trajectory tuple field index constants --- */
    mp_store_global(MP_QSTR_T_TIME,           MP_OBJ_NEW_SMALL_INT(0));
    mp_store_global(MP_QSTR_T_DISTANCE,       MP_OBJ_NEW_SMALL_INT(1));
    mp_store_global(MP_QSTR_T_VELOCITY,       MP_OBJ_NEW_SMALL_INT(2));
    mp_store_global(MP_QSTR_T_MACH,           MP_OBJ_NEW_SMALL_INT(3));
    mp_store_global(MP_QSTR_T_HEIGHT,         MP_OBJ_NEW_SMALL_INT(4));
    mp_store_global(MP_QSTR_T_SLANT_HEIGHT,   MP_OBJ_NEW_SMALL_INT(5));
    mp_store_global(MP_QSTR_T_DROP_ANGLE,     MP_OBJ_NEW_SMALL_INT(6));
    mp_store_global(MP_QSTR_T_WINDAGE,        MP_OBJ_NEW_SMALL_INT(7));
    mp_store_global(MP_QSTR_T_WINDAGE_ANGLE,  MP_OBJ_NEW_SMALL_INT(8));
    mp_store_global(MP_QSTR_T_SLANT_DISTANCE, MP_OBJ_NEW_SMALL_INT(9));
    mp_store_global(MP_QSTR_T_ANGLE,          MP_OBJ_NEW_SMALL_INT(10));
    mp_store_global(MP_QSTR_T_DENSITY_RATIO,  MP_OBJ_NEW_SMALL_INT(11));
    mp_store_global(MP_QSTR_T_DRAG,           MP_OBJ_NEW_SMALL_INT(12));
    mp_store_global(MP_QSTR_T_ENERGY,         MP_OBJ_NEW_SMALL_INT(13));
    mp_store_global(MP_QSTR_T_OGW,            MP_OBJ_NEW_SMALL_INT(14));
    mp_store_global(MP_QSTR_T_FLAG,           MP_OBJ_NEW_SMALL_INT(15));

    MP_DYNRUNTIME_INIT_EXIT
}
