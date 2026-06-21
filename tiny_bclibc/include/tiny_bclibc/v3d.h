#ifndef TINY_BCLIBC_V3D_H
#define TINY_BCLIBC_V3D_H

#include "platform.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct TINY_BCLIBC_V3dT
    {
        real_t x;
        real_t y;
        real_t z;
    } TINY_BCLIBC_V3dT;

    /* ── Construction ──────────────────────────────────────────────── */

    TINY_BCLIBC_INLINE_FUNC TINY_BCLIBC_V3dT TINY_BCLIBC_V3dT_make(real_t x, real_t y, real_t z)
    {
        TINY_BCLIBC_V3dT v;
        v.x = x;
        v.y = y;
        v.z = z;
        return v;
    }

    /* ── Arithmetic (return new vector) ────────────────────────────── */

    TINY_BCLIBC_INLINE_FUNC TINY_BCLIBC_V3dT TINY_BCLIBC_V3dT_add(TINY_BCLIBC_V3dT a, TINY_BCLIBC_V3dT b)
    {
        return TINY_BCLIBC_V3dT_make(a.x + b.x, a.y + b.y, a.z + b.z);
    }

    TINY_BCLIBC_INLINE_FUNC TINY_BCLIBC_V3dT TINY_BCLIBC_V3dT_neg(TINY_BCLIBC_V3dT v)
    {
        return TINY_BCLIBC_V3dT_make(-v.x, -v.y, -v.z);
    }

    TINY_BCLIBC_INLINE_FUNC TINY_BCLIBC_V3dT TINY_BCLIBC_V3dT_sub(TINY_BCLIBC_V3dT a, TINY_BCLIBC_V3dT b)
    {
        return TINY_BCLIBC_V3dT_make(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    TINY_BCLIBC_INLINE_FUNC TINY_BCLIBC_V3dT TINY_BCLIBC_V3dT_scale(TINY_BCLIBC_V3dT v, real_t s)
    {
        return TINY_BCLIBC_V3dT_make(v.x * s, v.y * s, v.z * s);
    }

    TINY_BCLIBC_INLINE_FUNC TINY_BCLIBC_V3dT TINY_BCLIBC_V3dT_div(TINY_BCLIBC_V3dT v, real_t s)
    {
        if (TINY_BCLIBC_FABS(s) < REAL_C(1e-10))
            return v;
        return TINY_BCLIBC_V3dT_scale(v, REAL_C(1.0) / s);
    }

    TINY_BCLIBC_INLINE_FUNC real_t TINY_BCLIBC_V3dT_dot(TINY_BCLIBC_V3dT a, TINY_BCLIBC_V3dT b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /* ── Compound assignment (modify in-place) ─────────────────────── */

    TINY_BCLIBC_INLINE_FUNC void TINY_BCLIBC_V3dT_add_assign(TINY_BCLIBC_V3dT *v, TINY_BCLIBC_V3dT other)
    {
        v->x += other.x;
        v->y += other.y;
        v->z += other.z;
    }

    TINY_BCLIBC_INLINE_FUNC void TINY_BCLIBC_V3dT_sub_assign(TINY_BCLIBC_V3dT *v, TINY_BCLIBC_V3dT other)
    {
        v->x -= other.x;
        v->y -= other.y;
        v->z -= other.z;
    }

    TINY_BCLIBC_INLINE_FUNC void TINY_BCLIBC_V3dT_scale_assign(TINY_BCLIBC_V3dT *v, real_t s)
    {
        v->x *= s;
        v->y *= s;
        v->z *= s;
    }

    TINY_BCLIBC_INLINE_FUNC void TINY_BCLIBC_V3dT_div_assign(TINY_BCLIBC_V3dT *v, real_t s)
    {
        if (TINY_BCLIBC_FABS(s) < REAL_C(1e-10))
            return;
        TINY_BCLIBC_V3dT_scale_assign(v, REAL_C(1.0) / s);
    }

    /* ── Fused operations (avoid temporaries) ──────────────────────── */

    /** v += other * s */
    TINY_BCLIBC_INLINE_FUNC void TINY_BCLIBC_V3dT_fma(TINY_BCLIBC_V3dT *v, TINY_BCLIBC_V3dT other, real_t s)
    {
        v->x += other.x * s;
        v->y += other.y * s;
        v->z += other.z * s;
    }

    /** v -= other * s */
    TINY_BCLIBC_INLINE_FUNC void TINY_BCLIBC_V3dT_fms(TINY_BCLIBC_V3dT *v, TINY_BCLIBC_V3dT other, real_t s)
    {
        v->x -= other.x * s;
        v->y -= other.y * s;
        v->z -= other.z * s;
    }

    /** v = a * sa + b * sb */
    TINY_BCLIBC_INLINE_FUNC void TINY_BCLIBC_V3dT_lc2(
        TINY_BCLIBC_V3dT *v,
        TINY_BCLIBC_V3dT a, real_t sa,
        TINY_BCLIBC_V3dT b, real_t sb)
    {
        v->x = a.x * sa + b.x * sb;
        v->y = a.y * sa + b.y * sb;
        v->z = a.z * sa + b.z * sb;
    }

    /** v = a * sa + b * sb + c * sc + d * sd */
    TINY_BCLIBC_INLINE_FUNC void TINY_BCLIBC_V3dT_lc4(
        TINY_BCLIBC_V3dT *v,
        TINY_BCLIBC_V3dT a, real_t sa,
        TINY_BCLIBC_V3dT b, real_t sb,
        TINY_BCLIBC_V3dT c, real_t sc,
        TINY_BCLIBC_V3dT d, real_t sd)
    {
        v->x = a.x * sa + b.x * sb + c.x * sc + d.x * sd;
        v->y = a.y * sa + b.y * sb + c.y * sc + d.y * sd;
        v->z = a.z * sa + b.z * sb + c.z * sc + d.z * sd;
    }

    /* ── Vector properties ─────────────────────────────────────────── */

    TINY_BCLIBC_INLINE_FUNC real_t TINY_BCLIBC_V3dT_mag_sq(TINY_BCLIBC_V3dT v)
    {
        return TINY_BCLIBC_V3dT_dot(v, v);
    }

    TINY_BCLIBC_INLINE_FUNC real_t TINY_BCLIBC_V3dT_mag(TINY_BCLIBC_V3dT v)
    {
        return TINY_BCLIBC_SQRT(TINY_BCLIBC_V3dT_mag_sq(v));
    }

    TINY_BCLIBC_INLINE_FUNC TINY_BCLIBC_V3dT TINY_BCLIBC_V3dT_norm(TINY_BCLIBC_V3dT v)
    {
        const real_t m_sq = v.x * v.x + v.y * v.y + v.z * v.z;
        if (m_sq < REAL_C(1e-20))
            return v;
        const real_t inv_mag = REAL_C(1.0) / TINY_BCLIBC_SQRT(m_sq);
        return TINY_BCLIBC_V3dT_make(v.x * inv_mag, v.y * inv_mag, v.z * inv_mag);
    }

    TINY_BCLIBC_INLINE_FUNC void TINY_BCLIBC_V3dT_normalize(TINY_BCLIBC_V3dT *v)
    {
        const real_t m_sq = v->x * v->x + v->y * v->y + v->z * v->z;
        if (m_sq < REAL_C(1e-20))
            return;
        const real_t inv_mag = REAL_C(1.0) / TINY_BCLIBC_SQRT(m_sq);
        v->x *= inv_mag;
        v->y *= inv_mag;
        v->z *= inv_mag;
    }

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* TINY_BCLIBC_V3D_H */
