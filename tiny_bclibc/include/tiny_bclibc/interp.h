#ifndef TINY_BCLIBC_INTERP_H
#define TINY_BCLIBC_INTERP_H

#include "platform.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* ── Cubic Hermite (Horner) ─────────────────────────────────────── */
    TINY_BCLIBC_INLINE_FUNC real_t
    tiny_bclibc_hermite(real_t x,
                    real_t xk, real_t xk1,
                    real_t yk, real_t yk1,
                    real_t mk, real_t mk1)
    {
        real_t h = xk1 - xk;
        real_t t = (x - xk) / h;
        real_t t2 = t * t;
        real_t t3 = t2 * t;
        real_t h00 = REAL_C(2.0) * t3 - REAL_C(3.0) * t2 + REAL_C(1.0);
        real_t h10 = (t - REAL_C(2.0)) * t2 + t;
        real_t h01 = -REAL_C(2.0) * t3 + REAL_C(3.0) * t2;
        real_t h11 = (t - REAL_C(1.0)) * t2;
        return h00 * yk + h * (h10 * mk + h11 * mk1) + h01 * yk1;
    }

    /* ── 3-point PCHIP slopes ────────────────────────────────────────── */
    TINY_BCLIBC_INLINE_FUNC void
    tiny_bclibc__pchip_slopes3(real_t x0, real_t y0,
                           real_t x1, real_t y1,
                           real_t x2, real_t y2,
                           real_t *m0, real_t *m1, real_t *m2)
    {
        real_t h0 = x1 - x0, h1 = x2 - x1;
        real_t d0 = (y1 - y0) / h0;
        real_t d1 = (y2 - y1) / h1;
        real_t hs = h0 + h1;

        /* m1 */
        int s0 = (d0 > REAL_C(0.0)) - (d0 < REAL_C(0.0));
        int s1 = (d1 > REAL_C(0.0)) - (d1 < REAL_C(0.0));
        if (s0 * s1 <= 0)
        {
            *m1 = REAL_C(0.0);
        }
        else
        {
            real_t w1 = REAL_C(2.0) * h1 + h0;
            real_t w2 = h1 + REAL_C(2.0) * h0;
            *m1 = (w1 + w2) / (w1 / d0 + w2 / d1);
        }
        /* m0 */
        {
            real_t r = ((REAL_C(2.0) * h0 + h1) * d0 - h0 * d1) / hs;
            if (s0 != ((r > REAL_C(0.0)) - (r < REAL_C(0.0))))
                *m0 = REAL_C(0.0);
            else
                *m0 = (TINY_BCLIBC_FABS(r) > REAL_C(3.0) * TINY_BCLIBC_FABS(d0)) ? REAL_C(3.0) * d0 : r;
        }
        /* m2 */
        {
            real_t r = ((REAL_C(2.0) * h1 + h0) * d1 - h1 * d0) / hs;
            if (s1 != ((r > REAL_C(0.0)) - (r < REAL_C(0.0))))
                *m2 = REAL_C(0.0);
            else
                *m2 = (TINY_BCLIBC_FABS(r) > REAL_C(3.0) * TINY_BCLIBC_FABS(d1)) ? REAL_C(3.0) * d1 : r;
        }
    }

    /* ── 3-point monotone PCHIP interpolation ────────────────────────── */
    TINY_BCLIBC_INLINE_FUNC real_t
    tiny_bclibc_interpolate3pt(real_t x,
                           real_t x0, real_t x1, real_t x2,
                           real_t y0, real_t y1, real_t y2)
    {
        /* sort ascending */
        real_t tx, ty;
        if (x1 < x0)
        {
            tx = x0;
            x0 = x1;
            x1 = tx;
            ty = y0;
            y0 = y1;
            y1 = ty;
        }
        if (x2 < x1)
        {
            if (x2 < x0)
            {
                tx = x0;
                x0 = x2;
                x2 = x1;
                x1 = tx;
                ty = y0;
                y0 = y2;
                y2 = y1;
                y1 = ty;
            }
            else
            {
                tx = x1;
                x1 = x2;
                x2 = tx;
                ty = y1;
                y1 = y2;
                y2 = ty;
            }
        }
        real_t m0, m1, m2;
        tiny_bclibc__pchip_slopes3(x0, y0, x1, y1, x2, y2, &m0, &m1, &m2);
        return (x <= x1)
                   ? tiny_bclibc_hermite(x, x0, x1, y0, y1, m0, m1)
                   : tiny_bclibc_hermite(x, x1, x2, y1, y2, m1, m2);
    }

    /* ── Linear 2-point interpolation ────────────────────────────────── */
    /* returns 0 on success, -1 on zero-division */
    TINY_BCLIBC_INLINE_FUNC int32_t
    tiny_bclibc_interpolate2pt(real_t x,
                           real_t x0, real_t y0,
                           real_t x1, real_t y1,
                           real_t *result)
    {
        if (x1 == x0)
            return -1;
        *result = y0 + (y1 - y0) * (x - x0) / (x1 - x0);
        return 0;
    }

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* TINY_BCLIBC_INTERP_H */
