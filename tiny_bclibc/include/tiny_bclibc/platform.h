#ifndef TINY_BCLIBC_PLATFORM_H
#define TINY_BCLIBC_PLATFORM_H

#include <math.h>
#include <stdint.h>

/* ── Numeric type ───────────────────────────────────────────────────── */
#if defined(TINY_BCLIBC_SINGLE_PRECISION)
typedef float real_t;
#define TINY_BCLIBC_SQRT sqrtf
#define TINY_BCLIBC_FABS fabsf
#define TINY_BCLIBC_ATAN2 atan2f
#define TINY_BCLIBC_COS cosf
#define TINY_BCLIBC_SIN sinf
#define TINY_BCLIBC_POW powf
#define TINY_BCLIBC_EXP expf
#define REAL_C(x) x##f
#else
typedef double real_t;
#define TINY_BCLIBC_SQRT sqrt
#define TINY_BCLIBC_FABS fabs
#define TINY_BCLIBC_ATAN2 atan2
#define TINY_BCLIBC_COS cos
#define TINY_BCLIBC_SIN sin
#define TINY_BCLIBC_POW pow
#define TINY_BCLIBC_EXP exp
#define REAL_C(x) x
#endif

/* ── Visibility / linkage of public functions ───────────────────────
 *
 *  (nothing)             → header-only: static inline
 *  TINY_BCLIBC_BUILD_SHARED  → building .so/.dll: export symbols
 *  TINY_BCLIBC_USE_SHARED    → consuming .so/.dll: import symbols
 *
 *  Small helpers (v3d, interp) — always TINY_BCLIBC_INLINE_FUNC.
 */
#if defined(TINY_BCLIBC_BUILD_SHARED)
#ifdef _WIN32
#define TINY_BCLIBC_FUNC __declspec(dllexport)
#else
#define TINY_BCLIBC_FUNC __attribute__((visibility("default")))
#endif
#elif defined(TINY_BCLIBC_USE_SHARED)
#ifdef _WIN32
#define TINY_BCLIBC_FUNC __declspec(dllimport)
#else
#define TINY_BCLIBC_FUNC extern
#endif
#else
/* Default: header-only */
#define TINY_BCLIBC_FUNC static inline
#endif

/* Always inline — for tiny helpers (v3d, interp, utilities) */
#define TINY_BCLIBC_INLINE_FUNC static inline

/* ── Large internal functions (engine.h) ────────────────────────────
 *  TINY_BCLIBC_INTERNAL: static + noinline.
 *
 *  Without this -O2/-O3 inlines tiny_bclibc__run_rk4 (~4 KB) into each
 *  of the 5 public functions → 20 KB of duplicate code in .so and native .mpy.
 *  noinline guarantees a single instance regardless of build flags.
 *
 *  Small helpers (v3d, interp, atmosphere_update) remain
 *  TINY_BCLIBC_INLINE_FUNC — they are tiny and benefit from inlining.
 */
#if defined(__GNUC__) || defined(__clang__)
#define TINY_BCLIBC_NOINLINE __attribute__((noinline))
#elif defined(_MSC_VER)
#define TINY_BCLIBC_NOINLINE __declspec(noinline)
#else
#define TINY_BCLIBC_NOINLINE
#endif
#define TINY_BCLIBC_INTERNAL TINY_BCLIBC_NOINLINE static

/* ── Thread-local storage ────────────────────────────────────────────
 *  Bare-metal without RTOS → define TINY_BCLIBC_NO_THREAD_LOCAL before including.
 *  You can also define TINY_BCLIBC_THREAD_LOCAL manually before including this file.
 */
#ifndef TINY_BCLIBC_THREAD_LOCAL
#if defined(TINY_BCLIBC_NO_THREAD_LOCAL)
#define TINY_BCLIBC_THREAD_LOCAL
#elif defined(_MSC_VER)
#define TINY_BCLIBC_THREAD_LOCAL __declspec(thread)
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#define TINY_BCLIBC_THREAD_LOCAL _Thread_local
#elif defined(__GNUC__) || defined(__clang__)
#define TINY_BCLIBC_THREAD_LOCAL __thread
#else
#define TINY_BCLIBC_THREAD_LOCAL
#endif
#endif

#endif /* TINY_BCLIBC_PLATFORM_H */
