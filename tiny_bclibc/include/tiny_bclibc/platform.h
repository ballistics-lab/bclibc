#ifndef TINY_BCLIBC_PLATFORM_H
#define TINY_BCLIBC_PLATFORM_H

#include <math.h>
#include <stdint.h>

/* ── Числовий тип ─────────────────────────────────────────────────── */
#ifdef TINY_BCLIBC_USE_FLOAT
typedef float real_t;
#define TINY_BCLIBC_SQRT  sqrtf
#define TINY_BCLIBC_FABS  fabsf
#define TINY_BCLIBC_ATAN2 atan2f
#define TINY_BCLIBC_COS   cosf
#define TINY_BCLIBC_SIN   sinf
#define TINY_BCLIBC_TAN   tanf
#define TINY_BCLIBC_POW   powf
#define TINY_BCLIBC_EXP   expf
#define REAL_C(x) x##f
#else
typedef double real_t;
#define TINY_BCLIBC_SQRT  sqrt
#define TINY_BCLIBC_FABS  fabs
#define TINY_BCLIBC_ATAN2 atan2
#define TINY_BCLIBC_COS   cos
#define TINY_BCLIBC_SIN   sin
#define TINY_BCLIBC_TAN   tan
#define TINY_BCLIBC_POW   pow
#define TINY_BCLIBC_EXP   exp
#define REAL_C(x) x
#endif

/* ── Видимість / linkage публічних функцій ──────────────────────────
 *
 *  (нічого)              → header-only: static inline
 *  TINY_BCLIBC_BUILD_SHARED  → компіляція .so/.dll: export символи
 *  TINY_BCLIBC_USE_SHARED    → споживання .so/.dll: import символи
 *
 *  Дрібні helpers (v3d, interp) — завжди TINY_BCLIBC_INLINE_FUNC.
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

/* Завжди inline — для tiny helpers (v3d, interp, утиліти) */
#define TINY_BCLIBC_INLINE_FUNC static inline

/* ── Великі внутрішні функції (engine.h) ────────────────────────────
 *  TINY_BCLIBC_INTERNAL: static + noinline.
 *
 *  Без цього -O2/-O3 inline-ує tiny_bclibc__run_rk4 (~4 KB) в кожну з
 *  5 публічних функцій → 20 KB дублікату в .so та native .mpy модулі.
 *  noinline гарантує один екземпляр незалежно від прапорів збірки.
 *
 *  Маленькі helpers (v3d, interp, atmosphere_update) лишаються
 *  TINY_BCLIBC_INLINE_FUNC — вони крихітні і профітують від inline.
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
 *  Bare-metal без RTOS → визначити TINY_BCLIBC_NO_THREAD_LOCAL перед включенням.
 *  Можна також визначити TINY_BCLIBC_THREAD_LOCAL вручну до включення цього файлу.
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
