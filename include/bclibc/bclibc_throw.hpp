/**
 * bclibc_throw.hpp
 *
 * Exception transport for MicroPython native modules compiled with
 * -fno-exceptions.  In a normal build BCLIBC_THROW(expr) is identical to
 * throw (expr).  In a natmod build it stores the error state in a global
 * struct and uses longjmp to unwind to the nearest ffi_call setjmp point.
 *
 * All C++ heap allocations in a natmod go through operator new → m_malloc →
 * MicroPython GC heap, so any objects orphaned by longjmp are recovered by
 * the GC on its next collection cycle.
 *
 * Usage:
 *   #include "bclibc/bclibc_throw.hpp"
 *   BCLIBC_THROW(BCLIBC_ZeroFindingError("msg", err, n, elev));
 *   BCLIBC_THROW(std::invalid_argument("bad input"));
 */

#ifndef BCLIBC_THROW_HPP
#define BCLIBC_THROW_HPP

#ifndef BCLIBC_BUILD_NATMOD

// ── Normal build: plain throw ────────────────────────────────────────────────
#define BCLIBC_THROW(expr) throw (expr)

#else // BCLIBC_BUILD_NATMOD

// ── Natmod build: __builtin_setjmp/__builtin_longjmp transport ───────────────
// Use GCC compiler builtins instead of libc setjmp/longjmp: the builtins are
// expanded inline by GCC and generate no external symbol references, which is
// required because libc.a is excluded from the natmod partial link.
// Buffer type must be void*[5] (GCC's requirement for these builtins).

#include <cstdint>
#include <cstring>

#include "bclibc/exceptions.hpp"
#include "bclibc/ffi/bclibc_ffi.h"

namespace bclibc {

struct BCLIBCThrowState {
    int32_t code;
    char    what[128];
    double  f64_0;
    double  f64_1;
    double  f64_2;
    int32_t i32_0;
};

} // namespace bclibc

extern "C" {
    extern void                      *g_bclibc_jmp_buf[5];
    extern bclibc::BCLIBCThrowState   g_bclibc_throw_state;
}

namespace bclibc {

// ── Helpers ──────────────────────────────────────────────────────────────────

inline void _bclibc_copy_what(const char *src) noexcept
{
    std::strncpy(g_bclibc_throw_state.what, src,
                 sizeof(g_bclibc_throw_state.what) - 1);
    g_bclibc_throw_state.what[sizeof(g_bclibc_throw_state.what) - 1] = '\0';
}

// ── Generic fallback (std::exception subclasses, etc.) ───────────────────────

template <typename E>
[[noreturn]] inline void bclibc_do_throw(const E &e) noexcept
{
    g_bclibc_throw_state.code  = BCLIBCFFI_ERR_GENERIC;
    g_bclibc_throw_state.f64_0 = 0.0;
    g_bclibc_throw_state.f64_1 = 0.0;
    g_bclibc_throw_state.f64_2 = 0.0;
    g_bclibc_throw_state.i32_0 = 0;
    _bclibc_copy_what(e.what());
    __builtin_longjmp(g_bclibc_jmp_buf, 1);
}

// ── bclibc-typed specialisations ─────────────────────────────────────────────

template <>
[[noreturn]] inline void bclibc_do_throw(const BCLIBC_SolverRuntimeError &e) noexcept
{
    g_bclibc_throw_state.code  = BCLIBCFFI_ERR_SOLVER_RUNTIME;
    g_bclibc_throw_state.f64_0 = 0.0;
    g_bclibc_throw_state.f64_1 = 0.0;
    g_bclibc_throw_state.f64_2 = 0.0;
    g_bclibc_throw_state.i32_0 = 0;
    _bclibc_copy_what(e.what());
    __builtin_longjmp(g_bclibc_jmp_buf, 1);
}

template <>
[[noreturn]] inline void bclibc_do_throw(const BCLIBC_OutOfRangeError &e) noexcept
{
    g_bclibc_throw_state.code  = BCLIBCFFI_ERR_OUT_OF_RANGE;
    g_bclibc_throw_state.f64_0 = e.requested_distance_ft;
    g_bclibc_throw_state.f64_1 = e.max_range_ft;
    g_bclibc_throw_state.f64_2 = e.look_angle_rad;
    g_bclibc_throw_state.i32_0 = 0;
    _bclibc_copy_what(e.what());
    __builtin_longjmp(g_bclibc_jmp_buf, 1);
}

template <>
[[noreturn]] inline void bclibc_do_throw(const BCLIBC_ZeroFindingError &e) noexcept
{
    g_bclibc_throw_state.code  = BCLIBCFFI_ERR_ZERO_FINDING;
    g_bclibc_throw_state.f64_0 = e.zero_finding_error;
    g_bclibc_throw_state.f64_1 = e.last_barrel_elevation_rad;
    g_bclibc_throw_state.f64_2 = 0.0;
    g_bclibc_throw_state.i32_0 = e.iterations_count;
    _bclibc_copy_what(e.what());
    __builtin_longjmp(g_bclibc_jmp_buf, 1);
}

template <>
[[noreturn]] inline void bclibc_do_throw(const BCLIBC_InterceptionError &e) noexcept
{
    g_bclibc_throw_state.code  = BCLIBCFFI_ERR_INTERCEPTION;
    g_bclibc_throw_state.f64_0 = 0.0;
    g_bclibc_throw_state.f64_1 = 0.0;
    g_bclibc_throw_state.f64_2 = 0.0;
    g_bclibc_throw_state.i32_0 = 0;
    _bclibc_copy_what(e.what());
    __builtin_longjmp(g_bclibc_jmp_buf, 1);
}

} // namespace bclibc

#define BCLIBC_THROW(expr) bclibc::bclibc_do_throw(expr)

#endif // BCLIBC_BUILD_NATMOD
#endif // BCLIBC_THROW_HPP
