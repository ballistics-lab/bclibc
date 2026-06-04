/**
 * cxx_stubs.cpp
 *
 * Provides:
 *   1. operator new / delete backed by MicroPython's heap allocator.
 *   2. Minimal C stdlib (strlen, memcpy, memset, memmove, memcmp, snprintf,
 *      errno) so that bclibc C++ code can build without a bare-metal libc.
 *   3. Minimal std::exception hierarchy implementations so that libstdc++.a
 *      need not be linked (it introduces .data sections that mpy_ld.py
 *      rejects).
 *   4. Newlib / libc syscall stubs (weak) for _sbrk, _write, etc.
 *   5. C++ ABI stubs (weak) for __cxa_atexit, __cxa_pure_virtual.
 *
 * Compiled with -fno-exceptions -fno-rtti -fno-builtin (Makefile).
 *
 * m_malloc/m_free are declared directly rather than via py/dynruntime.h to
 * avoid the -include build-$(ARCH)/.config.h dependency.
 */

#include <cstddef>
#include <cstdarg>

#include "bclibc/bclibc_throw.hpp"

/* natmod_stdexcept_compat.hpp is force-included first by -include in
 * CXXFLAGS, so std::exception / std::runtime_error etc. are already
 * declared when this file is compiled. */
#include <exception>   /* real std::exception base */
/* <functional> declares std::bad_function_call (in <bits/std_function.h>).
 * bad_function_call is NOT in <stdexcept>, so the compat header omits it.
 * Include <functional> here to get the declaration before we define the
 * virtual functions below. */
#include <functional>

extern "C" {
    void *m_malloc(size_t num_bytes);
    void  m_free(void *ptr);
}

// ============================================================================
// Global exception-transport state
// ============================================================================

extern "C" {
    void                     *g_bclibc_jmp_buf[5];
    bclibc::BCLIBCThrowState  g_bclibc_throw_state;
}

// ============================================================================
// operator new / delete — backed by MicroPython heap
// ============================================================================

void *operator new(size_t size) {
    return m_malloc(size);
}

void *operator new[](size_t size) {
    return m_malloc(size);
}

void operator delete(void *ptr) noexcept {
    m_free(ptr);
}

void operator delete[](void *ptr) noexcept {
    m_free(ptr);
}

void operator delete(void *ptr, size_t) noexcept {
    m_free(ptr);
}

void operator delete[](void *ptr, size_t) noexcept {
    m_free(ptr);
}

// ============================================================================
// Minimal C stdlib — compiled with -fno-builtin so GCC doesn't replace
// loop bodies with recursive calls to the same functions.
// ============================================================================

extern "C" {

size_t strlen(const char *s) {
    const char *p = s;
    while (*p) ++p;
    return (size_t)(p - s);
}

char *strncpy(char *dst, const char *src, size_t n) {
    char *d = dst;
    while (n > 0 && *src) { *d++ = *src++; --n; }
    while (n-- > 0) *d++ = '\0';
    return dst;
}

void *memcpy(void *dst, const void *src, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    while (n--) *d++ = *s++;
    return dst;
}

void *memset(void *dst, int c, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    while (n--) *d++ = (unsigned char)c;
    return dst;
}

void *memmove(void *dst, const void *src, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    if (d < s) {
        while (n--) *d++ = *s++;
    } else {
        d += n; s += n;
        while (n--) *--d = *--s;
    }
    return dst;
}

int memcmp(const void *a, const void *b, size_t n) {
    const unsigned char *p = (const unsigned char *)a;
    const unsigned char *q = (const unsigned char *)b;
    while (n--) {
        if (*p != *q) return (int)*p - (int)*q;
        ++p; ++q;
    }
    return 0;
}

/* errno: macro (*__errno()) on both arm-none-eabi and xtensa-esp-elf. */
#undef errno
static int s_errno;
int *__errno(void) { return &s_errno; }

// ============================================================================
// Minimal snprintf — handles %s, %d/%i, %u, %f/%g with optional .precision.
// Only used for building error messages (engine.cpp zero-finding errors).
// ============================================================================

static int _sn_write_uint(char *buf, int rem, unsigned long v) {
    char tmp[24]; int n = 0;
    if (v == 0) { tmp[n++] = '0'; }
    while (v) { tmp[n++] = '0' + (int)(v % 10); v /= 10; }
    int cnt = 0;
    while (n > 0 && rem > 0) { buf[cnt++] = tmp[--n]; rem--; }
    return cnt;
}

static int _sn_write_float(char *buf, int rem, double v, int prec) {
    int cnt = 0;
    if (v < 0.0) { if (rem > 0) { buf[cnt++] = '-'; rem--; } v = -v; }
    unsigned long ipart = (unsigned long)v;
    int n = _sn_write_uint(buf + cnt, rem, ipart);
    cnt += n; rem -= n;
    if (prec > 0 && rem > 0) {
        buf[cnt++] = '.'; rem--;
        double fpart = v - (double)ipart;
        for (int i = 0; i < prec && rem > 0; i++) {
            fpart *= 10.0;
            int d = (int)fpart;
            if (d < 0) d = 0;
            if (d > 9) d = 9;
            buf[cnt++] = '0' + d;
            fpart -= (double)d;
            rem--;
        }
    }
    return cnt;
}

int snprintf(char *buf, size_t sz, const char *fmt, ...) {
    if (!buf || sz == 0) return 0;
    int rem = (int)sz - 1;
    int total = 0;
    va_list ap;
    va_start(ap, fmt);
    const char *p = fmt;
    while (*p && rem > 0) {
        if (*p != '%') { buf[total++] = *p++; rem--; continue; }
        ++p; /* skip '%' */
        /* skip flags */
        while (*p == '-' || *p == '+' || *p == ' ' || *p == '#' || *p == '0') ++p;
        /* skip width */
        while (*p >= '1' && *p <= '9') ++p;
        /* optional .precision */
        int prec = 6;
        if (*p == '.') {
            ++p; prec = 0;
            while (*p >= '0' && *p <= '9') { prec = prec * 10 + (*p - '0'); ++p; }
        }
        /* skip length modifiers */
        while (*p == 'l' || *p == 'h' || *p == 'L' || *p == 'z') ++p;
        if (!*p) break;
        switch (*p++) {
        case 's': {
            const char *s = va_arg(ap, const char *);
            if (!s) s = "(null)";
            while (*s && rem > 0) { buf[total++] = *s++; rem--; }
            break;
        }
        case 'd': case 'i': {
            long v = (long)va_arg(ap, int);
            if (v < 0 && rem > 0) { buf[total++] = '-'; rem--; v = -v; }
            int n = _sn_write_uint(buf + total, rem, (unsigned long)v);
            total += n; rem -= n;
            break;
        }
        case 'u': {
            unsigned long v = (unsigned long)va_arg(ap, unsigned int);
            int n = _sn_write_uint(buf + total, rem, v);
            total += n; rem -= n;
            break;
        }
        case 'f': case 'g': case 'e': {
            double v = va_arg(ap, double);
            int n = _sn_write_float(buf + total, rem, v, prec);
            total += n; rem -= n;
            break;
        }
        case '%':
            if (rem > 0) { buf[total++] = '%'; rem--; }
            break;
        default:
            break;
        }
    }
    va_end(ap);
    buf[total] = '\0';
    return total;
}

// ============================================================================
// Newlib / libc syscall stubs (weak so toolchain-provided strong defs win)
// ============================================================================

void * __attribute__((weak)) _sbrk(int incr)
    { (void)incr; return (void *)-1; }

int __attribute__((weak)) _write(int fd, const char *buf, int len)
    { (void)fd; (void)buf; return len; }

int __attribute__((weak)) _close(int fd)
    { (void)fd; return -1; }

int __attribute__((weak)) _read(int fd, char *buf, int len)
    { (void)fd; (void)buf; (void)len; return -1; }

int __attribute__((weak)) _lseek(int fd, int off, int whence)
    { (void)fd; (void)off; (void)whence; return -1; }

int __attribute__((weak)) _isatty(int fd)
    { (void)fd; return 0; }

void __attribute__((weak)) _exit(int code)
    { (void)code; for (;;) {} }

void __attribute__((weak)) abort(void)
    { for (;;) {} }

/* C++ ABI stubs (weak so libsupc++.a can override if present) */
int  __attribute__((weak)) __cxa_atexit(void (*)(void *), void *, void *)
    { return 0; }

void __attribute__((weak)) __cxa_pure_virtual(void)
    { for (;;) {} }

} // extern "C"

// ============================================================================
// std::exception hierarchy — minimal implementations that avoid linking
// libstdc++.a (which has mutable .data sections mpy_ld.py rejects).
//
// std::exception base comes from the real <exception> header (no .data).
// Our runtime_error / logic_error etc. are declared in
// natmod_stdexcept_compat.hpp (force-included via CXXFLAGS -include).
// ============================================================================

namespace std {

/* std::exception — definitions to satisfy the virtual table without libstdc++ */
exception::~exception() noexcept {}
const char *exception::what() const noexcept { return "std::exception"; }

/* std::runtime_error */
runtime_error::~runtime_error() noexcept {}
const char *runtime_error::what() const noexcept {
    return _natmod_what ? _natmod_what : "";
}

/* std::logic_error */
logic_error::~logic_error() noexcept {}
const char *logic_error::what() const noexcept {
    return _natmod_what ? _natmod_what : "";
}

/* Subclasses — trivial destructors (base class handles everything) */
invalid_argument::~invalid_argument() noexcept {}
out_of_range::~out_of_range() noexcept {}
domain_error::~domain_error() noexcept {}
bad_function_call::~bad_function_call() noexcept {}
const char *bad_function_call::what() const noexcept { return "bad_function_call"; }

/* __throw_* stubs — replace libstdc++.a versions so they don't pull in
 * I/O or locale .data sections.  In -fno-exceptions builds these are
 * called instead of actual C++ throws, so we just halt. */
[[noreturn]] void __throw_bad_alloc()          { for (;;) {} }
[[noreturn]] void __throw_bad_function_call()   { for (;;) {} }
[[noreturn]] void __throw_length_error(const char *)   { for (;;) {} }
[[noreturn]] void __throw_out_of_range(const char *)   { for (;;) {} }
[[noreturn]] void __throw_out_of_range_fmt(const char *, ...) { for (;;) {} }
[[noreturn]] void __throw_logic_error(const char *)    { for (;;) {} }
[[noreturn]] void __throw_runtime_error(const char *)  { for (;;) {} }
[[noreturn]] void __throw_bad_cast()            { for (;;) {} }

} // namespace std
