// A bare WebAssembly build (no WASI, no Emscripten): the pieces of the runtime that would otherwise need WASI, or that
// the toolchain's wasm32-wasi does not have. Only cmake/BclibcWasmBare.cmake compiles this file.

#include <cstddef>
#include <cstdlib>

// libc++ reports a fatal error (a length_error in -fno-exceptions mode, a failed hardening check) through
// `__libcpp_verbose_abort`, whose default writes the message to stderr. That links in stdio, and the module then
// imports WASI's fd_write, fd_seek and fd_close. A trap needs nothing. Defining it here keeps the library's version
// (and the imports) out of the link.
#if defined(_LIBCPP_VERSION)
_LIBCPP_BEGIN_NAMESPACE_STD
[[noreturn]] void __libcpp_verbose_abort(const char *, ...) noexcept { __builtin_trap(); }
_LIBCPP_END_NAMESPACE_STD
#endif

// The core is built with -fno-exceptions (cmake/BclibcWasmBare.cmake) and never throws: every fallible call returns
// a Result instead. Neither toolchain's exception runtime (libunwind, libc++abi's unwinder) is linked in, so if
// something below the core ever still reaches `__cxa_throw` -- a `new` that libc++ itself throws bad_alloc from, on
// wasi-sdk, whose libc++abi was built with exceptions available -- it traps instead of unwinding into nothing.
// `cmake/check_no_imports.cmake` fails the build if a new version of a toolchain finds another way to pull in WASI.
extern "C"
{
    void *__cxa_allocate_exception(std::size_t) noexcept { __builtin_trap(); }

    [[noreturn]] void __cxa_throw(void *, void *, void (*)(void *)) { __builtin_trap(); }
}

#if defined(BCLIBC_WASI_SDK)

// wasi-sdk's libc (unlike zig's, which already defines these itself) pulls these in independently of exceptions:
// its buffered stdio (used by strtol/std::stoull's end-of-input path, which takes a file lock and, with it, futex
// and WASI's clock_time_get) and its default stack protector (whose canary is seeded from WASI's random_get).
// Nothing here is ever read from a file or times anything, so each gets an inert stand-in instead of the WASI
// import it would otherwise pull in.
extern "C"
{
    unsigned long __stdio_write(void *, const unsigned char *, unsigned long) { return 0; }
    long long __stdio_seek(void *, long long, int) { return -1; }
    int __stdio_close(void *) { return 0; }
    void __stdio_exit_needed() {}
    int __clock_gettime(int, void *) { return -1; }
    int clock_gettime(int, void *) { return -1; }
    unsigned long __stack_chk_guard = 0x2f1b6c4dUL;
    [[noreturn]] void __stack_chk_fail() { __builtin_trap(); }
}

#endif
