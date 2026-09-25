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

#if defined(BCLIBC_WASM_EXCEPTIONS)

// Real C++ exceptions (wasi-sdk: libc++abi and libunwind for WebAssembly exceptions). What they pull in from wasi-libc
// would make the module import WASI, so each culprit gets an inert stand-in here: nothing is ever printed, read from a
// file, locked or timed in this module, the code that does it (libunwind's debug logging, libc++abi's fatal-error
// path, the emergency allocator for exceptions) is never on the way of a working call. `cmake/check_no_imports.cmake`
// fails the build if a new version of the toolchain finds another way in.
extern "C"
{
    // libc++abi's fatal-error path (std::terminate, an exception that nobody catches) prints through it.
    [[noreturn]] void __abort_message(const char *, ...) { __builtin_trap(); }

    // libunwind's debug logging calls getenv, fprintf and fflush on stderr, which link in stdio and the environment:
    // fd_write, fd_seek, fd_close, environ_get, proc_exit.
    char *getenv(const char *) { return nullptr; }
    int fflush(void *) { return 0; }
    int fprintf(void *, const char *, ...) { return 0; }
    unsigned long __stdio_write(void *, const unsigned char *, unsigned long) { return 0; }
    long long __stdio_seek(void *, long long, int) { return -1; }
    int __stdio_close(void *) { return 0; }

    // strtol (libc++'s std::stoull helper) reads through stdio's buffered reader, whose end-of-input path takes a file
    // lock, and with it futex and WASI's clock_time_get. Nothing here is ever read from a file.
    void __stdio_exit_needed() {}
    int __clock_gettime(int, void *) { return -1; }
    int clock_gettime(int, void *) { return -1; }

    // libc++abi's emergency exception allocator takes a pthread mutex; with one thread there is nothing to lock, and
    // wasi-libc's mutex would pull in futex and the clock.
    int pthread_mutex_lock(void *) { return 0; }
    int pthread_mutex_trylock(void *) { return 0; }
    int pthread_mutex_unlock(void *) { return 0; }

    // wasi-libc's stack protector seeds its canary from WASI's random_get.
    unsigned long __stack_chk_guard = 0x2f1b6c4dUL;
    [[noreturn]] void __stack_chk_fail() { __builtin_trap(); }
}

#else

// No exception runtime (zig's wasm32-wasi has none): `__cxa_allocate_exception` and `__cxa_throw` would be left
// undefined. Here a throw is a trap, so a failed solve ends the call instead of being reported (and no `catch` runs).
extern "C"
{
    void *__cxa_allocate_exception(std::size_t) noexcept { __builtin_trap(); }

    [[noreturn]] void __cxa_throw(void *, void *, void (*)(void *)) { __builtin_trap(); }
}

#endif
