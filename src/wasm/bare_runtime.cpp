// A bare WebAssembly build (no WASI, no Emscripten): the pieces of the runtime that would otherwise need WASI or
// that zig's wasm32-wasi does not have. Only cmake/BclibcWasmBare.cmake compiles this file.
//
// What libc++ would otherwise pull in to be able to abort:
//
// libc++ reports a fatal error (a length_error in -fno-exceptions mode, a failed hardening check) through
// `__libcpp_verbose_abort`, whose default writes the message to stderr. That links in stdio, and the module then
// imports WASI's fd_write, fd_seek and fd_close. A trap needs nothing. Defining it here keeps the library's version
// (and the imports) out of the link.
#include <cstddef>
#include <cstdlib>

#if defined(_LIBCPP_VERSION)
_LIBCPP_BEGIN_NAMESPACE_STD
[[noreturn]] void __libcpp_verbose_abort(const char *, ...) noexcept { __builtin_trap(); }
_LIBCPP_END_NAMESPACE_STD
#endif

// Exceptions. The core throws (a solver that fails, a bad argument), and zig's wasm32-wasi has no runtime to throw
// with: it would leave `__cxa_allocate_exception` and `__cxa_throw` undefined. Here a throw is a trap, so a failed
// solve ends the call instead of being reported (and a `catch` never runs). It is a stopgap until the core reports
// errors without exceptions.
extern "C"
{
    void *__cxa_allocate_exception(std::size_t) noexcept { __builtin_trap(); }

    [[noreturn]] void __cxa_throw(void *, void *, void (*)(void *)) { __builtin_trap(); }
}
