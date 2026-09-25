// A bare WebAssembly build (no WASI, no Emscripten): what libc++ would otherwise pull in to be able to abort.
//
// libc++ reports a fatal error (a length_error in -fno-exceptions mode, a failed hardening check) through
// `__libcpp_verbose_abort`, whose default writes the message to stderr. That links in stdio, and the module then
// imports WASI's fd_write, fd_seek and fd_close. A trap needs nothing. Defining it here keeps the library's version
// (and the imports) out of the link.
#include <cstdlib>

#if defined(_LIBCPP_VERSION)
_LIBCPP_BEGIN_NAMESPACE_STD
[[noreturn]] void __libcpp_verbose_abort(const char *, ...) noexcept { __builtin_trap(); }
_LIBCPP_END_NAMESPACE_STD
#endif
