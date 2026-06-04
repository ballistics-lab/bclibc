/**
 * natmod_stdexcept_compat.hpp
 *
 * Force-included in every C++ translation unit for natmod builds
 * (-include flag in CXXFLAGS).  Blocks the real <stdexcept> (which
 * uses std::string and pulls in libstdc++.a with mutable .data sections)
 * and replaces it with minimal const-char*-backed types.
 *
 * std::exception itself is provided by the real <exception> header
 * (no std::string, no .data) so we don't block it.
 *
 * Implementations live in cxx_stubs.cpp.
 */
#pragma once
#ifdef BCLIBC_BUILD_NATMOD

/* Block the real <stdexcept> include guard so our definitions take effect
 * even when source files do #include <stdexcept>. */
#ifndef _GLIBCXX_STDEXCEPT
#define _GLIBCXX_STDEXCEPT 1

/* Set the guard BEFORE including <string> so that any transitive include of
 * <stdexcept> from within <string> is blocked.  <string> does not use
 * std::runtime_error directly, so this ordering is safe. */
#include <exception>  /* std::exception base — no std::string, safe */
#include <string>     /* required for the const string& overloads below */

namespace std {

class runtime_error : public exception {
    const char *_natmod_what;
public:
    explicit runtime_error(const char *msg) noexcept : _natmod_what(msg) {}
    /* Allows <system_error> constructors (string temporaries) to compile.
     * Never called in natmod builds (-fno-exceptions / jmp_buf path). */
    explicit runtime_error(const std::string&) noexcept : _natmod_what(nullptr) {}
    virtual ~runtime_error() noexcept;
    virtual const char *what() const noexcept;
};

class logic_error : public exception {
    const char *_natmod_what;
public:
    explicit logic_error(const char *msg) noexcept : _natmod_what(msg) {}
    explicit logic_error(const std::string&) noexcept : _natmod_what(nullptr) {}
    virtual ~logic_error() noexcept;
    virtual const char *what() const noexcept;
};

class invalid_argument : public logic_error {
public:
    explicit invalid_argument(const char *msg) noexcept : logic_error(msg) {}
    virtual ~invalid_argument() noexcept;
};

class out_of_range : public logic_error {
public:
    explicit out_of_range(const char *msg) noexcept : logic_error(msg) {}
    virtual ~out_of_range() noexcept;
};

class domain_error : public logic_error {
public:
    explicit domain_error(const char *msg) noexcept : logic_error(msg) {}
    virtual ~domain_error() noexcept;
};

/* bad_function_call is declared in <bits/std_function.h> (via <functional>),
 * NOT in <stdexcept>.  Defining it here would cause a "redefinition" error
 * when engine.hpp includes <functional>.  Let <functional> own the declaration;
 * cxx_stubs.cpp provides the virtual-function definitions. */

/* __throw_* stubs — replace libstdc++.a versions to avoid .data pull-in.
 * Called by std::vector, std::function, etc. under -fno-exceptions.
 * Definitions in cxx_stubs.cpp. */
[[noreturn]] void __throw_bad_alloc();
[[noreturn]] void __throw_length_error(const char *);
[[noreturn]] void __throw_out_of_range(const char *);
[[noreturn]] void __throw_out_of_range_fmt(const char *, ...)
    __attribute__((__format__(__printf__, 1, 2)));
[[noreturn]] void __throw_logic_error(const char *);
[[noreturn]] void __throw_runtime_error(const char *);
[[noreturn]] void __throw_bad_cast();

} // namespace std

#endif // _GLIBCXX_STDEXCEPT
#endif // BCLIBC_BUILD_NATMOD
