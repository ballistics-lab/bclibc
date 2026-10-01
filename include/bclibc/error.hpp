#ifndef BCLIBC_ERROR_HPP
#define BCLIBC_ERROR_HPP

#include "bclibc/result.hpp"

namespace bclibc
{
    struct BCLIBC_LogicError
    {
        const char *message = "Logic error";

        constexpr BCLIBC_LogicError() noexcept = default;
        constexpr explicit BCLIBC_LogicError(const char *message) noexcept
            : message(message != nullptr ? message : "Logic error") {}

        constexpr BCLIBC_LogicError(const BCLIBC_LogicError &) noexcept = default;
        constexpr BCLIBC_LogicError &operator=(const BCLIBC_LogicError &) noexcept = default;

        constexpr const char *what() const noexcept { return message; }
    };

    /** Invalid numeric domain, optionally carrying the conflicting values. */
    struct BCLIBC_DomainError : public BCLIBC_LogicError
    {
        using BCLIBC_LogicError::BCLIBC_LogicError;

        double lhs = 0.0;
        double rhs = 0.0;

        constexpr BCLIBC_DomainError() noexcept = default;
        constexpr BCLIBC_DomainError(const char *message, double lhs, double rhs) noexcept
            : BCLIBC_LogicError(message), lhs(lhs), rhs(rhs) {}

        constexpr BCLIBC_DomainError(const BCLIBC_DomainError &) noexcept = default;
        constexpr BCLIBC_DomainError &operator=(const BCLIBC_DomainError &) noexcept = default;
    };

    struct BCLIBC_RuntimeError
    {
        const char *message = "Runtime error";

        constexpr BCLIBC_RuntimeError() noexcept = default;
        constexpr explicit BCLIBC_RuntimeError(const char *message) noexcept
            : message(message != nullptr ? message : "Runtime error") {}

        constexpr BCLIBC_RuntimeError(const BCLIBC_RuntimeError &) noexcept = default;
        constexpr BCLIBC_RuntimeError &operator=(const BCLIBC_RuntimeError &) noexcept = default;

        constexpr const char *what() const noexcept { return message; }
    };

    struct BCLIBC_OutOfRangeError : public BCLIBC_RuntimeError
    {
        double requested = 0.0;
        double minimum = 0.0;
        double maximum = 0.0;

        constexpr BCLIBC_OutOfRangeError() noexcept = default;
        constexpr BCLIBC_OutOfRangeError(
            const char *message, double requested, double minimum, double maximum) noexcept
            : BCLIBC_RuntimeError(message),
              requested(requested),
              minimum(minimum),
              maximum(maximum) {}

        constexpr BCLIBC_OutOfRangeError(const BCLIBC_OutOfRangeError &) noexcept = default;
        constexpr BCLIBC_OutOfRangeError &operator=(const BCLIBC_OutOfRangeError &) noexcept = default;
    };

    /** Invalid caller input. */
    struct BCLIBC_InvalidArgumentError : public BCLIBC_RuntimeError
    {
        using BCLIBC_RuntimeError::BCLIBC_RuntimeError;

        constexpr BCLIBC_InvalidArgumentError() noexcept = default;

        constexpr BCLIBC_InvalidArgumentError(const BCLIBC_InvalidArgumentError &) noexcept = default;
        constexpr BCLIBC_InvalidArgumentError &operator=(const BCLIBC_InvalidArgumentError &) noexcept = default;
    };
}

#endif // BCLIBC_ERROR_HPP
