#ifndef BCLIBC_RESULT_HPP
#define BCLIBC_RESULT_HPP

#include <variant>

namespace bclibc
{
    /** A value-or-error result for APIs that do not throw. */
    template <class Error, class T>
    using Result = std::variant<Error, T>;

    template <class Error, class T>
    [[nodiscard]] constexpr bool has_error(const Result<Error, T> &result) noexcept
    {
        return std::holds_alternative<Error>(result);
    }

    template <class Error, class T>
    [[nodiscard]] constexpr bool is_ok(const Result<Error, T> &result) noexcept
    {
        return std::holds_alternative<T>(result);
    }
}

#endif // BCLIBC_RESULT_HPP
