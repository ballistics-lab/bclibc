#ifndef BCLIBC_RESULT_HPP
#define BCLIBC_RESULT_HPP

#include <cassert>
#include <type_traits>
#include <utility>
#include <variant>

namespace bclibc
{
    class BCLIBC_Error; // defined in exceptions.hpp, after the trajectory types its payloads carry

    /**
     * Value-or-error result for APIs that do not throw (the C++17 counterpart of C++23's std::expected).
     *
     * Unlike std::get, value() and error() never throw: checking has_value() first is the caller's job, and
     * violating it is a bug that asserts in debug builds. E defaults to BCLIBC_Error so signatures stay
     * `BCLIBC_Result<T>`; it may be incomplete wherever a function is only declared.
     */
    template <class T, class E = BCLIBC_Error>
    class BCLIBC_Result
    {
    public:
        using value_type = T;
        using error_type = E;

        BCLIBC_Result(T value) noexcept : v_(std::in_place_index<0>, std::move(value)) {}
        BCLIBC_Result(E error) noexcept : v_(std::in_place_index<1>, std::move(error)) {}

        /** An error payload (e.g. BCLIBC_DomainError) can be returned directly: it converts to E first. */
        template <class P,
                  class = std::enable_if_t<!std::is_same_v<P, T> && !std::is_same_v<P, E> &&
                                           !std::is_constructible_v<T, P> && std::is_constructible_v<E, P>>>
        BCLIBC_Result(P payload) noexcept : v_(std::in_place_index<1>, E(std::move(payload))) {}

        [[nodiscard]] bool has_value() const noexcept { return v_.index() == 0; }
        [[nodiscard]] explicit operator bool() const noexcept { return has_value(); }

        [[nodiscard]] T &value() noexcept
        {
            assert(has_value());
            return *std::get_if<0>(&v_);
        }
        [[nodiscard]] const T &value() const noexcept
        {
            assert(has_value());
            return *std::get_if<0>(&v_);
        }
        [[nodiscard]] const E &error() const noexcept
        {
            assert(!has_value());
            return *std::get_if<1>(&v_);
        }

    private:
        std::variant<T, E> v_;
    };

    template <class T, class E>
    [[nodiscard]] constexpr bool has_error(const BCLIBC_Result<T, E> &result) noexcept
    {
        return !result.has_value();
    }

    template <class T, class E>
    [[nodiscard]] constexpr bool is_ok(const BCLIBC_Result<T, E> &result) noexcept
    {
        return result.has_value();
    }
}

#endif // BCLIBC_RESULT_HPP
