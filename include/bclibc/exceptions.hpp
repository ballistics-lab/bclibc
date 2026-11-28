#ifndef BCLIBC_EXCEPTIONS_HPP
#define BCLIBC_EXCEPTIONS_HPP

#include <cstddef>
#include <type_traits>
#include <utility>
#include <variant>
#include "bclibc/error.hpp"
#include "bclibc/traj_data.hpp"

namespace bclibc
{
    /** Generic solver failure with no more specific payload (the catch-all of the solver errors). */
    struct BCLIBC_SolverRuntimeError
    {
        const char *message = "Solver runtime error";

        constexpr BCLIBC_SolverRuntimeError() noexcept = default;
        constexpr explicit BCLIBC_SolverRuntimeError(const char *message) noexcept
            : message(message != nullptr ? message : "Solver runtime error") {}

        constexpr const char *what() const noexcept { return message; }
    };

    /**
     * Zero-finding failed to converge; carries the last iteration's diagnostics.
     *
     * Unlike the other payloads, the message is OWNED: it is copied into a fixed buffer of the payload, so a text
     * formatted at the point of failure (the "No low zero trajectory in elevation range (A, B deg). Errors at bracket:
     * f(low)=X, f(high)=Y" of the Ridder search, the same text the Python reference raises) can be returned in the error
     * from a stack-local buffer. A longer text is cut to kMessageCapacity - 1 characters. what() points into this
     * object, so read it from the error you hold (copying the payload copies the text, no pointer is shared); no heap.
     */
    struct BCLIBC_SolverZeroFindingError
    {
        /** Size of the message buffer, terminator included; fits the longest bracket message (the numbers print with %.2f). */
        static constexpr std::size_t kMessageCapacity = 256;

        char message_text[kMessageCapacity] = {};
        double zero_finding_error = 0.0;
        int iterations_count = 0;
        double last_barrel_elevation_rad = 0.0;

        constexpr BCLIBC_SolverZeroFindingError() noexcept { assign("Zero finding error"); }
        constexpr BCLIBC_SolverZeroFindingError(
            const char *message,
            double zero_finding_error,
            int iterations_count,
            double last_barrel_elevation_rad) noexcept
            : zero_finding_error(zero_finding_error),
              iterations_count(iterations_count),
              last_barrel_elevation_rad(last_barrel_elevation_rad)
        {
            assign(message != nullptr ? message : "Zero finding error");
        }

        constexpr const char *what() const noexcept { return message_text; }

    private:
        constexpr void assign(const char *text) noexcept
        {
            std::size_t i = 0;
            for (; i + 1 < kMessageCapacity && text[i] != '\0'; ++i)
                message_text[i] = text[i];
            message_text[i] = '\0';
        }
    };

    /** Requested distance is beyond the trajectory's reachable range. */
    struct BCLIBC_SolverOutOfRangeError
    {
        const char *message = "Out of range";
        double requested_distance_ft = 0.0;
        double max_range_ft = 0.0;
        double look_angle_rad = 0.0;

        constexpr BCLIBC_SolverOutOfRangeError() noexcept = default;
        constexpr BCLIBC_SolverOutOfRangeError(
            const char *message,
            double requested_distance_ft,
            double max_range_ft,
            double look_angle_rad) noexcept
            : message(message != nullptr ? message : "Out of range"),
              requested_distance_ft(requested_distance_ft),
              max_range_ft(max_range_ft),
              look_angle_rad(look_angle_rad) {}

        constexpr const char *what() const noexcept { return message; }
    };

    /** Target point could not be located within the integrated trajectory; carries the last point reached. */
    struct BCLIBC_SolverInterceptionError
    {
        const char *message = "Interception error";
        BCLIBC_BaseTrajData raw_data;
        BCLIBC_TrajectoryData full_data;

        BCLIBC_SolverInterceptionError() noexcept = default;
        BCLIBC_SolverInterceptionError(
            const char *message,
            const BCLIBC_BaseTrajData &raw_data,
            const BCLIBC_TrajectoryData &full_data) noexcept
            : message(message != nullptr ? message : "Interception error"),
              raw_data(raw_data),
              full_data(full_data) {}

        const char *what() const noexcept { return message; }
    };

    namespace detail
    {
        using BCLIBC_ErrorVariant = std::variant<
            BCLIBC_LogicError,
            BCLIBC_DomainError,
            BCLIBC_RuntimeError,
            BCLIBC_OutOfRangeError,
            BCLIBC_InvalidArgumentError,
            BCLIBC_SolverZeroFindingError,
            BCLIBC_SolverOutOfRangeError,
            BCLIBC_SolverInterceptionError,
            BCLIBC_SolverRuntimeError>;

        template <class P, class V>
        struct is_error_payload;
        template <class P, class... Ps>
        struct is_error_payload<P, std::variant<Ps...>> : std::bool_constant<(std::is_same_v<P, Ps> || ...)>
        {
        };
    } // namespace detail

    /**
     * The one error type of bclibc: a tagged sum of every payload above and in error.hpp, each keeping its own
     * diagnostic fields. Nothing in bclibc throws: a fallible function returns BCLIBC_Result<T> and propagates
     * a failure by returning the callee's error unchanged, metadata included. Consumers dispatch on kind() and
     * read the payload with payload<Payload>(), which returns nullptr for any other alternative and never throws.
     */
    class BCLIBC_Error
    {
    public:
        enum class Kind
        {
            Logic,
            Domain,
            Runtime,
            OutOfRange,
            InvalidArgument,
            SolverZeroFinding,
            SolverOutOfRange,
            SolverInterception,
            SolverRuntime,
        };

        /** A placeholder, not a real failure: lets a BCLIBC_Result (and Cython's stack-allocated one) be default-constructed. */
        BCLIBC_Error() noexcept : v_(std::in_place_type<BCLIBC_LogicError>, BCLIBC_LogicError{"Uninitialized result"}) {}

        template <class P, class = std::enable_if_t<detail::is_error_payload<P, detail::BCLIBC_ErrorVariant>::value>>
        BCLIBC_Error(P payload) noexcept : v_(std::in_place_type<P>, std::move(payload)) {}

        [[nodiscard]] Kind kind() const noexcept { return static_cast<Kind>(v_.index()); }

        template <class P>
        [[nodiscard]] const P *payload() const noexcept { return std::get_if<P>(&v_); }

        [[nodiscard]] const char *what() const noexcept { return what_from<0>(); }

    private:
        template <std::size_t I>
        [[nodiscard]] const char *what_from() const noexcept
        {
            if constexpr (I == std::variant_size_v<detail::BCLIBC_ErrorVariant>)
                return "";
            else if (const auto *p = std::get_if<I>(&v_))
                return p->what();
            else
                return what_from<I + 1>();
        }

        detail::BCLIBC_ErrorVariant v_;
    };

} // namespace bclibc

#endif //  BCLIBC_EXCEPTIONS_HPP
