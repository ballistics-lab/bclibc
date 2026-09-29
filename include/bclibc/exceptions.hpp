#ifndef BCLIBC_EXCEPTIONS_HPP
#define BCLIBC_EXCEPTIONS_HPP

#include <variant>
#include "bclibc/error.hpp"
#include "bclibc/traj_data.hpp"

namespace bclibc
{
    /** Zero-finding failed to converge; carries the last iteration's diagnostics. */
    struct BCLIBC_SolverZeroFindingError
    {
        const char *message = "Zero finding error";
        double zero_finding_error = 0.0;
        int iterations_count = 0;
        double last_barrel_elevation_rad = 0.0;

        constexpr BCLIBC_SolverZeroFindingError() noexcept = default;
        constexpr BCLIBC_SolverZeroFindingError(
            const char *message,
            double zero_finding_error,
            int iterations_count,
            double last_barrel_elevation_rad) noexcept
            : message(message != nullptr ? message : "Zero finding error"),
              zero_finding_error(zero_finding_error),
              iterations_count(iterations_count),
              last_barrel_elevation_rad(last_barrel_elevation_rad) {}

        constexpr const char *what() const noexcept { return message; }
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

    /**
     * Every error BCLIBC_BaseEngine can produce: the trajectory/domain-independent
     * BCLIBC_BaseError alternatives plus the ones specific to zero-finding and
     * interception. BCLIBC_BaseEngine never throws — every method that can fail
     * returns a BCLIBC_EngineResult and callers propagate it with has_error(),
     * exactly like the rest of bclibc.
     */
    using BCLIBC_EngineError = std::variant<
        BCLIBC_LogicError,
        BCLIBC_DomainError,
        BCLIBC_RuntimeError,
        BCLIBC_OutOfRangeError,
        BCLIBC_InvalidArgumentError,
        BCLIBC_SolverZeroFindingError,
        BCLIBC_SolverOutOfRangeError,
        BCLIBC_SolverInterceptionError>;

    template <class T>
    using BCLIBC_EngineResult = Result<BCLIBC_EngineError, T>;
} // namespace bclibc

#endif //  BCLIBC_EXCEPTIONS_HPP
