#ifndef BCLIBC_EXCEPTIONS_HPP
#define BCLIBC_EXCEPTIONS_HPP

#include <stdexcept>
#include "bclibc/base_types.hpp"

namespace bclibc
{
    class BCLIBC_SolverRuntimeException : public std::runtime_error
    {
    public:
        BCLIBC_SolverRuntimeException(const std::string &message)
            : std::runtime_error(message) {};
    };

    class BCLIBC_OutOfRangeException : public BCLIBC_SolverRuntimeException
    {
    public:
        double requested_distance_ft;
        double max_range_ft;
        double look_angle_rad;

        BCLIBC_OutOfRangeException(
            const std::string &message,
            double requested_distance_ft,
            double max_range_ft,
            double look_angle_rad)
            : BCLIBC_SolverRuntimeException(message),
              requested_distance_ft(requested_distance_ft),
              max_range_ft(max_range_ft),
              look_angle_rad(look_angle_rad) {};
    };

    class BCLIBC_ZeroFindingException : public BCLIBC_SolverRuntimeException
    {
    public:
        double zero_finding_error;
        int iterations_count;
        double last_barrel_elevation_rad;

        BCLIBC_ZeroFindingException(
            const std::string &message,
            double zero_finding_error,
            int iterations_count,
            double last_barrel_elevation_rad)
            : BCLIBC_SolverRuntimeException(message),
              zero_finding_error(zero_finding_error),
              iterations_count(iterations_count),
              last_barrel_elevation_rad(last_barrel_elevation_rad) {};
    };

    class BCLIBC_InterceptionException : public BCLIBC_SolverRuntimeException
    {
    public:
        BCLIBC_BaseTrajData raw_data;
        BCLIBC_TrajectoryData full_data;
        BCLIBC_InterceptionException(
            const std::string &message,
            const BCLIBC_BaseTrajData &raw_data,
            const BCLIBC_TrajectoryData &full_data)
            : BCLIBC_SolverRuntimeException(message),
              raw_data(raw_data),
              full_data(full_data) {};
    };
}; // bclibc

#endif //  BCLIBC_EXCEPTIONS_HPP
