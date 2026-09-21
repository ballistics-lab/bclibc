#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "bclibc/cash_karp.hpp"
#include "bclibc/embedded_rk45.hpp"

namespace bclibc
{
    namespace
    {
        // Standard Numerical Recipes `rkck` Cash-Karp tableau. Preserved exactly
        // from the pre-refactor standalone implementation -- do not "clean up"
        // these numbers without re-running the accuracy sweep referenced in
        // cash_karp.hpp's doc comment.
        //
        // Free-standing (not class-static) constexpr arrays: a `static
        // constexpr` array *class member* here trips a GCC PIC-relocation
        // quirk when this translation unit is linked into a shared object
        // (the array, being ODR-used through indexing, wants an internal
        // symbol the non-PIC-looking codegen for anonymous-namespace class
        // statics can't satisfy) -- "ld: ... can not be used when making a
        // shared object; recompile with -fPIC". Plain namespace-scope arrays
        // don't hit this.
        //
        // Row i-1 holds the A-coefficients for stage i (i=1..5); stage 0 has
        // no A-row since it starts from the step's own initial state.
        constexpr double kCkA[5][5] = {
            {1.0 / 5.0, 0.0, 0.0, 0.0, 0.0},
            {3.0 / 40.0, 9.0 / 40.0, 0.0, 0.0, 0.0},
            {3.0 / 10.0, -9.0 / 10.0, 6.0 / 5.0, 0.0, 0.0},
            {-11.0 / 54.0, 2.5, -70.0 / 27.0, 35.0 / 27.0, 0.0},
            {1631.0 / 55296.0, 175.0 / 512.0, 575.0 / 13824.0, 44275.0 / 110592.0, 253.0 / 4096.0},
        };

        // 5th-order solution weights.
        constexpr double kCkB[6] = {
            37.0 / 378.0, 0.0, 250.0 / 621.0, 125.0 / 594.0, 0.0, 512.0 / 1771.0};

        // Error weights: 5th-order minus embedded 4th-order.
        constexpr double kCkE[6] = {
            37.0 / 378.0 - 2825.0 / 27648.0,
            0.0,
            250.0 / 621.0 - 18575.0 / 48384.0,
            125.0 / 594.0 - 13525.0 / 55296.0,
            0.0 - 277.0 / 14336.0,
            512.0 / 1771.0 - 0.25,
        };

        struct CashKarp54Tableau
        {
            static constexpr int stages = 6;
            static constexpr bool fsal = false;

            static inline double A(int i, int j) { return kCkA[i - 1][j]; }
            static inline double B(int i) { return kCkB[i]; }
            static inline double E(int i) { return kCkE[i]; }
        };

        // Preserves the original standalone Cash-Karp controller exactly:
        // asymmetric growth (-0.20 exponent, clamped [1, 5]) vs shrink (-0.25
        // exponent, clamped [0.1, 0.9]), no memory of prior rejections within
        // the same step, and no wind-boundary step limiting.
        struct CashKarpController
        {
            static constexpr double kSafety = 0.9;

            static inline void on_wind_change(const BCLIBC_V3dT &velocity, const BCLIBC_V3dT &wind, BCLIBC_V3dT &vr)
            {
                vr = velocity - wind;
            }

            static inline void limit_step_at_wind_boundary(
                double /*next_range*/, const BCLIBC_V3dT & /*pos*/, const BCLIBC_V3dT & /*vr*/,
                const BCLIBC_V3dT & /*wind*/, double & /*dt*/)
            {
                // Historical behavior: Cash-Karp never limited its step at wind-zone
                // boundaries. Left as a no-op to preserve exact prior output.
            }

            static inline double accepted_factor(double error_norm, bool /*was_rejected*/)
            {
                const double grow = (error_norm > 1.89e-4)
                                        ? kSafety * std::pow(error_norm, -0.20)
                                        : 5.0;
                return std::max(1.0, std::min(grow, 5.0));
            }

            static inline double rejected_factor(double error_norm)
            {
                const double shrink = kSafety * std::pow(error_norm, -0.25);
                return std::max(0.1, std::min(shrink, 0.9));
            }
        };
    } // namespace

    void BCLIBC_integrateCashKarp(
        BCLIBC_BaseEngine &eng,
        BCLIBC_BaseTrajDataHandlerInterface &handler,
        BCLIBC_TerminationReason &reason)
    {
        BCLIBC_CashKarpIntegrator integrator;
        integrator(eng, handler, reason);
    }

    void BCLIBC_CashKarpIntegrator::operator()(
        BCLIBC_BaseEngine &eng,
        BCLIBC_BaseTrajDataHandlerInterface &handler,
        BCLIBC_TerminationReason &reason)
    {
        embedded_rk45_detail::run<CashKarp54Tableau, CashKarpController>(
            eng, handler, reason,
            accepted_steps_, rejected_steps_,
            relative_tolerance_, absolute_tolerance_);
    }

    void BCLIBC_CashKarpIntegrator::get_stats(int &out_accepted, int &out_rejected) const noexcept
    {
        out_accepted = accepted_steps_;
        out_rejected = rejected_steps_;
    }

    void BCLIBC_CashKarpIntegrator::set_relative_tolerance(double tolerance)
    {
        if (!std::isfinite(tolerance) || tolerance <= 0.0)
            throw std::invalid_argument("Cash-Karp relative tolerance must be finite and positive");
        relative_tolerance_ = tolerance;
    }

    void BCLIBC_CashKarpIntegrator::set_absolute_tolerance(double tolerance)
    {
        if (!std::isfinite(tolerance) || tolerance < 0.0)
            throw std::invalid_argument("Cash-Karp absolute tolerance must be finite and non-negative");
        absolute_tolerance_ = tolerance;
    }

}; // namespace bclibc
