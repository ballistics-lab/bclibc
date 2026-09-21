#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "bclibc/dormand_prince.hpp"
#include "bclibc/embedded_rk45.hpp"

namespace bclibc
{
    namespace
    {
        // Dormand-Prince 5(4) ("DOPRI5"), the same tableau scipy.integrate's
        // RK45 uses. 7 stages: stages 0-5 are the classic 6-stage DOPRI core;
        // stage 6 reuses the FSAL trick (its A-row equals the 5th-order B
        // weights, so it evaluates the derivative at the accepted new state
        // for free) both to seed the next step's first stage and to supply
        // the 7th term the embedded error estimate needs.
        //
        // Free-standing (not class-static) constexpr arrays -- see cash_karp.cpp's
        // identical comment for why: a `static constexpr` array class member
        // here trips a GCC PIC-relocation quirk when linked into a shared object.
        //
        // Row i-1 holds the A-coefficients for stage i (i=1..6).
        constexpr double kDpA[6][6] = {
            {1.0 / 5.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {3.0 / 40.0, 9.0 / 40.0, 0.0, 0.0, 0.0, 0.0},
            {44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0, 0.0, 0.0, 0.0},
            {19372.0 / 6561.0, -25360.0 / 2187.0, 64448.0 / 6561.0, -212.0 / 729.0, 0.0, 0.0},
            {9017.0 / 3168.0, -355.0 / 33.0, 46732.0 / 5247.0, 49.0 / 176.0, -5103.0 / 18656.0, 0.0},
            {35.0 / 384.0, 0.0, 500.0 / 1113.0, 125.0 / 192.0, -2187.0 / 6784.0, 11.0 / 84.0},
        };

        // 5th-order solution weights; stage 6 (the FSAL point) contributes
        // nothing further since the state is already fully determined by
        // stages 0-5 through row 6 of kDpA above.
        constexpr double kDpB[7] = {
            35.0 / 384.0, 0.0, 500.0 / 1113.0, 125.0 / 192.0, -2187.0 / 6784.0, 11.0 / 84.0, 0.0};

        // Error weights (5th-order minus embedded 4th-order), matching
        // scipy.integrate._ivp.rk.RK45.E exactly.
        constexpr double kDpE[7] = {
            71.0 / 57600.0, 0.0, -71.0 / 16695.0, 71.0 / 1920.0,
            -17253.0 / 339200.0, 22.0 / 525.0, -1.0 / 40.0};

        struct DormandPrince54Tableau
        {
            static constexpr int stages = 7;
            static constexpr bool fsal = true;

            static inline double A(int i, int j) { return kDpA[i - 1][j]; }
            static inline double B(int i) { return kDpB[i]; }
            static inline double E(int i) { return kDpE[i]; }
        };

        // Matches scipy.integrate._ivp.rk.RungeKutta's step-size controller:
        // same exponent (-1/(error_estimator_order+1) = -1/5) for growth and
        // shrink, safety 0.9, factor clamped to [0.2, 10], and growth capped
        // at 1x for one step immediately following a rejection in the same
        // step attempt sequence.
        struct ScipyRKController
        {
            static constexpr double kSafety = 0.9;
            static constexpr double kMinFactor = 0.2;
            static constexpr double kMaxFactor = 10.0;
            static constexpr double kErrorExponent = -1.0 / 5.0;

            static inline void on_wind_change(const BCLIBC_V3dT &velocity, const BCLIBC_V3dT &wind, BCLIBC_V3dT &vr)
            {
                vr = velocity - wind;
            }

            // Wind is only re-sampled once, at the *start* of a step, and then
            // held constant across every stage evaluation within it. A large
            // adaptive step that both starts before and ends past next_range
            // would silently integrate its tail through the wrong wind zone --
            // FSAL cache invalidation (handled centrally in
            // integrate_embedded_rk45) only fixes the *next* step's stale
            // derivative, not this step's own wind model. Shrinking dt so the
            // step lands at (never past) the boundary keeps every stage's wind
            // sample valid for where it's actually evaluated.
            static inline void limit_step_at_wind_boundary(
                double next_range, const BCLIBC_V3dT &pos, const BCLIBC_V3dT &vr,
                const BCLIBC_V3dT &wind, double &dt)
            {
                const double ground_vx = vr.x + wind.x;
                if (ground_vx <= 0.0)
                    return;
                const double remaining = next_range - pos.x;
                // Below this floor, treat the boundary as already reached rather than
                // keep shrinking dt to match: once a prior iteration's limiting has
                // already gotten pos.x within a fraction of a foot of next_range, the
                // stage combination's own rounding can land the accepted step a few ULPs
                // *short* of the boundary instead of exactly on or past it. Without this
                // floor, the next iteration re-limits dt to that now-tinier remaining
                // distance, and the one after that to a tinier one still: a Zeno's-
                // paradox loop that in practice never terminates (reproduced, and fixed
                // the same way, in tiny_bclibc__run_tsitouras for a 3+ wind-zone shot;
                // this is the same controller logic, so the same failure mode applies
                // here even though no existing test happens to trigger it). 1e-7 ft
                // matches the wind-change trigger's own epsilon in embedded_rk45.hpp.
                if (remaining <= 1e-7)
                    return;
                const double time_to_boundary = remaining / ground_vx;
                if (time_to_boundary < dt)
                    dt = time_to_boundary;
            }

            static inline double accepted_factor(double error_norm, bool was_rejected)
            {
                double factor = (error_norm == 0.0)
                                    ? kMaxFactor
                                    : std::min(kMaxFactor, kSafety * std::pow(error_norm, kErrorExponent));
                if (was_rejected)
                    factor = std::min(1.0, factor);
                return factor;
            }

            static inline double rejected_factor(double error_norm)
            {
                return std::max(kMinFactor, kSafety * std::pow(error_norm, kErrorExponent));
            }
        };
    } // namespace

    void BCLIBC_integrateDormandPrince(
        BCLIBC_BaseEngine &eng,
        BCLIBC_BaseTrajDataHandlerInterface &handler,
        BCLIBC_TerminationReason &reason)
    {
        BCLIBC_DormandPrinceIntegrator integrator;
        integrator(eng, handler, reason);
    }

    void BCLIBC_DormandPrinceIntegrator::operator()(
        BCLIBC_BaseEngine &eng,
        BCLIBC_BaseTrajDataHandlerInterface &handler,
        BCLIBC_TerminationReason &reason)
    {
        embedded_rk45_detail::run<DormandPrince54Tableau, ScipyRKController>(
            eng, handler, reason,
            accepted_steps_, rejected_steps_,
            relative_tolerance_, absolute_tolerance_);
    }

    void BCLIBC_DormandPrinceIntegrator::get_stats(int &out_accepted, int &out_rejected) const noexcept
    {
        out_accepted = accepted_steps_;
        out_rejected = rejected_steps_;
    }

    void BCLIBC_DormandPrinceIntegrator::set_relative_tolerance(double tolerance)
    {
        if (!std::isfinite(tolerance) || tolerance <= 0.0)
            throw std::invalid_argument("Dormand-Prince relative tolerance must be finite and positive");
        relative_tolerance_ = tolerance;
    }

    void BCLIBC_DormandPrinceIntegrator::set_absolute_tolerance(double tolerance)
    {
        if (!std::isfinite(tolerance) || tolerance < 0.0)
            throw std::invalid_argument("Dormand-Prince absolute tolerance must be finite and non-negative");
        absolute_tolerance_ = tolerance;
    }

}; // namespace bclibc
