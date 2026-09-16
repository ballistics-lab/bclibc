#ifndef BCLIBC_CASH_KARP_HPP
#define BCLIBC_CASH_KARP_HPP

#include "bclibc/v3d.hpp"
#include "bclibc/base_types.hpp"
#include "bclibc/engine.hpp"
#include "bclibc/traj_data.hpp"

namespace bclibc
{
    /**
     * @brief Performs a full projectile trajectory simulation using the embedded
     * Cash-Karp Runge-Kutta 4(5) method (Numerical Recipes `rkck`), with adaptive
     * step-size control driven by the embedded 4th/5th-order error estimate.
     *
     * Unlike @ref BCLIBC_integrateRK4's fixed step, this grows the step during
     * smooth flight (up to 64x the configured base step) and shrinks it near the
     * transonic drag transition or whenever the local error estimate exceeds
     * tolerance, retrying the same attempted step rather than accepting it.
     *
     * IMPORTANT, KNOWN LIMITATION (see project issue tracker before relying on
     * this for anything beyond experimentation): the drag coefficient AND
     * atmosphere sample are recomputed fresh at each of the 6 stages, from
     * that stage's own intermediate velocity/altitude -- unlike
     * @ref BCLIBC_integrateRK4, which freezes both once per step. This is not
     * an oversight: an earlier variant that froze them (mirroring RK4's own
     * per-step memoization, extended to 6 stages) produced real, tolerance-
     * independent accuracy failures, because the embedded error estimator
     * cannot see model error from a stale drag/atmosphere sample, only
     * discretization error of the frozen sub-problem. Full per-stage
     * recompute is the validated fix -- do not reintroduce step-level
     * freezing to "optimize" this without re-running the full accuracy
     * sweep.
     *
     * SEPARATELY, AND MORE IMPORTANT: this integrator produces raw trajectory
     * points that are sparser and much less uniformly spaced than
     * BCLIBC_integrateRK4's fixed-step output. @ref BCLIBC_interpolate3pt
     * (used by @ref BCLIBC_TrajectoryDataFilter and
     * @ref BCLIBC_SinglePointHandler for RANGE-step/APEX/MACH/ZERO-crossing
     * interpolation) derives its Hermite slopes via finite differences across
     * 3 raw points -- accurate when those points are dense and uniform, but
     * measurably wrong once they are not, which is exactly what this
     * integrator produces on purpose during smooth flight. Real accuracy
     * regressions (not float rounding) have been observed at exactly this
     * boundary (a multi-yard ZERO_UP distance miss). Not a bug in this file --
     * see the project issue tracker for the write-up and suggested fix
     * (reconstruct position via exact-derivative Hermite using the already-
     * known velocity at each raw point, instead of a finite-difference
     * estimate) before using this integrator's output for anything
     * accuracy-sensitive.
     *
     * @param eng Ballistics engine: shot parameters, atmosphere, wind layers, configuration.
     * @param handler Interface receiving trajectory points as they are accepted.
     * @param reason Output parameter describing why the simulation ended.
     */
    void BCLIBC_integrateCashKarp(
        BCLIBC_BaseEngine &eng,
        BCLIBC_BaseTrajDataHandlerInterface &handler,
        BCLIBC_TerminationReason &reason);

    /**
     * @brief Returns the accepted/rejected step counts from the most recent
     * @ref BCLIBC_integrateCashKarp call on this thread -- the fairest
     * apples-to-apples comparison against a fixed-step method's own
     * `eng.integration_step_count` (which only counts accepted/emitted
     * steps for both methods; this additionally exposes how many attempted
     * steps were rejected and retried at a smaller step size).
     */
    void BCLIBC_cashKarpGetStats(int &out_accepted, int &out_rejected);

    /**
     * @brief Set the relative local-error tolerance for Cash-Karp on this thread.
     *
     * The default is 1e-6. The setting is thread-local so independent engine
     * instances can integrate concurrently with different tolerances.
     *
     * @throws std::invalid_argument if @p tolerance is not finite and positive.
     */
    void BCLIBC_cashKarpSetRelativeTolerance(double tolerance);

}; // namespace bclibc

#endif // BCLIBC_CASH_KARP_HPP
