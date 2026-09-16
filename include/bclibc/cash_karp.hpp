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
     * DO NOT tighten this default without re-measuring -- it is not an
     * arbitrary "safe" choice, it is empirically the best available tradeoff
     * among the values actually tested (project issue #350; see
     * py-ballisticcalc's tests/test_cashkarp.py::test_cashkarp_accuracy_across_tolerances
     * for the harness). Measured against a 5x-finer fixed-step RK4 reference
     * on one shot profile (0.22 BC / 800 m/s / 3 MOA zero / 2000 m):
     *
     *   rtol   total steps   max height err   max event-root-distance err
     *   1e-6   48            0.045 ft         0.258 ft   <- best
     *   1e-7   50            0.042 ft         0.978 ft
     *   1e-8   54            0.043 ft         1.818 ft
     *   1e-9   52            0.051 ft         0.730 ft
     *
     * Height/velocity accuracy plateaus past 1e-6 (no benefit from going
     * tighter); event-root distance (ZERO/MACH/APEX crossings, found by
     * Hermite root-finding within whichever accepted-step interval brackets
     * them) does NOT improve monotonically with tolerance -- it depends on
     * where that interval's boundaries happen to fall relative to the
     * crossing, not on the global error tolerance directly. 1e-6 costs the
     * *fewest* total steps of the four and gives the *best* event accuracy
     * of the four in this measurement -- tightening further is a pure loss
     * (more compute, no better and sometimes worse accuracy), not a
     * conservative choice. Not yet verified across multiple shot profiles --
     * if this default is ever revisited, re-run the same sweep on at least
     * 2-3 different calibers/BCs/ranges first, not just trust this one.
     *
     * @throws std::invalid_argument if @p tolerance is not finite and positive.
     */
    void BCLIBC_cashKarpSetRelativeTolerance(double tolerance);

}; // namespace bclibc

#endif // BCLIBC_CASH_KARP_HPP
