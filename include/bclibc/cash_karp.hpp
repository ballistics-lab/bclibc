#ifndef BCLIBC_CASH_KARP_HPP
#define BCLIBC_CASH_KARP_HPP

#include <atomic>

#include "bclibc/v3d.hpp"
#include "bclibc/base_types.hpp"
#include "bclibc/engine.hpp"
#include "bclibc/traj_data.hpp"
#include "bclibc/embedded_rk45.hpp"

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
     * Error control follows scipy.integrate.solve_ivp's Runge-Kutta convention: a scalar
     * absolute tolerance and relative tolerance independently scale every position/velocity
     * component as `atol + rtol * max(abs(y), abs(y_new))`; the six scaled errors use an RMS norm.
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
     * @brief Stateful Cash-Karp integrator: a @ref BCLIBC_IntegrateCallable
     * target that owns its own tolerances and accepted/rejected step counts,
     * instead of that state living in a thread_local shared by every
     * BCLIBC_BaseEngine on the thread that happens to use Cash-Karp.
     *
     * Each instance is independent, so distinct engines -- even on the same
     * thread -- can run Cash-Karp with different tolerances at the same time.
     * `eng.integrate_func` (a `std::function`) copies whatever is assigned to
     * it, so assigning the integrator by value gives the engine its own copy
     * and later calls on your own variable won't see that copy's stats; wrap
     * it in `std::ref` to keep one shared instance whose stats you can read
     * back after the run:
     *
     * @code
     * bclibc::BCLIBC_CashKarpIntegrator integrator(1e-8);   // rtol=1e-8, atol defaults to 1e-6
     * eng.integrate_func = std::ref(integrator);   // this engine only, same instance
     * // ... eng.integrate(...) / eng.integrate_filtered(...) ...
     * int accepted, rejected;
     * integrator.get_stats(accepted, rejected);   // reflects the run above
     * @endcode
     *
     * Assigning the free function @ref BCLIBC_integrateCashKarp to
     * `integrate_func` instead runs Cash-Karp at the default 1e-6/1e-6
     * tolerances with no accessible stats -- use it when neither is needed.
     *
     * Tolerances and stats are each stored in a `std::atomic`, so a single
     * instance CAN be shared (via `std::ref`) across threads -- e.g. one
     * `BCLIBC_CashKarpIntegrator` driving several `BCLIBC_BaseEngine`s
     * concurrently -- without a data race: `set_relative_tolerance()` /
     * `set_absolute_tolerance()` from one thread can safely race with
     * `operator()` (which snapshots both once at the start of its run, so a
     * whole integration sees one consistent tolerance pair even if it
     * changes mid-run) and `get_stats()` from others. `get_stats()`'s two
     * loads are independent, though: a concurrent run can still make
     * `out_accepted`/`out_rejected` a non-atomic snapshot pair (e.g. the new
     * accepted count paired with the previous run's rejected count) -- fine
     * for approximate monitoring, not for anything requiring the two to
     * agree exactly.
     */
    class BCLIBC_CashKarpIntegrator
    {
    public:
        /**
         * @brief Construct with explicit tolerances (each defaulting to 1e-6).
         * @throws std::invalid_argument under the same conditions as
         * @ref set_relative_tolerance / @ref set_absolute_tolerance.
         */
        explicit BCLIBC_CashKarpIntegrator(
            double relative_tolerance = embedded_rk45_detail::default_tolerance,
            double absolute_tolerance = embedded_rk45_detail::default_tolerance);

        /** @brief Copies the current tolerances (not the running stats'
         * transient state) -- each field is copied independently via an
         * atomic load, so this itself cannot race unsafely with concurrent
         * use of @p other, though it is not a single atomic snapshot of all
         * four fields together. */
        BCLIBC_CashKarpIntegrator(const BCLIBC_CashKarpIntegrator &other) noexcept;
        BCLIBC_CashKarpIntegrator &operator=(const BCLIBC_CashKarpIntegrator &other) noexcept;

        void operator()(
            BCLIBC_BaseEngine &eng,
            BCLIBC_BaseTrajDataHandlerInterface &handler,
            BCLIBC_TerminationReason &reason);

        /**
         * @brief Returns the accepted/rejected step counts from this
         * instance's most recent integration run -- the fairest
         * apples-to-apples comparison against a fixed-step method's own
         * `eng.integration_step_count` (which only counts accepted/emitted
         * steps for both methods; this additionally exposes how many
         * attempted steps were rejected and retried at a smaller step size).
         */
        void get_stats(int &out_accepted, int &out_rejected) const noexcept;

        /**
         * @brief Set this instance's relative local-error tolerance.
         *
         * The default is 1e-6.
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
        void set_relative_tolerance(double tolerance);

        /**
         * @brief Set this instance's scalar absolute local-error tolerance.
         *
         * This follows scipy.integrate.solve_ivp semantics: every component of
         * the six-value state vector (three position and three velocity values)
         * is independently scaled by `atol + rtol * max(abs(y), abs(y_new))`.
         * The default is 1e-6.
         *
         * @throws std::invalid_argument if @p tolerance is not finite or is negative.
         */
        void set_absolute_tolerance(double tolerance);

    private:
        std::atomic<int> accepted_steps_{0};
        std::atomic<int> rejected_steps_{0};
        std::atomic<double> relative_tolerance_{embedded_rk45_detail::default_tolerance};
        std::atomic<double> absolute_tolerance_{embedded_rk45_detail::default_tolerance};
    };

}; // namespace bclibc

#endif // BCLIBC_CASH_KARP_HPP
