#ifndef BCLIBC_DORMAND_PRINCE_HPP
#define BCLIBC_DORMAND_PRINCE_HPP

#include "bclibc/embedded_rk45.hpp"
#include "bclibc/engine.hpp"
#include "bclibc/traj_data.hpp"

namespace bclibc
{
    /** Adaptive Dormand--Prince 5(4) trajectory integrator.  Its error scale
     * and controller follow scipy.integrate.RK45. */
    void BCLIBC_integrateDormandPrince(
        BCLIBC_BaseEngine &eng,
        BCLIBC_BaseTrajDataHandlerInterface &handler,
        BCLIBC_TerminationReason &reason);

    /**
     * @brief Stateful Dormand-Prince integrator: a @ref BCLIBC_IntegrateCallable
     * target that owns its own tolerances and accepted/rejected step counts
     * per instance, instead of that state living in a thread_local shared by
     * every BCLIBC_BaseEngine on the thread that happens to use
     * Dormand-Prince. See @ref bclibc::BCLIBC_CashKarpIntegrator for the
     * usage pattern; assigning the free function
     * @ref BCLIBC_integrateDormandPrince to `integrate_func` instead runs at
     * the default 1e-6/1e-6 tolerances with no accessible stats.
     */
    class BCLIBC_DormandPrinceIntegrator
    {
    public:
        /**
         * @brief Construct with explicit tolerances (each defaulting to 1e-6).
         * @throws std::invalid_argument under the same conditions as
         * @ref set_relative_tolerance / @ref set_absolute_tolerance.
         */
        explicit BCLIBC_DormandPrinceIntegrator(
            double relative_tolerance = embedded_rk45_detail::default_tolerance,
            double absolute_tolerance = embedded_rk45_detail::default_tolerance);

        void operator()(
            BCLIBC_BaseEngine &eng,
            BCLIBC_BaseTrajDataHandlerInterface &handler,
            BCLIBC_TerminationReason &reason);

        /** @brief Accepted/rejected step counts from this instance's most
         * recent integration run. */
        void get_stats(int &out_accepted, int &out_rejected) const noexcept;

        /** @brief Set this instance's relative local-error tolerance (default 1e-6).
         * @throws std::invalid_argument if @p tolerance is not finite and positive. */
        void set_relative_tolerance(double tolerance);

        /** @brief Set this instance's scalar absolute local-error tolerance (default 1e-6).
         * @throws std::invalid_argument if @p tolerance is not finite or is negative. */
        void set_absolute_tolerance(double tolerance);

    private:
        int accepted_steps_ = 0;
        int rejected_steps_ = 0;
        double relative_tolerance_ = embedded_rk45_detail::default_tolerance;
        double absolute_tolerance_ = embedded_rk45_detail::default_tolerance;
    };
}

#endif // BCLIBC_DORMAND_PRINCE_HPP
