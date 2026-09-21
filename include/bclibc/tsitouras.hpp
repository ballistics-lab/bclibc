#ifndef BCLIBC_TSITOURAS_HPP
#define BCLIBC_TSITOURAS_HPP

#include "bclibc/embedded_rk45.hpp"
#include "bclibc/engine.hpp"
#include "bclibc/traj_data.hpp"

namespace bclibc
{
    /** Adaptive Tsitouras 5(4) trajectory integrator ("Tsit5"): a 7-stage
     * FSAL Runge--Kutta pair (Tsitouras, 2011, "Runge-Kutta pairs of order
     * 5(4) satisfying only the first column simplifying assumption",
     * Computers & Mathematics with Applications 62(2), 770-775). Same
     * error scale and step-size controller as @ref BCLIBC_integrateDormandPrince
     * (scipy.integrate.RK45-style), which it is a drop-in structural twin of --
     * same stage count, same FSAL property -- but with coefficients tuned to
     * give a smaller leading error term at each order, so it typically needs
     * fewer rejected/retried steps for the same tolerance. */
    void BCLIBC_integrateTsitouras(
        BCLIBC_BaseEngine &eng,
        BCLIBC_BaseTrajDataHandlerInterface &handler,
        BCLIBC_TerminationReason &reason);

    /**
     * @brief Stateful Tsitouras integrator: a @ref BCLIBC_IntegrateCallable
     * target that owns its own tolerances and accepted/rejected step counts
     * per instance, instead of that state living in a thread_local shared by
     * every BCLIBC_BaseEngine on the thread that happens to use Tsitouras.
     * See @ref bclibc::BCLIBC_CashKarpIntegrator for the usage pattern;
     * assigning the free function @ref BCLIBC_integrateTsitouras to
     * `integrate_func` instead runs at the default 1e-6/1e-6 tolerances with
     * no accessible stats.
     */
    class BCLIBC_TsitourasIntegrator
    {
    public:
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

#endif // BCLIBC_TSITOURAS_HPP
