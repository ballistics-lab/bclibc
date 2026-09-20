#ifndef BCLIBC_TSITOURAS_HPP
#define BCLIBC_TSITOURAS_HPP

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
    void BCLIBC_tsitourasGetStats(int &out_accepted, int &out_rejected);
    void BCLIBC_tsitourasSetRelativeTolerance(double tolerance);
    void BCLIBC_tsitourasSetAbsoluteTolerance(double tolerance);
}

#endif // BCLIBC_TSITOURAS_HPP
