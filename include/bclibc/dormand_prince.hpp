#ifndef BCLIBC_DORMAND_PRINCE_HPP
#define BCLIBC_DORMAND_PRINCE_HPP

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
    void BCLIBC_dormandPrinceGetStats(int &out_accepted, int &out_rejected);
    void BCLIBC_dormandPrinceSetRelativeTolerance(double tolerance);
    void BCLIBC_dormandPrinceSetAbsoluteTolerance(double tolerance);
}

#endif // BCLIBC_DORMAND_PRINCE_HPP
