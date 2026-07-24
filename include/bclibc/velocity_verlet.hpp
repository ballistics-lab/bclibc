#ifndef BCLIBC_VELOCITY_VERLET_HPP
#define BCLIBC_VELOCITY_VERLET_HPP

#include "v3d.hpp"
#include "base_types.hpp"
#include "engine.hpp"
#include "bclibc/traj_data.hpp"

namespace bclibc
{
    /**
     * @brief Performs projectile trajectory simulation using the Velocity Verlet integration method.
     *
     * The Velocity Verlet method is a symplectic, time-reversible second-order integrator
     * that updates position and velocity together:
     * ```
     * x(t + dt) = x(t) + v(t)*dt + 0.5*a(t)*dt^2
     * v(t + dt) = v(t) + 0.5*[a(t) + a(t + dt)]*dt
     * ```
     * Because it re-uses the acceleration computed at the end of the previous step,
     * it stays synchronized and conserves energy better than explicit Euler over
     * long integration periods, at roughly the cost of one extra acceleration
     * evaluation per step compared to RK4's four.
     *
     * The simulation accounts for:
     * - Gravity (constant downward acceleration)
     * - Aerodynamic drag (velocity and altitude dependent)
     * - Wind effects (variable with range)
     * - Coriolis forces (optional, for long-range shots)
     *
     * Integration continues until one of these conditions is met:
     * - Maximum range exceeded
     * - Velocity drops below minimum threshold
     * - Projectile altitude drops below minimum
     * - Drop exceeds maximum allowed value
     *
     * @param eng The ballistics engine containing shot properties, atmospheric conditions, and configuration.
     * @param handler Interface for processing computed trajectory data points.
     * @param reason Output parameter indicating why the simulation terminated.
     */
    void BCLIBC_integrateVELOCITY_VERLET(
        BCLIBC_BaseEngine &eng,
        BCLIBC_BaseTrajDataHandlerInterface &handler,
        BCLIBC_TerminationReason &reason);

}; // namespace bclibc

#endif // BCLIBC_VELOCITY_VERLET_HPP
