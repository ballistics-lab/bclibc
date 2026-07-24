#include <cmath>
#include "bclibc/velocity_verlet.hpp"
#include "bclibc/log.hpp"

namespace bclibc
{
    /**
     * @brief Computes acceleration (dv/dt) for a given velocity: gravity + coriolis - drag.
     *
     * @param eng Engine providing coriolis settings and drag model.
     * @param velocity_for_coriolis Velocity vector to evaluate the (optional) Coriolis term at.
     * @param relative_velocity Velocity relative to the air mass (velocity minus wind).
     * @param relative_speed Magnitude of relative_velocity (precomputed to avoid a repeated sqrt).
     * @param gravity_vector Precomputed gravity vector.
     * @param density_ratio Air density ratio at the current altitude.
     * @param mach Local speed of sound at the current altitude.
     * @param out Output acceleration vector.
     */
    static inline void BCLIBC_velocity_verlet_acceleration(
        BCLIBC_BaseEngine &eng,
        const BCLIBC_V3dT &velocity_for_coriolis,
        const BCLIBC_V3dT &relative_velocity,
        double relative_speed,
        const BCLIBC_V3dT &gravity_vector,
        double density_ratio,
        double mach,
        BCLIBC_V3dT &out)
    {
        const double inv_mach = (mach != 0.0) ? (1.0 / mach) : 1.0;
        const double km = density_ratio * eng.shot.drag_by_mach(relative_speed * inv_mach);

        out = gravity_vector;
        if (!eng.shot.coriolis.flat_fire_only)
        {
            BCLIBC_V3dT coriolis_accel;
            eng.shot.coriolis.coriolis_acceleration_local(velocity_for_coriolis, coriolis_accel);
            out += coriolis_accel;
        }
        // out -= km * relative_speed * relative_velocity  (i.e. -drag*relative_velocity)
        out.fused_multiply_subtract(relative_velocity, km * relative_speed);
    }

    /**
     * @brief Performs projectile trajectory simulation using the Velocity Verlet integration method.
     *
     * See bclibc/velocity_verlet.hpp for the algorithm description. This implementation carries the
     * acceleration computed at the end of one step into the start of the next, so each step
     * only requires one extra acceleration evaluation (at the predicted end-of-step state).
     *
     * @param eng The ballistics engine containing shot properties, atmospheric conditions, and configuration.
     * @param handler Interface for processing computed trajectory data points.
     * @param reason Output parameter indicating why the simulation terminated.
     */
    void BCLIBC_integrateVELOCITY_VERLET(
        BCLIBC_BaseEngine &eng,
        BCLIBC_BaseTrajDataHandlerInterface &handler,
        BCLIBC_TerminationReason &reason)
    {
        // Scalars
        double velocity = 0.0;
        double density_ratio = 0.0;
        double mach = 0.0;
        double time = 0.0;

        // Vectors
        BCLIBC_V3dT range_vector{};
        BCLIBC_V3dT velocity_vector{};
        BCLIBC_V3dT relative_velocity{};
        BCLIBC_V3dT gravity_vector{};
        BCLIBC_V3dT wind_vector{};
        BCLIBC_V3dT acceleration_vector{};
        BCLIBC_V3dT new_acceleration_vector{};
        BCLIBC_V3dT predicted_velocity{};

        // Initialize working variables
        reason = BCLIBC_TerminationReason::NO_TERMINATE;
        eng.integration_step_count = 0;

        const double delta_time = eng.shot.calc_step;

        if (delta_time <= 0.0)
        {
            BCLIBC_ERROR(
                "Invalid calc_step=%.9f (must be > 0); integration aborted", delta_time);
            reason = BCLIBC_TerminationReason::MINIMUM_VELOCITY_REACHED;
            return;
        }

        // Initialize gravity vector (pointing downward in y-axis)
        gravity_vector.x = 0.0;
        gravity_vector.y = eng.config.cGravityConstant;
        gravity_vector.z = 0.0;

        // Get initial wind conditions
        wind_vector = eng.shot.wind_sock.current_vector();

        // Initialize projectile state
        velocity = eng.shot.muzzle_velocity;

        // Set initial position accounting for sight height and cant angle
        range_vector.x = 0.0;
        range_vector.y = -eng.shot.cant_cosine * eng.shot.sight_height;
        range_vector.z = -eng.shot.cant_sine * eng.shot.sight_height;

        // Calculate initial direction vector from barrel elevation and azimuth
        const double cos_elev = std::cos(eng.shot.barrel_elevation);
        BCLIBC_V3dT dir_vector;
        dir_vector.x = cos_elev * std::cos(eng.shot.barrel_azimuth);
        dir_vector.y = std::sin(eng.shot.barrel_elevation);
        dir_vector.z = cos_elev * std::sin(eng.shot.barrel_azimuth);

        // Calculate initial velocity vector
        velocity_vector = dir_vector * velocity;

        // Acceleration at t=0; carried forward across steps thereafter
        eng.shot.atmo.update_density_factor_and_mach_for_altitude(
            eng.shot.alt0 + range_vector.y,
            density_ratio,
            mach);
        relative_velocity = velocity_vector - wind_vector;
        BCLIBC_velocity_verlet_acceleration(
            eng, velocity_vector, relative_velocity, relative_velocity.mag(),
            gravity_vector, density_ratio, mach, acceleration_vector);

        // Main trajectory integration loop
        // Continue until range limit is reached or termination condition is met
        while (reason == BCLIBC_TerminationReason::NO_TERMINATE)
        {
            eng.integration_step_count++;

            // Update wind vector if we've crossed into a new wind zone
            if (range_vector.x >= eng.shot.wind_sock.next_range)
            {
                wind_vector = eng.shot.wind_sock.vector_for_range(range_vector.x);
            }

            // Update atmospheric density and speed of sound at current altitude
            eng.shot.atmo.update_density_factor_and_mach_for_altitude(
                eng.shot.alt0 + range_vector.y,
                density_ratio,
                mach);

            // Record current trajectory point
            handler.handle(
                BCLIBC_BaseTrajData(time, range_vector, velocity_vector, mach));

            // === Velocity Verlet Integration Step ===
            // 1. Update position using the acceleration carried over from the previous step:
            //    x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt^2
            range_vector.fused_multiply_add(velocity_vector, delta_time);
            range_vector.fused_multiply_add(acceleration_vector, 0.5 * delta_time * delta_time);

            // 2. Predict velocity at t+dt using only a(t): v_pred = v(t) + a(t)*dt
            predicted_velocity = velocity_vector;
            predicted_velocity.fused_multiply_add(acceleration_vector, delta_time);

            // 3. Evaluate acceleration at the predicted end-of-step state: a(t+dt)
            relative_velocity = predicted_velocity - wind_vector;
            BCLIBC_velocity_verlet_acceleration(
                eng, predicted_velocity, relative_velocity, relative_velocity.mag(),
                gravity_vector, density_ratio, mach, new_acceleration_vector);

            // 4. Update velocity using the average of a(t) and a(t+dt):
            //    v(t+dt) = v(t) + 0.5*[a(t) + a(t+dt)]*dt
            velocity_vector.fused_multiply_add(acceleration_vector, 0.5 * delta_time);
            velocity_vector.fused_multiply_add(new_acceleration_vector, 0.5 * delta_time);

            // 5. Carry the new acceleration forward into the next step
            acceleration_vector = new_acceleration_vector;

            // 6. Update scalar velocity magnitude and simulation time
            velocity = velocity_vector.mag();
            time += delta_time;
        }

        // Record final trajectory point
        handler.handle(
            BCLIBC_BaseTrajData(time, range_vector, velocity_vector, mach));

        BCLIBC_DEBUG("Function exit, reason=%d\n", reason);
    }

}; // namespace bclibc
