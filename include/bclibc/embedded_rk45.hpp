#ifndef BCLIBC_EMBEDDED_RK45_HPP
#define BCLIBC_EMBEDDED_RK45_HPP

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "bclibc/base_types.hpp"
#include "bclibc/engine.hpp"
#include "bclibc/log.hpp"
#include "bclibc/traj_data.hpp"

namespace bclibc
{
    namespace embedded_rk45_detail
    {
        constexpr double default_tolerance = 1e-6;
        constexpr int max_retries = 24;
        constexpr double min_dt_divisor = 64.0;
        constexpr double max_dt_multiplier = 64.0;

        struct Deriv
        {
            BCLIBC_V3dT dvr;
            BCLIBC_V3dT dp;
        };

        inline Deriv derivative(const BCLIBC_ShotProps &shot,
                                const BCLIBC_V3dT &wind,
                                const BCLIBC_V3dT &gravity_plus_coriolis,
                                const BCLIBC_V3dT &vr,
                                const BCLIBC_V3dT &pos)
        {
            double density_ratio, mach_fps;
            shot.atmo.update_density_factor_and_mach_for_altitude(
                shot.alt0 + pos.y, density_ratio, mach_fps);
            const double speed = vr.mag();
            const double mach = speed / (mach_fps != 0.0 ? mach_fps : 1.0);
            const double km = density_ratio * shot.drag_by_mach(mach);
            Deriv result;
            result.dvr.linear_combination(gravity_plus_coriolis, 1.0, vr, -km * speed);
            result.dp = vr + wind;
            return result;
        }

        /** Generic compile-time adaptive embedded Runge--Kutta trajectory core.
         *
         * Accepted/rejected step counts are written to @p accepted_steps /
         * @p rejected_steps, and tolerances are read from @p relative_tolerance /
         * @p absolute_tolerance, both owned by the caller -- see @ref
         * bclibc::BCLIBC_CashKarpIntegrator and friends, which each hold their
         * own copy of this state per instance instead of it living in a
         * thread_local shared by every BCLIBC_BaseEngine on the thread using
         * the same method. */
        template <typename Tableau, typename Controller>
        void run(BCLIBC_BaseEngine &eng,
                BCLIBC_BaseTrajDataHandlerInterface &handler,
                BCLIBC_TerminationReason &reason,
                int &accepted_steps,
                int &rejected_steps,
                double relative_tolerance,
                double absolute_tolerance)
        {
            static_assert(Tableau::stages > 1, "embedded RK requires at least two stages");

            reason = BCLIBC_TerminationReason::NO_TERMINATE;
            eng.integration_step_count = 0;
            accepted_steps = rejected_steps = 0;

            const double base_dt = eng.shot.calc_step;
            if (base_dt <= 0.0)
            {
                BCLIBC_ERROR("Invalid calc_step=%.9f (must be > 0); integration aborted", base_dt);
                reason = BCLIBC_TerminationReason::MINIMUM_VELOCITY_REACHED;
                return;
            }
            const double min_dt = base_dt / min_dt_divisor;
            const double max_dt = base_dt * max_dt_multiplier;
            double dt = base_dt;

            const BCLIBC_V3dT gravity{0.0, eng.config.cGravityConstant, 0.0};
            BCLIBC_V3dT wind = eng.shot.wind_sock.current_vector();
            BCLIBC_V3dT pos{0.0, -eng.shot.cant_cosine * eng.shot.sight_height,
                           -eng.shot.cant_sine * eng.shot.sight_height};
            const double cos_elev = std::cos(eng.shot.barrel_elevation);
            const BCLIBC_V3dT direction{
                cos_elev * std::cos(eng.shot.barrel_azimuth),
                std::sin(eng.shot.barrel_elevation),
                cos_elev * std::sin(eng.shot.barrel_azimuth)};
            BCLIBC_V3dT velocity = direction * eng.shot.muzzle_velocity;
            BCLIBC_V3dT vr = velocity - wind;
            double time = 0.0;
            Deriv cached_first{};
            bool have_cached_first = false;

            double density_ratio, mach_fps;
            eng.shot.atmo.update_density_factor_and_mach_for_altitude(
                eng.shot.alt0 + pos.y, density_ratio, mach_fps);
            BCLIBC_BaseTrajData step_start(time, pos, velocity, mach_fps);
            handler.handle(step_start);

            while (reason == BCLIBC_TerminationReason::NO_TERMINATE)
            {
                ++eng.integration_step_count;
                bool wind_changed = false;
                if (pos.x + 1e-7 >= eng.shot.wind_sock.next_range)
                {
                    wind = eng.shot.wind_sock.vector_for_range(pos.x);
                    Controller::on_wind_change(velocity, wind, vr);
                    wind_changed = true;
                }

                BCLIBC_V3dT gravity_plus_coriolis = gravity;
                if (!eng.shot.coriolis.flat_fire_only)
                {
                    BCLIBC_V3dT coriolis{};
                    eng.shot.coriolis.coriolis_acceleration_local(velocity, coriolis);
                    gravity_plus_coriolis += coriolis;
                }
                if (wind_changed || !eng.shot.coriolis.flat_fire_only)
                    have_cached_first = false;

                Controller::limit_step_at_wind_boundary(
                    eng.shot.wind_sock.next_range, pos, vr, wind, dt);

                BCLIBC_V3dT vr_next{}, pos_next{};
                double dt_used = dt;
                bool rejected_attempt = false;
                for (int attempt = 0; attempt < max_retries; ++attempt)
                {
                    Deriv k[Tableau::stages];
                    for (int i = 0; i < Tableau::stages; ++i)
                    {
                        if (i == 0 && have_cached_first)
                        {
                            k[0] = cached_first;
                            continue;
                        }
                        BCLIBC_V3dT dv{}, dp{};
                        for (int j = 0; j < i; ++j)
                        {
                            dv += k[j].dvr * Tableau::A(i, j);
                            dp += k[j].dp * Tableau::A(i, j);
                        }
                        k[i] = derivative(eng.shot, wind, gravity_plus_coriolis,
                                          vr + dv * dt, pos + dp * dt);
                    }
                    cached_first = k[0];
                    have_cached_first = true;

                    BCLIBC_V3dT sum_v{}, sum_p{}, err_v{}, err_p{};
                    for (int i = 0; i < Tableau::stages; ++i)
                    {
                        sum_v += k[i].dvr * Tableau::B(i);
                        sum_p += k[i].dp * Tableau::B(i);
                        err_v += k[i].dvr * Tableau::E(i);
                        err_p += k[i].dp * Tableau::E(i);
                    }
                    vr_next = vr + sum_v * dt;
                    pos_next = pos + sum_p * dt;
                    err_v *= dt;
                    err_p *= dt;

                    const double svx = absolute_tolerance + relative_tolerance * std::max(std::abs(vr.x), std::abs(vr_next.x));
                    const double svy = absolute_tolerance + relative_tolerance * std::max(std::abs(vr.y), std::abs(vr_next.y));
                    const double svz = absolute_tolerance + relative_tolerance * std::max(std::abs(vr.z), std::abs(vr_next.z));
                    const double spx = absolute_tolerance + relative_tolerance * std::max(std::abs(pos.x), std::abs(pos_next.x));
                    const double spy = absolute_tolerance + relative_tolerance * std::max(std::abs(pos.y), std::abs(pos_next.y));
                    const double spz = absolute_tolerance + relative_tolerance * std::max(std::abs(pos.z), std::abs(pos_next.z));
                    const double error_norm = std::sqrt((
                        std::pow(err_v.x / svx, 2) + std::pow(err_v.y / svy, 2) + std::pow(err_v.z / svz, 2) +
                        std::pow(err_p.x / spx, 2) + std::pow(err_p.y / spy, 2) + std::pow(err_p.z / spz, 2)) / 6.0);

                    if (error_norm <= 1.0 || dt <= min_dt * 1.0001)
                    {
                        ++accepted_steps;
                        dt_used = dt;
                        dt = std::min(dt * Controller::accepted_factor(error_norm, rejected_attempt), max_dt);
                        if (Tableau::fsal)
                            cached_first = k[Tableau::stages - 1];
                        else
                            have_cached_first = false;
                        break;
                    }
                    ++rejected_steps;
                    rejected_attempt = true;
                    dt = std::max(dt * Controller::rejected_factor(error_norm), min_dt);
                }

                time += dt_used;
                vr = vr_next;
                pos = pos_next;
                velocity = vr + wind;
                eng.shot.atmo.update_density_factor_and_mach_for_altitude(
                    eng.shot.alt0 + pos.y, density_ratio, mach_fps);
                BCLIBC_BaseTrajData step_end(time, pos, velocity, mach_fps);
                handler.handle_step(step_start, step_end);
                step_start = step_end;
            }
        }
    }
}; // namespace bclibc

#endif  // BCLIBC_EMBEDDED_RK45_HPP
