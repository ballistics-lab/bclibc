#include <cmath>
#include <algorithm>
#include <stdexcept>
#include "bclibc/cash_karp.hpp"
#include "bclibc/base_types.hpp"
#include "bclibc/log.hpp"

namespace bclibc
{
    namespace
    {
        // Standard Numerical Recipes `rkck` Cash-Karp tableau.
        constexpr double A2 = 1.0 / 5.0;
        constexpr double A3 = 3.0 / 10.0;
        constexpr double A4 = 3.0 / 5.0;
        constexpr double A5 = 1.0;
        constexpr double A6 = 7.0 / 8.0;

        constexpr double B21 = 1.0 / 5.0;
        constexpr double B31 = 3.0 / 40.0, B32 = 9.0 / 40.0;
        constexpr double B41 = 3.0 / 10.0, B42 = -9.0 / 10.0, B43 = 6.0 / 5.0;
        constexpr double B51 = -11.0 / 54.0, B52 = 2.5, B53 = -70.0 / 27.0, B54 = 35.0 / 27.0;
        constexpr double B61 = 1631.0 / 55296.0, B62 = 175.0 / 512.0, B63 = 575.0 / 13824.0,
                          B64 = 44275.0 / 110592.0, B65 = 253.0 / 4096.0;

        // 5th-order solution coefficients.
        constexpr double C1 = 37.0 / 378.0, C3 = 250.0 / 621.0, C4 = 125.0 / 594.0, C6 = 512.0 / 1771.0;
        // Error coefficients (5th-order minus embedded 4th-order).
        constexpr double D1 = C1 - 2825.0 / 27648.0;
        constexpr double D3 = C3 - 18575.0 / 48384.0;
        constexpr double D4 = C4 - 13525.0 / 55296.0;
        constexpr double D5 = 0.0 - 277.0 / 14336.0;
        constexpr double D6 = C6 - 0.25;

        constexpr double kSafety = 0.9;
        constexpr double kAtolVelocity = 1e-3; // fps floor
        constexpr double kAtolPosition = 1e-4; // ft floor
        constexpr double kDefaultRelTolerance = 1e-6;
        constexpr int kMaxRetryPerStep = 24;
        constexpr double kMinDtDivisor = 64.0;
        constexpr double kMaxDtMultiplier = 64.0;

        thread_local int g_ck_accepted = 0;
        thread_local int g_ck_rejected = 0;
        thread_local double g_ck_rel_tolerance = kDefaultRelTolerance;

        struct Deriv
        {
            BCLIBC_V3dT dvr; // d(v_rel)/dt
            BCLIBC_V3dT dp;  // d(pos)/dt
        };

        static inline void calculate_dvdt(
            const BCLIBC_V3dT &v_rel,
            const BCLIBC_V3dT &gravity_plus_coriolis,
            double km_coeff,
            double v_mag,
            BCLIBC_V3dT &acceleration) noexcept
        {
            acceleration.linear_combination(gravity_plus_coriolis, 1.0, v_rel, -km_coeff * v_mag);
        }

        // Per-stage derivative: drag coefficient AND atmosphere sample are both
        // re-evaluated fresh from this stage's own intermediate velocity/altitude.
        // See cash_karp.hpp's doc comment for why this is required, not optional.
        static inline Deriv ck_deriv(
            const BCLIBC_ShotProps &shot,
            const BCLIBC_V3dT &wind,
            const BCLIBC_V3dT &gravity_plus_coriolis,
            const BCLIBC_V3dT &vr,
            const BCLIBC_V3dT &pos)
        {
            double density_ratio, mach_fps;
            shot.atmo.update_density_factor_and_mach_for_altitude(
                shot.alt0 + pos.y, density_ratio, mach_fps);
            const double inv_mach = (mach_fps != 0.0) ? (1.0 / mach_fps) : 1.0;
            const double speed = vr.mag();
            const double mach = speed * inv_mach;
            const double km = density_ratio * shot.drag_by_mach(mach);

            Deriv d;
            calculate_dvdt(vr, gravity_plus_coriolis, km, speed, d.dvr);
            d.dp = vr + wind;
            return d;
        }
    } // namespace

    void BCLIBC_cashKarpGetStats(int &out_accepted, int &out_rejected)
    {
        out_accepted = g_ck_accepted;
        out_rejected = g_ck_rejected;
    }

    void BCLIBC_cashKarpSetRelativeTolerance(double tolerance)
    {
        if (!std::isfinite(tolerance) || tolerance <= 0.0)
            throw std::invalid_argument("Cash-Karp relative tolerance must be finite and positive");
        g_ck_rel_tolerance = tolerance;
    }

    void BCLIBC_integrateCashKarp(
        BCLIBC_BaseEngine &eng,
        BCLIBC_BaseTrajDataHandlerInterface &handler,
        BCLIBC_TerminationReason &reason)
    {
        reason = BCLIBC_TerminationReason::NO_TERMINATE;
        eng.integration_step_count = 0;
        g_ck_accepted = 0;
        g_ck_rejected = 0;

        const double base_dt = eng.shot.calc_step;
        if (base_dt <= 0.0)
        {
            BCLIBC_ERROR("Invalid calc_step=%.9f (must be > 0); integration aborted", base_dt);
            reason = BCLIBC_TerminationReason::MINIMUM_VELOCITY_REACHED;
            return;
        }
        const double min_dt = base_dt / kMinDtDivisor;
        const double max_dt = base_dt * kMaxDtMultiplier;
        double dt = base_dt;

        BCLIBC_V3dT gravity_vector{0.0, eng.config.cGravityConstant, 0.0};
        BCLIBC_V3dT wind_vector = eng.shot.wind_sock.current_vector();

        BCLIBC_V3dT range_vector{
            0.0,
            -eng.shot.cant_cosine * eng.shot.sight_height,
            -eng.shot.cant_sine * eng.shot.sight_height};

        const double cos_elev = std::cos(eng.shot.barrel_elevation);
        BCLIBC_V3dT dir_vector{
            cos_elev * std::cos(eng.shot.barrel_azimuth),
            std::sin(eng.shot.barrel_elevation),
            cos_elev * std::sin(eng.shot.barrel_azimuth)};
        BCLIBC_V3dT velocity_vector = dir_vector * eng.shot.muzzle_velocity;
        BCLIBC_V3dT vr = velocity_vector - wind_vector;

        double time = 0.0;

        double density_ratio, mach_fps;
        eng.shot.atmo.update_density_factor_and_mach_for_altitude(
            eng.shot.alt0 + range_vector.y, density_ratio, mach_fps);
        BCLIBC_BaseTrajData step_start(time, range_vector, velocity_vector, mach_fps);
        handler.handle(step_start);

        while (reason == BCLIBC_TerminationReason::NO_TERMINATE)
        {
            eng.integration_step_count++;

            if (range_vector.x >= eng.shot.wind_sock.next_range)
                wind_vector = eng.shot.wind_sock.vector_for_range(range_vector.x);

            double density_ratio, mach_fps;
            eng.shot.atmo.update_density_factor_and_mach_for_altitude(
                eng.shot.alt0 + range_vector.y, density_ratio, mach_fps);

            BCLIBC_V3dT gravity_plus_coriolis = gravity_vector;
            if (!eng.shot.coriolis.flat_fire_only)
            {
                BCLIBC_V3dT coriolis_acc{};
                eng.shot.coriolis.coriolis_acceleration_local(velocity_vector, coriolis_acc);
                gravity_plus_coriolis += coriolis_acc;
            }

            BCLIBC_V3dT vr_next{}, pos_next{};
            double dt_used = dt;
            for (int attempt = 0; attempt < kMaxRetryPerStep; ++attempt)
            {
                const Deriv k1 = ck_deriv(eng.shot, wind_vector, gravity_plus_coriolis, vr, range_vector);

                const BCLIBC_V3dT vr2 = vr + k1.dvr * (dt * A2);
                const BCLIBC_V3dT p2 = range_vector + k1.dp * (dt * A2);
                const Deriv k2 = ck_deriv(eng.shot, wind_vector, gravity_plus_coriolis, vr2, p2);

                const BCLIBC_V3dT vr3 = vr + (k1.dvr * B31 + k2.dvr * B32) * dt;
                const BCLIBC_V3dT p3 = range_vector + (k1.dp * B31 + k2.dp * B32) * dt;
                const Deriv k3 = ck_deriv(eng.shot, wind_vector, gravity_plus_coriolis, vr3, p3);

                const BCLIBC_V3dT vr4 = vr + (k1.dvr * B41 + k2.dvr * B42 + k3.dvr * B43) * dt;
                const BCLIBC_V3dT p4 = range_vector + (k1.dp * B41 + k2.dp * B42 + k3.dp * B43) * dt;
                const Deriv k4 = ck_deriv(eng.shot, wind_vector, gravity_plus_coriolis, vr4, p4);

                const BCLIBC_V3dT vr5s = vr + (k1.dvr * B51 + k2.dvr * B52 + k3.dvr * B53 + k4.dvr * B54) * dt;
                const BCLIBC_V3dT p5s = range_vector + (k1.dp * B51 + k2.dp * B52 + k3.dp * B53 + k4.dp * B54) * dt;
                const Deriv k5 = ck_deriv(eng.shot, wind_vector, gravity_plus_coriolis, vr5s, p5s);

                const BCLIBC_V3dT vr6 = vr + (k1.dvr * B61 + k2.dvr * B62 + k3.dvr * B63 + k4.dvr * B64 + k5.dvr * B65) * dt;
                const BCLIBC_V3dT p6 = range_vector + (k1.dp * B61 + k2.dp * B62 + k3.dp * B63 + k4.dp * B64 + k5.dp * B65) * dt;
                const Deriv k6 = ck_deriv(eng.shot, wind_vector, gravity_plus_coriolis, vr6, p6);

                vr_next = vr + (k1.dvr * C1 + k3.dvr * C3 + k4.dvr * C4 + k6.dvr * C6) * dt;
                pos_next = range_vector + (k1.dp * C1 + k3.dp * C3 + k4.dp * C4 + k6.dp * C6) * dt;

                const BCLIBC_V3dT err_v = (k1.dvr * D1 + k3.dvr * D3 + k4.dvr * D4 + k5.dvr * D5 + k6.dvr * D6) * dt;
                const BCLIBC_V3dT err_p = (k1.dp * D1 + k3.dp * D3 + k4.dp * D4 + k5.dp * D5 + k6.dp * D6) * dt;

                const double scale_v = kAtolVelocity + g_ck_rel_tolerance * vr_next.mag();
                const double scale_p = kAtolPosition + g_ck_rel_tolerance * pos_next.mag();
                const double err_norm = std::max(err_v.mag() / scale_v, err_p.mag() / scale_p);

                if (err_norm <= 1.0 || dt <= min_dt * 1.0001)
                {
                    ++g_ck_accepted;
                    dt_used = dt; // the step actually just integrated, before growing dt for next time
                    double grow = (err_norm > 1.89e-4)
                                      ? kSafety * std::pow(err_norm, -0.20)
                                      : 5.0;
                    // clamp(grow, 1.0, 5.0) -- never shrink on an accepted step
                    grow = std::max(1.0, std::min(grow, 5.0));
                    dt = std::min(dt * grow, max_dt);
                    break;
                }
                ++g_ck_rejected;
                // clamp(shrink, 0.1, 0.9) -- must actually shrink, but not collapse
                double shrink = kSafety * std::pow(err_norm, -0.25);
                shrink = std::max(0.1, std::min(shrink, 0.9));
                dt = std::max(dt * shrink, min_dt);
            }

            time += dt_used;
            vr = vr_next;
            range_vector = pos_next;
            velocity_vector = vr + wind_vector;
            eng.shot.atmo.update_density_factor_and_mach_for_altitude(
                eng.shot.alt0 + range_vector.y, density_ratio, mach_fps);
            BCLIBC_BaseTrajData step_end(time, range_vector, velocity_vector, mach_fps);
            handler.handle_step(step_start, step_end);
            step_start = step_end;
        }
    }

}; // namespace bclibc
