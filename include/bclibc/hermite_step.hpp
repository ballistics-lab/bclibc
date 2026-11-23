#pragma once

// The one interpolation convention for everything that works with an integration step: the cubic Hermite interpolant of the
// step, built from the two states with derivatives (position and velocity at the start and at the end of the step).
//
//   position(u)  cubic Hermite of the end positions with the end velocities as slopes, u = (t - t0) / (t1 - t0)
//   velocity(u)  the derivative of that polynomial (a quadratic; it reproduces both end velocities exactly)
//   mach(u)      linear
//
// Used by the trajectory filter (rows by range / time, apex, Mach and zero events), by BCLIBC_SinglePointHandler (find_apex,
// integrate_at / integrate_raw_at, every key), by BCLIBC_ZeroCrossingHandler and by the dense-output queries. The 3-point
// PCHIP of raw points is only the fallback of a query that no step brackets.
//
// Root finding. Every key of a step is a polynomial of u of degree <= 3 (positions: cubic, velocities: quadratic, Mach:
// linear), so the crossing is found on the polynomial: a bracketed Newton iteration (the bracket [0, 1] is kept and
// narrowed on every evaluation, a Newton step that leaves it is replaced by a bisection), which reaches the roundoff
// level in 3-5 evaluations. The state at the root is then evaluated once. Quantities that are not such a polynomial (the
// filter's speed-minus-Mach event) use hermite_at_value(): bisection on the full interpolant.

#include <cmath>
#include <functional>

#include "bclibc/traj_data.hpp"

// The root solves run once per query (or per sampled row), not per step: keep them out of line so the per-step test of the step
// loop (find_apex, integrate_at) stays small.
#if defined(_MSC_VER)
#define BCLIBC_HERMITE_COLD __declspec(noinline)
#else
#define BCLIBC_HERMITE_COLD __attribute__((noinline))
#endif

namespace bclibc
{
    namespace hermite_step
    {
        /** a u^3 + b u^2 + c u + d, u in [0, 1] over a step. */
        struct Cubic
        {
            double a, b, c, d;
            double value(double u) const { return ((a * u + b) * u + c) * u + d; }
            double slope(double u) const { return (3.0 * a * u + 2.0 * b) * u + c; }
        };

        /** Position polynomial of a step from the end values y0, y1 and the end slopes m0, m1 (per unit of time). */
        inline Cubic position_cubic(double y0, double y1, double m0, double m1, double dt)
        {
            const double c = dt * m0;
            return Cubic{2.0 * y0 - 2.0 * y1 + c + dt * m1, -3.0 * y0 + 3.0 * y1 - 2.0 * c - dt * m1, c, y0};
        }

        /** The derivative with respect to time of a position polynomial (the velocity of the point), as a polynomial of u. */
        inline Cubic velocity_cubic(const Cubic &p, double dt)
        {
            return Cubic{0.0, 3.0 * p.a / dt, 2.0 * p.b / dt, p.c / dt};
        }

        inline Cubic combine(double k0, const Cubic &p0, double k1, const Cubic &p1)
        {
            return Cubic{k0 * p0.a + k1 * p1.a, k0 * p0.b + k1 * p1.b, k0 * p0.c + k1 * p1.c, k0 * p0.d + k1 * p1.d};
        }

        /**
         * The root of c(u) = target in [0, 1], given the exact values f0 = start value - target and f1 = end value - target
         * (they decide the bracket). Returns false when the end values do not bracket the target.
         */
        inline bool solve(const Cubic &c, double f0, double f1, double target, double &u_out)
        {
            if (f0 == 0.0)
            {
                u_out = 0.0;
                return true;
            }
            if (f1 == 0.0)
            {
                u_out = 1.0;
                return true;
            }
            if (!((f0 < 0.0) != (f1 < 0.0)))
                return false;
            double lo = 0.0, hi = 1.0, flo = f0;
            double u = f0 / (f0 - f1); // the secant of the two ends: exact for a linear key
            for (int i = 0; i < 64; ++i)
            {
                const double f = c.value(u) - target;
                if (f == 0.0)
                    break;
                if ((f < 0.0) == (flo < 0.0))
                {
                    lo = u;
                    flo = f;
                }
                else
                {
                    hi = u;
                }
                double next;
                const double df = c.slope(u);
                if (i < 40 && df != 0.0)
                    next = u - f / df;
                else
                    next = 0.5 * (lo + hi);
                if (!(next > lo && next < hi))
                    next = 0.5 * (lo + hi);
                const double step = std::fabs(next - u);
                u = next;
                if (step <= 1e-15 || hi - lo <= 1e-15)
                    break;
            }
            u_out = u;
            return true;
        }

        /** The state of the step at `time`: the Hermite position, its derivative as velocity, linear Mach. */
        inline bool at_time(const BCLIBC_BaseTrajData &start, const BCLIBC_BaseTrajData &end, double time, BCLIBC_BaseTrajData &out)
        {
            const double dt = end.time - start.time;
            if (!(dt > 0.0))
                return false;
            const double u = (time - start.time) / dt;
            out.time = time;
            const Cubic px = position_cubic(start.px, end.px, start.vx, end.vx, dt);
            const Cubic py = position_cubic(start.py, end.py, start.vy, end.vy, dt);
            const Cubic pz = position_cubic(start.pz, end.pz, start.vz, end.vz, dt);
            out.px = px.value(u);
            out.py = py.value(u);
            out.pz = pz.value(u);
            // The velocity is the derivative of the position polynomial: it keeps both end velocities exactly and the
            // state internally consistent without any earlier step.
            out.vx = px.slope(u) / dt;
            out.vy = py.slope(u) / dt;
            out.vz = pz.slope(u) / dt;
            out.mach = start.mach + u * (end.mach - start.mach);
            return true;
        }

        /** The polynomial of a key over the step; false for TIME (linear in u, handled by the callers) or an unknown key. */
        inline bool key_cubic(const BCLIBC_BaseTrajData &s, const BCLIBC_BaseTrajData &e, BCLIBC_BaseTrajData_InterpKey key, double dt, Cubic &out)
        {
            switch (key)
            {
            case BCLIBC_BaseTrajData_InterpKey::POS_X:
                out = position_cubic(s.px, e.px, s.vx, e.vx, dt);
                return true;
            case BCLIBC_BaseTrajData_InterpKey::POS_Y:
                out = position_cubic(s.py, e.py, s.vy, e.vy, dt);
                return true;
            case BCLIBC_BaseTrajData_InterpKey::POS_Z:
                out = position_cubic(s.pz, e.pz, s.vz, e.vz, dt);
                return true;
            case BCLIBC_BaseTrajData_InterpKey::VEL_X:
                out = velocity_cubic(position_cubic(s.px, e.px, s.vx, e.vx, dt), dt);
                return true;
            case BCLIBC_BaseTrajData_InterpKey::VEL_Y:
                out = velocity_cubic(position_cubic(s.py, e.py, s.vy, e.vy, dt), dt);
                return true;
            case BCLIBC_BaseTrajData_InterpKey::VEL_Z:
                out = velocity_cubic(position_cubic(s.pz, e.pz, s.vz, e.vz, dt), dt);
                return true;
            case BCLIBC_BaseTrajData_InterpKey::MACH:
                out = Cubic{0.0, 0.0, e.mach - s.mach, s.mach};
                return true;
            default:
                return false;
            }
        }

        /** The field of the state selected by `key` (inline: the step loop of find_apex / integrate_at calls it per step). */
        inline double key_value(const BCLIBC_BaseTrajData &p, BCLIBC_BaseTrajData_InterpKey key)
        {
            switch (key)
            {
            case BCLIBC_BaseTrajData_InterpKey::TIME: return p.time;
            case BCLIBC_BaseTrajData_InterpKey::MACH: return p.mach;
            case BCLIBC_BaseTrajData_InterpKey::POS_X: return p.px;
            case BCLIBC_BaseTrajData_InterpKey::POS_Y: return p.py;
            case BCLIBC_BaseTrajData_InterpKey::POS_Z: return p.pz;
            case BCLIBC_BaseTrajData_InterpKey::VEL_X: return p.vx;
            case BCLIBC_BaseTrajData_InterpKey::VEL_Y: return p.vy;
            case BCLIBC_BaseTrajData_InterpKey::VEL_Z: return p.vz;
            default: return 0.0;
            }
        }

        /** Whether `target` lies between the two key values, ends included. */
        inline bool brackets(double k0, double k1, double target)
        {
            return (k0 <= target && target <= k1) || (k1 <= target && target <= k0);
        }

        inline void assign_key(BCLIBC_BaseTrajData &out, BCLIBC_BaseTrajData_InterpKey key, double value)
        {
            switch (key)
            {
            case BCLIBC_BaseTrajData_InterpKey::TIME: out.time = value; break;
            case BCLIBC_BaseTrajData_InterpKey::MACH: out.mach = value; break;
            case BCLIBC_BaseTrajData_InterpKey::POS_X: out.px = value; break;
            case BCLIBC_BaseTrajData_InterpKey::POS_Y: out.py = value; break;
            case BCLIBC_BaseTrajData_InterpKey::POS_Z: out.pz = value; break;
            case BCLIBC_BaseTrajData_InterpKey::VEL_X: out.vx = value; break;
            case BCLIBC_BaseTrajData_InterpKey::VEL_Y: out.vy = value; break;
            case BCLIBC_BaseTrajData_InterpKey::VEL_Z: out.vz = value; break;
            default: break;
            }
        }

        /**
         * The state of the step where `key` equals `target`, or false when the step does not bracket it (the key at the two
         * ends, ends included) or is degenerate. The key of the result is set to the exact target.
         */
        BCLIBC_HERMITE_COLD inline bool at_key(const BCLIBC_BaseTrajData &start, const BCLIBC_BaseTrajData &end, BCLIBC_BaseTrajData_InterpKey key,
                           double target, BCLIBC_BaseTrajData &out)
        {
            const double k0 = key_value(start, key), k1 = key_value(end, key);
            if (!brackets(k0, k1, target))
                return false;
            const double dt = end.time - start.time;
            if (!(dt > 0.0))
                return false;
            double u;
            if (key == BCLIBC_BaseTrajData_InterpKey::TIME)
            {
                u = (target - k0) / dt;
            }
            else
            {
                Cubic c;
                if (!key_cubic(start, end, key, dt, c))
                    return false;
                if (!solve(c, k0 - target, k1 - target, target, u))
                    return false;
            }
            if (!at_time(start, end, start.time + u * dt, out))
                return false;
            // The query axis gets the exact target: the root is converged in u, and re-deriving the key from it would put
            // the polynomial's own rounding on top (see tiny_bclibc, which does this in single precision).
            assign_key(out, key, target);
            return true;
        }

        /**
         * The state where the slant height py cos(a) - px sin(a) equals `target` (cos/sin of the look angle given), within the
         * step, or false when the step does not bracket it. Slant height is a combination of two positions: a cubic of u.
         */
        BCLIBC_HERMITE_COLD inline bool at_slant(const BCLIBC_BaseTrajData &start, const BCLIBC_BaseTrajData &end, double ca, double sa, double target,
                             BCLIBC_BaseTrajData &out)
        {
            const double dt = end.time - start.time;
            if (!(dt > 0.0))
                return false;
            const Cubic c = combine(ca, position_cubic(start.py, end.py, start.vy, end.vy, dt), -sa,
                                    position_cubic(start.px, end.px, start.vx, end.vx, dt));
            double u;
            if (!solve(c, (start.py * ca - start.px * sa) - target, (end.py * ca - end.px * sa) - target, target, u))
                return false;
            return at_time(start, end, start.time + u * dt, out);
        }

        /**
         * The state where an arbitrary scalar of the state (speed minus Mach, ...) is `target`: bisection on the full
         * interpolant, for quantities that are not a polynomial of u. `out = start` / `end` when it is exact there.
         */
        inline bool at_value(const BCLIBC_BaseTrajData &start, const BCLIBC_BaseTrajData &end,
                             const std::function<double(const BCLIBC_BaseTrajData &)> &value, double target, BCLIBC_BaseTrajData &out)
        {
            double lo = start.time;
            double hi = end.time;
            double flo = value(start) - target;
            const double fhi = value(end) - target;
            if (flo == 0.0)
            {
                out = start;
                return true;
            }
            if (fhi == 0.0)
            {
                out = end;
                return true;
            }
            if ((flo < 0.0) == (fhi < 0.0))
                return false;
            for (int i = 0; i < 40; ++i)
            {
                const double mid = 0.5 * (lo + hi);
                BCLIBC_BaseTrajData sample;
                if (!at_time(start, end, mid, sample))
                    return false;
                const double fmid = value(sample) - target;
                if ((flo < 0.0) != (fmid < 0.0))
                {
                    hi = mid;
                }
                else
                {
                    lo = mid;
                    flo = fmid;
                }
            }
            return at_time(start, end, 0.5 * (lo + hi), out);
        }
    } // namespace hermite_step
} // namespace bclibc
