#pragma once
// compare.hpp — tolerance-based comparison of bclibc vs tbclibc trajectory points.

#include <cmath>
#include <cstdio>
#include <string>

#include "bclibc/traj_data.hpp"
#include "tiny_bclibc/traj_data.h"

namespace identity
{
    // Double mode: bclibc (double) vs tbclibc (double) → tight tolerance.
    static constexpr double kAbsTol = 1e-9;

    struct FieldCmp {
        const char *name;
        double bclibc_val;
        double tbclibc_val;
    };

    inline bool check_field(const char *name, double a, double b, double tol,
                             int point_idx, bool verbose)
    {
        double diff = std::fabs(a - b);
        bool ok = (diff <= tol);
        if (!ok || verbose) {
            std::printf("  [pt %d] %-22s  bclibc=%.12g  tbclibc=%.12g  diff=%.3e  %s\n",
                        point_idx, name, a, b, diff, ok ? "OK" : "FAIL");
        }
        return ok;
    }

    // Compare one full TrajectoryData point.
    // Returns true if all fields within tolerance.
    inline bool compare_point(const bclibc::BCLIBC_TrajectoryData &bc,
                               const TINY_BCLIBC_TrajectoryData &tb,
                               int idx, double tol = kAbsTol, bool verbose = false)
    {
        bool ok = true;
#define CMP(field) ok &= check_field(#field, bc.field, tb.field, tol, idx, verbose)
        CMP(time);
        CMP(distance_ft);
        CMP(velocity_fps);
        CMP(mach);
        CMP(height_ft);
        CMP(slant_height_ft);
        CMP(drop_angle_rad);
        CMP(windage_ft);
        CMP(windage_angle_rad);
        CMP(slant_distance_ft);
        CMP(angle_rad);
        CMP(density_ratio);
        CMP(drag);
        CMP(energy_ft_lb);
        CMP(ogw_lb);
#undef CMP
        {
            bool flag_ok = (static_cast<int>(bc.flag) == tb.flag);
            if (!flag_ok || verbose)
                std::printf("  [pt %d] %-22s  bclibc=%d  tbclibc=%d  %s\n",
                            idx, "flag", static_cast<int>(bc.flag), tb.flag,
                            flag_ok ? "OK" : "FAIL");
            ok &= flag_ok;
        }
        return ok;
    }

    // Compare full trajectories (must be same length, matched by index).
    inline bool compare_trajectories(
        const std::vector<bclibc::BCLIBC_TrajectoryData> &bc_traj,
        const std::vector<TINY_BCLIBC_TrajectoryData>        &tb_traj,
        const char *label, double tol = kAbsTol)
    {
        std::printf("\n=== %s ===\n", label);

        if (bc_traj.size() != tb_traj.size()) {
            std::printf("  FAIL: different point counts: bclibc=%zu  tbclibc=%zu\n",
                        bc_traj.size(), tb_traj.size());
            return false;
        }
        bool all_ok = true;
        int failures = 0;
        for (int i = 0; i < static_cast<int>(bc_traj.size()); ++i) {
            bool ok = compare_point(bc_traj[i], tb_traj[i], i, tol, false);
            if (!ok) {
                compare_point(bc_traj[i], tb_traj[i], i, tol, true); // verbose on fail
                ++failures;
            }
            all_ok &= ok;
        }
        if (all_ok)
            std::printf("  PASS  (%zu points, tol=%.0e)\n", bc_traj.size(), tol);
        else
            std::printf("  FAIL  (%d/%zu points differ)\n", failures, bc_traj.size());
        return all_ok;
    }

    // Compare a single scalar (e.g., zero_angle result).
    inline bool compare_scalar(const char *label, double bc_val, double tb_val,
                                double tol = kAbsTol)
    {
        double diff = std::fabs(bc_val - tb_val);
        bool ok = (diff <= tol);
        std::printf("  %-30s  bclibc=%.12g  tbclibc=%.12g  diff=%.3e  %s\n",
                    label, bc_val, tb_val, diff, ok ? "PASS" : "FAIL");
        return ok;
    }

} // namespace identity
