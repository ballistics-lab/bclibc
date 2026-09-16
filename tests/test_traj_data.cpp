#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <vector>
#include "bclibc/traj_data.hpp"
#include "bclibc/traj_filter.hpp"

using namespace bclibc;

namespace
{
    BCLIBC_BaseTrajSeq make_increasing_seq()
    {
        // position.x runs 0..4, matching the repro from GitHub issue #19.
        BCLIBC_BaseTrajSeq seq;
        for (int t = 0; t < 5; ++t)
        {
            seq.append(BCLIBC_BaseTrajData(
                static_cast<double>(t), static_cast<double>(t), 0.0, 0.0,
                100.0, 0.0, 0.0, 1.0));
        }
        return seq;
    }

    BCLIBC_BaseTrajSeq make_decreasing_seq()
    {
        // position.x runs 4..0 (decreasing), to exercise the other monotonic branch.
        BCLIBC_BaseTrajSeq seq;
        for (int t = 0; t < 5; ++t)
        {
            seq.append(BCLIBC_BaseTrajData(
                static_cast<double>(t), static_cast<double>(4 - t), 0.0, 0.0,
                100.0, 0.0, 0.0, 1.0));
        }
        return seq;
    }

    // GitHub issue #19: get_at() must raise instead of silently extrapolating
    // when key_value falls outside the sequence's range.
    void test_get_at_rejects_out_of_range_above_increasing()
    {
        auto seq = make_increasing_seq();
        BCLIBC_BaseTrajData out;
        bool threw = false;
        try
        {
            seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, 104.0, 0.0, out);
        }
        catch (const std::out_of_range &)
        {
            threw = true;
        }
        assert(threw && "expected std::out_of_range for value above range");
    }

    void test_get_at_rejects_out_of_range_below_increasing()
    {
        auto seq = make_increasing_seq();
        BCLIBC_BaseTrajData out;
        bool threw = false;
        try
        {
            seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, -10.0, 0.0, out);
        }
        catch (const std::out_of_range &)
        {
            threw = true;
        }
        assert(threw && "expected std::out_of_range for value below range");
    }

    void test_get_at_rejects_out_of_range_decreasing()
    {
        auto seq = make_decreasing_seq();
        BCLIBC_BaseTrajData out;
        bool threw = false;
        try
        {
            seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, -1.0, 0.0, out);
        }
        catch (const std::out_of_range &)
        {
            threw = true;
        }
        assert(threw && "expected std::out_of_range for value beyond a decreasing sequence");
    }

    void test_get_at_interpolates_in_range()
    {
        auto seq = make_increasing_seq();
        BCLIBC_BaseTrajData out;
        seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, 2.5, 0.0, out);
        assert(std::fabs(out.px - 2.5) < 1e-9);
    }

    void test_get_at_matches_exact_endpoints()
    {
        auto seq = make_increasing_seq();
        BCLIBC_BaseTrajData out;

        seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, 0.0, 0.0, out);
        assert(std::fabs(out.px - 0.0) < 1e-9);

        seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, 4.0, 0.0, out);
        assert(std::fabs(out.px - 4.0) < 1e-9);
    }

    // A boundary value that's outside by only floating-point dust must still
    // resolve rather than being rejected by the new range check.
    void test_get_at_tolerates_epsilon_boundary_jitter()
    {
        auto seq = make_increasing_seq();
        BCLIBC_BaseTrajData out;

        seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, 4.0 + 1e-10, 0.0, out);
        assert(std::fabs(out.px - 4.0) < 1e-6);

        seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, 0.0 - 1e-10, 0.0, out);
        assert(std::fabs(out.px - 0.0) < 1e-6);
    }

    void test_get_at_requires_at_least_three_points()
    {
        BCLIBC_BaseTrajSeq seq;
        seq.append(BCLIBC_BaseTrajData(0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 1.0));
        seq.append(BCLIBC_BaseTrajData(1.0, 1.0, 0.0, 0.0, 100.0, 0.0, 0.0, 1.0));

        BCLIBC_BaseTrajData out;
        bool threw = false;
        try
        {
            seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, 0.5, 0.0, out);
        }
        catch (const std::domain_error &)
        {
            threw = true;
        }
        assert(threw && "expected std::domain_error for fewer than 3 points");
    }

    BCLIBC_ShotProps make_filter_test_props()
    {
        // TrajectoryData construction evaluates drag, so provide the smallest
        // valid constant drag curve even though this test feeds raw step data.
        BCLIBC_Curve curve = {BCLIBC_CurvePoint(0.0, 0.0, 0.0, 0.0)};
        BCLIBC_MachList mach_list = {0.0, 2.0};
        return BCLIBC_ShotProps(
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.1, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            curve, mach_list,
            BCLIBC_Atmosphere::from_conditions(15.0, 1013.25, 0.0),
            BCLIBC_Coriolis::from_lat_az(
                std::numeric_limits<double>::quiet_NaN(), 0.0,
                std::numeric_limits<double>::quiet_NaN()),
            BCLIBC_WindSock(),
            BCLIBC_TRAJ_FLAG_NONE);
    }

    void test_streaming_step_coalesces_zero_and_range()
    {
        std::vector<BCLIBC_TrajectoryData> records;
        BCLIBC_TerminationReason reason = BCLIBC_TerminationReason::NO_TERMINATE;
        BCLIBC_ShotProps props = make_filter_test_props();

        BCLIBC_TrajectoryDataFilter filter(
            records, props, BCLIBC_TRAJ_FLAG_ZERO, reason,
            100.0, 35.0, 0.0);
        BCLIBC_BaseTrajDataHandlerCompositor handler(&filter);

        // One deliberately wide accepted step. At t=1, the endpoint-Hermite
        // path is x=35 and y=0; linear x interpolation would not find that
        // range row at t=1. The ZERO_UP event must merge into the same row.
        const BCLIBC_BaseTrajData start(0.0, 0.0, -1.0, 0.0,
                                        20.0, 1.0, 0.0, 1100.0);
        const BCLIBC_BaseTrajData end(2.0, 100.0, 1.0, 0.0,
                                      80.0, 1.0, 0.0, 1100.0);

        handler.handle(start);
        handler.handle_step(start, end);

        int coalesced_count = 0;
        for (const BCLIBC_TrajectoryData &row : records)
        {
            const bool is_zero_up = (row.flag & BCLIBC_TRAJ_FLAG_ZERO_UP) != 0;
            const bool is_range = (row.flag & BCLIBC_TRAJ_FLAG_RANGE) != 0;
            if (is_zero_up && is_range)
            {
                ++coalesced_count;
                assert(std::fabs(row.distance_ft - 35.0) < 1e-9);
                assert(std::fabs(row.time - 1.0) < 1e-9);
                assert(std::fabs(row.height_ft) < 1e-9);
            }
        }
        assert(coalesced_count == 1);
    }
}

int main()
{
    test_get_at_rejects_out_of_range_above_increasing();
    test_get_at_rejects_out_of_range_below_increasing();
    test_get_at_rejects_out_of_range_decreasing();
    test_get_at_interpolates_in_range();
    test_get_at_matches_exact_endpoints();
    test_get_at_tolerates_epsilon_boundary_jitter();
    test_get_at_requires_at_least_three_points();
    test_streaming_step_coalesces_zero_and_range();

    std::printf("test_traj_data: all tests passed\n");
    return 0;
}
