#include <cassert>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include "bclibc/traj_data.hpp"

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

    std::printf("test_traj_data: all tests passed\n");
    return 0;
}
