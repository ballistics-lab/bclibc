#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <vector>
#include "bclibc/exceptions.hpp"
#include "bclibc/traj_data.hpp"
#include "bclibc/traj_filter.hpp"

using namespace bclibc;

namespace
{
    template <typename Error, typename Value>
    const Error &expect_error(const BCLIBC_Result<Value> &result)
    {
        assert(has_error(result));
        const auto *error = result.error().template as<Error>();
        assert(error != nullptr);
        return *error;
    }

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

    BCLIBC_BaseTrajSeq make_slant_height_seq()
    {
        BCLIBC_BaseTrajSeq seq;
        for (int t = 0; t < 5; ++t)
        {
            seq.append(BCLIBC_BaseTrajData(
                static_cast<double>(t), static_cast<double>(t), static_cast<double>(t), 0.0,
                100.0, 0.0, 0.0, 1.0));
        }
        return seq;
    }

    // GitHub issue #19: get_at() must report an error instead of silently extrapolating
    // when key_value falls outside the sequence's range.
    void test_get_at_rejects_out_of_range_above_increasing()
    {
        auto seq = make_increasing_seq();
        BCLIBC_BaseTrajData out;
        const auto result = seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, 104.0, 0.0, out);
        const auto &error = expect_error<BCLIBC_OutOfRangeError>(result);
        assert(error.requested == 104.0 && error.minimum == 0.0 && error.maximum == 4.0);
    }

    void test_get_at_rejects_out_of_range_below_increasing()
    {
        auto seq = make_increasing_seq();
        BCLIBC_BaseTrajData out;
        const auto result = seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, -10.0, 0.0, out);
        const auto &error = expect_error<BCLIBC_OutOfRangeError>(result);
        assert(error.requested == -10.0 && error.minimum == 0.0 && error.maximum == 4.0);
    }

    void test_get_at_rejects_out_of_range_decreasing()
    {
        auto seq = make_decreasing_seq();
        BCLIBC_BaseTrajData out;
        const auto result = seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, -1.0, 0.0, out);
        const auto &error = expect_error<BCLIBC_OutOfRangeError>(result);
        assert(error.requested == -1.0 && error.minimum == 0.0 && error.maximum == 4.0);
    }

    void test_get_at_interpolates_in_range()
    {
        auto seq = make_increasing_seq();
        BCLIBC_BaseTrajData out;
        assert(!has_error(seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, 2.5, 0.0, out)));
        assert(std::fabs(out.px - 2.5) < 1e-9);
    }

    void test_get_at_matches_exact_endpoints()
    {
        auto seq = make_increasing_seq();
        BCLIBC_BaseTrajData out;

        assert(!has_error(seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, 0.0, 0.0, out)));
        assert(std::fabs(out.px - 0.0) < 1e-9);

        assert(!has_error(seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, 4.0, 0.0, out)));
        assert(std::fabs(out.px - 4.0) < 1e-9);
    }

    // A boundary value that's outside by only floating-point dust must still
    // resolve rather than being rejected by the new range check.
    void test_get_at_tolerates_epsilon_boundary_jitter()
    {
        auto seq = make_increasing_seq();
        BCLIBC_BaseTrajData out;

        assert(!has_error(seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, 4.0 + 1e-10, 0.0, out)));
        assert(std::fabs(out.px - 4.0) < 1e-6);

        assert(!has_error(seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, 0.0 - 1e-10, 0.0, out)));
        assert(std::fabs(out.px - 0.0) < 1e-6);
    }

    void test_get_at_requires_at_least_three_points()
    {
        BCLIBC_BaseTrajSeq seq;
        seq.append(BCLIBC_BaseTrajData(0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 1.0));
        seq.append(BCLIBC_BaseTrajData(1.0, 1.0, 0.0, 0.0, 100.0, 0.0, 0.0, 1.0));

        BCLIBC_BaseTrajData out;
        const auto result = seq.get_at(BCLIBC_BaseTrajData_InterpKey::POS_X, 0.5, 0.0, out);
        expect_error<BCLIBC_DomainError>(result);
    }

    void test_sequence_index_returns_value_result()
    {
        auto seq = make_increasing_seq();

        const auto first = seq[0];
        const auto *first_ref = (first.has_value() ? &first.value() : nullptr);
        assert(first_ref != nullptr);
        assert(first_ref->get().time == 0.0);

        const auto last = seq[-1];
        const auto *last_ref = (last.has_value() ? &last.value() : nullptr);
        assert(last_ref != nullptr);
        assert(last_ref->get().time == 4.0);

        const auto out_of_range = seq[5];
        const auto &error = expect_error<BCLIBC_OutOfRangeError>(out_of_range);
        assert(error.requested == 5.0 && error.minimum == 0.0 && error.maximum == 4.0);
    }

    void test_get_at_slant_height_returns_value_result()
    {
        auto seq = make_slant_height_seq();
        BCLIBC_BaseTrajData out;
        assert(!has_error(seq.get_at_slant_height(0.0, 2.5, out)));
        assert(std::fabs(out.py - 2.5) < 1e-9);

        BCLIBC_BaseTrajSeq short_seq;
        short_seq.append(BCLIBC_BaseTrajData(0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 1.0));
        short_seq.append(BCLIBC_BaseTrajData(1.0, 1.0, 1.0, 0.0, 100.0, 0.0, 0.0, 1.0));
        expect_error<BCLIBC_DomainError>(short_seq.get_at_slant_height(0.0, 0.5, out));

        const auto degenerate = make_increasing_seq().get_at_slant_height(0.0, 0.0, out);
        expect_error<BCLIBC_DomainError>(degenerate);
    }

    void test_base_interpolate_returns_value_error()
    {
        const BCLIBC_BaseTrajData p0(0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 1.0);
        const BCLIBC_BaseTrajData p1(1.0, 1.0, 2.0, 3.0, 2.0, 3.0, 4.0, 2.0);
        const BCLIBC_BaseTrajData p2(2.0, 2.0, 4.0, 6.0, 3.0, 4.0, 5.0, 3.0);
        BCLIBC_BaseTrajData out;

        const auto success = BCLIBC_BaseTrajData::interpolate(
            BCLIBC_BaseTrajData_InterpKey::TIME, 0.5, p0, p1, p2, out);
        assert(!has_error(success));
        assert(std::fabs(out.time - 0.5) < 1e-12);

        const auto failure = BCLIBC_BaseTrajData::interpolate(
            BCLIBC_BaseTrajData_InterpKey::TIME, 0.5, p0, p0, p2, out);
        assert(has_error(failure));

        const auto *domain = failure.error().as<BCLIBC_DomainError>();
        assert(domain != nullptr);
        assert(domain->lhs == 0.0 && domain->rhs == 0.0);
    }

    void test_trajectory_interpolate_returns_value_result()
    {
        BCLIBC_TrajectoryData p0, p1, p2;
        p0.time = 0.0;
        p1.time = 1.0;
        p2.time = 2.0;

        const auto success = BCLIBC_TrajectoryData::interpolate(
            BCLIBC_TrajectoryData_InterpKey::TIME, 0.5, p0, p1, p2, BCLIBC_TRAJ_FLAG_MACH);
        const auto *data = (success.has_value() ? &success.value() : nullptr);
        assert(data != nullptr);
        assert(std::fabs(data->time - 0.5) < 1e-12);
        assert(data->flag == BCLIBC_TRAJ_FLAG_MACH);

        const auto bad_key = BCLIBC_TrajectoryData::interpolate(
            static_cast<BCLIBC_TrajectoryData_InterpKey>(-1), 0.0, p0, p1, p2, BCLIBC_TRAJ_FLAG_NONE);
        expect_error<BCLIBC_LogicError>(bad_key);

        const auto bad_method = BCLIBC_TrajectoryData::interpolate(
            BCLIBC_TrajectoryData_InterpKey::TIME, 0.5, p0, p1, p2, BCLIBC_TRAJ_FLAG_NONE,
            static_cast<BCLIBC_InterpMethod>(-1));
        expect_error<BCLIBC_InvalidArgumentError>(bad_method);

        p1.time = 0.0;
        const auto zero_division = BCLIBC_TrajectoryData::interpolate(
            BCLIBC_TrajectoryData_InterpKey::TIME, 0.0, p0, p1, p2, BCLIBC_TRAJ_FLAG_NONE,
            BCLIBC_InterpMethod::LINEAR);
        expect_error<BCLIBC_DomainError>(zero_division);
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

    void test_filter_get_record_returns_value_result()
    {
        std::vector<BCLIBC_TrajectoryData> records;
        BCLIBC_TerminationReason reason = BCLIBC_TerminationReason::NO_TERMINATE;
        const BCLIBC_ShotProps props = make_filter_test_props();
        BCLIBC_TrajectoryDataFilter filter(
            records, props, BCLIBC_TRAJ_FLAG_NONE, reason, 100.0, 35.0, 0.0);

        BCLIBC_TrajectoryData record;
        record.time = 1.0;
        filter.append(record);

        const auto last = filter.get_record(-1);
        const auto *last_ref = (last.has_value() ? &last.value() : nullptr);
        assert(last_ref != nullptr && last_ref->get().time == 1.0);

        const auto out_of_range = filter.get_record(1);
        const auto &error = expect_error<BCLIBC_OutOfRangeError>(out_of_range);
        assert(error.requested == 1.0 && error.minimum == 0.0 && error.maximum == 0.0);
    }

    void test_single_point_handler_returns_value_results()
    {
        BCLIBC_SinglePointHandler handler(BCLIBC_BaseTrajData_InterpKey::TIME, 1.0, nullptr);
        expect_error<BCLIBC_RuntimeError>(handler.get_result());
        expect_error<BCLIBC_OutOfRangeError>(handler.get_last());

        handler.handle(BCLIBC_BaseTrajData(0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 1.0));
        const auto first = handler.get_last();
        const auto *first_ref = (first.has_value() ? &first.value() : nullptr);
        assert(first_ref != nullptr && first_ref->get().time == 0.0);

        handler.handle(BCLIBC_BaseTrajData(1.0, 1.0, 0.0, 0.0, 100.0, 0.0, 0.0, 1.0));
        handler.handle(BCLIBC_BaseTrajData(2.0, 2.0, 0.0, 0.0, 100.0, 0.0, 0.0, 1.0));
        const auto result = handler.get_result();
        const auto *result_ref = (result.has_value() ? &result.value() : nullptr);
        assert(result_ref != nullptr && result_ref->get().time == 1.0);
    }

    void test_streaming_step_keeps_zero_and_range_separate()
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
        // range row at t=1. The ZERO_UP event stays a separate row from it.
        const BCLIBC_BaseTrajData start(0.0, 0.0, -1.0, 0.0,
                                        20.0, 1.0, 0.0, 1100.0);
        const BCLIBC_BaseTrajData end(2.0, 100.0, 1.0, 0.0,
                                      80.0, 1.0, 0.0, 1100.0);

        handler.handle(start);
        handler.handle_step(start, end);

        // Event roots and scheduled samples are separate observations
        // (see BCLIBC_TrajectoryDataFilter::handle_step): the crossing and the
        // range sample at the same point are two rows, never one merged row.
        int event_rows = 0;
        int range_rows_at_event = 0;
        for (const BCLIBC_TrajectoryData &row : records)
        {
            assert(!((row.flag & BCLIBC_TRAJ_FLAG_ZERO_UP) && (row.flag & BCLIBC_TRAJ_FLAG_RANGE)));
            if (row.flag == BCLIBC_TRAJ_FLAG_ZERO_UP)
            {
                ++event_rows;
                assert(std::fabs(row.distance_ft - 35.0) < 1e-9);
                assert(std::fabs(row.time - 1.0) < 1e-9);
                assert(std::fabs(row.height_ft) < 1e-9);
            }
            else if (row.flag == BCLIBC_TRAJ_FLAG_RANGE && std::fabs(row.time - 1.0) < 1e-9)
            {
                ++range_rows_at_event;
                assert(std::fabs(row.distance_ft - 35.0) < 1e-9);
            }
        }
        assert(event_rows == 1);
        assert(range_rows_at_event == 1);
    }

    void test_streaming_step_keeps_zero_down_and_range_separate()
    {
        std::vector<BCLIBC_TrajectoryData> records;
        BCLIBC_TerminationReason reason = BCLIBC_TerminationReason::NO_TERMINATE;
        BCLIBC_ShotProps props = make_filter_test_props();

        BCLIBC_TrajectoryDataFilter filter(
            records, props, BCLIBC_TRAJ_FLAG_ZERO, reason,
            100.0, 35.0, 0.0);
        BCLIBC_BaseTrajDataHandlerCompositor handler(&filter);

        // Symmetric to ZERO_UP: the Hermite path crosses the sight line
        // downward at x=35, t=1 inside this single accepted step.
        const BCLIBC_BaseTrajData start(0.0, 0.0, 1.0, 0.0,
                                        20.0, -1.0, 0.0, 1100.0);
        const BCLIBC_BaseTrajData end(2.0, 100.0, -1.0, 0.0,
                                      80.0, -1.0, 0.0, 1100.0);

        handler.handle(start);
        handler.handle_step(start, end);

        // Event roots and scheduled samples are separate observations
        // (see BCLIBC_TrajectoryDataFilter::handle_step): the crossing and the
        // range sample at the same point are two rows, never one merged row.
        int event_rows = 0;
        int range_rows_at_event = 0;
        for (const BCLIBC_TrajectoryData &row : records)
        {
            assert(!((row.flag & BCLIBC_TRAJ_FLAG_ZERO_DOWN) && (row.flag & BCLIBC_TRAJ_FLAG_RANGE)));
            if (row.flag == BCLIBC_TRAJ_FLAG_ZERO_DOWN)
            {
                ++event_rows;
                assert(std::fabs(row.distance_ft - 35.0) < 1e-9);
                assert(std::fabs(row.time - 1.0) < 1e-9);
                assert(std::fabs(row.height_ft) < 1e-9);
            }
            else if (row.flag == BCLIBC_TRAJ_FLAG_RANGE && std::fabs(row.time - 1.0) < 1e-9)
            {
                ++range_rows_at_event;
                assert(std::fabs(row.distance_ft - 35.0) < 1e-9);
            }
        }
        assert(event_rows == 1);
        assert(range_rows_at_event == 1);
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
    test_sequence_index_returns_value_result();
    test_get_at_slant_height_returns_value_result();
    test_base_interpolate_returns_value_error();
    test_trajectory_interpolate_returns_value_result();
    test_filter_get_record_returns_value_result();
    test_single_point_handler_returns_value_results();
    test_streaming_step_keeps_zero_and_range_separate();
    test_streaming_step_keeps_zero_down_and_range_separate();

    std::printf("test_traj_data: all tests passed\n");
    return 0;
}
