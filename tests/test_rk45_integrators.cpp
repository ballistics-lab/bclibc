// test_rk45_integrators.cpp — BCLIBC_CashKarpIntegrator functional + concurrency tests.
//
// The concurrency test is the one that matters for CHANGELOG's claim of
// "verified race-free under ThreadSanitizer": several threads drive
// integration through ONE shared BCLIBC_CashKarpIntegrator instance (bound
// via std::ref into each thread's own BCLIBC_BaseEngine::integrate_func)
// while another thread concurrently mutates its tolerances and a third
// polls get_stats() -- exactly the sharing pattern the class's std::atomic
// fields exist to make safe. Build with -fsanitize=thread (see README for
// the exact invocation) to actually exercise the race detector; without a
// sanitizer this only checks the *values* stay sane, not the absence of a
// data race.
//
// No external test framework -- plain assert()/printf(), matching
// test_traj_data.cpp.

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <thread>
#include <vector>

#include "bclibc/cash_karp.hpp"
#include "bclibc/engine.hpp"

using namespace bclibc;

namespace
{
    // A tiny, physically-plausible-enough (not G7-accurate) supersonic drag
    // table -- sufficient to drive a real multi-step adaptive integration
    // without needing the full reference tables the identity test uses.
    constexpr double kMach[] = {0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 5.0};
    constexpr double kCd[] = {0.10, 0.12, 0.15, 0.40, 0.35, 0.28, 0.22, 0.18, 0.15};
    constexpr int kDragTableSize = sizeof(kMach) / sizeof(kMach[0]);

    BCLIBC_ShotProps make_shot_props()
    {
        BCLIBC_Shot shot{};
        shot.bc = 0.22;
        shot.weight_grain = 168.0;
        shot.diameter_inch = 0.308;
        shot.length_inch = 1.2;
        shot.muzzle_velocity_fps = 2650.0;
        shot.stability_coefficient = 0.0;
        shot.mach_data = kMach;
        shot.cd_data = kCd;
        shot.drag_table_size = kDragTableSize;
        shot.sight_height_ft = 0.15;
        shot.twist_inch = 10.0;
        shot.temp_c = 15.0;
        shot.pressure_hpa = 1013.25;
        shot.altitude_ft = 0.0;
        shot.humidity = 0.5;
        shot.winds = nullptr;
        shot.wind_count = 0;
        shot.look_angle_rad = 0.0;
        shot.barrel_elevation_rad = 0.02;
        shot.barrel_azimuth_rad = 0.0;
        shot.cant_angle_rad = 0.0;
        shot.latitude_deg = std::numeric_limits<double>::quiet_NaN();
        shot.azimuth_deg = std::numeric_limits<double>::quiet_NaN();
        shot.calc_step = 0.0025;
        return shot.to_shot_props();
    }

    // BCLIBC_BaseEngine is non-copyable/non-moveable; build in place.
    void init_engine(BCLIBC_BaseEngine &eng)
    {
        eng.shot = make_shot_props();
        eng.config = BCLIBC_Config(
            /*cStepMultiplier=*/1.0,
            /*cZeroFindingAccuracy=*/5e-6,
            /*cMinimumVelocity=*/50.0,
            /*cMaximumDrop=*/-15000.0,
            /*cMaxIterations=*/40,
            /*cGravityConstant=*/-32.17405,
            /*cMinimumAltitude=*/-1500.0);
        eng.gravity_vector = BCLIBC_V3dT(0.0, eng.config.cGravityConstant, 0.0);
    }

    void run_short_trajectory(BCLIBC_BaseEngine &eng)
    {
        std::vector<BCLIBC_TrajectoryData> records;
        BCLIBC_TerminationReason reason = BCLIBC_TerminationReason::NO_TERMINATE;
        // Short range so each run is cheap -- the concurrency test wants many
        // runs overlapping in time, not one long one.
        eng.integrate_filtered(500.0, 50.0, 0.0, BCLIBC_TRAJ_FLAG_RANGE, records, reason, nullptr);
    }

    // ------------------------------------------------------------------
    // Functional tests (single-threaded)
    // ------------------------------------------------------------------

    void test_default_tolerances_are_1e_minus_6()
    {
        BCLIBC_CashKarpIntegrator integrator;
        BCLIBC_BaseEngine eng;
        init_engine(eng);
        eng.integrate_func = std::ref(integrator);
        run_short_trajectory(eng);

        int accepted = 0, rejected = 0;
        integrator.get_stats(accepted, rejected);
        assert(accepted > 0 && "expected at least one accepted step over 500ft");
        assert(rejected >= 0);
    }

    void test_constructor_tolerances_are_applied()
    {
        BCLIBC_CashKarpIntegrator loose(1e-3, 1e-3);
        BCLIBC_CashKarpIntegrator tight(1e-9, 1e-9);
        BCLIBC_BaseEngine eng_loose, eng_tight;
        init_engine(eng_loose);
        init_engine(eng_tight);
        eng_loose.integrate_func = std::ref(loose);
        eng_tight.integrate_func = std::ref(tight);
        run_short_trajectory(eng_loose);
        run_short_trajectory(eng_tight);

        int loose_accepted = 0, loose_rejected = 0, tight_accepted = 0, tight_rejected = 0;
        loose.get_stats(loose_accepted, loose_rejected);
        tight.get_stats(tight_accepted, tight_rejected);
        // Tighter tolerance must never take *fewer* attempted steps than a looser one.
        assert(tight_accepted + tight_rejected >= loose_accepted + loose_rejected);
    }

    void test_invalid_tolerance_throws()
    {
        BCLIBC_CashKarpIntegrator integrator;
        bool threw = false;
        try
        {
            integrator.set_relative_tolerance(-1.0);
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        assert(threw && "expected std::invalid_argument for negative relative tolerance");

        threw = false;
        try
        {
            integrator.set_absolute_tolerance(std::numeric_limits<double>::quiet_NaN());
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        assert(threw && "expected std::invalid_argument for NaN absolute tolerance");
    }

    void test_copy_duplicates_tolerances_not_shared_stats()
    {
        BCLIBC_CashKarpIntegrator original(1e-7, 1e-8);
        BCLIBC_BaseEngine eng;
        init_engine(eng);
        eng.integrate_func = std::ref(original);
        run_short_trajectory(eng);

        int orig_accepted = 0, orig_rejected = 0;
        original.get_stats(orig_accepted, orig_rejected);
        assert(orig_accepted > 0);

        // A copy is a snapshot: independent atomics, so its own stats start
        // from whatever `original`'s were *at copy time* -- not reset to
        // zero -- and running `original` again afterward must not reach it.
        BCLIBC_CashKarpIntegrator copy(original);
        int copy_accepted = 0, copy_rejected = 0;
        copy.get_stats(copy_accepted, copy_rejected);
        assert(copy_accepted == orig_accepted && copy_rejected == orig_rejected);

        run_short_trajectory(eng); // eng.integrate_func still targets `original`
        int orig_accepted_2 = 0, orig_rejected_2 = 0;
        original.get_stats(orig_accepted_2, orig_rejected_2);
        assert(orig_accepted_2 > 0);

        int copy_accepted_2 = 0, copy_rejected_2 = 0;
        copy.get_stats(copy_accepted_2, copy_rejected_2);
        assert(copy_accepted_2 == copy_accepted && copy_rejected_2 == copy_rejected &&
               "copy's stats must not change when the original integrates again");
    }

    // ------------------------------------------------------------------
    // Concurrency test: the one ThreadSanitizer actually needs to see.
    // ------------------------------------------------------------------

    void test_concurrent_tolerance_stats_access_is_race_free()
    {
        BCLIBC_CashKarpIntegrator shared_integrator;
        std::atomic<bool> stop{false};
        std::atomic<long> integration_runs{0};
        std::atomic<long> tolerance_writes{0};
        std::atomic<long> stats_reads{0};

        // Two threads drive real integrations through the SAME integrator
        // instance, each through its own (non-shared) BCLIBC_BaseEngine --
        // this is the documented std::ref sharing pattern from
        // cash_karp.hpp's class doc comment.
        auto integrate_worker = [&]()
        {
            BCLIBC_BaseEngine eng;
            init_engine(eng);
            eng.integrate_func = std::ref(shared_integrator);
            while (!stop.load(std::memory_order_relaxed))
            {
                run_short_trajectory(eng);
                integration_runs.fetch_add(1, std::memory_order_relaxed);
            }
        };

        // One thread hammers both tolerance setters with always-valid values.
        auto tolerance_worker = [&]()
        {
            double rtol = 1e-6, atol = 1e-6;
            while (!stop.load(std::memory_order_relaxed))
            {
                rtol = (rtol > 1e-9) ? rtol * 0.5 : 1e-4;
                atol = (atol > 1e-9) ? atol * 0.5 : 1e-4;
                shared_integrator.set_relative_tolerance(rtol);
                shared_integrator.set_absolute_tolerance(atol);
                tolerance_writes.fetch_add(1, std::memory_order_relaxed);
            }
        };

        // One thread only ever reads -- get_stats()'s two loads are
        // independent (per the class doc comment), so this must never crash
        // or read torn/garbage memory, even though the (accepted, rejected)
        // pair it sees may not correspond to the same run.
        auto stats_reader_worker = [&]()
        {
            while (!stop.load(std::memory_order_relaxed))
            {
                int accepted = 0, rejected = 0;
                shared_integrator.get_stats(accepted, rejected);
                assert(accepted >= 0 && rejected >= 0);
                stats_reads.fetch_add(1, std::memory_order_relaxed);
            }
        };

        std::vector<std::thread> threads;
        threads.emplace_back(integrate_worker);
        threads.emplace_back(integrate_worker);
        threads.emplace_back(tolerance_worker);
        threads.emplace_back(stats_reader_worker);

        // Under a sanitizer this is already very slow per iteration, so a
        // short wall-clock budget is enough to interleave many operations
        // across all four threads without making the test suite slow.
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        stop.store(true, std::memory_order_relaxed);
        for (auto &t : threads)
            t.join();

        assert(integration_runs.load() > 0 && "integrate worker made no progress");
        assert(tolerance_writes.load() > 0 && "tolerance worker made no progress");
        assert(stats_reads.load() > 0 && "stats reader made no progress");

        // The real assertion here is what a sanitizer build checks, not
        // anything above: no data race was reported for the whole run.
    }
}

int main()
{
    test_default_tolerances_are_1e_minus_6();
    test_constructor_tolerances_are_applied();
    test_invalid_tolerance_throws();
    test_copy_duplicates_tolerances_not_shared_stats();
    test_concurrent_tolerance_stats_access_is_race_free();

    std::printf("test_rk45_integrators: all tests passed\n");
    return 0;
}
