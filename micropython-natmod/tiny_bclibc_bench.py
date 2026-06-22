"""
tiny_bclibc performance benchmark.
Run with:
    micropython test_bclibc_bench.py
"""

import sys
import time
import math
import gc

_HERE = __file__.rsplit("/", 1)[0] if "/" in __file__ else "."
sys.path.append(_HERE)

import tiny_bclibc as bclibc
from tiny_bclibc_types import Shot, Request, DRAG_G7

# ── Test configuration ──────────────────────────────────────────────────────
SHOT = Shot(
    bc=0.310,
    weight_grain=168.0,
    diameter_inch=0.308,
    length_inch=1.2,
    muzzle_velocity_fps=2750.0,
    sight_height_ft=0.125,
    twist_inch=11.0,
    temp_c=15.0,
    pressure_hpa=1013.25,
    altitude_ft=0.0,
    humidity=0.5,
    drag_type=DRAG_G7,
)

# Request for 1 km trajectory with fine steps
REQUEST_1KM = Request(
    range_limit_ft=1000.0 * 3.28084,  # 1 km in feet
    range_step_ft=10.0 * 3.28084,  # 10 m steps
    filter_flags=bclibc.TRAJ_FLAG_RANGE,
)

# Request for 3 km trajectory with coarse steps
REQUEST_3KM = Request(
    range_limit_ft=3000.0 * 3.28084,  # 3 km in feet
    range_step_ft=100.0 * 3.28084,  # 100 m steps
    filter_flags=bclibc.TRAJ_FLAG_RANGE,
)

# ── Benchmark functions ──────────────────────────────────────────────────────


def bench_integrate(req, iterations=10):
    """Benchmark integrate() function."""
    shot_buf = SHOT.pack()
    req_buf = req.pack()

    times = []
    rows_count = 0

    for _ in range(iterations):
        gc.collect()
        start = time.ticks_us()
        rows, reason = bclibc.integrate(shot_buf, req_buf)
        end = time.ticks_us()
        times.append(time.ticks_diff(end, start))
        rows_count = len(rows)

    avg_us = sum(times) / len(times)
    avg_ms = avg_us / 1000.0

    return {
        "avg_us": avg_us,
        "avg_ms": avg_ms,
        "min_us": min(times),
        "max_us": max(times),
        "iterations": iterations,
        "rows": rows_count,
        "reason": reason,
    }


def bench_integrate_at(iterations=100):
    """Benchmark integrate_at() function."""
    shot_buf = SHOT.pack()
    targets = [100.0, 500.0, 1000.0, 1500.0, 2000.0]  # feet

    times = []

    for _ in range(iterations):
        for target in targets:
            gc.collect()
            start = time.ticks_us()
            raw, full = bclibc.integrate_at(shot_buf, bclibc.INTERP_POS_X, target)
            end = time.ticks_us()
            times.append(time.ticks_diff(end, start))

    avg_us = sum(times) / len(times)
    avg_ms = avg_us / 1000.0

    return {
        "avg_us": avg_us,
        "avg_ms": avg_ms,
        "min_us": min(times),
        "max_us": max(times),
        "iterations": iterations * len(targets),
        "calls_per_sec": 1.0 / (avg_us / 1_000_000) if avg_us > 0 else 0,
    }


def bench_find_zero_angle(iterations=50):
    """Benchmark find_zero_angle() function."""
    shot_buf = SHOT.pack()
    zero_dist_ft = 300.0 * 3.28084  # 300 m

    times = []
    results = []

    for _ in range(iterations):
        gc.collect()
        start = time.ticks_us()
        elev = bclibc.find_zero_angle(shot_buf, zero_dist_ft)
        end = time.ticks_us()
        times.append(time.ticks_diff(end, start))
        results.append(elev)

    avg_us = sum(times) / len(times)
    avg_ms = avg_us / 1000.0

    return {
        "avg_us": avg_us,
        "avg_ms": avg_ms,
        "min_us": min(times),
        "max_us": max(times),
        "iterations": iterations,
        "elev_rad_avg": sum(results) / len(results),
    }


def bench_find_apex(iterations=50):
    """Benchmark find_apex() function."""
    # Need a zeroed shot first
    shot_buf = SHOT.pack()
    zero_dist_ft = 300.0 * 3.28084
    elev = bclibc.find_zero_angle(shot_buf, zero_dist_ft)

    # Create zeroed shot
    zeroed = Shot(
        bc=SHOT.bc,
        weight_grain=SHOT.weight_grain,
        diameter_inch=SHOT.diameter_inch,
        length_inch=SHOT.length_inch,
        muzzle_velocity_fps=SHOT.muzzle_velocity_fps,
        sight_height_ft=SHOT.sight_height_ft,
        twist_inch=SHOT.twist_inch,
        barrel_elevation_rad=elev,
        drag_type=DRAG_G7,
    )
    zeroed_buf = zeroed.pack()

    times = []

    for _ in range(iterations):
        gc.collect()
        start = time.ticks_us()
        apex = bclibc.find_apex(zeroed_buf)
        end = time.ticks_us()
        times.append(time.ticks_diff(end, start))

    avg_us = sum(times) / len(times)
    avg_ms = avg_us / 1000.0

    return {
        "avg_us": avg_us,
        "avg_ms": avg_ms,
        "min_us": min(times),
        "max_us": max(times),
        "iterations": iterations,
        "apex_dist_ft": apex[1],
        "apex_height_ft": apex[4],
    }


def bench_memory_usage():
    """Measure memory usage during trajectory calculation."""
    gc.collect()
    mem_before = gc.mem_alloc()
    mem_free_before = gc.mem_free()

    shot_buf = SHOT.pack()
    req_buf = REQUEST_3KM.pack()

    gc.collect()
    mem_after_pack = gc.mem_alloc()

    rows, reason = bclibc.integrate(shot_buf, req_buf)
    mem_after_integrate = gc.mem_alloc()

    # Store results to prevent optimization
    result = len(rows)
    gc.collect()
    mem_after_gc = gc.mem_alloc()
    mem_free_after = gc.mem_free()

    return {
        "mem_before": mem_before,
        "mem_after_pack": mem_after_pack,
        "mem_after_integrate": mem_after_integrate,
        "mem_after_gc": mem_after_gc,
        "mem_free_before": mem_free_before,
        "mem_free_after": mem_free_after,
        "rows": len(rows),
        "reason": reason,
        "peak_alloc": mem_after_integrate - mem_before,
    }


# ── Run benchmarks ──────────────────────────────────────────────────────────

print("=" * 60)
print("tiny_bclibc Performance Benchmark")
print("=" * 60)
print("Version:", bclibc.version())
print("")

# 1. integrate() benchmark
print("--- integrate() (1 km, 10 m steps) ---")
result = bench_integrate(REQUEST_1KM, iterations=10)
print(f"  Rows: {result['rows']}")
print(f"  Stop reason: {result['reason']}")
print(f"  Avg: {result['avg_ms']:.2f} ms  ({result['avg_us']:.0f} µs)")
print(f"  Min: {result['min_us']:.0f} µs  Max: {result['max_us']:.0f} µs")
print(f"  Iterations: {result['iterations']}")
print("")

# 2. integrate() 3 km benchmark
print("--- integrate() (3 km, 100 m steps) ---")
result = bench_integrate(REQUEST_3KM, iterations=10)
print(f"  Rows: {result['rows']}")
print(f"  Stop reason: {result['reason']}")
print(f"  Avg: {result['avg_ms']:.2f} ms  ({result['avg_us']:.0f} µs)")
print(f"  Min: {result['min_us']:.0f} µs  Max: {result['max_us']:.0f} µs")
print(f"  Iterations: {result['iterations']}")
print("")

# 3. integrate_at() benchmark
print("--- integrate_at() (single point interpolation) ---")
result = bench_integrate_at(iterations=100)
print(f"  Avg: {result['avg_ms']:.3f} ms  ({result['avg_us']:.1f} µs)")
print(f"  Min: {result['min_us']:.0f} µs  Max: {result['max_us']:.0f} µs")
print(f"  Calls: {result['iterations']}")
print(f"  ~{result['calls_per_sec']:.0f} calls/sec")
print("")

# 4. find_zero_angle() benchmark
print("--- find_zero_angle() (300 m zero) ---")
result = bench_find_zero_angle(iterations=50)
print(f"  Avg: {result['avg_ms']:.3f} ms  ({result['avg_us']:.1f} µs)")
print(f"  Min: {result['min_us']:.0f} µs  Max: {result['max_us']:.0f} µs")
print(f"  Elevation avg: {math.degrees(result['elev_rad_avg']):.4f}°")
print(f"  Iterations: {result['iterations']}")
print("")

# 5. find_apex() benchmark
print("--- find_apex() ---")
result = bench_find_apex(iterations=50)
print(f"  Avg: {result['avg_ms']:.3f} ms  ({result['avg_us']:.1f} µs)")
print(f"  Min: {result['min_us']:.0f} µs  Max: {result['max_us']:.0f} µs")
print(f"  Apex: {result['apex_dist_ft']:.1f} ft, {result['apex_height_ft']:.1f} ft")
print(f"  Iterations: {result['iterations']}")
print("")

# 6. Memory usage
print("--- Memory usage (3 km trajectory) ---")
mem = bench_memory_usage()
print(f"  Before: {mem['mem_before']:,} B")
print(f"  After pack: {mem['mem_after_pack']:,} B")
print(f"  After integrate: {mem['mem_after_integrate']:,} B")
print(f"  After GC: {mem['mem_after_gc']:,} B")
print(f"  Peak allocation: {mem['peak_alloc']:,} B")
print(f"  Rows: {mem['rows']}")
print(f"  Free memory: {mem['mem_free_before']:,} B → {mem['mem_free_after']:,} B")
print("")

# 7. Summary
print("=" * 60)
print("Benchmark Summary")
print("=" * 60)

# Estimate shots per second for different operations
integrate_time_ms = bench_integrate(REQUEST_1KM, iterations=5)["avg_ms"]
integrate_at_time_ms = bench_integrate_at(iterations=50)["avg_ms"]
find_zero_time_ms = bench_find_zero_angle(iterations=20)["avg_ms"]

print(f"  Trajectory (1 km, 10 m steps): {1000 / integrate_time_ms:.1f} shots/sec")
print(f"  Interpolation: {1000 / integrate_at_time_ms:.1f} calls/sec")
print(f"  Zero finding: {1000 / find_zero_time_ms:.1f} calls/sec")
print("=" * 60)
print("Benchmark complete.")
