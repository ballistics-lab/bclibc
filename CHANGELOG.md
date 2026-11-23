# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
- The **upper bound of the lofted zero search is 89.9 degrees**, as in the Python reference (`math.radians(89.9)` = 1.5690509975429023). The C++
  core used 1.5690308719637473 (a mistyped constant, 89.8988 degrees), in the code shared by all six engines (RK4, Euler, Velocity Verlet, Cash-Karp,
  Dormand-Prince, Tsitouras). It shows as a different `last_barrel_elevation` in "No lofted zero trajectory" errors and as a slightly different
  bracket for a lofted zero near the bound. `tiny_bclibc` has no lofted search and is not affected.
- The **single-point handler finds a key crossing inside the first integration step** (`BCLIBC_SinglePointHandler`, used by `find_apex`,
  `integrate_at` / `integrate_raw_at`). Its three-point window was only tested between its 2nd and 3rd points, so the segment from the start
  point to the end of the first step was never examined: the apex of a shot with a tiny elevation (relative angle 0.001 deg, apex at 1.4 ms)
  failed with "No apex flagged", and `integrate_at(time, t)` failed for every `t` before the end of the first step, `t = 0` included. The
  segment is now tested once, when the third point arrives, with the same three-point interpolation as the later steps; crossings in later steps
  are unchanged (recorded-value guard). `tiny_bclibc` had the same defect and is fixed the same way (identity test added).
- The Ridder zero search **fails when it runs out of iterations** instead of accepting an unconverged result. After the loop
  `find_zero_angle_ridder` accepted a bracket narrower than 10 angle tolerances, or a bracket end whose error was below
  10 * `cZeroFindingAccuracy`, so a search that had not converged returned an angle that could miss by feet (a lofted zero changes
  its miss by up to 6e6 ft per radian). It now returns `BCLIBC_SolverZeroFindingError` ("Ridder's method failed to converge.",
  `BCLIBCFFI_ERR_ZERO_FINDING`), as the Python reference raises `ZeroFindingError`. The accepting paths (which returned the bracket midpoint
  without a point and the last angle with one) and the unreachable "converged but no return" block are removed. `tiny_bclibc` fails the same way
  (`TINY_BCLIBC_ERR_ZERO_FINDING`, "not converged"). A search that converged is unchanged; one that used to be accepted on the fallback now
  fails and needs a larger `cMaxIterations`.
- **RK4 evaluates the Coriolis acceleration in each of its four stages**, with the stage's own ground velocity, as the Python RK4
  (`py_ballisticcalc/engines/rk4.py`, `coriolis_fn(ground_vel)` per stage). It used to evaluate it once per step, at the velocity the step
  started with, and reuse it in all four stages. On a 30 degree, 800 m/s G7 shot at 4000 m (latitude 45, azimuth 90) the result was off the
  Python one by 1.5e-3 ft in height and 1.8e-3 ft in windage (3e-3 ft windage at latitude -30, azimuth 0); it now agrees to 1e-6 ft. Shots
  without Coriolis (no latitude, or `flat_fire_only`) are unchanged. Euler, Velocity Verlet and the embedded RK45 engines were not changed.
- A shot that **starts outside a termination limit gets its terminal row**. The essential terminators (minimum velocity, maximum drop,
  minimum altitude) were also run on the initial point, so a shot that began outside a limit (a minimum velocity above the muzzle velocity, a
  minimum altitude above the start with the bullet falling) stopped before its first integration step: the integrator never called
  `handle_step()`, `BCLIBC_TrajectoryDataFilter::finalize()` found nothing newer than the start row and the result was `[RANGE t=0]` alone. The
  limits are now tested after a step, as the Python engines do (they integrate one step, then test), and the result is
  `[RANGE t=0, NONE t=dt]` with the same termination reason, for RK4, Euler, Velocity Verlet and the embedded RK45 engines. `tiny_bclibc`
  tests the limits at step ends only and is not affected.
- The **wind sock is rewound at the start of every integration** (`BCLIBC_WindSock::reset()`, called by RK4, Euler, Velocity Verlet and the
  embedded RK45 engines). Every integration used to continue from the wind segment the previous one had ended on, so the trial shots of a
  zero or maximum-range search (`find_zero_angle`, `find_zero_point`, `find_max_range`, a lofted zero, Newton's `zero_angle` with more than
  one segment) saw the wind of their predecessors: a wind that ended inside the trajectory was ignored, and the zero or maximum range was that of
  still air (a 30 m/s head wind to 1000 m changed the zero of an 800 m/s shot by 0.2 ft, a 5 m/s wind to 3000 m made a lofted zero miss by
  thousands of feet). The Python engines build a new sock per integration and were right. `tiny_bclibc` copies the sock per integration and
  is not affected.
- `find_zero_angle` returns the same angle as `find_zero_point`: the bracket-width exit of the Ridder loop returned the midpoint of the
  bracket without a point and `next_angle` with one; it now returns `next_angle` in both (`BCLIBC_BaseEngine` and `tiny_bclibc`). The returned
  angle changes by up to the bracket width; the angle of a search that did not take that exit is unchanged.
- The angle tolerance of the Ridder zero search is **1e-7 rad**, as in the Python reference (py_ballisticcalc `base_engine.py`); it was 1e-5 rad
  (the constant introduced to decouple it from the height accuracy `cZeroFindingAccuracy`, with an unjustified value). A near-vertical lofted
  zero changes its miss by up to 6e6 ft per radian, so the search used to return an angle that missed by feet (3.5, 16.8 and 2.1 ft at 50, 100
  and 300 m for an 800 m/s G7 shot). `tiny_bclibc` keeps 1e-5 (single-precision floats cannot resolve 1e-7 rad above 1 rad).
- The zero angle of a shot with a **downhill look angle** no longer fails with `OutOfRangeError` / `BCLIBCFFI_ERR_OUT_OF_RANGE`
  ("maximum possible range" of a few feet) for a target well within range. The Ridder zero search (`find_zero_angle_ridder`,
  `find_zero_point`, `BCLIBCFFI_find_zero_{angle,point}_lofted_*` with `lofted == 0`, `BCLIBCFFI_find_max_range_*`, and `tiny_bclibc`'s
  Ridder fallback and `tiny_bclibc_find_max_range`) first finds the maximum range with a golden-section search over the barrel
  elevation, measuring each shot by the slant distance at which it comes back down through the line of sight. A downhill line of
  sight lies *below* the muzzle (at 10 degrees down it is 0.17 ft lower per foot of range), so a shot crosses it far below the
  muzzle altitude. The search ran with `cMaximumDrop = 0` ("stop at muzzle level"; in `tiny_bclibc` the caller's drop limit) and the
  caller's `cMinimumAltitude`, so every shot was stopped at muzzle level (or at the altitude limit) before the crossing: `range_for_angle()`
  answered 0 ("no crossing") for all but near-vertical elevations, the search converged on about 90 degrees with a maximum range of a few feet
  (2.98 ft, 5.87 ft ...), and every farther target was "out of range". For a downhill look angle the search now lifts the two limits
  (restored on return; the integration stops at the crossing). A level or uphill look angle is untouched, and so is every result that did
  not depend on the defect: the Newton-first entry points (`BCLIBCFFI_find_zero_angle_shot`, `zero_angle`, `zero_angle_with_fallback`) worked
  on every downhill shot tested, and their answers, the zeros of level and uphill shots and their maximum range are bit-for-bit what they were.
  `find_max_range` of a downhill shot is now the real maximum slant range (for example 15262 ft at 33.5 degrees for a 5 degree downhill
  shot of 2750 fps, G7 0.223) instead of a few feet at 89.9 degrees.
  For a downhill look angle the search ignores the caller's `cMinimumAltitude` and `cMaximumDrop` (lifted to 1e6 ft, so the
  reported maximum range can lie below the altitude floor the caller set), and two things keep that from running for ever:
  `range_for_angle` answers "no crossing" without integrating for a launch that cannot rise above the line of sight (the vacuum
  peak above it, `h0 + (v sin(elevation - look))^2 / (2 g cos(look))`, is not above it: this includes every elevation at or below
  the look angle, so a wide bracket such as (-90, 90) is cheap), and the flight is bounded to 600 s (a wind that carries a steep shot
  along the line of sight never lets it cross).
  The 600 s is a bound on a single probe's flight time (realistic flights are under about 400 s); a probe that exceeds it reports "no
  crossing", so for extreme inputs (for example look -20 degrees at 6000 fps in 0.05 hPa) `find_max_range` can return 0 ft instead of
  hanging, and a later zero search then reports out of range.
  `tiny_bclibc_find_zero_point` also widens the drop and altitude limits for its intercept shot (a long downhill target, for example
  1500 m at -30 degrees, lies below the default altitude limit and used to end with "target intercept not found").
  Tests: `tests/test_zero_downhill.cpp` (C ABI and C++ core: zero of downhill shots from -5 to -45 degrees, targets 0.5 to 300 m, must hit within
  2 mm; the maximum range equals a brute-force scan; recorded values of unaffected results) and the `tiny_bclibc` identity test.
- The zero angle of a very close target (closer than about twice the sight height) is right now, and the cutoff of the
  "very close shot" shortcut of `init_zero_calculation` (and `tiny_bclibc`'s `find_zero_angle`) is what it was meant to be.
  The shortcut returns a straight-line angle without a search ("ignore gravity and drag") and had two defects:
  - **The cutoff used the wrong quantity.** It was `2 * max(|sight height|, cStepMultiplier)`, and `cStepMultiplier` is a
    dimensionless factor (1.0 by default), so the cutoff was about 2 ft (0.6 m) whatever the sight height. It is now
    `2 * max(|sight height|, calc_step)` with the shot's `calc_step`, following the Python reference (`props.calc_step`).
    This is not a dimensional fix: `calc_step` is a time step in seconds for RK4, Velocity Verlet and the adaptive methods
    (distance-like only for Euler, 0.5), so in practice the cutoff is 2 * sight height. A target closer than 0.6 m with a low
    sight (for example 0.1 to 0.6 m with a 0.05 or 0.1 m sight) used to take the shortcut and is now searched for like any
    other target (a target exactly at the cutoff, such as 0.2 m with a 0.1 m sight, is searched: the test is `<`).
  - **The angle pointed the wrong way.** It was `atan2(target_y + start_height, target_x)`, but `start_height` is the *negative*
    sight height (the bore is below the line of sight), so for a sight above the bore the angle was negative: the bore pointed
    down, away from the line of sight, and the shot missed it by twice the sight height (0.2 m for a 0.1 m sight, 1 m for 0.5 m, at any
    close target). It is now `atan2(target_y - start_height, target_x)`, the line from the bore up to the target.
  The zero angle of a target closer than the cutoff **changes on purpose**: with a 0.1 m sight, a 0.2 m target is +26.565 deg
  (it was -26.565 deg), 0.4 m is +14.036 deg, 0.6 m is +9.46 deg (found by the search now); with a 0.5 m sight, 0.2 / 0.4 / 0.6 m
  give +68.2 / +51.3 / +39.8 deg (they were the negatives of these, a miss of 1 m). The shot fired with the zero reaches the line of sight
  at the target within 0.3 mm. The zero of any target at or beyond the cutoff (every target 2 m and farther, whatever the sight) is
  bit-for-bit what it was, and so is the answer for a target closer than 0.01 ft (the look angle) and for a vertical shot. For the
  lofted (higher) arc of a target that was between the old and the new cutoff, `BCLIBCFFI_find_zero_*_lofted_*` with `lofted != 0`
  now reports `BCLIBCFFI_ERR_ZERO_FINDING` (a flat shot has no higher arc) instead of the wrong shortcut angle, as for any
  target beyond the cutoff.
  `BCLIBCFFI_ZeroInit::look_angle_rad` of a `DONE` close shot is the corrected angle.
  Tests: `tests/test_zero_close_target.cpp` (C ABI and C++ core: the miss of the zero shot for close targets over sight heights,
  velocities, drag tables, look angles and integrators; the cutoff; recorded zeros of far targets), and the `tiny_bclibc` identity test.

## [2.2.1] - 2026-11-21

### Added
- C ABI `BCLIBCFFI_integrate_dense_shot_stats()`: `BCLIBCFFI_integrate_dense_shot()` with an `out_stats`. For a fixed-step
  method (RK4, Euler, Velocity Verlet) `accepted` is the engine's step count of the call, `rejected` is 0, so a binding can
  report the step count the exts engine reports. The records and the status are those of `BCLIBCFFI_integrate_dense_shot()`.
  `BCLIBCFFI_get_layout()` is unchanged: no struct is added (`BCLIBCFFI_StepStats` is already in it).

## [2.2.0] - 2026-11-19

Closes the gaps a Cython prototype over the C ABI and `py_ballisticcalc.ctypes` found in `bclibc_ffi.h`: the pieces of the C++
API a binding had to rewrite (a trajectory buffer, interpolation over points) and the error kinds the ABI merged into one code. One
thing readers of the ABI have to know, under "Changed".

### Added
- C ABI for a growable buffer of base trajectory points: `BCLIBCFFI_BaseTrajBuf`, `BCLIBCFFI_base_traj_buf_append` and
  `BCLIBCFFI_base_traj_buf_free` (what `BCLIBC_BaseTrajSeq` is in C++). The memory is allocated and released by the library, next to
  `BCLIBCFFI_free_base_trajectory`, so a module with another C runtime (Windows) does not free it itself.
- C ABI for interpolation over points: `BCLIBCFFI_base_traj_interpolate` (`BCLIBC_BaseTrajData::interpolate`, three points) and
  `BCLIBCFFI_base_traj_interpolate_at` (`BCLIBC_BaseTrajSeq::interpolate_at`, the points around an index, a negative index counts
  from the end). The results are those of the C++ code; an unknown key fails with `BCLIBCFFI_ERR_INVALID_ARGUMENT`. Indices of the
  new calls are `int64_t` (the counts of the older ones stay `int32_t`).
- Status codes for the error kinds the ABI did not tell apart: `BCLIBCFFI_ERR_DOMAIN`, `BCLIBCFFI_ERR_INVALID_ARGUMENT`,
  `BCLIBCFFI_ERR_OUT_OF_BOUNDS` (an index or a value outside the data), `BCLIBCFFI_ERR_LOGIC`, `BCLIBCFFI_ERR_RUNTIME`. The fields
  of `BCLIBCFFI_Error` (`f64_0..f64_2`, `i32_0`) are described per code in a table in the header; `OUT_OF_BOUNDS` carries the
  requested value and the allowed minimum and maximum, `DOMAIN` the two values (for example the first two equal keys).
- `BCLIBC_FFI_PARSE_ONLY`: a macro for binding generators that read the header without a preprocessor (cffi, autopxd2); with it
  `BCLIBC_API` expands to nothing. Defining it together with `BCLIBC_FFI_EXPORT` (the build of the library) is an `#error`: a DLL built that way would export nothing.
- `tests/test_ffi_adaptive.cpp`: the error kinds and their fields, the buffer (growth, free, reuse), the interpolation (by every
  kind of key, negative index, the ends, equality with `BCLIBCFFI_dense_get_at`), and the new layout block.

### Changed
- **`BCLIBCFFI_get_layout()` returns 100 values (was 96):** `BCLIBCFFI_BaseTrajBuf` (its size and 3 offsets) is appended at the end, so
  no earlier offset moves. A reader that checks the exact count (js-ballistics, the web layer of dart-bclibc) must raise its
  expected count with the library.
- **A failed call no longer returns `BCLIBCFFI_ERR_GENERIC` for every kind of error that is not a solver's.** `setEngineError` maps
  all kinds of `BCLIBC_Error` (it translated only the `Solver*` ones), and the places that returned `GENERIC` for an engine error
  (the drag curve and shot properties of a shot, the trajectory record of the apex or of a dense step) go through it too. A drag
  table of fewer than 2 points is now `INVALID_ARGUMENT`; a tolerance that is not finite and positive, `INVALID_ARGUMENT`; a key outside
  the dense output, `OUT_OF_BOUNDS` (with the range of the stream); too few steps for an interpolation, `DOMAIN`. A caller that
  tested for `GENERIC` has to handle the new codes (a binding with a `default:` branch needs nothing). `GENERIC` stays for the
  checks of the C layer (a method that does not fit the call, no memory for a result array).
- `BCLIBCFFI_dense_get_at` and `BCLIBCFFI_dense_get_at_slant_height` no longer answer an empty stream with the text "dense output is
  empty" and `GENERIC`: the C++ code decides, as it does for any stream of fewer than 3 steps (`DOMAIN`). A negative count or a NULL
  array with a positive count is `INVALID_ARGUMENT`.
- The header notes that `sizeof(BCLIBCFFI_TrajectoryData)` is 128 bytes on most platforms and 124 on 32-bit x86 Linux, and that the
  size to use is the one `BCLIBCFFI_get_layout()` reports.

## [2.1.0] - 2026-11-18

Adds the C ABI the bindings were missing (adaptive integrators, dense output, the pieces of the solvers) and makes the zero
search independent of the early-termination limits (#45). Two things readers of the ABI have to know, under "Changed".

### Added
- C ABI for the adaptive integrators (Cash-Karp, Dormand-Prince, Tsitouras) in `bclibc_ffi.h`: `BCLIBCFFI_integrate_adaptive_shot`, `BCLIBCFFI_free_base_trajectory`, and the zero-angle, zero-point, max-range and apex variants `BCLIBCFFI_*_adaptive_shot`. Each takes per-call relative and absolute tolerances and reports accepted and rejected step counts (`BCLIBCFFI_StepStats`); the integrate call can also return the accepted-step stream as dense output. Each call owns its integrator, so no state is shared between calls.
- C ABI for the remaining engine paths: `BCLIBCFFI_integrate_dense_shot` (dense output for the fixed-step RK4, Euler and
  Velocity Verlet methods), `BCLIBCFFI_find_zero_angle_lofted_shot` and `BCLIBCFFI_find_zero_point_lofted_shot` (lofted
  solvers, fixed-step), their `*_lofted_adaptive_shot` variants, and `BCLIBCFFI_integrate_at_adaptive_shot` (interpolated
  point with per-call tolerances). Errors use the same `BCLIBCFFI_Status` codes.
- C ABI for the pieces the solvers are made of: `BCLIBCFFI_error_at_distance_shot`, `BCLIBCFFI_range_for_angle_shot`,
  `BCLIBCFFI_find_zero_angle_newton_shot` and `BCLIBCFFI_zero_init_shot` (each with an `_adaptive_shot` form taking
  per-call tolerances and reporting step counts), so a caller can run its own zero or range search.
- C ABI for working with dense output: `BCLIBCFFI_dense_get_at` and `BCLIBCFFI_dense_get_at_slant_height`
  (interpolation inside the accepted-step array) and `BCLIBCFFI_trajectory_data_from_step_shot` (a step as a full
  trajectory record), so dense output is usable without reimplementing PCHIP or the shot physics per language.
- `BCLIBCFFI_get_layout()` also reports `BCLIBCFFI_StepStats` and `BCLIBCFFI_ZeroInit`: appended at the end, so no earlier
  offset moves, and it now returns 96 values (was 85). A reader that checks the exact count (js-ballistics, the web layer of
  dart-bclibc) must raise its expected count with the library.
- `tests/test_ffi_adaptive.cpp`: C ABI tests for all of the above (status codes, tolerances, step statistics, dense
  output, every integration method, error paths), run by CTest.
- CI (`pr-check.yml`): an ASan + UBSan job (the C++ tests and the `tiny_bclibc` identity test) and a packaging job on
  Linux and macOS (the Makefile targets, and `cmake/check_package.sh`, which installs `bclibc` and `tiny_bclibc` and
  builds, links and runs a `find_package` consumer of each at `-O0`).
- `tests/wasm_parity/parity.py` covers the new entry points too (adaptive integrators with step statistics, dense
  output and its helpers, the lofted and Newton solvers, `error_at_distance`, `range_for_angle`, `zero_init`, error
  paths): 204 cases, identical between the native library and the wasi-sdk and zig modules (angles within a few ulp).

### Changed
- **BREAKING for a reader that checks the exact count: `BCLIBCFFI_get_layout()` returns 96 values (was 85).** `StepStats`
  and `ZeroInit` are appended at its end, so no earlier offset moves; a reader that passes the old buffer size gets `-1`. The
  web layer of dart-bclibc (`_Layout.fieldCount`) and js-ballistics (`LAYOUT_FIELD_COUNT`) must be raised with the library.
- **A zero beyond the maximum range is reported as `BCLIBCFFI_ERR_OUT_OF_RANGE` (2), not `BCLIBCFFI_ERR_SOLVER_RUNTIME` (1)**
  (`BCLIBC_SolverOutOfRangeError` in C++, with the requested and the maximum distance), as the documentation always said: the
  first shot of Newton's method ended short of the target, which no fallback caught, and now Ridder's method takes over and
  checks the range. Code that matched on status 1 for this case has to match 2.
- Ridder's method lost a branch that could not run any more (a "nudge" of the low bracket edge when `error_at_distance`
  failed): it returns a finite miss now, never a sentinel.

### Fixed
- The zero search no longer depends on the early-termination limits (#45). `find_zero_angle` failed with "Trajectory too
  short to determine error at distance" (or "Failed to interpolate trajectory at target distance") whenever a shot of the search
  was cut off before the target, which a tight `cMaximumDrop` or `cMinimumAltitude` (0 means "stop at muzzle level") does
  to the flat edge of the Ridder bracket and to the first Newton guess; the same inputs worked with the default limits. Now:
  - Ridder's method and Newton's method widen both limits (never narrow them) for the duration of the search, so the answer is the
    unconstrained zero, the same as with the default limits; the caller's limits apply to every later integration.
  - `error_at_distance` measures a trajectory that ends before the target at its last point (a finite, signed error), as the Python
    engine does, instead of failing; Newton does the same, so a trajectory that falls short is a failure to converge and
    `zero_angle_with_fallback` switches to Ridder's method (it was a `SolverRuntimeError`, which nothing caught).
  - The widening for `cMinimumAltitude` had the sign of its test inverted (`cMinimumAltitude - alt0 > required_drop`, ported
    from the Python engine): the limit was almost never widened, so long zeros failed even with the default limits. Over a grid
    of 1314 zeros (three shots, six engines, sight heights, look angles, 25 to 5000 yd) at the default limits, 72 that raised
    `SolverRuntimeError` now solve (3500 and 5000 yd), none got worse and no solved value changed.
  - The higher (lofted) zero, whose near-vertical bracket edge never reaches the target, is found instead of failing with
    "Trajectory too short".
  - `tiny_bclibc` has the same changes (its `error_at_distance` returned a `+1e9` sentinel, the wrong sign for a shot that falls
    short, and nothing widened the limits), so it solves the same zeros, to rounding, as bclibc.
- `BCLIBC_BaseEngine::range_for_angle` located the line-of-sight crossing by linear interpolation between two
  integrator steps. With the large steps of the adaptive integrators that is far off where the trajectory crosses the
  line of sight at a shallow angle (a 984 ft crossing came out at 952-978 ft; RK4's small steps were off by about
  0.004 ft). It now uses the cubic Hermite interpolant of the step, as the trajectory filter's `ZERO_DOWN` rows and
  `tiny_bclibc` already do, so every method gives the same answer. `find_max_range` uses it, but its landing crossing
  is steep: for a G7 2750 fps shot (about 18,420 ft) the maximum range moves by hundredths of a foot and the angle of the
  maximum by up to 0.06 degrees (adaptive methods), RK4 only at the milli-foot level. The py-ballisticcalc test suite
  (all six Cython engines) passes unchanged, and the `tiny_bclibc` identity test now also compares `find_max_range`
  (bclibc and `tiny_bclibc` agree to rounding).
- The `BCLIBCFFI_integrate_*` calls that return dense output now set their output pointers to `NULL`/`0` on failure
  instead of leaving them untouched.
- `make linux` and `make macos` failed (`cd build && cd build`): the Makefile's `MAKE_COMMAND` is GNU make's own variable
  (`$(MAKE)` is defined as `$(MAKE_COMMAND)`), so overriding it broke `$(MAKE)`. Renamed to `BUILD_COMMAND`.
- `cmake --install` left out `result.hpp`, `error.hpp` and `velocity_verlet.hpp`, so `#include <bclibc.hpp>` from the
  installed tree did not compile; the whole header tree is installed now instead of a hand-kept list.
- The installed `bclibc::bclibc_core` and `bclibc::bclibc_ffi` targets carried no include directory (`CMAKE_INSTALL_INCLUDEDIR`
  was still empty where the targets were declared), so `find_package(bclibc)` consumers could not find the headers.
  `bclibc_ffi` also exports its public include directory now.
- `tiny_bclibc` as a `find_package` package: the short target names of the README (`tiny_bclibc::headers`, `::baremetal`,
  `::static`, `::shared`) now exist there (and `::shared` in a subdirectory build); the exported long names are unchanged.
  `libm` is propagated to consumers, which compile the header-only math calls themselves, so linking the shared library
  or the headers no longer fails with undefined `sin`/`cos`/`atan2`.
- `tiny_bclibc` as a shared library did not link on macOS: Apple's `ld` has no `--strip-all` (it uses `-x` there now).

### Removed
- `build.sh`, `build.bat`, `build.ps1` and `clean.ps1`. Nothing used them (no workflow, no consumer: dart-bclibc,
  js-ballistics, micropython-bclibc and tiny-bclibc-wasm-py all build through `make`/`cmake`), `build.sh` reported
  "libraries not found" after a successful build and `build.ps1` needed a Developer prompt for `dumpbin`. The README
  describes `make` and `cmake` instead.

## [2.0.0] - 2026-10-03

Stable release, covering the whole arc from 1.1.8 through 2.0.0-rc.4. Highlights below; full details are in
the beta/rc entries beneath this one. The only change since rc.4 itself is CI-only: `release.yml` gates the
GitHub release job behind a manual-approval environment and no longer deletes a pre-existing release for the
same tag before publishing.

### Changed
- **BREAKING: bclibc no longer throws.** Every fallible function returns `BCLIBC_Result<T>` wrapping one
  tagged `BCLIBC_Error`, replacing the old exception classes and the `BCLIBC_BaseResult`/`BCLIBC_EngineResult`
  split. The flat C ABI, which already returned error codes, is unaffected at that boundary.
- **BREAKING: the RK45 tolerance/stats API is now per-instance.** The thread-local
  `BCLIBC_cashKarpSetRelativeTolerance`-style functions are replaced by `BCLIBC_CashKarpIntegrator`/
  `BCLIBC_DormandPrinceIntegrator`/`BCLIBC_TsitourasIntegrator` classes, each owning its own tolerances and
  accepted/rejected step counts (safe to share across threads via `std::ref`).
- **Event/row interpolation is interval-based, not per-raw-point**, and a scheduled sample no longer silently
  merges with a physical event at a nearby timestamp — both fix real accuracy/row-count bugs that adaptive
  stepping exposed.
- `tiny_bclibc` migrated its adaptive core from Cash-Karp to Tsitouras 5(4), matching bclibc's own default.
- The bare WebAssembly build (`BCLIBC_WASM_BARE=ON`) is one flavour for both toolchains (wasi-sdk, zig) now
  that the core never throws.

### Added
- Dormand-Prince 5(4) and Tsitouras 5(4) adaptive integrators, alongside the existing Cash-Karp.
- Bare (import-nothing) WebAssembly builds for both `bclibc` and `tiny_bclibc`, verified bit-identical to
  native.
- `BCLIBCFFI_interpolate_trajectory_data`, `BCLIBCFFI_hermite`, `BCLIBCFFI_interpolate_3pt`/`_2pt` in the flat
  C ABI.
- CI actually runs the C++ test suite and the `tiny_bclibc` identity test on every PR, plus a dedicated
  "no exceptions or RTTI in the core" guard.

### Removed
- The Emscripten-based wasm build (`build_wasm.sh`) and its automated emsdk-bump workflow, superseded by the
  bare wasm build.

## [2.0.0-rc.4] - 2026-10-01

### Changed
- **bclibc no longer throws.** Every fallible function returns `BCLIBC_Result<T>` (`include/bclibc/result.hpp`), the
  C++17 counterpart of `std::expected<T, BCLIBC_Error>` with no third-party dependency: `has_value()`/`has_error()`,
  `value()` and `error()` never throw (no `std::get`/`std::visit`; a wrong access asserts in debug). A
  default-constructed result holds a placeholder error, so one can be declared before it is assigned (Cython needs that).
- **One error type.** `BCLIBC_Error` (`include/bclibc/exceptions.hpp`) is a tagged sum of the payload structs
  `BCLIBC_LogicError`, `BCLIBC_DomainError`, `BCLIBC_RuntimeError`, `BCLIBC_OutOfRangeError`,
  `BCLIBC_InvalidArgumentError`, `BCLIBC_SolverZeroFindingError`, `BCLIBC_SolverOutOfRangeError`,
  `BCLIBC_SolverInterceptionError` and `BCLIBC_SolverRuntimeError`, each keeping the diagnostic fields the old exception
  classes carried. A failure propagates by returning the callee's error unchanged; consumers switch on `kind()` and read
  the payload with `payload<Payload>()` (`nullptr` for any other alternative). It replaces the
  `BCLIBC_BaseResult`/`BCLIBC_EngineResult` pair and `BCLIBC_BaseError`/`BCLIBC_EngineError` (and `widen()`).
  `BCLIBC_Error` needs the trajectory types its payloads carry, so a translation unit that reads a result includes
  `bclibc/exceptions.hpp`; the headers only declare `BCLIBC_Result`-returning functions (the default body of
  `BCLIBC_BaseTrajDataHandlerInterface::handle_step` moved to `src/traj_data.cpp` for that reason).
- Returning a result: `BCLIBC_BaseEngine::integrate`, `integrate_at`, `integrate_filtered`, `find_apex`,
  `error_at_distance`, `init_zero_calculation`, `zero_angle_newton`, `zero_angle_with_fallback`,
  `zero_point_with_fallback`, `find_zero_angle_ridder`, `find_max_range`, `zero_angle`, `find_zero_angle` and
  `find_zero_point`; the integrators; `BCLIBC_BaseTrajSeq`, `BCLIBC_TrajectoryData` and `BCLIBC_TrajectoryDataFilter`;
  `BCLIBC_ShotProps::drag_by_mach`/`update_stability_coefficient`, `BCLIBC_Shot::to_shot_props` and
  `build_pchip_curve_from_arrays`. Numerics are unchanged: only error propagation and handling moved.
- `BCLIBC_CashKarpIntegrator`/`BCLIBC_DormandPrinceIntegrator`/`BCLIBC_TsitourasIntegrator`'s
  `set_relative_tolerance`/`set_absolute_tolerance` return `BCLIBC_Result<std::monostate>` instead of
  throwing `std::invalid_argument`; their constructors log a warning and keep the field at its default
  (`embedded_rk45_detail::default_tolerance`) on an invalid value instead of failing to construct.
- `src/ffi/bclibc_ffi.cpp`: the `try`/`catch` safety net in `ffi_call` is gone, since nothing throws; every entry
  point maps a `BCLIBC_Error` to a `BCLIBCFFI_ERR_*` code through `setEngineError()` (`switch (error.kind())`),
  filling the same per-kind diagnostic fields as before. `BCLIBC_SolverRuntimeError` maps to
  `BCLIBCFFI_ERR_SOLVER_RUNTIME`, every other non-solver error to `BCLIBCFFI_ERR_GENERIC`, as on the previous release.
- **The bare WebAssembly build (`BCLIBC_WASM_BARE=ON`) is one flavour for both toolchains now.** Because the
  core never throws, it no longer needs real C++ exceptions: the `BCLIBC_WASM_EXCEPTIONS` option is gone, both
  `cmake/wasi-sdk-wasm32.cmake` and `cmake/zig-wasm32-wasi.cmake` build with `-fno-exceptions` and `-flto`, and wasi-sdk
  drops `-fwasm-exceptions`/`-lunwind` (so any wasm32 host works, not only one with WebAssembly's final exception
  encoding). A failed solve returns the same `BCLIBCFFI_ERR_*` code as the native library on both toolchains; zig's build
  no longer traps on an ordinary failed solve, only on something that would already be fatal natively.
  `src/wasm/bare_runtime.cpp`'s stubs are trimmed to what each toolchain's libc still pulls in without an exception
  runtime: the `__cxa_allocate_exception`/`__cxa_throw` traps are gone too (neither toolchain's link needs them any
  more). The module is built with `-fno-rtti` as well (zig: ~87 KB → ~84 KB); verified with zig and wasi-sdk 34 by
  `tests/wasm_parity/parity.py` on wasmtime and Node, bit-identical to native including the error path. The zig module was
  also run on wasmtime 24/36/49, wasm3, Node 18/20/22/24, bun 1.1/1.4, Deno 1.46/2.9 and Chrome 113/120/132/141, with
  identical results.
- `wasm-bare.yml` and the docs use `uv` (`uv pip install ziglang`, `uv run --with ziglang make wasm-zig`) instead of `pip`.
- `tests/wasm_parity/parity.py`: dropped `--errors codes|trap`; a trap during a normal case is always a failure now.

### Added
- `BCLIBCFFI_interpolate_trajectory_data` (with `BCLIBCFFI_TrajectoryInterpKey` and `BCLIBCFFI_InterpMethod`) in the flat C
  ABI: the point of a trajectory where a chosen field has a given value, interpolated from the three points around
  it (`BCLIBC_TrajectoryData::interpolate`, PCHIP or linear). Until now only the Embind bindings of js-ballistics
  had it, which the bare WebAssembly build has no use for. No struct changes: `BCLIBCFFI_get_layout()` is the same.
- `BCLIBCFFI_hermite`, `BCLIBCFFI_interpolate_3pt` and `BCLIBCFFI_interpolate_2pt` in the flat C ABI: the scalar interpolation
  functions of `interp.hpp`, for the same reason.
- `pr-check.yml` runs the C++ tests (`ctest`, Debug and Release, since the tests are assert-based) and the `tiny_bclibc`
  identity test; neither ran in CI before.
- `cmake/check_no_exceptions.sh` and a `No exceptions or RTTI in the core` job in `pr-check.yml`: builds the core and
  the C ABI natively with `-fno-exceptions -fno-rtti` (a `throw`, `try` or `catch` is a compile error) and fails if an
  object still references the C++ exception runtime or RTTI, so a regression shows up in the regular PR check and not
  only in the wasm one.

### Fixed
- `cmake/check_no_imports.cmake` now fails on an import section of any kind, by walking the module's sections, not only on
  a `wasi_snapshot_preview1` string.
- `tests/test_traj_data.cpp`: `test_streaming_step_coalesces_*` asserted that a ZERO_UP/ZERO_DOWN row merges with the
  RANGE row at the same point, which contradicts `BCLIBC_TrajectoryDataFilter::handle_step` (events and scheduled
  samples are separate rows by design since #30). They failed only with asserts enabled (Debug); CI's Release build
  compiles `assert` out. Renamed to `test_streaming_step_keeps_*_separate`, they now assert the real contract.

### Removed
- `build_wasm.sh` and `.github/workflows/emsdk-update.yml` (the Emscripten-based wasm build and its automated
  emsdk-bump PRs). CI (`build-lib.yml`, `build-libs.yml`, `pr-check.yml`, `release.yml`) now builds and publishes
  the bare wasm module(s) via `wasm-bare.yml` (made reusable via `workflow_call`) instead.

## [2.0.0-rc.3] - 2026-09-25

### Changed
- `BCLIBC_BaseEngine`'s lock is now the type `BCLIBC_Mutex`: `std::recursive_mutex` as before, except where the
  standard library has no threads (libc++ for a bare `wasm32-wasi`, or `-DBCLIBC_NO_THREADS`), where it is a lock
  that does nothing. WebAssembly's bare libc++ has no `std::recursive_mutex`, so the engine did not compile there.
  It follows libc++'s `_LIBCPP_HAS_THREADS`, not the target, so Emscripten (with or without `-pthread`) and every
  native build keep the real lock.
- `log.hpp` no longer includes `<iostream>` or uses exceptions: `BCLIBC_LOG_LEVEL` is read with `std::strtol`
  (the same values as `std::stoi` gave, anything else keeps the default) and the log line goes out through
  `fprintf(stderr)`, with the same text as before. A translation unit that got `<iostream>` (`std::cout`, `std::cerr`)
  only through `bclibc/log.hpp` has to include it itself now.
- Under `__wasm__` the log prints nothing by default (and does not even read `BCLIBC_LOG_LEVEL`, which would make
  a bare module import WASI's `environ_get`), so that it needs no WASI; define `BCLIBC_WASM_LOG` to have it (with WASI).

### Added
- `BCLIBC_WASM_BARE=ON`: builds the core and its flat C ABI (`bclibc_ffi.h`) into one WebAssembly module,
  `bclibc_wasm.wasm`, that imports nothing, so it runs with an empty import object in any host that has WebAssembly, and
  needs no Emscripten and no embind: CMake with one of two toolchain files (or `make wasm WASI_SDK_PATH=...` and
  `make wasm-zig`). The module exports `malloc` and `free`, is a reactor (the host calls `_initialize` first), its memory
  is its own and grows up to 2 GiB (`BCLIBC_WASM_MAX_MEMORY`, as Emscripten's `MAXIMUM_MEMORY`) with a 1 MiB stack
  (`BCLIBC_WASM_STACK_SIZE`), and the build fails if it imports from WASI.
  - `cmake/wasi-sdk-wasm32.cmake` (wasi-sdk 34, `-DWASI_SDK_PATH=`): **with C++ exceptions**, so the flat C ABI returns
    the same error codes as the native library. About 1.6 MB. It uses WebAssembly's final exception encoding
    (`try_table`), the only one wasi-sdk's libraries have, so the host must have it: wasmtime 49, wasm3 (a build from
    git), Node 25 and JavaScriptCore (WebKitGTK) were tried, and iOS 26 (an iPhone 16, Pythonista's `JSContext`, through
    `wasmhost.selftest`); `wasmhost.selftest` tells which encodings an engine takes. The build is deliberately not LTO:
    with LTO the option that selects the encoding is lost, and the module never catches.
  - `cmake/zig-wasm32-wasi.cmake` (zig 0.16): about 78 KB, but zig has no exception runtime, so **a `throw` is a trap**
    there: a failed solve ends the call instead of coming back as an error code, and the host makes a new instance
    after it.
- `src/wasm/bare_runtime.cpp` (only that build compiles it, it is not among `src/*.cpp`): libc++'s default
  `__libcpp_verbose_abort` writes to stderr, which makes a module import WASI's `fd_write`, `fd_seek` and `fd_close`
  as soon as it uses `std::vector`, `std::string` or `std::sort`; this one traps instead. Under wasi-sdk it also stands
  in for the parts of wasi-libc that libunwind and libc++abi would call (stderr, the environment, locks, the clock, the
  stack protector seed), and under zig it defines `__cxa_allocate_exception` and `__cxa_throw` as traps.
- `tests/wasm_parity/parity.py` and the `WASM (bare module)` workflow (`.github/workflows/wasm-bare.yml`): both
  flavours are built and the results of the same shots are compared with `libbclibc_ffi` on wasmtime and Node: every
  double is bit-identical except the angle fields (`drop_angle_rad`, `windage_angle_rad`, `angle_rad`), which differ by
  1 ulp (`atan`/`atan2` are not the same in glibc and in the module's musl; measured on x86-64, for all integration
  methods and atmospheres, and the same on wasmtime, wasm3, JavaScriptCore and Node), and a failed solve returns the
  native status (wasi-sdk) or traps (zig).

## [2.0.0-rc.2] - 2026-09-23

### Added
- `tiny_bclibc/build_wasm.sh` + `tiny_bclibc/wasm/tiny_bclibc_wasm.c`: builds tiny_bclibc
  (double and single precision) into WebAssembly modules that import nothing, so they run in a
  bare JS engine such as JavaScriptCore's `JSContext` on iOS (Pythonista) with no WASI or
  Emscripten glue. The wrapper exposes a flat, numbers-only ABI over two `double` buffers
  (`tbw_integrate` -> `tiny_bclibc_integrate_stream`, `tbw_find_zero_point` ->
  `tiny_bclibc_find_zero_point`). Toolchain: zig (`pip install ziglang`) or clang with a
  wasi-sdk sysroot.

## [2.0.0-rc.1] - 2026-09-22

### Changed
- **BREAKING**: The embedded RK45 methods' per-method tolerance/stats API
  (`BCLIBC_cashKarpSetRelativeTolerance`, `BCLIBC_cashKarpSetAbsoluteTolerance`,
  `BCLIBC_cashKarpGetStats`, and the equivalent `dormandPrince`/`tsitouras`
  functions) is replaced by stateful integrator classes
  (`BCLIBC_CashKarpIntegrator`, `BCLIBC_DormandPrinceIntegrator`,
  `BCLIBC_TsitourasIntegrator` — `set_relative_tolerance()`,
  `set_absolute_tolerance()`, `get_stats()`). The old functions read/wrote a
  `thread_local` keyed by `<Tableau, Controller>`, so every `BCLIBC_BaseEngine`
  on a thread using the same method shared one tolerance and one accepted/
  rejected step count, silently clobbering each other for any caller running
  more than one engine per thread (e.g. with intentionally different
  tolerances, or reading stats between two engines' runs). Each new
  integrator class instance owns its tolerances and stats independently, so
  it can be assigned straight to one engine's `BCLIBC_IntegrateCallable
  integrate_func` (wrap it in `std::ref` to keep reading its stats back
  afterward — `std::function` copies whatever is assigned to it otherwise).
  `BCLIBC_integrateCashKarp`/`BCLIBC_integrateDormandPrince`/
  `BCLIBC_integrateTsitouras` are unchanged as plain
  `BCLIBC_IntegrateCallable`-compatible free functions for the common case of
  running at the default 1e-6/1e-6 tolerances with no need to read back stats
  (e.g. `eng.integrate_func = bclibc::BCLIBC_integrateTsitouras;`).
- Each RK45 integrator class also gained an explicit constructor taking
  `(relative_tolerance, absolute_tolerance)` (each defaulting to 1e-6, same
  validation as the setters), so a configured instance can be built in one
  expression: `BCLIBC_CashKarpIntegrator(1e-8)`.
- The integrator classes' tolerances and step counts are each stored in a
  `std::atomic` (relaxed ordering), so a single instance can safely be
  shared across threads (e.g. via `std::ref`, driving several
  `BCLIBC_BaseEngine`s concurrently) — `set_relative_tolerance()` /
  `set_absolute_tolerance()` from one thread cannot race with a concurrent
  `operator()` or `get_stats()` call from another. `operator()` snapshots
  both tolerances once at the start of its run, so one integration always
  sees one consistent tolerance pair even if changed mid-run from another
  thread. `get_stats()`'s two fields are read independently, so a
  concurrent run can still return a non-atomic pairing of them (e.g. a
  fresh `accepted` count against the previous run's `rejected` count) —
  fine for approximate monitoring, not for anything requiring the two to
  agree exactly. Verified race-free under ThreadSanitizer with multiple
  engine threads sharing one integrator instance while a separate thread
  concurrently mutates tolerance and another polls stats
  (`tests/test_rk45_integrators.cpp::test_concurrent_tolerance_stats_access_is_race_free`,
  build with the new `-DBCLIBC_SANITIZE_THREAD=ON` CMake option); the same
  test against the pre-`std::atomic` version reproduces four real data
  races (one per atomic field), confirming both the fix and the test.
- Cash-Karp's doc comment (`BCLIBC_CashKarpIntegrator`, and by reference
  Dormand-Prince/Tsitouras) now also documents `std::function::target<T>()`
  as an alternative to `std::ref` for binding layers where
  `std::reference_wrapper`'s lack of a default constructor is inconvenient
  (e.g. Cython's C++ temp-variable codegen, which requires stack-allocated
  types to be default-constructible).

## [2.0.0-beta.8] - 2026-09-21

### Added
- New `BCLIBC_integrateTsitouras` (`bclibc/tsitouras.hpp`): Tsitouras 5(4) ("Tsit5"), a 7-stage
  FSAL RK45 pair structurally identical to `BCLIBC_integrateDormandPrince` (same
  `ScipyRKController`, same tolerance API shape), with coefficients verified against
  `ARKODE_TSITOURAS_7_4_5` in SUNDIALS/ARKODE. Adds `BCLIBCFFI_INTEGRATION_TSITOURAS` to the FFI
  enum and `build_wasm.sh`'s source list.
- `tiny_bclibc` migrated from Cash-Karp to Tsitouras as its baked-in adaptive core
  (`tiny_bclibc__run_tsitouras` in `engine.h`), including the FSAL derivative cache and
  wind-boundary step limiting the C++ `ScipyRKController` also does. Measured against the
  previous Cash-Karp build: within noise on wall-clock and accepted-step-count benchmarks for
  smooth ballistic trajectories — see [tiny_bclibc's README](tiny_bclibc/README.md#adaptive-integration-tsitouras)
  and the top-level [Adaptive integration](README.md#adaptive-integration) section for the
  measured numbers and why it was switched anyway.

### Fixed
- `bclibc_ffi.cpp`'s `calcStep()`/`selectIntegrateFunc()` grouped `BCLIBCFFI_INTEGRATION_EULER`
  with the switch's `default:` label, so any unrecognized/out-of-range method value silently ran
  (and step-sized for) Euler instead of RK4, the library's documented default. `default:` now
  falls back to RK4.
- `ScipyRKController::limit_step_at_wind_boundary` (shared logic duplicated in
  `dormand_prince.cpp` and `tsitouras.cpp`) could shrink `dt` toward zero without ever actually
  crossing a wind-zone boundary: it re-limits `dt` to the exact remaining distance every
  iteration, and floating-point rounding of that "land at, never past" step can leave `pos.x` a
  few ULPs short of the boundary instead of on or past it -- a Zeno's-paradox loop. Confirmed via
  `tiny_bclibc`'s C port of the same controller (which hit it in practice for a 3+ wind-zone
  shot, `tests/test_computer.py::test_multiple_wind` in py-ballisticcalc); Dormand-Prince simply
  hadn't hit it on the existing test suite, not because the logic was safe. Fixed by treating the
  boundary as already reached below a `1e-7` ft floor, matching the wind-change trigger's own
  epsilon.
- `tiny_bclibc`'s `ZERO_UP`/`ZERO_DOWN` event detection compared only each accepted interval's
  two endpoint signs, missing an up/down pair of crossings entirely contained within one interval
  (both endpoints land on the same side of the sight line). Cash-Karp's more conservative
  controller never grew a step wide enough to do this; Tsitouras's FSAL/larger-growth-cap
  controller does. Fixed by porting bclibc's own cubic-Hermite multi-root finder
  (`hermite_scalar_roots` in `src/traj_filter.cpp`) to `tiny_bclibc__hermite_scalar_roots` in
  `engine.h`, used the same way: split the interval at the cubic's own critical points so each
  sub-interval is monotonic, then bisect each for a sign change.

### Fixed
- `tiny_bclibc__run_tsitouras` folded `dt` into each stage's coefficient and added the result
  directly onto the (much larger-magnitude) `vr`/`pos` base, one term at a time -- unlike the
  C++ generic core (`embedded_rk45.hpp`), which accumulates the unscaled weighted sum of prior
  derivatives first and applies `dt` in a single multiply-then-add at the end. Both orders are
  algebraically identical, but repeatedly perturbing a large base with small per-term
  corrections rounds differently (and less favorably) than summing the small corrections
  together first. Rewrote all 6 stage computations (and the final FSAL `vr_next`/`pos_next`) to
  match the C++ core's accumulate-then-scale-then-add order. Confirmed via the identity test:
  per-field diffs on a simple no-wind shot dropped from ~1e-9 to ~1e-16-1e-13 (several fields
  now bit-identical). This alone didn't close py-ballisticcalc's
  `tests/test_hitresult.py::test_flags` divergence (see the next entry for why and how that
  got fixed too).

- `tiny_bclibc__hermite_at_time` reconstructed its interval-interior `.mach` (speed / local
  speed of sound) by linearly interpolating the *ratio* itself between the two accepted-step
  endpoints. Unlike position/velocity, `.mach` isn't part of the ODE's state, so there's no
  Hermite basis for it directly -- but the ratio's curvature over an interval comes almost
  entirely from velocity's curvature, and velocity *is* exactly reconstructable (it's what the
  Hermite derivative above already computes). Linearly interpolating the pre-divided ratio
  threw that away, costing real accuracy specifically at MACH crossings -- and specifically in
  the transonic region, where the drag curve's "bump" makes velocity's curvature largest.
  Confirmed via a step-by-step trajectory diff against the C++ engine on
  py-ballisticcalc's `tests/test_hitresult.py::test_flags` (a shot with wind and calculated
  powder sensitivity): every regularly-sampled point on both sides of the MACH crossing agreed
  to ~1e-14 (the accumulation-order fix above worked), but the crossing itself landed 0.67 yd
  off out of 963 yd (0.07%) -- isolated entirely to that one interpolated point, not a
  drifting/diverging step sequence as this entry previously (incorrectly) surmised. Fixed by
  matching bclibc's own C++ `traj_filter.cpp`: back out each endpoint's speed of sound (fps)
  from its already-known ratio and speed, linearly interpolate *that* (it varies smoothly with
  altitude, unlike the ratio near the transonic bump), and divide the accurately-reconstructed
  velocity magnitude by it. `test_flags` now passes bit-for-bit-adjacent under
  `TinyBclibcDoubleIntegrationEngine` (previously the suite's one known tiny_bclibc-only
  failure); no regressions across either precision's full suite.

### Known issues
- Single precision still shows two additional tolerance misses since the Tsitouras switch
  (`test_computer.py::test_wind_lag_rule`/`test_multiple_wind`), a side effect of the
  accumulation-order fix above shifting float32 rounding at already-marginal self-consistency
  tolerances -- same class as the rest of `TinyBclibcSingleIntegrationEngine`'s pre-existing
  documented precision-floor failures (see its docstring in py-ballisticcalc's
  `examples/tiny_bclibc/__init__.py`), not a logic bug.

## [2.0.0-beta.7] - 2026-09-18

### Changed
- `tiny_bclibc` full migration to Cash-Karp done

### Fix
- Got `bclibc` and `tiny_bclibc` identity test pass

## [2.0.0-beta.6] - 2026-09-17

### Fixed

- WASM builds now regenerate `build/generated/bclibc/version.h` on every invocation, preventing
  a header left by an earlier native build from embedding a stale `BCLIBC_VERSION` in the shipped
  `.wasm` artifact. The Emscripten export list now also includes `BCLIBCFFI_find_zero_point` and
  `BCLIBCFFI_find_zero_point_shot`, making the zero-point FFI API available to WASM consumers.

## [2.0.0-beta.5] - 2026-09-17

### Fixed
- `BCLIBCFFI_get_layout` add BCLIBCFFI_ZeroPointResult

## [2.0.0-beta.4] - 2026-09-17

### Added

- `tiny_bclibc`: `tiny_bclibc_find_zero_point()` returns the lower-arc zero angle together with
  the full `TINY_BCLIBC_TrajectoryData` RANGE point at the requested distance, without mutating
  the caller's `TINY_BCLIBC_ShotProps`.

## [2.0.0-beta.3] - 2026-09-17

- `tiny_bclibc`: Cash-Karp adaptive error control now matches the C++ engine's default
  solve_ivp-style policy: one `1e-6` absolute and relative tolerance for all six position and
  relative-velocity components, each independently scaled before calculating the RMS acceptance
  norm. This replaces the unequal position/velocity floors and max-of-vector-norms criterion.

## [2.0.0-beta.2] - 2026-09-17

### Added

- Public Dormand--Prince 5(4) integration (`BCLIBC_integrateDormandPrince`)
  and `BCLIBCFFI_INTEGRATION_DORMAND_PRINCE`.
- Compile-time embedded RK tableau/controller core (`integrate_embedded_rk45<Tableau, Controller>`,
  `bclibc/embedded_rk45.hpp`). Cash--Karp and Dormand--Prince are now both thin `.cpp` files that
  instantiate this core with their own tableau/controller instead of duplicating the adaptive
  step-control loop; `State<Tableau, Controller>` gives each method's tolerance/step-count
  thread-locals independent storage (previously, before this was templated, they would have been
  shared namespace-scope statics -- fine for Cash-Karp alone, but wrong once a second method
  shares the core). Cash--Karp's tableau, controller factors, and lack of wind-boundary step
  limiting are preserved exactly, byte-for-byte behavior compatible with the pre-refactor
  standalone implementation.
- Dormand--Prince additionally clamps its step at a wind-zone boundary
  (`ScipyRKController::limit_step_at_wind_boundary`): wind is sampled once per step and held
  constant across all of that step's stage evaluations, so a large adaptive step that both starts
  before and would end past the next wind segment's boundary would otherwise integrate its tail
  through the wrong wind vector. FSAL cache invalidation (already centralized in the core) only
  fixes the *next* step's stale derivative, not this step's own wind model, so this needed its own
  fix. Cash-Karp does not get this fix, matching its "preserve historical behavior" mandate above.

### Changed
- Cash-Karp error control now matches `scipy.integrate.solve_ivp`'s Runge-Kutta convention:
  `BCLIBC_cashKarpSetAbsoluteTolerance()` sets one thread-local scalar `atol` (default `1e-6`)
  for all six position/velocity components, replacing unequal hidden position/velocity floors.
  Each component is scaled by `atol + rtol * max(abs(y), abs(y_new))`, and the adaptive
  acceptance norm is the RMS across those six scaled errors.

## [2.0.0-beta.1] - 2026-09-16

### Added
- Cash-Karp adaptive RK45 integrator (`BCLIBC_integrateCashKarp`, `bclibc/cash_karp.hpp`) —
  Numerical Recipes' `rkck` embedded 4th/5th-order method, selectable via
  `BCLIBC_BaseEngine::integrate_func` like `BCLIBC_integrateRK4`. Grows its step up to 64x the
  configured base step during smooth flight and shrinks to base/64 on error-estimate rejection
  (retrying rather than accepting), typically needing 2-6x fewer total steps than fixed-step
  RK4 for comparable accuracy. `BCLIBC_cashKarpSetRelativeTolerance()` sets the thread-local
  relative error tolerance (default `1e-6`, empirically Pareto-optimal on the one profile
  measured so far — see its doc comment for the data); `BCLIBC_cashKarpGetStats()` returns the
  accepted/rejected step counts from the most recent call on the current thread.
- `tiny_bclibc`: adaptive Cash-Karp core (`tiny_bclibc__run_cashkarp`, `engine.h`), now backing
  `tiny_bclibc_integrate()`/`tiny_bclibc_integrate_stream()`.
- `tiny_bclibc`/`interp.h`: `tiny_bclibc_hermite_derivative()` — the exact derivative of
  `tiny_bclibc_hermite()`'s cubic, used to reconstruct a physically consistent velocity from an
  interval's endpoint (position, velocity) pairs instead of a separate finite-difference
  velocity estimate.

### Changed
- **Event/row interpolation is now interval-based, not per-raw-point.** Every integrator
  (`BCLIBC_integrateRK4`, `BCLIBC_integrateCashKarp`, Euler, Velocity Verlet; `tiny_bclibc`'s
  Cash-Karp core) streams each accepted `(start, end)` interval to the handler
  (`BCLIBC_BaseTrajDataHandlerInterface::handle_step`, added with a default that falls back to
  `handle(end)`). `BCLIBC_TrajectoryDataFilter`/`BCLIBC_SinglePointHandler` and `tiny_bclibc`'s
  own filter now reconstruct RANGE/time-step rows and APEX/MACH/ZERO event roots with a 2-point
  cubic Hermite built from each interval's *exact* endpoint state (bisecting on time where
  needed), replacing per-point interpolation over a 3-point window of raw samples
  (`BCLIBC_interpolate3pt`/`tiny_bclibc_interpolate3pt`, finite-difference PCHIP slopes). The
  old scheme's slopes were estimated from neighboring raw-sample spacing — accurate when RK4's
  fixed step keeps samples dense and uniform, but measurably wrong once an adaptive integrator's
  accepted steps are sparse and irregular (a multi-yard event-crossing miss was observed and is
  what motivated this fix).
- **Scheduled samples and physical events are no longer merged into one row when their
  timestamps happen to land close together.** `BCLIBC_TrajectoryDataFilter::add_row` and
  `merge_sorted_record`'s implicit time-tolerance merge is gone for the interval-based path —
  a RANGE-step row and a ZERO/MACH/APEX event are always independent records now, even at an
  identical timestamp. Merging depended on raw-sample spacing that adaptive stepping no longer
  guarantees, and previously produced a real bug: making event-time reconstruction more accurate
  could push a previously-"accidentally" coincident event/row pair just far enough apart that
  they stopped merging, silently changing output row counts. Downstream consumers that want the
  old "closest sample annotated with nearby event flags" view should compute that projection
  themselves from the exact records (see `py_ballisticcalc`'s `HitResult.trajectory` for a
  reference implementation).
- `tiny_bclibc`: `tiny_bclibc_integrate()`/`tiny_bclibc_integrate_stream()` now run the
  Cash-Karp core instead of fixed-step RK4. `tiny_bclibc_integrate_raw()`, `integrate_at()`,
  `find_apex()`, and `find_zero_angle()` are unchanged (still RK4-based internally).
- `tiny_bclibc`: the MACH-crossing and ZERO-crossing detection conditions in the interval-based
  filter now use strict `>`/`<` on both sides of the threshold, matching
  `py_ballisticcalc`'s `TrajectoryDataFilter.record_step` reference. The previous non-strict
  `>=`/`<=` treated a state exactly on the threshold (e.g. a muzzle velocity exactly at Mach 1)
  as already past it, which under interval-based (start-inclusive) detection could register a
  false crossing essentially at t=0 instead of at the real, later crossing.

### Fixed
- Cash-Karp: the drag coefficient **and** the atmosphere sample are now recomputed fresh at
  each of the 6 stages, from that stage's own intermediate velocity/altitude — unlike
  `BCLIBC_integrateRK4`, which freezes both once per step. An earlier variant that reused RK4's
  once-per-step freeze (extended to 6 stages) produced real, tolerance-independent accuracy
  failures: the embedded error estimator can't see model error from a stale drag/atmosphere
  sample, only discretization error of the frozen sub-problem, so tightening tolerance never
  helped.
- Cash-Karp: `time` now accumulates the step size actually integrated (captured before growing
  `dt` for the next attempt), not the already-grown value — the previous code produced a real
  ~4.3% flight-time error.
- `hermite_at_x()` (`src/traj_filter.cpp`, and its `tiny_bclibc` port in `engine.h`) now assigns
  the exact query target to the result's downrange position instead of re-deriving it from the
  bisection-converged Hermite sample. Invisible at double precision (residual is far below any
  existing tolerance), but at `tiny_bclibc`'s single-precision (`float`) build, one `float`
  ULP at typical trajectory ranges (~0.004 ft at 36000 ft) exceeded the bisection residual,
  measurably missing an exact RANGE-step target.
- `get_step_stats()`-equivalent (`BCLIBC_cashKarpGetStats`): documented as reading thread-local
  counters shared across every instance integrating on the same thread — callers needing a
  specific instance's own stats must snapshot immediately after that instance's own `integrate()`
  call returns (see `py_ballisticcalc.exts`' `CythonizedCashKarpIntegrationEngine` for the
  pattern), not read it lazily afterward.
- `std::clamp` (C++17) replaced with manual `std::min`/`std::max` clamping in `cash_karp.cpp`,
  matching `rk45.cpp`'s style — this project targets `-std=c++11`.

## [1.1.8] - 2026-09-26

### Added
- `tiny_bclibc`: `tiny_bclibc_integrate_raw()` — streams every raw RK4 step
  (`TINY_BCLIBC_BaseTrajData`: time/position/velocity/mach, no C-side filtering or
  interpolation) to a caller-supplied callback. Intended for external drivers (e.g. ctypes/FFI
  bindings) that want to reuse a host-language trajectory filter, zero-finding, apex, and
  max-range implementation while delegating only the numerically-sensitive RK4 stepping to
  tiny_bclibc, so the host can exercise tiny_bclibc's compiled precision (float or double) as
  the physics core of its own engine.
- `tiny_bclibc`: `tiny_bclibc_sizeof_shot_props()` / `tiny_bclibc_sizeof_curve_point()` — let
  external callers validate an opaque buffer size, or a struct-layout mirror (e.g. a ctypes
  `Structure`), against the actual compiled layout.
- `tiny_bclibc`: `tiny_bclibc_integrate_stream()` gains an optional `out_final_raw` parameter
  (`TINY_BCLIBC_BaseTrajData *`, may be `NULL`) exposing the exact terminal raw kinematic state
  of the integration, regardless of whether the C-side filter happened to emit a row for it.
  Lets a caller reconstruct the equivalent of `BCLIBC_TrajectoryDataFilter`'s
  destructor-time finalize (append the exact terminal point when a trajectory ends other than
  by reaching the requested range) without tiny_bclibc itself needing to grow that logic.

### Fixed
- `tiny_bclibc`: `tiny_bclibc__run_rk4`'s stop control (minimum velocity, maximum drop, minimum
  altitude) now reads from `TINY_BCLIBC_ShotProps::cfg` instead of hardcoded constants, and
  `tiny_bclibc_build_shot_props()` now populates `cfg` from `TINY_BCLIBC_Shot::config` (it
  previously left `cfg` unset). Previously every trajectory was silently capped at a hardcoded
  -15000 ft drop / -1500 ft altitude / 0 fps minimum velocity regardless of the caller's actual
  config, which in particular broke long-search algorithms (e.g. max-range / zero-angle search
  over a wide angle bracket) that rely on temporarily relaxing these limits.
- `tiny_bclibc`: gravity acceleration in `tiny_bclibc__run_rk4` now uses
  `props->cfg.cGravityConstant` instead of a hardcoded `-32.17405`, so a caller-supplied
  non-default gravity constant is honored.
- `tiny_bclibc__integrate_on_step`'s MACH crossing check compared ground velocity (fps)
  directly against `BaseTrajData::mach` (a dimensionless Mach *ratio* in this codebase, not a
  speed) -- `vel < pt->mach` was therefore false for any realistic velocity, so a MACH crossing
  could never be detected via this path. Now compares the ratio to `1.0` directly.
- `tiny_bclibc__integrate_on_step`'s ZERO crossing interpolated raw `py` against a reference
  height computed from the *current* point's `px` (`px * tan(look_angle)`), which drifts as
  `px` moves across the 3-point window and is only accurate for shallow look angles (confirmed
  ~27 ft off at a 30° look angle in a downstream consumer's test). Now interpolates the
  slant height directly (`py*cos(look_angle) - px*sin(look_angle)`, the same geometric
  quantity bclibc's C++ `BCLIBC_TrajectoryDataFilter`/`BCLIBC_TrajectoryData::interpolate`
  interpolates to zero) via the existing `tiny_bclibc_interpolate3pt`, which is exact
  regardless of angle.
- `tiny_bclibc__integrate_on_step`'s ZERO_UP/ZERO_DOWN branch tested `(bit_is_set &&
  geometric_condition)` for both directions in an if/else-if chain, rather than branching on
  which bit is still live first (as bclibc's C++ `BCLIBC_TrajectoryDataFilter::record` and
  py_ballisticcalc's `TrajectoryDataFilter.record` both do via `if ZERO_UP: ... elif
  ZERO_DOWN: ...`, gated on the bit alone). Concretely: while still waiting for a shot to cross
  *up* through the sight line, every step where it was still below the line (a legitimate,
  common state, not just a brief transient) satisfied the `ZERO_DOWN` branch's combined
  condition and got wrongly reported as a ZERO_DOWN crossing, since the ZERO_DOWN bit hadn't
  been cleared yet either. Fixed to branch on bit membership only, matching both reference
  implementations.
- `tiny_bclibc__integrate_on_step`'s RANGE-step target was accumulated via repeated
  `next_range_dist += range_step_ft`; replaced with a step index (`range_step_ft *
  step_index`), so per-target rounding no longer compounds across iterations. Note this does
  *not* recover full `range_limit_ft`/`range_step_ft` precision on its own: those fields are
  `real_t`, so a caller requesting a target more precise than `real_t` can represent already
  loses that precision at the call boundary, before this loop runs at all -- e.g. a
  single-precision consumer asking for an exact ~741 m range step is off by ~3e-5 m purely from
  that one rounding. Requesting sub-`real_t`-precision output from a `TINY_BCLIBC_SINGLE_PRECISION`
  build is not something any amount of downstream arithmetic can fix; the index-based change
  here only removes the *additional* drift repeated float32 addition was contributing on top of
  that inherent limit.

## [1.1.7] - 2026-07-24

### Added
- Velocity Verlet integration engine (`src/velocity_verlet.cpp`, `include/bclibc/velocity_verlet.hpp`,
  `BCLIBC_integrateVELOCITY_VERLET`): symplectic, time-reversible second-order integrator alongside
  RK4 and Euler. Like RK4 it uses a fixed (non-adaptive) time step and carries the
  end-of-step acceleration into the next step, so it costs one extra acceleration
  evaluation per step versus RK4's four. Wired into the FFI as
  `BCLIBCFFI_INTEGRATION_VELOCITY_VERLET = 2` (`BCLIBCFFI_IntegrationMethod`), selectable the same
  way as `BCLIBCFFI_INTEGRATION_RK4`/`BCLIBCFFI_INTEGRATION_EULER`.

## [1.1.6] - 2026-07-22

### Added
- WebAssembly build: `build_wasm.sh` compiles the existing flat C ABI
  (`bclibc_ffi.cpp`, `include/bclibc/ffi/bclibc_ffi.h`) to a WebAssembly +
  Emscripten JS-glue module (`build/web/bclibc_ffi.js` + `.wasm`), for
  consumers without `dart:ffi`/native FFI (e.g. Flutter/Dart web). No
  Embind — same `BCLIBCFFI_*` exports as the native shared library.
  Self-installs a pinned Emscripten SDK (`tool/emsdk/`, gitignored) on first
  run if `emcc` isn't already on `PATH`.
  - Everything the script writes stays inside `bclibc/` itself (not the
    embedding parent repo's directory), so it works whether `bclibc` is
    checked out standalone or as a submodule of any of its several
    consumers (dart-bclibc, js-ballistics, py-ballisticcalc,
    micropython-bclibc).
  - Do not add `-sSINGLE_FILE=1` back to the build flags: as of emsdk 6.0.3
    it produces a wasm blob Chrome's `WebAssembly.instantiate` rejects
    (`invalid value type 0x1`), even though the identical bytes load fine
    under Node. The two-file (`.js` + `.wasm`) layout is the
    verified-working one.
- `BCLIBCFFI_get_layout()`: new FFI export returning every
  `BCLIBCFFI_Shot`-family struct's field byte offsets/sizes, computed via
  `offsetof()`/`sizeof()` by whichever compiler built the library. Lets a
  hand-written JS/wasm binding (no Embind, no struct-aware FFI shim)
  marshal structs into wasm linear memory without hardcoding — or risking
  drift from — the C struct layout. Unused by the native `dart:ffi` path,
  which already gets struct layout from `ffigen` at bindings-generation
  time.
- CI: `arch: wasm` option on the reusable `build-lib.yml` (and wired into
  `build-libs.yml` / `pr-check.yml`), building via `build_wasm.sh` instead
  of CMake and caching the emsdk install.

## [1.1.5] - 2026-07-21

### Fixed
- `BCLIBC_BaseTrajSeq::get_at()` (`src/traj_data.cpp`) silently extrapolated instead of raising when
  `key_value` was outside the trajectory's range: `bisect_center_idx_buf()`/`find_target_index()` clamp
  their bracket index to `[1, n-2]` regardless of how far `key_value` falls outside the sequence, so
  `interpolate_at()` only ever validated the *index*, never the *value*. `get_at()` now checks `key_value`
  against `[min(buffer[0][key], buffer[n-1][key]), max(...)]` (± a `1e-9` epsilon) before searching and
  throws `std::out_of_range` if it falls outside. ([#19](https://github.com/ballistics-lab/bclibc/issues/19))
- `pre-commit-check.sh`: artifact check looked for `libbclibc_core.a` in the build root, but
  `ARCHIVE_OUTPUT_DIRECTORY` in `CMakeLists.txt` places it under `lib/`, so step 3 always failed
  (silently, since `pr-check.yml` invoked the script with `|| true`). Fixed the expected path.

### Added
- `tests/`: minimal CTest-based unit test target (`bclibc_tests`) covering `BCLIBC_BaseTrajSeq::get_at()`,
  wired up via the previously-unused `BCLIBC_BUILD_TESTS` CMake option
- `make test`: configures with `-DBCLIBC_BUILD_TESTS=ON`, builds, and runs `ctest`

### Changed
- `pre-commit-check.sh` now builds with `-DBCLIBC_BUILD_TESTS=ON` and runs `ctest` as a required step

### Removed
- Deprecated define `-DTINY_BCLIBC_USE_FLOAT` deleted from lib

## [1.1.4] - 2026-06-26

### Added

#### `tiny_bclibc_integrate_stream` — zero-allocation streaming integration
- New public API: `tiny_bclibc_integrate_stream(props, req, cb, cb_ctx, out_total, out_reason)`
  - Calls a C callback `tiny_bclibc_StreamCb` once per filtered output point instead of writing to a heap buffer
  - Callback returns `TINY_BCLIBC_TERM_HANDLER_STOP` (or any non-zero) to abort integration early
  - No intermediate `TrajectoryData` buffer allocated — suitable for RAM-constrained MCUs
  - Shares 100 % of the filtering logic with `tiny_bclibc_integrate` via the existing `tiny_bclibc__integrate_on_step` path; no code duplication
- New public typedef: `tiny_bclibc_StreamCb` — `int32_t (*)(const TINY_BCLIBC_TrajectoryData *, void *)`

#### Drag tables extracted to separate header
- `src/drag_tables.h`: G1 and G7 built-in drag tables extracted from the MicroPython binding into a standalone header with include guard

#### Experimental status documented
- `tiny_bclibc` and the MicroPython module are explicitly marked **experimental** in all README files. APIs, binary layout, and build system may change without notice until stabilised.

### Changed

- MicroPython module (natmod, usermod, FFI) moved to a dedicated repository: **[github.com/ballistics-lab/micropython-bclibc](https://github.com/ballistics-lab/micropython-bclibc)**

## [1.1.3] - 2026-06-22

### Fixed
- `tiny_bclibc`: `TINY_BCLIBC_FAST_ZERO_FIND` returned wrong zero angle (~0.078° instead of ~0.143° for a 300 m zero).
  Root cause: `acc = 0.01` (a height tolerance in feet) was also used for the Ridder's angle-bracket
  convergence checks (`|next_angle − mid_angle|` and `|high_angle − low_angle|`).  With `acc = 0.01 rad =
  0.573°`, the bracket triggered premature convergence before the true zero angle (~0.0025 rad) was reached.
  Fix: introduce a separate `angle_tol = 1e-5 rad` for the angle-difference checks; `acc` now governs only
  height-error convergence (`|f_mid|`, `|f_next|`) as intended.
- `bclibc` (C++ engine): same units mismatch in `find_zero_angle` — `cZeroFindingAccuracy` (height in ft) was
  used for Ridder's angle-bracket convergence.  Introduced `kRiddersAngleTol = 1e-5 rad` to decouple them.
  No observable regression at the default accuracy (`0.001`), but protects against incorrect results if a
  larger accuracy value is supplied.
- `test_bclibc.py` now asserts `find_zero_angle` returns within 1e-4 rad of the reference value
  (0.002502 rad = 0.1434° for G7 BC=0.310, 168 gr, 2750 fps, 1.5 in sight, 300 m zero).
  The test exits with a non-zero code when any assertion fails, making CI catch value regressions.

### Added

#### Experimental status
- `tiny_bclibc` (C99 engine) and `micropython-mod` are now explicitly marked **experimental**
  in all `README.md` files (`tiny_bclibc/README.md`, `micropython-mod/README.md`, root
  `README.md`). APIs, binary layout, and build system may change without notice until
  the features are stabilised.

#### Float32 vs Float64 precision comparison (natmod)
- Added `precision_run.py` (MicroPython worker) and `precision_compare.py` (CPython runner)
  to `micropython-mod/` for measuring accumulated trajectory deviation between the
  `float32` (`-DTINY_BCLIBC_USE_FLOAT`) and `float64` natmod builds.
- Test conditions: G7, BC=0.310, 168 gr, mv=2750 fps, T=15°C, P=1013.25 hPa, RH=0.5,
  0–3000 m, output step=25 m (120 sample points), MicroPython v1.26 unix x64.
  `range_step_ft` is the output sampling step only; internal RK4 sub-step is controlled
  independently by `step_multiplier` (default 0.5).
- Results (f32 − f64, double as reference):
  - Max vertical drop deviation: **0.108 cm** at 2975 m
  - Max velocity deviation: **0.0015 fps** (0.0005 m/s)
  - Max Mach deviation: **1.32 × 10⁻⁶**
  - `find_zero_angle` (300 m zero): **5 × 10⁻¹⁰ rad** (< 0.001 mrad)
  - Float32 is sufficient for all supported MCU targets over distances up to 3000 m.
- Documentation with full methodology added to `micropython-mod/README.md`,
  `tiny_bclibc/README.md`, and root `README.md`.

#### `tiny_bclibc` — Pure C99 ballistics engine
- New `tiny_bclibc/` subtree: header-only C99 port of the ballistics engine
  - `real_t` = `double` by default; `float` with `-DTINY_BCLIBC_USE_FLOAT`
  - Three usage modes: header-only (`static inline`), shared library, static library via single TU `src/tiny_bclibc_impl.c`
  - Public API: `tiny_bclibc_build_shot_props`, `tiny_bclibc_integrate`, `tiny_bclibc_integrate_at`, `tiny_bclibc_find_zero_angle`, `tiny_bclibc_find_apex`, `tiny_bclibc_find_max_range`, `tiny_bclibc_last_error`
  - CIPM-2007 atmosphere, PCHIP drag curves, Coriolis, spin drift, Ridder zero-finding, RK4 integration
  - Bare-metal / RTOS compatible: `TINY_BCLIBC_NO_THREAD_LOCAL`, `TINY_BCLIBC_NO_ERR_BUF`
  - CMake package with `tiny_bclibc::headers` / `tiny_bclibc::shared` / `tiny_bclibc::static` targets
  - Identity test suite (`tests/test_identity.cpp`) verifying numerical agreement with the C++ engine

#### `micropython-mod` — MicroPython native module
- New `micropython-mod/` subtree: `.mpy` native module wrapping `tiny_bclibc`
  - Supports 11 architectures: x64, x86, armv6m, armv7m, armv7emsp, armv7emdp, xtensa, xtensawin, rv32imc, rv64imc (single and double precision variants)
  - Bundled math: `libm_dbl` (musl-derived, x64/x86 double), fdlibm (x64/x86 single, RISC-V); ARM/Xtensa uses newlib via `LINK_RUNTIME`
  - `math_shim.c`: `sincos`/`sincosf` shim for GCC `-O2` merge optimisation
  - `mem_shim.c`: `memset`/`memcpy` shim for bare-metal targets
  - `math_shadow/math.h`: intercepts glibc `<math.h>` to prevent `__sin`/`__cos` signature conflict with musl libm_dbl
  - `tiny_bclibc_types.py`: `Shot`, `Wind`, `Config`, `Request` data classes with `pack()`/`unpack()`
  - `test_bclibc.py`: full test suite (integrate, find_zero_angle, find_apex, integrate_at, RAM test)
  - `tests/test_ffi.py`: mirror test suite using MicroPython `ffi` module against `libtiny_bclibc.so` — works on any unix port architecture (aarch64, mipsel, …) without a native module
  - `ci/run_qemu.py`: QEMU pty bridge for running natmod tests on emulated MCU targets; supports `--machine` and `--qemu-extra` for any QEMU ARM board
- CI workflow `.github/workflows/natmod.yml`:
  - Builds all arch/precision matrix in parallel
  - Tests on x64 and x86 unix port (both precisions)
  - Tests on QEMU Cortex-M3 (`MPS2_AN385` / armv7m)
  - `workflow_dispatch` trigger with `mpy_tag` input to test against any MicroPython release
- `TINY_BCLIBC_FAST_ZERO_FIND` compile-time flag for `find_zero_angle` on soft-float MCUs (Cortex-M0+, RISC-V without FPU):
  - GSS bracket search uses 8× coarser RK4 step — reduces steps per trajectory ~8×
  - GSS convergence threshold relaxed to `1e-2 rad` (~13 iterations vs ~25) — halves trajectory count
  - Ridder's height-error tolerance `acc` relaxed to `0.01 ft` (3 mm) — within `float` precision floor; angle-bracket convergence uses a separate `1e-5 rad` constant, unchanged
  - Final angle is computed by Ridder's at full `calc_step`; output accuracy is unchanged
  - Enabled automatically by natmod `Makefile` when `USE_FLOAT=1`; independent of `TINY_BCLIBC_USE_FLOAT`
- `micropython-mod/natmod/RISC-V_picolibc.md`: documents two `mpy_ld.py` bugs triggered by picolibc on RISC-V and the patch in `natmod/patches/micropython/mpy_ld_srodata.patch`
- `micropython-mod/src/sincosf_shim.md`: documents why `src/math_shim.c` is compiled only for x64/x86

### Changed
- `README.md`: added repository structure overview; sections for `tiny_bclibc` and `micropython-mod`
- Updated `Makefile`, `CMakeLists`, `build-libs` to be consistent and better structured
- natmod `math_shim.c` (`sincosf` shim) removed from RISC-V build — GCC does not generate `sincosf` calls on ARM/RISC-V with the flags used; saves 68 B of flash
- natmod armv6m QEMU test (`MICROBIT` board) removed — MICROBIT firmware does not support loading native `.mpy` for Cortex-M0; build verification in the `build` job is sufficient

## [1.1.2] - 2026-05-29

### Fixed
- `CMakeLists.txt`: removed `-ffast-math` from Release build flags; the flag broke trajectory event detection (apex, zero-angle) by relaxing IEEE 754 semantics required by the solver's sign-change logic.

## [1.1.1] - 2026-05-29

### Fixed
- `BCLIBC_Atmosphere::update_density_factor_and_mach_for_altitude()`: added vacuum guard (`_p0 <= 0`) before the barometric formula; previously `_p0 = 0` (set by `from_conditions(p_hpa=0)`) caused `0/0 = NaN` for `density_delta`, propagating NaN drag through any vacuum trajectory where altitude changes by more than 30 ft from base.
- `BCLIBC_Shot::to_shot_props()`: `BCLIBC_MachList` now move-constructed from `mach_v` instead of copy-constructed; eliminates one redundant O(N) heap allocation per shot.

## [1.1.0] - 2026-05-26

### Added
- `BCLIBC_Coriolis::from_lat_az(lat_deg, muzzle_velocity_fps, az_deg = NaN)` — static factory; computes all trig pre-computation (sin/cos lat, sin/cos az, range/cross offsets) from geographic degrees; NaN lat → no Coriolis; NaN az → flat-fire drift only
- `BCLIBC_Atmosphere::from_conditions(t_c, p_hpa, alt_ft, humidity = 0.0)` — static factory; CIPM-2007 moist-air density, Rankine Mach formula matching Python `machF()` exactly; p_hpa ≤ 0 → vacuum (zero density, no drag)
- `BCLIBC_cStandardDensityMetric` constant (1.2250 kg/m³) — ICAO sea-level standard density used to normalise CIPM-2007 output
- `BCLIBC_Shot` struct — user-facing shot descriptor (natural units, no pre-computation, non-owning pointers for drag table and winds arrays); `to_shot_props()` assembles engine-ready `BCLIBC_ShotProps` performing all conversions in one place: cant cos/sin, atmosphere, Coriolis, PCHIP drag curve, wind sock
- `BCLIBCFFI_Shot` C struct in `bclibc_ffi.h` — C-compatible mirror of `BCLIBC_Shot` for Dart FFI / other C consumers; takes raw user-facing units (`temp_c`, `pressure_hpa`, `latitude_deg`, `azimuth_deg`) and two parallel `mach_data`/`cd_data` arrays (same layout as `BCLIBC_Shot`); `pressure_hpa == 0` → vacuum
- `BCLIBCFFI_find_apex_shot`, `BCLIBCFFI_find_max_range_shot`, `BCLIBCFFI_find_zero_angle_shot`, `BCLIBCFFI_integrate_shot`, `BCLIBCFFI_integrate_at_shot` — FFI entry points accepting `BCLIBCFFI_Shot`; all physics conversion delegated to `BCLIBC_Shot::to_shot_props()` in C++

### Changed (**breaking** — requires new minor version for all C FFI consumers)
- `bclibc_ffi.h`: all C struct and enum types renamed from `BC*` to `BCLIBCFFI_*` to be consistent with the `BCLIBCFFI_` function prefix:
  - `BCConfig` → `BCLIBCFFI_Config`
  - `BCAtmosphere` → `BCLIBCFFI_Atmosphere`
  - `BCCoriolis` → `BCLIBCFFI_Coriolis`
  - `BCWind` → `BCLIBCFFI_Wind`
  - `BCDragPoint` → `BCLIBCFFI_DragPoint`
  - `BCShotProps` → `BCLIBCFFI_ShotProps`
  - `BCShot` → `BCLIBCFFI_Shot` (also changed `drag_table: BCDragPoint*` to separate `mach_data/cd_data: const double*` arrays, matching `BCLIBC_Shot` layout)
  - `BCTrajFlag` enum → `BCLIBCFFI_TrajFlag`; values `BC_TRAJ_FLAG_*` → `BCLIBCFFI_TRAJ_FLAG_*`
  - `BCTerminationReason` enum → `BCLIBCFFI_TerminationReason`; values `BC_TERM_*` → `BCLIBCFFI_TERM_*`
  - `BCBaseTrajInterpKey` enum → `BCLIBCFFI_BaseTrajInterpKey`; values `BC_INTERP_KEY_*` → `BCLIBCFFI_INTERP_KEY_*`
  - `BCIntegrationMethod` enum → `BCLIBCFFI_IntegrationMethod`; values `BC_INTEGRATION_*` → `BCLIBCFFI_INTEGRATION_*`
  - `BCTrajectoryRequest` → `BCLIBCFFI_TrajectoryRequest`
  - `BCBaseTrajData` → `BCLIBCFFI_BaseTrajData`
  - `BCTrajectoryData` → `BCLIBCFFI_TrajectoryData`
  - `BCMaxRangeResult` → `BCLIBCFFI_MaxRangeResult`
  - `BCInterception` → `BCLIBCFFI_Interception`
  - `BCLIBCFFIStatus` enum → `BCLIBCFFI_Status` (consistent with `BCLIBCFFI_<Name>` pattern)
  - `BCLIBCFFIError` struct → `BCLIBCFFI_Error`
- C++ internal types (`BCLIBC_TRAJ_FLAG_*`, `BCLIBC_Shot`, `BCLIBC_ShotProps`, etc. in `base_types.hpp`) are **unchanged** — this rename affects only the C FFI layer (`bclibc_ffi.h`)

## [1.0.5] - 2026-05-14

### Added
- Flatpak module descriptor (`bclibc.json`) for use in Flatpak/Flathub projects
- CMake install rules (`GNUInstallDirs`) — `cmake --install` now installs headers, libraries and cmake package config to prefix
- CMake package export (`find_package(bclibc)`) via `bclibc-config.cmake` and `bclibc-targets.cmake`
- `BCLIBC_BUILD_TESTS` cmake option

### Changed
- `git describe` no longer uses `--dirty` flag (required for Flathub sandbox builds)
- Source files listed explicitly instead of `GLOB_RECURSE`
- `target_include_directories` uses generator expressions for correct install-tree paths

### CI
- Auto-update `bclibc.json` on every release tag via GitHub Actions

## [1.0.4] - 2026-04-20

### Changed
- Replaced Ukrainian source code comments with English

## [1.0.3] - 2026-04-11

### Added
- `ffi_call<F>` template replacing `BCLIBCFFI_CATCH` macro — uniform catch logic across FFI boundary including non-std exceptions

### Changed
- `try_get_exact` return type changed from `void` (exception-as-control-flow) to `bool`; call sites converted from `try/catch` to plain `if` checks
- `build_pchip_curve_from_arrays` extracted into `base_types.hpp`/`base_types.cpp` eliminating code duplication between Cython and FFI paths

### CI
- Multi-platform build support (Linux x86\_64/arm64, Windows x86\_64/arm64, macOS universal)
- Fixed release assets upload

## [1.0.2] - 2026-03-30

### CI
- Added cross-platform CI builds for Linux, Windows and macOS

## [1.0.1] - 2026-03-25

### Added
- `build_pchip_curve_from_arrays` FFI function
- `buildCurve` FFI wrapper

## [1.0.0] - 2026-03-23

### Added
- Initial release

[Unreleased]: https://github.com/ballistics-lab/bclibc/compare/v2.2.1...HEAD
[2.2.1]: https://github.com/ballistics-lab/bclibc/compare/v2.2.0...v2.2.1
[2.2.0]: https://github.com/ballistics-lab/bclibc/compare/v2.1.0...v2.2.0
[2.1.0]: https://github.com/ballistics-lab/bclibc/compare/v2.0.0...v2.1.0
[2.0.0]: https://github.com/ballistics-lab/bclibc/compare/v1.1.8...v2.0.0
[2.0.0-rc.4]: https://github.com/ballistics-lab/bclibc/compare/v2.0.0-rc.3...v2.0.0-rc.4
[2.0.0-rc.3]: https://github.com/ballistics-lab/bclibc/compare/v2.0.0-rc.2...v2.0.0-rc.3
[2.0.0-rc.2]: https://github.com/ballistics-lab/bclibc/compare/v2.0.0-rc.1...v2.0.0-rc.2
[2.0.0-rc.1]: https://github.com/ballistics-lab/bclibc/compare/v2.0.0-beta.8...v2.0.0-rc.1
[2.0.0-beta.8]: https://github.com/ballistics-lab/bclibc/compare/v2.0.0-beta.7...v2.0.0-beta.8
[2.0.0-beta.7]: https://github.com/ballistics-lab/bclibc/compare/v2.0.0-beta.6...v2.0.0-beta.7
[2.0.0-beta.6]: https://github.com/ballistics-lab/bclibc/compare/v2.0.0-beta.5...v2.0.0-beta.6
[2.0.0-beta.5]: https://github.com/ballistics-lab/bclibc/compare/v2.0.0-beta.4...v2.0.0-beta.5
[2.0.0-beta.4]: https://github.com/ballistics-lab/bclibc/compare/v2.0.0-beta.3...v2.0.0-beta.4
[2.0.0-beta.3]: https://github.com/ballistics-lab/bclibc/compare/v2.0.0-beta.2...v2.0.0-beta.3
[2.0.0-beta.2]: https://github.com/ballistics-lab/bclibc/compare/v2.0.0-beta.1...v2.0.0-beta.2
[2.0.0-beta.1]: https://github.com/ballistics-lab/bclibc/compare/v1.1.8...v2.0.0-beta.1
[1.1.8]: https://github.com/ballistics-lab/bclibc/compare/v1.1.7...v1.1.8
[1.1.7]: https://github.com/ballistics-lab/bclibc/compare/v1.1.6...v1.1.7
[1.1.6]: https://github.com/ballistics-lab/bclibc/compare/v1.1.5...v1.1.6
[1.1.5]: https://github.com/ballistics-lab/bclibc/compare/v1.1.4...v1.1.5
[1.1.4]: https://github.com/ballistics-lab/bclibc/compare/v1.1.3...v1.1.4
[1.1.3]: https://github.com/ballistics-lab/bclibc/compare/v1.1.2...v1.1.3
[1.1.2]: https://github.com/ballistics-lab/bclibc/compare/v1.1.1...v1.1.2
[1.1.1]: https://github.com/ballistics-lab/bclibc/compare/v1.1.0...v1.1.1
[1.1.0]: https://github.com/ballistics-lab/bclibc/compare/v1.0.5...v1.1.0
[1.0.5]: https://github.com/ballistics-lab/bclibc/compare/v1.0.4...v1.0.5
[1.0.4]: https://github.com/ballistics-lab/bclibc/compare/v1.0.3...v1.0.4
[1.0.3]: https://github.com/ballistics-lab/bclibc/compare/v1.0.2...v1.0.3
[1.0.2]: https://github.com/ballistics-lab/bclibc/compare/v1.0.1...v1.0.2
[1.0.1]: https://github.com/ballistics-lab/bclibc/compare/v1.0.0...v1.0.1
[1.0.0]: https://github.com/ballistics-lab/bclibc/releases/tag/v1.0.0
