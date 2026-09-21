# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/ballistics-lab/bclibc/compare/v2.0.0-beta.7...HEAD
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
