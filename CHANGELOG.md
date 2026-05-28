# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/ballistics-lab/bclibc/compare/v1.0.5...HEAD
[1.0.5]: https://github.com/ballistics-lab/bclibc/compare/v1.0.4...v1.0.5
[1.0.4]: https://github.com/ballistics-lab/bclibc/compare/v1.0.3...v1.0.4
[1.0.3]: https://github.com/ballistics-lab/bclibc/compare/v1.0.2...v1.0.3
[1.0.2]: https://github.com/ballistics-lab/bclibc/compare/v1.0.1...v1.0.2
[1.0.1]: https://github.com/ballistics-lab/bclibc/compare/v1.0.0...v1.0.1
[1.0.0]: https://github.com/ballistics-lab/bclibc/releases/tag/v1.0.0
