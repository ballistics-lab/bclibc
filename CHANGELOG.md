# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
