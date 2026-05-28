# bclibc — C++ Ballistic Solver Engine

High-performance ballistic trajectory solver with RK4 and Euler integration, Ridder's method for zero-finding, and a stable C FFI layer for use from Dart/Flutter, Python, Rust, or any language with C bindings.

---

## Architecture

Two build artifacts:

| Artifact | Type | Purpose |
|---|---|---|
| `libbclibc_core.a` / `bclibc_core.lib` | Static | Pure C++ engine logic. Use for C++ projects and unit tests. |
| `libbclibc_ffi.so` / `libbclibc_ffi.dylib` / `bclibc_ffi.dll` | Shared | Stable C API (`BCLIBCFFI_*`). Use for FFI bindings. |

---

## Dependencies

- CMake 3.13+
- C++17 compiler: GCC, Clang, or MSVC
- Make (Linux/macOS) or Visual Studio 2022 (Windows)

---

## Building

### Linux / macOS

```bash
./build.sh
```

Or via Make:

```bash
make          # Build everything (Core + FFI)
make core     # Static core only
make ffi      # Shared FFI only
make clean    # Remove build/
```

### Windows

PowerShell:
```powershell
.\build.ps1              # Release (default)
.\build.ps1 -Configuration Debug
```

CMD:
```bat
build.bat Release
build.bat Debug
```

### Manual CMake

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Windows:
```powershell
cmake -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

---

## Output locations

| Platform | DLL/SO | Static |
|---|---|---|
| Linux | `build/libbclibc_ffi.so` | `build/libbclibc_core.a` |
| macOS | `build/libbclibc_ffi.dylib` | `build/libbclibc_core.a` |
| Windows | `build/bin/Release/bclibc_ffi.dll` | `build/lib/Release/bclibc_core.lib` |

---

## FFI API

The public C API is declared in `include/bclibc/ffi/bclibc_ffi.h`. All symbols are prefixed with `BCLIBCFFI_`.

**`BCLIBCFFI_ShotProps`-based (pre-computed physics, legacy path):**

| Function | Description |
|---|---|
| `BCLIBCFFI_get_version()` | Library version string |
| `BCLIBCFFI_find_apex()` | Highest point of trajectory |
| `BCLIBCFFI_find_max_range()` | Maximum range and angle |
| `BCLIBCFFI_find_zero_angle()` | Barrel elevation to zero at distance |
| `BCLIBCFFI_integrate()` | Full trajectory, filtered by step/flags |
| `BCLIBCFFI_integrate_at()` | Single interpolated point at key value |
| `BCLIBCFFI_free_trajectory()` | Free memory from `BCLIBCFFI_integrate` |
| `BCLIBCFFI_get_correction()` | Angular correction for offset at distance |
| `BCLIBCFFI_calculate_energy()` | Kinetic energy (ft-lb) |
| `BCLIBCFFI_calculate_ogw()` | Optimal Game Weight |

**`BCLIBCFFI_Shot`-based (natural units, preferred — all physics conversion in C++):**

| Function | Description |
|---|---|
| `BCLIBCFFI_find_apex_shot()` | Highest point of trajectory |
| `BCLIBCFFI_find_max_range_shot()` | Maximum range and angle |
| `BCLIBCFFI_find_zero_angle_shot()` | Barrel elevation to zero at distance |
| `BCLIBCFFI_integrate_shot()` | Full trajectory, filtered by step/flags |
| `BCLIBCFFI_integrate_at_shot()` | Single interpolated point at key value |

`BCLIBCFFI_Shot` accepts raw user-facing units (`temp_c`, `pressure_hpa`, `latitude_deg`, `azimuth_deg`, parallel `mach_data`/`cd_data` arrays). All atmosphere density, Coriolis trig, PCHIP drag curve, and cant pre-computation are performed inside C++ by `BCLIBC_Shot::to_shot_props()`.

**Key types:**

| Type | Description |
|---|---|
| `BCLIBCFFI_Shot` | Preferred shot input (natural units) |
| `BCLIBCFFI_ShotProps` | Legacy shot input (pre-computed physics) |
| `BCLIBCFFI_TrajectoryData` | One filtered trajectory record |
| `BCLIBCFFI_TrajectoryRequest` | Step / range / filter config for `integrate` |
| `BCLIBCFFI_Interception` | Single interpolated point from `integrate_at` |
| `BCLIBCFFI_MaxRangeResult` | Max range + angle from `find_max_range` |
| `BCLIBCFFI_Error` | Error code + message + typed extra fields |

### Symbol visibility

On Windows, the DLL exports are controlled via `__declspec(dllexport)` (defined automatically when building the library). On Linux, a version script (`src/ffi/bclibc_ffi.version`) restricts the export table to `BCLIBCFFI_*` symbols only.

Verify exports:
```bash
# Linux
nm -D build/libbclibc_ffi.so | grep " T "

# macOS
nm -g build/libbclibc_ffi.dylib | grep " T "

# Windows
dumpbin /exports build\bin\Release\bclibc_ffi.dll
```

---

## Dart / Flutter integration

1. Copy the platform library to your Flutter project's native assets folder
2. Load with `dart:ffi`
3. Generate Dart bindings automatically using [`ffigen`](https://pub.dev/packages/ffigen) with the header:
   ```yaml
   headers:
     entry-points:
       - include/bclibc/ffi/bclibc_ffi.h
   ```

---

## CI / CD

| Workflow | Trigger | Description |
|---|---|---|
| `pr-check.yml` | PR to `main`/`develop` | Builds on Linux, macOS, Windows × Debug/Release |
| `build-libs.yml` | Manual | Build specific platform and upload artifacts |
| `release.yml` | Push tag `v*` | Builds all platforms and creates GitHub Release |

---

## Pre-commit check

Runs a clean build and validates artifacts, version metadata, and symbol visibility:

```bash
chmod +x pre-commit-check.sh
./pre-commit-check.sh
```

---

## Project structure

```
.
├── include/
│   ├── bclibc.hpp
│   └── bclibc/
│       ├── base_types.hpp
│       ├── engine.hpp
│       ├── euler.hpp
│       ├── exceptions.hpp
│       ├── interp.hpp
│       ├── log.hpp
│       ├── rk4.hpp
│       ├── scope_guard.hpp
│       ├── traj_data.hpp
│       ├── traj_filter.hpp
│       ├── v3d.hpp
│       └── ffi/
│           └── bclibc_ffi.h       # Public C FFI API
│
├── src/
│   ├── base_types.cpp
│   ├── engine.cpp
│   ├── euler.cpp
│   ├── interp.cpp
│   ├── rk4.cpp
│   ├── traj_data.cpp
│   ├── traj_filter.cpp
│   └── ffi/
│       ├── bclibc_ffi.cpp
│       └── bclibc_ffi.version     # Linux symbol visibility script
│
├── .github/workflows/
│   ├── build-libs.yml
│   ├── pr-check.yml
│   └── release.yml
│
├── CMakeLists.txt
├── Makefile
├── build.sh                       # Linux/macOS build script
├── build.ps1                      # Windows PowerShell build script
├── build.bat                      # Windows CMD build script
├── clean.ps1                      # Windows clean script
├── pre-commit-check.sh
├── version.h.in
└── LICENSE
```


> [!WARNING]
>
> ## RISK NOTICE
>
> This library performs approximate simulations of complex physical processes.
> Therefore, the calculation results MUST NOT be considered as completely and reliably > reflecting actual behavior of projectiles. While these results may be used for educational purpose, they must NOT be considered as reliable for the areas where incorrect calculation may cause making a wrong decision, financial harm, or can put a human life at risk.
> 
> THE CODE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE MATERIALS OR THE USE OR OTHER DEALINGS IN THE MATERIALS.
