# bclibc — Ballistic Solver Engine

High-performance ballistic trajectory solver with RK4, Euler, Velocity Verlet, and adaptive Cash-Karp / Dormand-Prince / Tsitouras RK45 integration, Ridder's method for zero-finding, PCHIP drag curves, Coriolis, and spin drift.

`BCLIBC_integrateDormandPrince` and `BCLIBC_integrateTsitouras` provide two
structurally-identical 7-stage FSAL 5(4) pairs with SciPy RK45-style component
scaling and adaptive controller behavior. Cash--Karp keeps its own
compatibility controller. All three use the compile-time embedded-RK core.

[![Release][release badge]][release]
[![Codecov][codecov badge]][codecov]

![Linux] ![Windows] ![macOS] ![WebAssembly]

---

## Part of the Ballistics Lab ecosystem

`bclibc` is the shared C++/C99 physics core behind the [**Ballistics Lab**][ballistics-lab] ecosystem —
one engine, bound natively into [py-ballisticcalc][py-ballisticcalc] (Python/Cython),
[js-ballistics][js-ballistics] (TypeScript/WASM via Embind), [dart-bclibc][dart-bclibc]
(Dart FFI natively, and WASM on web via `build_wasm.sh` — see [WASM build](#wasm-build)), and
[micropython-bclibc][micropython-bclibc] (MCUs via the bundled `tiny_bclibc` C99 subset).

---

## Coverage

`bclibc` has no test suite or coverage tooling of its own. Its C++ engine is exercised and
correctness-tested through the Cython wrapper in [`py_ballisticcalc.exts`][py_ballisticcalc.exts],
part of the [py-ballisticcalc][py-ballisticcalc] Python library — its pytest suite
(`euler` / `rk4` / `rk45`) drives every `bclibc` and `tiny_bclibc` code path
end-to-end via the Cython bindings. The badge above reflects that project's overall
coverage; line-level coverage of the compiled `.pyx`/C++ layer itself is not tracked separately.

---

## Repository structure

| Directory | Description |
|-----------|-------------|
| `src/` / `include/` | C++ engine + `libbclibc_ffi` — Dart/Flutter, Python, Rust FFI |
| [`tiny_bclibc/`](tiny_bclibc/README.md) | Pure C99 engine — header-only, embeddable on MCUs |
| **[micropython-bclibc](https://github.com/ballistics-lab/micropython-bclibc)** | MicroPython module — natmod (`.mpy`), usermod (baked-in firmware), FFI |

---

## tiny_bclibc — C99 engine *(experimental)*

> [!WARNING]
> `tiny_bclibc` is an **experimental** feature. The API, CMake interface, and binary layout
> may change without notice. Validate thoroughly before using in production.

[`tiny_bclibc/`](tiny_bclibc/README.md) is a pure C99 reimplementation of the ballistic engine.
Header-only by default (`static inline`); can also be compiled as a shared or static library
from a single TU (`src/tiny_bclibc_impl.c`).

**Features:** Tsitouras 5(4) adaptive RK45 (`tiny_bclibc_integrate`/`_stream`/`_raw`), fixed-step RK4
(used internally for zero-angle/apex finding), PCHIP drag, CIPM-2007 atmosphere, Coriolis, spin
drift, Ridder zero-finding, `float` or `double` precision, bare-metal / RTOS compatible (no TLS,
no heap required).

```cmake
add_subdirectory(tiny_bclibc)
target_link_libraries(my_target PRIVATE tiny_bclibc::headers)
```

See [tiny_bclibc/README.md](tiny_bclibc/README.md) for full API and CMake options.

---

## MicroPython module *(experimental)*

> [!WARNING]
> The MicroPython module is an **experimental** feature. Build system, binary format,
> and Python API may change without notice in future releases.

The MicroPython module is maintained in a separate repository:
**[github.com/ballistics-lab/micropython-bclibc](https://github.com/ballistics-lab/micropython-bclibc)**

Three integration modes are available:

| Mode | When to use |
|------|-------------|
| **natmod** (`.mpy`) | Deploy `.mpy` to device filesystem; works on any `mpy_ld.py`-supported arch |
| **usermod** (baked-in) | Own the firmware build; module is always available as a built-in |
| **FFI** (`libtiny_bclibc.so`) | Any unix port arch without native module support (aarch64, mipsel, …) |

See the [micropython-bclibc README](https://github.com/ballistics-lab/micropython-bclibc) for build instructions, target matrix, API reference, and CI setup.

---

## C++ engine / libbclibc_ffi

The `src/` / `include/` tree contains the original C++ engine with a stable C FFI layer
(`libbclibc_ffi.so` / `.dll`) for use from Dart/Flutter, Python, Rust, and any language
with C bindings.

---

## Adaptive integration

Three embedded RK45 methods share one compile-time core (`bclibc/embedded_rk45.hpp`,
`embedded_rk45_detail::run<Tableau, Controller>`), each selectable via
`BCLIBC_BaseEngine::integrate_func` like any other integrator: **Cash-Karp**, **Dormand-Prince**,
and **Tsitouras**. All three retry a rejected attempt rather than accepting it, and grow/shrink
`dt` between `base_step/64` and `base_step*64`.

Each method also has a stateful integrator class (`BCLIBC_CashKarpIntegrator`,
`BCLIBC_DormandPrinceIntegrator`, `BCLIBC_TsitourasIntegrator`) that owns its own tolerances and
accepted/rejected step counts *per instance* — assign one (wrapped in `std::ref` if you want to
read its stats back afterward) to a specific engine's `integrate_func` instead of calling a
free function that mutated shared thread-local state, which meant every `BCLIBC_BaseEngine` on a
thread using the same method shared one tolerance/stats, even engines integrating concurrently
with intentionally different settings. The plain free functions
(`BCLIBC_integrateCashKarp` and friends) remain for the common case of just wanting that method
at its default `1e-6`/`1e-6` tolerances with no need to read back stats. Each class also takes
its tolerances directly in its constructor (e.g. `BCLIBC_CashKarpIntegrator(1e-8)`), and stores
tolerances/stats in `std::atomic`s, so one instance can safely be shared (via `std::ref`) across
threads — `set_relative_tolerance()`/`set_absolute_tolerance()` from one thread cannot race with
a concurrent `operator()` or `get_stats()` call from another (verified under ThreadSanitizer).

Unlike `BCLIBC_integrateRK4`, which freezes the drag coefficient once per step, every adaptive
method recomputes both the drag coefficient *and* the atmosphere sample fresh at each stage: an
earlier variant that reused RK4's once-per-step freeze produced real, tolerance-independent
accuracy failures, because the embedded error estimator is blind to model error from a stale
drag/atmosphere sample once the step grows tens of times past the fixed-step size.

Adaptive steps are also far sparser and less uniformly spaced than fixed-step RK4's output,
which broke the original per-raw-point, 3-point finite-difference (PCHIP) event/row
interpolation: it estimates slopes from neighboring raw-point spacing, which is accurate when
points are dense and uniform but measurably wrong once they are not. The fix, now used by every
integrator (`BCLIBC_TrajectoryDataFilter::handle_step` in `src/traj_filter.cpp`): integrators
stream each accepted `(start, end)` interval to the handler via `handle_step`, and RANGE/time-step
rows and APEX/MACH/ZERO event roots are reconstructed with a 2-point cubic Hermite built from
each interval's *exact* endpoint positions and velocities (not a finite-difference estimate),
solved by bisection where needed. Scheduled samples and physical events are kept as independent
records rather than merged when their timestamps happen to land close together — merging them
depended on raw-sample spacing that adaptive stepping no longer guarantees.

### Cash-Karp

`BCLIBC_integrateCashKarp` (`bclibc/cash_karp.hpp`) is Numerical Recipes' `rkck`, a 6-stage,
non-FSAL pair with its own asymmetric grow/shrink controller (safety `0.9`, growth capped at
`5x`, shrink exponent `-0.25` vs growth exponent `-0.20`) rather than the SciPy-style one below —
preserved exactly from the pre-refactor standalone implementation. It never limits its step at a
wind-zone boundary (historical behavior, kept for output-compatibility).
`BCLIBC_CashKarpIntegrator::set_relative_tolerance()` / `set_absolute_tolerance()` each default to
`1e-6`; the scalar `atol` and `rtol` scale every one of the three position and three velocity
components as `atol + rtol * max(abs(y), abs(y_new))`, and their errors use an RMS norm —
typically 2-6x fewer total steps than fixed-step RK4 for the same accuracy.

### Dormand-Prince

`BCLIBC_integrateDormandPrince` (`bclibc/dormand_prince.hpp`) is DOPRI5 — the same tableau
`scipy.integrate`'s `RK45` uses — a 7-stage FSAL (First-Same-As-Last) pair: its 7th stage's A-row
equals the 5th-order solution weights, so it doubles as the accepted state *and* seeds the next
step's first stage, both saving a derivative evaluation and improving accuracy at wind-zone
transitions over a non-FSAL method. Its `ScipyRKController` matches
`scipy.integrate._ivp.rk.RungeKutta`'s own controller exactly: exponent `-1/5`, safety `0.9`,
factor clamped to `[0.2, 10]`, growth capped at `1x` for one step immediately following a
rejection — and it limits `dt` so a step never overshoots the next wind-zone boundary (needed so
every stage's wind sample stays valid for a step that starts before and would otherwise end past
a wind-zone change). Same tolerance API shape as Cash-Karp:
`BCLIBC_DormandPrinceIntegrator::set_relative_tolerance()` / `set_absolute_tolerance()`,
default `1e-6` each.

### Tsitouras

`BCLIBC_integrateTsitouras` (`bclibc/tsitouras.hpp`) is Tsit5 (Tsitouras, 2011 — "Runge-Kutta
pairs of order 5(4) satisfying only the first column simplifying assumption", *Computers &
Mathematics with Applications* 62(2), 770-775; coefficients verified against
`ARKODE_TSITOURAS_7_4_5` in SUNDIALS/ARKODE). It is Dormand-Prince's structural twin — same
7-stage FSAL shape, same `ScipyRKController` (see above), same `1e-6` default tolerances via
`BCLIBC_TsitourasIntegrator::set_relative_tolerance()` / `set_absolute_tolerance()` — but with
smaller leading error-term coefficients at each order, and is `tiny_bclibc`'s current default
adaptive core (see [tiny_bclibc's README](tiny_bclibc/README.md#adaptive-integration-tsitouras)).

**Measured, not assumed:** across a small sweep of shot profiles at `rtol=atol=1e-6`, Tsitouras'
accepted+rejected step count comes out within 1-2 steps of Cash-Karp's and Dormand-Prince's on
smooth, well-conditioned ballistic trajectories — i.e. no consistent step-count or wall-clock win
over the other two for *this* problem class, despite the smaller leading error term. It was added
as a well-regarded, actively-used modern default elsewhere (e.g. `Tsit5` in Julia's
OrdinaryDiffEq.jl/SciML) and a structurally-compatible Dormand-Prince alternative, not because it
measurably outperforms the existing methods here — don't assume it will win on your own workload
either without measuring `get_step_stats()` (or your language binding's equivalent) yourself.

See `CHANGELOG.md` for the specific fixes (per-stage recompute, streaming handler contract,
exact-derivative Hermite reconstruction).

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
make wasm WASI_SDK_PATH=...   # Bare WebAssembly module with exceptions (see WASM build)
make wasm-zig                 # ... or the small one with zig, where a throw is a trap
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

## WASM build

`build_wasm.sh` compiles the same `BCLIBCFFI_*` C ABI (`bclibc_ffi.cpp`) to WebAssembly via Emscripten, for platforms without `dart:ffi` (e.g. Flutter/Dart web). It self-installs a pinned Emscripten SDK into `tool/emsdk` on first run if `emcc` isn't already on `PATH`:

```bash
./build_wasm.sh
```

Output: `build/web/bclibc_ffi.js` + `build/web/bclibc_ffi.wasm` (Emscripten JS-glue module, `MODULARIZE=1`). Ship both files together — do **not** re-add `-sSINGLE_FILE=1`: as of emsdk 6.0.3 it produces a wasm blob Chrome's `WebAssembly.instantiate` rejects (`invalid value type 0x1`), even though the identical bytes load fine under Node. The two-file layout is the verified-working one.

The module exports the flat `BCLIBCFFI_*` functions directly (no Embind) plus `BCLIBCFFI_get_layout()`, which returns every `BCLIBCFFI_Shot`-family struct's field byte offsets/sizes, computed via `offsetof()`/`sizeof()` by whichever compiler built the module. Callers marshal structs into wasm linear memory (`_malloc`/`HEAPU8`) using those offsets instead of hardcoding them — see `dart-bclibc`'s `lib/ffi/bclibc_ffi_web.dart` for a complete `dart:js_interop` binding built this way.

### Bare WebAssembly build (no Emscripten, no imports)

The same C ABI as one module that imports **nothing**, built with CMake and one of two toolchains, neither of them
Emscripten:

| toolchain | `-DCMAKE_TOOLCHAIN_FILE=` | C++ exceptions | size |
|---|---|---|---|
| [wasi-sdk](https://github.com/WebAssembly/wasi-sdk/releases) (34 tested; `-DWASI_SDK_PATH=` or `$WASI_SDK_PATH`) | `cmake/wasi-sdk-wasm32.cmake` | **yes**: the core throws and the flat C ABI returns the same `BCLIBCFFI_ERR_*` codes as the native library | ~1.6 MB |
| [zig](https://ziglang.org) (`zig` on `PATH`, `-DZIG=`, or `pip install ziglang`) | `cmake/zig-wasm32-wasi.cmake` | no: a `throw` is a trap | ~78 KB (`-Oz -flto`) |

```bash
cmake -S . -B build/wasm -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/wasi-sdk-wasm32.cmake -DWASI_SDK_PATH=/opt/wasi-sdk-34.0 -DBCLIBC_WASM_BARE=ON
cmake --build build/wasm          # -> build/wasm/bclibc_wasm.wasm; the build fails if it imports from WASI
```

Or `make wasm WASI_SDK_PATH=/opt/wasi-sdk-34.0` (wasi-sdk, `build/wasm/`) and `make wasm-zig` (zig, `build/wasm-zig/`).

`tests/wasm_parity/parity.py` runs the same shots through the native library and the module (wasmhost, on wasmtime and Node) and
fails on any difference beyond 1 ulp in the angle fields, or on a failed solve that does not return the native status
(wasi-sdk) or trap (zig); the `WASM (bare module)` workflow builds both flavours and runs it.

It runs with an empty import object in any host that has WebAssembly: Node, browsers, JavaScriptCore, wasmtime, wasm3,
or Python through [wasmhost](https://github.com/ballistics-lab/py-wasmhost). What a host has to know, since there is
no Emscripten glue to do it:

- Call `_initialize()` once before the first call (it is a reactor: static initializers).
- The memory is the module's own (exported as `memory`, 17 pages to start, grows on its own up to 2 GiB,
  `BCLIBC_WASM_MAX_MEMORY`; the shadow stack is 1 MiB, `BCLIBC_WASM_STACK_SIZE`). Nothing is passed in. After any call
  that may allocate, take `memory.buffer` again: a grown memory detaches the old `ArrayBuffer`.
- `malloc` and `free` are exported: put arguments into the module's memory and free what a call hands back
  (`BCLIBCFFI_free_trajectory` for the records).
- Pointers and `size_t` are 4 bytes, so read struct fields at the offsets `BCLIBCFFI_get_layout()` reports.
- **wasi-sdk build: exceptions need the host to have WebAssembly's final exception encoding (`try_table`).** wasi-sdk's
  libraries use only that one (a module mixing it with the older `try`/`catch` is invalid, so the code is built with
  `-mllvm -wasm-use-legacy-eh=false`). Measured: wasmtime 49, wasm3 (git, 2026), Node 25 and JavaScriptCore (WebKitGTK)
  run it; hosts older than the encoding (older Node, Safari/iOS before it) do not, and `wasmhost.selftest` can tell.
  The build is **not** link-time optimized on purpose: that option does not reach the code generator of an LTO build, and
  the module then compiles but never catches (the exception escapes the call).
- **zig build: a `throw` is a trap** (no exception runtime): a solve that fails (`ZeroFinding`, `OutOfRange`, ...) ends
  the call with `unreachable` instead of returning a `BCLIBCFFI_ERR_*` code, and a trap leaves the shadow stack where it
  was, so **make a new instance after any trap**.
- `src/wasm/bare_runtime.cpp` is what keeps WASI out: libc++'s abort, and for wasi-sdk a set of inert stand-ins for the
  parts of wasi-libc that libunwind and libc++abi call (stderr, the environment, locks, the clock, the stack protector
  seed). A newer wasi-sdk may find another way in; the post-build check says so.

Numerically it matches the native library: the same inputs through `libbclibc_ffi.so` (x86-64, glibc) and the
module (both builds), on wasmtime, wasm3, JavaScriptCore and Node, for the six integration methods, `find_zero_*`, `find_apex`,
`find_max_range` and `integrate_at`, with sea-level, high-altitude and vacuum atmospheres, with and without Coriolis, cant
and look angle: 2098 values compared, **all bit-identical except 71 that differ by 1 ulp**, and only in
`drop_angle_rad`, `windage_angle_rad` and `angle_rad`, the ones computed with `atan`/`atan2` (libm differs between
glibc and the module's musl; `+ - * / sqrt` are exact everywhere). 1 ulp is the spacing between two neighbouring
`double`s, about 2.2e-16 relative: for a 0.01 rad angle 1.7e-18 rad, some 10^12 times finer than the solver's own
zero-finding accuracy. So compare results across platforms with a tolerance (say relative 1e-12), not with `==`. The
engines agree with each other exactly; on arm64 (FMA) it has not been measured.

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
│       ├── cash_karp.hpp
│       ├── dormand_prince.hpp
│       ├── embedded_rk45.hpp
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
│       ├── velocity_verlet.hpp
│       └── ffi/
│           └── bclibc_ffi.h       # Public C FFI API
│
├── src/
│   ├── base_types.cpp
│   ├── cash_karp.cpp
│   ├── dormand_prince.cpp
│   ├── engine.cpp
│   ├── euler.cpp
│   ├── interp.cpp
│   ├── rk4.cpp
│   ├── traj_data.cpp
│   ├── traj_filter.cpp
│   ├── velocity_verlet.cpp
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


<!-- REUSABLE LINKS -->

[release badge]: https://img.shields.io/github/v/release/ballistics-lab/bclibc?labelColor=%23181717&logo=github&logoColor=white
[release]: https://github.com/ballistics-lab/bclibc/releases/latest

[codecov badge]: https://codecov.io/gh/o-murphy/py-ballisticcalc/graph/badge.svg
[codecov]: https://codecov.io/gh/o-murphy/py-ballisticcalc

[Linux]: https://img.shields.io/badge/Linux-x86__64%20%7C%20arm64-grey?logo=linux&logoColor=black&labelColor=FCC624
[Windows]: https://img.shields.io/badge/x86__64%20%7C%20arm64-grey?logo=windows&logoColor=black&label=Windows&labelColor=0078D4
[macOS]: https://img.shields.io/badge/macOS-arm64%20%7C%20x86__64-grey?logo=apple&logoColor=white&labelColor=000000
[WebAssembly]: https://img.shields.io/badge/WebAssembly-grey?logo=webassembly&logoColor=white&labelColor=654FF0

[ballistics-lab]: https://github.com/ballistics-lab
[py-ballisticcalc]: https://github.com/o-murphy/py-ballisticcalc
[py_ballisticcalc.exts]: https://github.com/o-murphy/py-ballisticcalc/tree/master/py_ballisticcalc.exts
[js-ballistics]: https://github.com/ballistics-lab/js-ballistics
[dart-bclibc]: https://github.com/ballistics-lab/dart-bclibc
[micropython-bclibc]: https://github.com/ballistics-lab/micropython-bclibc
