# tiny_bclibc

Pure C99 ballistics engine — header-only by default, or compiled as a shared/static library.

Implements RK4 integration, PCHIP drag curves, Coriolis, spin drift, CIPM-2007 atmosphere,
and Ridder's method for zero-finding. Designed for embedded targets (MicroPython natmod,
bare-metal MCUs) as well as desktop use.

## Usage modes

| Mode | How | When |
|------|-----|-------|
| **Header-only** | `#include "tiny_bclibc/engine.h"` | app controls compilation; one TU or LTO |
| **Shared library** | `TINY_BCLIBC_BUILD_SHARED` when building, `TINY_BCLIBC_USE_SHARED` when consuming | FFI, plugins, runtime loading |
| **Static library** | `TINY_BCLIBC_BUILD_SHARED` off, link `tiny_bclibc_impl.c` | embedding without header pollution |

### Header-only

```c
#include "tiny_bclibc/engine.h"   // pulls in all headers; every function is static inline
```

All logic lives in `engine.h` (and the headers it includes). No separate compilation needed.

### Shared / static library

One translation unit does all the work:

```bash
gcc -O2 -shared -fPIC -DTINY_BCLIBC_BUILD_SHARED \
    -Iinclude src/tiny_bclibc_impl.c -o libtiny_bclibc.so
```

Consumers add `-DTINY_BCLIBC_USE_SHARED` and link against the library:

```c
#define TINY_BCLIBC_USE_SHARED
#include "tiny_bclibc/engine.h"
```

## CMake

```cmake
add_subdirectory(tiny_bclibc)

# Header-only (no compiled library needed)
target_link_libraries(my_target PRIVATE tiny_bclibc::headers)

# Or with a compiled library
target_link_libraries(my_target PRIVATE tiny_bclibc::shared)  # or tiny_bclibc::static
```

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `TINY_BCLIBC_USE_FLOAT` | `OFF` | Use `float` instead of `double` for `real_t` |
| `TINY_BCLIBC_BUILD_SHARED` | `OFF` | Build shared library |
| `TINY_BCLIBC_BUILD_STATIC` | `OFF` | Build static library |
| `TINY_BCLIBC_INSTALL` | `ON` | Install targets and headers |
| `TINY_BCLIBC_BUILD_IDENTITY_TEST` | `OFF` | Build bclibc↔tiny_bclibc identity test |

```bash
cmake -B build -DTINY_BCLIBC_BUILD_SHARED=ON -DTINY_BCLIBC_USE_FLOAT=OFF
cmake --build build
```

## Precision

`real_t` is `double` by default. Define `TINY_BCLIBC_USE_FLOAT` before including any header
(or pass `-DTINY_BCLIBC_USE_FLOAT` to the compiler) to switch to `float`.

All math macros (`TINY_BCLIBC_SIN`, `TINY_BCLIBC_SQRT`, etc.) resolve to the matching
single- or double-precision libc functions automatically.

## API

### Setup

```c
// 1. Fill TINY_BCLIBC_Shot with user-facing values
TINY_BCLIBC_Shot shot = { .bc = 0.310, .muzzle_velocity_fps = 2750.0, ... };

// 2. Build physics (PCHIP drag curve, atmosphere, Coriolis, ...)
TINY_BCLIBC_CurvePoint curve[82];   // >= shot.drag_table_size
TINY_BCLIBC_ShotProps  props;
int32_t rc = tiny_bclibc_build_shot_props(&shot, curve, &props);
```

### Trajectory functions

```c
// Full trajectory at fixed range / time steps
int32_t tiny_bclibc_integrate(
    const TINY_BCLIBC_ShotProps        *props,
    const TINY_BCLIBC_TrajectoryRequest *req,
    TINY_BCLIBC_TrajectoryData          *buf,
    int32_t                              capacity,
    int32_t                             *out_written,
    int32_t                             *out_total,
    int32_t                             *out_reason);

// Single interpolated point
int32_t tiny_bclibc_integrate_at(
    const TINY_BCLIBC_ShotProps *props,
    int32_t                      key,          // TINY_BCLIBC_KEY_*
    real_t                       target_value,
    TINY_BCLIBC_BaseTrajData    *out_raw,
    TINY_BCLIBC_TrajectoryData  *out_full);    // may be NULL

// Barrel elevation to zero at distance
int32_t tiny_bclibc_find_zero_angle(
    const TINY_BCLIBC_ShotProps *props,
    real_t                       distance_ft,
    real_t                      *out_angle_rad);

// Highest point of trajectory
int32_t tiny_bclibc_find_apex(
    const TINY_BCLIBC_ShotProps *props,
    TINY_BCLIBC_TrajectoryData  *out);

// Maximum range (golden-section search over elevation)
int32_t tiny_bclibc_find_max_range(
    const TINY_BCLIBC_ShotProps *props,
    real_t low_deg, real_t high_deg,
    real_t *out_range_ft, real_t *out_angle_rad);

// Last error string (thread-local; "" if no error)
const char *tiny_bclibc_last_error(void);
```

### Inline helpers

```c
real_t tiny_bclibc_get_correction(real_t distance_ft, real_t offset_ft);   // → angle (rad)
real_t tiny_bclibc_calculate_energy(real_t weight_grain, real_t vel_fps);  // → ft·lbf
real_t tiny_bclibc_calculate_ogw(real_t weight_grain, real_t vel_fps);     // → lb
```

### Key types

| Type | Description |
|------|-------------|
| `TINY_BCLIBC_Shot` | User-facing shot descriptor (natural units) |
| `TINY_BCLIBC_Config` | Integration tuning (step multiplier, limits) |
| `TINY_BCLIBC_Wind` | Single wind layer (`velocity_fps`, `direction_from_rad`, `until_distance_ft`) |
| `TINY_BCLIBC_CurvePoint` | PCHIP segment (a, b, c, d coefficients) |
| `TINY_BCLIBC_ShotProps` | Pre-computed physics (output of `build_shot_props`) |
| `TINY_BCLIBC_Atmosphere` | CIPM-2007 atmosphere state |
| `TINY_BCLIBC_Coriolis` | Coriolis pre-computed sines/cosines |
| `TINY_BCLIBC_TrajectoryRequest` | Step / range / filter config |
| `TINY_BCLIBC_TrajectoryData` | Full output row (16 fields) |
| `TINY_BCLIBC_BaseTrajData` | Raw RK4 state (time, position, velocity, mach) |

### Interpolation keys (`TINY_BCLIBC_InterpKey`)

`TINY_BCLIBC_KEY_TIME`, `TINY_BCLIBC_KEY_MACH`, `TINY_BCLIBC_KEY_POS_X/Y/Z`,
`TINY_BCLIBC_KEY_VEL_X/Y/Z`

### Trajectory filter flags (`TINY_BCLIBC_TrajFlag`)

`TRAJ_FLAG_RANGE` (8), `TRAJ_FLAG_APEX` (16), `TRAJ_FLAG_ZERO` (3),
`TRAJ_FLAG_MACH` (4), `TRAJ_FLAG_MRT` (32), `TRAJ_FLAG_ALL` (31)

### Termination reasons (`TINY_BCLIBC_TerminationReason`)

| Value | Reason |
|-------|--------|
| 0 | No termination (integration still running) |
| 1 | Target range reached |
| 2 | Minimum velocity reached |
| 3 | Maximum drop reached |
| 4 | Minimum altitude reached |
| 5 | Callback requested stop |

## Compile-time macros

| Macro | Effect |
|-------|--------|
| `TINY_BCLIBC_USE_FLOAT` | `real_t = float`; all math uses `*f` variants |
| `TINY_BCLIBC_BUILD_SHARED` | Emit exported symbols (building `.so`/`.dll`) |
| `TINY_BCLIBC_USE_SHARED` | Import symbols (consuming `.so`/`.dll`) |
| `TINY_BCLIBC_NO_THREAD_LOCAL` | Disable TLS error buffer (bare-metal, RTOS) |
| `TINY_BCLIBC_NO_ERR_BUF` | No error string at all (natmod / BSS=0 targets) |

## Error handling

All public functions return `int32_t` (`TINY_BCLIBC_OK = 0` on success).
On failure, `tiny_bclibc_last_error()` returns a thread-local string describing the problem.
When `TINY_BCLIBC_NO_ERR_BUF` is defined (natmod builds), `last_error()` always returns
a generic string — use the return code instead.

## Project structure

```
tiny_bclibc/
├── include/tiny_bclibc/
│   ├── platform.h      # real_t, macros, visibility
│   ├── v3d.h           # 3-component vector helpers
│   ├── interp.h        # 3-point PCHIP interpolation
│   ├── base_types.h    # Shot, ShotProps, Wind, Config, Atmosphere, Coriolis
│   ├── traj_data.h     # TrajectoryData, BaseTrajData, TrajectoryRequest
│   └── engine.h        # Public API + RK4 implementation
├── src/
│   └── tiny_bclibc_impl.c   # Single-file library entry point
├── tests/
│   └── test_identity.cpp    # bclibc↔tiny_bclibc result comparison
├── CMakeLists.txt
└── version.h.in
```
