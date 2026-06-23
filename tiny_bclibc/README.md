# tiny_bclibc *(experimental)*

> [!WARNING]
> `tiny_bclibc` is an **experimental** feature. The API, CMake interface, and binary layout
> may change without notice until stabilised. Validate thoroughly before production use.

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
| `TINY_BCLIBC_SINGLE_PRECISION` | `OFF` | Use `float` instead of `double` for `real_t` |
| `TINY_BCLIBC_USE_FLOAT` | `OFF` | Deprecated alias for `TINY_BCLIBC_SINGLE_PRECISION` |
| `TINY_BCLIBC_BUILD_SHARED` | `OFF` | Build shared library |
| `TINY_BCLIBC_BUILD_STATIC` | `OFF` | Build static library |
| `TINY_BCLIBC_INSTALL` | `ON` | Install targets and headers |
| `TINY_BCLIBC_BUILD_IDENTITY_TEST` | `OFF` | Build bclibc↔tiny_bclibc identity test |

### Performance tuning macros

These are compiler defines, not CMake options — pass via `-D` flag or CFLAGS:

| Define | Description |
|--------|-------------|
| `TINY_BCLIBC_FAST_ZERO_FIND` | Reduce `find_zero_angle` computation on resource-constrained targets. Uses an 8× coarser RK4 step during the GSS bracket search and relaxed convergence tolerances (`h < 1e-2 rad`). Ridder's height-error tolerance is relaxed to `0.01 ft` (vs `0.001 ft`); angle-bracket convergence always uses `1e-5 rad`. The final angle is computed by Ridder's at full `calc_step`; output accuracy is unchanged. Enabled automatically by the natmod Makefile when `PRECISION=single`. |

```bash
cmake -B build -DTINY_BCLIBC_BUILD_SHARED=ON -DTINY_BCLIBC_SINGLE_PRECISION=OFF
cmake --build build
```

## Precision

`real_t` is `double` by default. Define `TINY_BCLIBC_SINGLE_PRECISION` before including any header
(or pass `-DTINY_BCLIBC_SINGLE_PRECISION` to the compiler) to switch to `float`.
`TINY_BCLIBC_VERSION_FULL` contains the version string with precision suffix: `"1.1.3-dp"` or `"1.1.3-sp"`.

All math macros (`TINY_BCLIBC_SIN`, `TINY_BCLIBC_SQRT`, etc.) resolve to the matching
single- or double-precision libc functions automatically.

### Float32 vs Float64 accumulated deviation

Measured via the `micropython-natmod` comparison tool (see
[`micropython-natmod/precision_compare.py`](../micropython-natmod/precision_compare.py)).

**Test conditions:**
- Shot: G7, BC=0.310, 168 gr, dia=0.308", mv=2750 fps, sight=0.125 ft (1.5"), twist=11"
- Atmosphere: T=15°C, P=1013.25 hPa, RH=0.5, alt=0 ft
- Range: 0–3000 m, output step=25 m (120 sample points)
- Internal RK4 step: controlled by `step_multiplier=0.5` (independent of output step)
- Host: MicroPython v1.26 unix port, x64, Python float=64-bit

| Metric | Max deviation | At distance |
|--------|--------------|-------------|
| Vertical drop (height) | **0.108 cm** | 2975 m |
| Velocity | **0.0015 fps** (0.0005 m/s) | — |
| Mach number | **1.32 × 10⁻⁶** | — |
| `find_zero_angle` (300 m zero) | **5 × 10⁻¹⁰ rad** (<0.001 mrad) | — |

**Conclusion:** float32 accumulates ≈ 0.1 cm of drop error at 3000 m relative to float64.
This is negligible compared to any practical uncertainty source (wind, BC variance, MV
spread) and float32 is sufficient for all supported embedded targets.

## Usage

### 1. Basic trajectory

```c
#include "tiny_bclibc/engine.h"

// Fill user-facing shot (natural units)
TINY_BCLIBC_Shot shot = {0};
shot.bc                  = 0.310;
shot.drag_type           = TINY_BCLIBC_DRAG_G7;
shot.muzzle_velocity_fps = 2750.0;
shot.weight_grain        = 168.0;
shot.diameter_inch       = 0.308;
shot.twist_inch          = 11.0;
shot.sight_height_ft     = 0.125;   // 1.5 inch
shot.barrel_elevation_rad = 0.0;    // set after find_zero_angle
// atmosphere: leave zeroed for ICAO standard

// Build physics (PCHIP drag curve + atmosphere + Coriolis)
TINY_BCLIBC_CurvePoint curve[82];
TINY_BCLIBC_ShotProps  props;
if (tiny_bclibc_build_shot_props(&shot, curve, &props) != TINY_BCLIBC_OK) {
    fprintf(stderr, "%s\n", tiny_bclibc_last_error());
    return 1;
}

// Integrate to 1000 m in 100 m steps
TINY_BCLIBC_TrajectoryRequest req = {0};
req.range_limit_ft = 3280.84;  // 1000 m
req.range_step_ft  = 328.084;  // 100 m
req.filter_flags   = TINY_BCLIBC_TRAJ_FLAG_RANGE;

TINY_BCLIBC_TrajectoryData rows[32];
int32_t written, total, reason;
tiny_bclibc_integrate(&props, &req, rows, 32, &written, &total, &reason);

for (int i = 0; i < written; i++)
    printf("%.0f ft  %.1f fps  %.3f ft\n",
           rows[i].distance_ft, rows[i].velocity_fps, rows[i].height_ft);
```

### 2. Zero + corrections

`find_zero_angle` returns the barrel elevation in radians. You must store it in
`shot.barrel_elevation_rad` (or `props.barrel_elevation`) before calling `integrate`.
Without this step the shot flies with 0° elevation.

```c
// Step 1 — find zero angle at 100 m (328.084 ft)
real_t zero_angle_rad = 0.0;
if (tiny_bclibc_find_zero_angle(&props, 328.084, &zero_angle_rad) != TINY_BCLIBC_OK) {
    fprintf(stderr, "%s\n", tiny_bclibc_last_error());
    return 1;
}

// Step 2 — store in shot and rebuild props (or set directly on props)
shot.barrel_elevation_rad = zero_angle_rad;
tiny_bclibc_build_shot_props(&shot, curve, &props);

// Step 3 — integrate to target distance
req.range_limit_ft = 1640.42;   // 500 m
tiny_bclibc_integrate(&props, &req, rows, 32, &written, &total, &reason);

// Step 4 — read correction at target row
// slant_height_ft: height above/below the look-angle line
// drop_angle_rad:  trajectory angle minus look_angle — the elevation correction to dial
TINY_BCLIBC_TrajectoryData *last = &rows[written - 1];
double corr_mrad = -last->drop_angle_rad * 1000.0;   // positive → aim higher
double wind_mrad = -last->windage_angle_rad * 1000.0; // positive → aim right
printf("Elevation: %.2f mrad  Windage: %.2f mrad\n", corr_mrad, wind_mrad);
```

`drop_angle_rad` (= trajectory angle − look angle) is the ready-to-use angular correction:
negate it to get the hold or dial value. The same field is available as `T_DROP_ANGLE` in the natmod.

### 3. look_angle + hold (uphill / different distance)

`barrel_elevation_rad` is the total absolute angle from horizontal:

```c
// zero_angle_rad was computed for 100 m, flat
double look_angle_rad  = 0.26;   // target is 15° uphill
double hold_rad        = 0.003;  // additional correction for 500 m
shot.look_angle_rad       = look_angle_rad;
shot.barrel_elevation_rad = look_angle_rad + zero_angle_rad + hold_rad;
tiny_bclibc_build_shot_props(&shot, curve, &props);
```

`zero_angle_rad` is the *relative* offset computed at zero — you add it to whatever
look angle is current. This mirrors the `look_angle + zero_elevation + relative_angle`
decomposition used in higher-level wrappers.

### 4. Single interpolated point (`integrate_at`)

Cheaper than a full trajectory when only one distance matters:

```c
TINY_BCLIBC_TrajectoryData point;
tiny_bclibc_integrate_at(&props, TINY_BCLIBC_KEY_POS_X, 1640.42, NULL, &point);
printf("At 500 m: %.1f fps  drop=%.3f mrad\n",
       point.velocity_fps, -point.drop_angle_rad * 1000.0);
```

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
| `TINY_BCLIBC_SINGLE_PRECISION` | `real_t = float`; all math uses `*f` variants |
| `TINY_BCLIBC_USE_FLOAT` | Deprecated alias for `TINY_BCLIBC_SINGLE_PRECISION` |
| `TINY_BCLIBC_BUILD_SHARED` | Emit exported symbols (building `.so`/`.dll`) |
| `TINY_BCLIBC_USE_SHARED` | Import symbols (consuming `.so`/`.dll`) |
| `TINY_BCLIBC_NO_THREAD_LOCAL` | Disable TLS error buffer (bare-metal, RTOS) |
| `TINY_BCLIBC_NO_ERR_BUF` | No error string at all (natmod / BSS=0 targets) |

## Error handling

All public functions return `int32_t` (`TINY_BCLIBC_OK = 0` on success).
On failure, `tiny_bclibc_last_error()` returns a thread-local string describing the problem.
When `TINY_BCLIBC_NO_ERR_BUF` is defined (natmod builds), `last_error()` always returns
a generic string — use the return code instead.

## MicroPython integration

The `micropython-natmod/` directory wraps `tiny_bclibc` as a native `.mpy` module
for embedded MicroPython targets. For unix MicroPython on architectures without native
module support (aarch64, mipsel, …), `examples/tiny_bclibc_mp_ffi/tiny_bclibc_mp_ffi.py`
exposes the same Python API by calling `libtiny_bclibc.so` via the built-in `ffi` module.

See [micropython-natmod/README.md](../micropython-natmod/README.md) for build instructions
and a full API reference.

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
