# bclibc for MicroPython and CircuitPython

Two integration paths are provided:

| Path | When to use |
|------|-------------|
| **Native module (`.mpy`)** | Load on any MicroPython port that supports native modules; no firmware rebuild needed |
| **External C module** | Compile directly into the MicroPython or CircuitPython firmware image |

---

## Prerequisites

- MicroPython source tree (clone from https://github.com/micropython/micropython)
- Cross-compiler for your target architecture:
  - ARM: `arm-none-eabi-gcc` / `arm-none-eabi-g++`
  - ESP32: `xtensa-esp32-elf-gcc` (from ESP-IDF)
  - ESP32-C3: `riscv32-esp-elf-gcc` (from ESP-IDF)
- Python 3 (for `mpy_ld.py`, called by the Makefile)

---

## Path 1 – Native module (.mpy)

The `.mpy` file is a self-contained native module.  Copy it to the board (e.g. via `mpremote`) and `import bclibc` like any Python module.

### Build

```sh
cd micropython/natmod

# STM32 / Cortex-M4 (e.g. STM32F4xx)
make ARCH=armv7emsp MPY_DIR=/path/to/micropython

# STM32 / Cortex-M7 with double-precision FPU (e.g. STM32H7xx)
make ARCH=armv7emdp MPY_DIR=/path/to/micropython

# Raspberry Pi Pico (RP2040 – Cortex-M0+, software double)
make ARCH=armv6m MPY_DIR=/path/to/micropython

# ESP32
make ARCH=xtensawin MPY_DIR=/path/to/micropython

# ESP32-C3
make ARCH=rv32imc MPY_DIR=/path/to/micropython
```

The output is `build-<ARCH>/bclibc.mpy` (also copied to `bclibc.mpy` in the current directory by dynruntime.mk).

### Deploy

```sh
mpremote cp bclibc.mpy :bclibc.mpy
```

### Notes on floating-point precision

`bclibc` uses `double` (64-bit) throughout its C++ core.  On Cortex-M0+ (`armv6m`) and Cortex-M4 (`armv7emsp`) targets the hardware FPU only covers 32-bit `float`; `double` operations are handled by the compiler's soft-float library.  Results are numerically correct; the computation is simply slower than on a Cortex-M7 (`armv7emdp`) which has hardware `double` support.

---

## Path 2 – External C module (firmware build)

Use this when you need to compile bclibc into a custom firmware image.

### MicroPython

```sh
cd micropython                              # MicroPython source root
mkdir build && cd build
cmake .. \
    -DBOARD=RPI_PICO \                      # adjust for your board
    -DUSER_C_MODULES=/path/to/bclibc/micropython/micropython.cmake
make -j$(nproc)
```

### CircuitPython

CircuitPython uses a similar mechanism called "extra modules":

```sh
cd circuitpython                            # CircuitPython source root
make BOARD=raspberry_pi_pico \
     EXTRA_MODULES_DIR=/path/to/bclibc/micropython
```

(Check the CircuitPython docs for your specific board/version as the flag name varies.)

---

## Python API

```python
import bclibc

# Library version string
bclibc.version()  # e.g. "1.1.2"

# Utility functions
bclibc.get_correction(distance_ft, offset_ft)   # -> float (radians)
bclibc.calculate_energy(weight_grain, vel_fps)  # -> float (ft-lb)
bclibc.calculate_ogw(weight_grain, vel_fps)     # -> float (lb)

# Shot dict (all keys optional – sensible defaults apply)
shot = {
    "bc":                   0.331,
    "weight_grain":         168.0,
    "diameter_inch":        0.308,
    "length_inch":          1.220,
    "muzzle_velocity_fps":  2650.0,
    "sight_height_ft":      0.1667,   # 2 inches
    "twist_inch":           11.0,
    "temp_c":               15.0,
    "pressure_hpa":         1013.25,
    "altitude_ft":          0.0,
    "humidity":             0.5,
    # Drag table (required)
    "mach_data": [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0],
    "cd_data":   [0.5, 0.4, 0.6, 0.4, 0.34, 0.3, 0.27],
    # Winds (optional list)
    "winds": [
        {"velocity_fps": 10.0, "direction_from_rad": 1.5708,
         "until_distance_ft": 1e16, "max_distance_ft": 1e16}
    ],
    # Aiming
    "look_angle_rad":       0.0,
    "barrel_elevation_rad": 0.0,
    "barrel_azimuth_rad":   0.0,
    "cant_angle_rad":       0.0,
    # Coriolis: pass NaN (or omit) to disable
    "latitude_deg":         float("nan"),
    "azimuth_deg":          float("nan"),
    # Integration method: bclibc.METHOD_RK4 or bclibc.METHOD_EULER
    "method": bclibc.METHOD_RK4,
    # Solver config (optional sub-dict)
    "config": {
        "cStepMultiplier":      1.0,
        "cZeroFindingAccuracy": 0.001,
        "cMinimumVelocity":     50.0,
        "cMaximumDrop":         15000.0,
        "cMaxIterations":       100,
        "cGravityConstant":     -32.17405,
        "cMinimumAltitude":     -1000.0,
    },
}

# Zero angle
angle = bclibc.find_zero_angle(shot, 300.0)   # distance in ft -> radians

# Apex
apex = bclibc.find_apex(shot)                 # -> trajectory tuple (see below)

# Max range
max_range_ft, angle_at_max_rad = bclibc.find_max_range(shot, 0.0, 45.0)

# Full trajectory
request = {
    "range_limit_ft": 3000.0,
    "range_step_ft":  100.0,
    "time_step":      0.0,
    "filter_flags":   bclibc.TRAJ_FLAG_RANGE,
}
points, reason = bclibc.integrate(shot, request)
for p in points:
    dist_ft   = p[bclibc.T_DISTANCE]
    height_ft = p[bclibc.T_HEIGHT]
    vel_fps   = p[bclibc.T_VELOCITY]
    print(dist_ft, height_ft, vel_fps)

# Single-point interception
raw, full = bclibc.integrate_at(shot, bclibc.INTERP_POS_X, 1000.0)
```

### Trajectory tuple field indices

| Constant | Index | Field |
|---|---|---|
| `T_TIME` | 0 | time (s) |
| `T_DISTANCE` | 1 | distance_ft |
| `T_VELOCITY` | 2 | velocity_fps |
| `T_MACH` | 3 | mach |
| `T_HEIGHT` | 4 | height_ft |
| `T_SLANT_HEIGHT` | 5 | slant_height_ft |
| `T_DROP_ANGLE` | 6 | drop_angle_rad |
| `T_WINDAGE` | 7 | windage_ft |
| `T_WINDAGE_ANGLE` | 8 | windage_angle_rad |
| `T_SLANT_DISTANCE` | 9 | slant_distance_ft |
| `T_ANGLE` | 10 | angle_rad |
| `T_DENSITY_RATIO` | 11 | density_ratio |
| `T_DRAG` | 12 | drag |
| `T_ENERGY` | 13 | energy_ft_lb |
| `T_OGW` | 14 | ogw_lb |
| `T_FLAG` | 15 | flag (TRAJ_FLAG_* bitmask) |

### Constants

```python
# Integration methods
bclibc.METHOD_RK4    # 0
bclibc.METHOD_EULER  # 1

# TrajFlag bitmask values
bclibc.TRAJ_FLAG_NONE       # 0
bclibc.TRAJ_FLAG_ZERO_UP    # 1
bclibc.TRAJ_FLAG_ZERO_DOWN  # 2
bclibc.TRAJ_FLAG_ZERO       # 3
bclibc.TRAJ_FLAG_MACH       # 4
bclibc.TRAJ_FLAG_RANGE      # 8
bclibc.TRAJ_FLAG_APEX       # 16
bclibc.TRAJ_FLAG_ALL        # 31
bclibc.TRAJ_FLAG_MRT        # 32

# Interpolation keys (for integrate_at)
bclibc.INTERP_TIME           # 0
bclibc.INTERP_MACH           # 1
bclibc.INTERP_POS_X          # 2 (downrange distance, ft)
bclibc.INTERP_POS_Y          # 3 (height, ft)
bclibc.INTERP_POS_Z          # 4 (windage, ft)
bclibc.INTERP_VEL_X          # 5
bclibc.INTERP_VEL_Y          # 6
bclibc.INTERP_VEL_Z          # 7
```

### Memory considerations

On constrained devices (e.g. RP2040 with 264 KB RAM) keep the trajectory short:
- Use `filter_flags = bclibc.TRAJ_FLAG_RANGE` and a large `range_step_ft` (e.g. 300 ft) to reduce the number of returned points.
- Use `bclibc.METHOD_EULER` for faster (lower-accuracy) integration.
- The drag table should have at most ~20 points; 5–10 usually suffice.
