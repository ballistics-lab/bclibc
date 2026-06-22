# tiny_bclibc MicroPython natmod

Native module (`.mpy`) that exposes the `tiny_bclibc` ballistics library to MicroPython.
Each supported architecture produces its own file: `tiny_bclibc_<arch>[_d].mpy`.

### Architecture support

The native `.mpy` format is supported only on architectures that `mpy_ld.py` knows how to
link. On other architectures (aarch64, mipsel, …) MicroPython can still **run** as a unix
port, and the full API is available via [`test_bclibc_ffi.py`](#ffi-based-access-any-unix-architecture)
which calls `libtiny_bclibc.so` directly through the `ffi` module — no native module needed.

| Approach | Architectures | Requires |
|----------|--------------|---------|
| Native `.mpy` natmod | x64, x86, armv6m–armv7emdp, xtensa, rv32/64imc | `mpy_ld.py` linker support |
| FFI (`libtiny_bclibc.so`) | **any** unix port arch (aarch64, mipsel, …) | `libffi`, shared library build |

## Prerequisites

### Python tooling

```bash
pip install pyelftools ar      # required by mpy_ld.py
```

### MicroPython source

The build system needs MicroPython v1.28 (for `dynruntime.mk`, `mpy-cross`,
and `lib/libm`).  Pass `MPY_DIR` explicitly or place it at `../micropython-1.28.0`:

```bash
wget https://github.com/micropython/micropython/releases/download/v1.28.0/micropython-1.28.0.tar.xz
tar xf micropython-1.28.0.tar.xz
export MPY_DIR=$(pwd)/micropython-1.28.0
```

### Cross-compilers

| Target | Package (Debian/Ubuntu) |
|--------|-------------------------|
| x64 | `gcc` (host compiler, already installed) |
| x86 | `gcc-multilib` |
| RP2040 / Cortex-M | `gcc-arm-none-eabi libnewlib-arm-none-eabi` |
| ESP32-C3/C6 (RISC-V 32/64) | `gcc-riscv64-unknown-elf picolibc-riscv64-unknown-elf` |
| ESP32 / ESP32-S3 | `xtensa-esp32{s3}-elf-gcc` (from ESP-IDF) |

```bash
sudo apt-get install gcc-arm-none-eabi libnewlib-arm-none-eabi \
                     gcc-multilib gcc-riscv64-unknown-elf
```

## Build

All commands are run from this directory (`micropython-natmod/`).

Precision suffix: none = single (`float`), `_d` = double.
Default precision: **double** for x64/x86 host, **single** for all MCU targets.

```bash
make            # x64 double (default) → tiny_bclibc_x64_d.mpy
make x64        # x64 double
make x64s       # x64 single           → tiny_bclibc_x64.mpy
make x86        # x86 double
make x86s       # x86 single
make rp2040     # armv6m    single — Raspberry Pi Pico
make rp2350     # armv7emsp single — RP2350
make stm32f4    # armv7emsp single — STM32F4 (alias of rp2350)
make stm32h7    # armv7emdp single — STM32H7
make stm32h7dp  # armv7emdp double — STM32H7 (double-precision FPU)
make esp32s3    # xtensawin single — ESP32-S3
make esp32      # xtensa    single — ESP32
make esp32c3    # rv32imc   single — ESP32-C3
make esp32c6    # rv32imc   single — ESP32-C6 (alias of esp32c3)
```

Output: `tiny_bclibc_<arch>[_d].mpy`

```bash
make clean      # remove all build-* directories and tiny_bclibc_*.mpy
```

### Custom MPY_DIR or precision

```bash
make ARCH=armv6m MPY_DIR=/path/to/micropython-1.28.0
make ARCH=armv7emdp USE_FLOAT=0   # double on Cortex-M7
make ARCH=x64 USE_FLOAT=1         # single on host
```

## Test (x64 / x86 host)

```bash
# Build MicroPython unix binary (must match the .mpy version)
make -C "$MPY_DIR/ports/unix" VARIANT=standard
MPY="$MPY_DIR/ports/unix/build-standard/micropython"

# Build natmod and symlink
make x64        # or: make x64s / make x86 / make x86s
ln -sf tiny_bclibc_x64_d.mpy tiny_bclibc.mpy

# Run tests (natmod)
$MPY test_bclibc.py

# Run tests (ffi — calls libtiny_bclibc.so directly)
$MPY test_bclibc_ffi.py
```

Expected output ends with `=== done ===` and all lines read `PASS`.

## FFI-based access (any unix architecture)

On architectures where `mpy_ld.py` does not yet support native modules (aarch64, mipsel,
and others), `test_bclibc_ffi.py` uses MicroPython's built-in `ffi` module to call
`libtiny_bclibc.so` directly. This works on every platform where MicroPython unix port
is available and `libffi` is present.

```bash
# 1. Build libtiny_bclibc.so for the target platform (native or cross)
cmake -B ../tiny_bclibc/build-shared \
      -S ../tiny_bclibc \
      -DTINY_BCLIBC_BUILD_SHARED=ON \
      -DCMAKE_BUILD_TYPE=Release
cmake --build ../tiny_bclibc/build-shared

# 2. Build MicroPython unix port for the target (with ffi support)
make -C "$MPY_DIR/ports/unix" VARIANT=standard

MPY="$MPY_DIR/ports/unix/build-standard/micropython"

# 3. Run — no .mpy needed
$MPY test_bclibc_ffi.py
```

`test_bclibc_ffi.py` automatically skips on 32-bit platforms (pointer size ≠ 8 bytes)
because the struct layout in the shared library uses `double` (64-bit `real_t`).

## Test (QEMU — Cortex-M3 / armv7m)

```bash
sudo apt-get install qemu-system-arm
pip install pyserial

# Build MicroPython cross-compiler and QEMU firmware (one-time)
make -C "$MPY_DIR/mpy-cross"
make -C "$MPY_DIR/ports/qemu" BOARD=MPS2_AN385

# Build natmod
make ARCH=armv7m
ln -sf tiny_bclibc_armv7m.mpy tiny_bclibc.mpy

# Run tests through the QEMU pty bridge
python3 ci/run_qemu.py \
    "$MPY_DIR/ports/qemu/build-MPS2_AN385/firmware.elf" \
    .
```

## Test (QEMU — Cortex-M0 / armv6m)

Uses MicroPython's `MICROBIT` QEMU board (nRF51 SOC, `cortex-m0`).

```bash
sudo apt-get install qemu-system-arm
pip install pyserial

# Build MicroPython cross-compiler and QEMU firmware (one-time)
make -C "$MPY_DIR/mpy-cross"
make -C "$MPY_DIR/ports/qemu" BOARD=MICROBIT

# Build natmod
make ARCH=armv6m
ln -sf tiny_bclibc_armv6m.mpy tiny_bclibc.mpy

# Run tests through the QEMU pty bridge
python3 ci/run_qemu.py \
    "$MPY_DIR/ports/qemu/build-MICROBIT/firmware.elf" \
    . \
    --machine microbit \
    --qemu-extra "-global nrf51-soc.flash-size=1048576 -global nrf51-soc.sram-size=262144"
```

## Module API

```python
import tiny_bclibc
from tiny_bclibc_types import Shot, Request, Wind, Config, DRAG_G1, DRAG_G7, DRAG_CUSTOM

tiny_bclibc.version()              # → "1.2.3"

# Scalar helpers
tiny_bclibc.calculate_energy(weight_grain, velocity_fps)   # → ft·lbf (float)
tiny_bclibc.calculate_ogw(weight_grain, velocity_fps)       # → optimal game weight (float)
tiny_bclibc.get_correction(distance_ft, drop_ft)            # → angle correction (rad)

# Trajectory integration
rows, stop_reason = tiny_bclibc.integrate(shot.pack(), request.pack())
# rows: list of 16-tuples — use T_* indices to access fields

# Zero-angle search
elevation_rad = tiny_bclibc.find_zero_angle(shot.pack(), zero_distance_ft)

# Maximum range (golden-section search over [low_deg, high_deg])
range_ft, angle_rad = tiny_bclibc.find_max_range(shot.pack(), low_deg, high_deg)

# Single-point interpolation
raw, full = tiny_bclibc.integrate_at(shot.pack(), tiny_bclibc.INTERP_POS_X, distance_ft)
# raw: 8-tuple (time, px, py, pz, vx, vy, vz, mach)
# full: same 16-tuple as integrate() rows

# Apex
apex = tiny_bclibc.find_apex(shot.pack())   # → single trajectory row 16-tuple

# ── Trajectory flag constants ─────────────────────────────────────────────────
tiny_bclibc.TRAJ_FLAG_NONE       # 0
tiny_bclibc.TRAJ_FLAG_ZERO_UP    # 1  — rising zero crossing
tiny_bclibc.TRAJ_FLAG_ZERO_DOWN  # 2  — falling zero crossing
tiny_bclibc.TRAJ_FLAG_ZERO       # 3  — any zero crossing
tiny_bclibc.TRAJ_FLAG_MACH       # 4  — Mach 1 crossing
tiny_bclibc.TRAJ_FLAG_RANGE      # 8  — range-step output
tiny_bclibc.TRAJ_FLAG_APEX       # 16 — apex
tiny_bclibc.TRAJ_FLAG_ALL        # 31
tiny_bclibc.TRAJ_FLAG_MRT        # 32 — max range trajectory

# ── Interpolation key constants ───────────────────────────────────────────────
tiny_bclibc.INTERP_TIME          # 0
tiny_bclibc.INTERP_MACH          # 1
tiny_bclibc.INTERP_POS_X         # 2  — horizontal distance
tiny_bclibc.INTERP_POS_Y         # 3  — height
tiny_bclibc.INTERP_POS_Z         # 4  — lateral
tiny_bclibc.INTERP_VEL_X         # 5
tiny_bclibc.INTERP_VEL_Y         # 6
tiny_bclibc.INTERP_VEL_Z         # 7

# ── Trajectory tuple field indices ────────────────────────────────────────────
tiny_bclibc.T_TIME           # 0  — time (s)
tiny_bclibc.T_DISTANCE       # 1  — horizontal distance (ft)
tiny_bclibc.T_VELOCITY       # 2  — total velocity (fps)
tiny_bclibc.T_MACH           # 3  — Mach number
tiny_bclibc.T_HEIGHT         # 4  — height (ft)
tiny_bclibc.T_SLANT_HEIGHT   # 5  — height relative to look angle (ft)
tiny_bclibc.T_DROP_ANGLE     # 6  — trajectory angle minus look angle (rad)
tiny_bclibc.T_WINDAGE        # 7  — windage + spin drift (ft)
tiny_bclibc.T_WINDAGE_ANGLE  # 8  — windage angle (rad)
tiny_bclibc.T_SLANT_DISTANCE # 9  — slant distance (ft)
tiny_bclibc.T_ANGLE          # 10 — trajectory angle (rad)
tiny_bclibc.T_DENSITY_RATIO  # 11
tiny_bclibc.T_DRAG           # 12 — drag coefficient
tiny_bclibc.T_ENERGY         # 13 — kinetic energy (ft·lbf)
tiny_bclibc.T_OGW            # 14 — optimal game weight (lb)
tiny_bclibc.T_FLAG           # 15 — TRAJ_FLAG_* bitmask
```

See [tiny_bclibc_types.py](tiny_bclibc_types.py) for `Shot`, `Wind`, `Config`, `Request` constructors and `.pack()` / `.unpack()`.

## Architecture notes

| ARCH | Precision | Math library | BSS |
|------|-----------|-------------|-----|
| x64 / x86 | double (default) | musl libm_dbl (bundled in MicroPython) | 0 |
| x64 / x86 | float (optional) | fdlibm single (bundled in MicroPython) | 0 |
| armv6m | float only | newlib libm.a (via LINK_RUNTIME) | 0 |
| armv7m | float only | newlib libm.a (via LINK_RUNTIME) | 0 |
| armv7emsp | float only | newlib libm.a (via LINK_RUNTIME) | 0 |
| armv7emdp | float (default) / double | newlib libm.a (via LINK_RUNTIME) | 0 |
| xtensawin / xtensa | float only | newlib libm.a (via LINK_RUNTIME) | 0 |
| rv32imc / rv64imc | float only | fdlibm single + libgcc soft-float | 0 |

> **RISC-V note:** picolibc triggers a `mpy_ld.py` bug on current MicroPython.
> fdlibm is used as a workaround until the fix lands upstream.
> See [RISC-V_picolibc.md](RISC-V_picolibc.md) for details and the patch.

BSS must be 0 — MicroPython natmod ABI does not allow uninitialized static data.

### `find_zero_angle` performance (`TINY_BCLIBC_FAST_ZERO_FIND`)

`find_zero_angle` uses a Golden-Section Search (GSS) to bracket the max-range angle,
then Ridder's method to find the zero angle. Each GSS iteration runs a full RK4
trajectory, which is expensive on soft-float MCUs (Cortex-M0+, RISC-V without FPU).

`TINY_BCLIBC_FAST_ZERO_FIND` is automatically defined when building with `USE_FLOAT=1`.
It applies two optimisations that do **not** affect the final angle accuracy:

| Parameter | Default | Fast |
|-----------|---------|------|
| GSS step multiplier | 1× | 8× coarser (fewer RK4 steps per trajectory) |
| GSS convergence `h` | `1e-5 rad` | `1e-2 rad` (~13 iterations vs ~25) |
| Ridder's `acc` | `0.001 ft` | `0.01 ft` (3 mm — more than sufficient for `float`) |

The bracket bound (`angle_at_max`) is used only to constrain Ridder's search interval;
its precision does not affect the output. Ridder's method always uses the original
`calc_step`.

To build without `FAST_ZERO_FIND` even on `USE_FLOAT=1`, remove
`-DTINY_BCLIBC_FAST_ZERO_FIND` from `CFLAGS_EXTRA` in the Makefile.

See [sincosf_shim.md](sincosf_shim.md) for why `src/math_shim.c` is only compiled on x64/x86 and how to add it back for MCU targets if needed.

## Memory budget

Measured on x64 host, MicroPython v1.28, G7 drag, 168 gr @ 2750 fps.
Each output row costs ~**653 B** of heap (allocated by `tiny_bclibc.integrate()`).

| Range step | Rows (3 km) | Heap delta |
|-----------|-------------|------------|
| 100 m | 30 | ~19.5 KB |
| 50 m | 60 | ~39 KB |
| 25 m | 120 | ~78 KB |
| 10 m | 300 | ~197 KB |

### Per-platform recommendation

| Board | MCU | Arch | Usable heap¹ | Max step @ 3 km |
|-------|-----|------|-------------|-----------------|
| Raspberry Pi Pico | RP2040 | armv6m | ~192 KB | 10 m ✓ |
| Raspberry Pi Pico 2 | RP2350 | armv7emsp | ~480 KB | 10 m ✓ |
| STM32F401 (128 KB RAM) | Cortex-M4 | armv7emsp | ~64 KB | 50 m |
| STM32F405/F407 (192 KB RAM) | Cortex-M4 | armv7emsp | ~128 KB | 25 m |
| STM32H743 (1 MB RAM) | Cortex-M7 | armv7emdp | ~512 KB | 10 m ✓ |
| ESP32 | Xtensa LX6 | xtensa | ~200 KB | 25 m |
| ESP32-S3 | Xtensa LX7 | xtensawin | ~300 KB | 10 m ✓ |
| ESP32-S3 + PSRAM | Xtensa LX7 | xtensawin | ~8 MB | 1 m ✓ |
| ESP32-C3 | RISC-V | rv32imc | ~390 KB | 10 m ✓ |
| ESP32-C6 | RISC-V | rv32imc | ~490 KB | 10 m ✓ |

¹ Approximate free heap after MicroPython runtime starts. Actual value depends on
firmware variant, frozen modules, and Wi-Fi stack (ESP32).

For MCUs where the result list must fit in constrained RAM, stream results row by row
using `integrate_at()` + a range loop instead of storing the full trajectory.
