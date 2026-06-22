# ── Arch selection ─────────────────────────────────────────────────────────────
#
#   make                       → x64 (double, default)
#   make ARCH=x86              → x86 32-bit (double, default)
#   make ARCH=armv6m           → RP2040 / Cortex-M0+
#   make ARCH=armv7m           → Cortex-M3
#   make ARCH=armv7emsp        → Cortex-M4F / M7 single-precision FPU
#   make ARCH=armv7emdp        → Cortex-M7 double-precision FPU
#   make ARCH=xtensawin        → ESP32 / ESP32-S3 (Xtensa windowed)
#   make ARCH=xtensa           → ESP8266 (Xtensa no-window)
#   make ARCH=rv32imc          → ESP32-C3 / ESP32-C6 (RISC-V 32)
#   make ARCH=rv64imc          → RISC-V 64
#
ARCH    ?= x64
MPY_DIR ?= micropython

# ── Precision ─────────────────────────────────────────────────────────────────
# USE_FLOAT=0 → double  → tiny_bclibc_<arch>_d.mpy  (default on x64/x86)
# USE_FLOAT=1 → float   → tiny_bclibc_<arch>.mpy    (default on MCU)
#
ifeq ($(ARCH),$(filter $(ARCH),x64 x86))
USE_FLOAT ?= 0
else
USE_FLOAT ?= 1
endif

ifeq ($(USE_FLOAT),1)
_PREC_SUFFIX :=
else
_PREC_SUFFIX := _d
endif

MOD         = tiny_bclibc_$(ARCH)$(_PREC_SUFFIX)
BCLIBC_ROOT = ..
BUILD       = build-$(ARCH)$(_PREC_SUFFIX)
VERSION_H   = generated/bclibc_mp/version.h
VERSION_IN  = version.h.in

SRC = src/bclibc_mp.c

# ── Shared math-library sources ───────────────────────────────────────────────
#
# fdlibm (single-precision, BSD types) — used on x64/x86 single and RISC-V.
# fdlibm.h uses BSD __int32_t → alias with -D and -include stdint.h.
#
_FDLIBM_DIR   := $(MPY_DIR)/lib/libm
_FDLIBM_SRCS  := $(_FDLIBM_DIR)/sf_sin.c      \
                 $(_FDLIBM_DIR)/sf_cos.c      \
                 $(_FDLIBM_DIR)/kf_sin.c      \
                 $(_FDLIBM_DIR)/kf_cos.c      \
                 $(_FDLIBM_DIR)/ef_rem_pio2.c \
                 $(_FDLIBM_DIR)/kf_rem_pio2.c \
                 $(_FDLIBM_DIR)/atan2f.c      \
                 $(_FDLIBM_DIR)/atanf.c       \
                 $(_FDLIBM_DIR)/ef_sqrt.c     \
                 $(_FDLIBM_DIR)/math.c
_FDLIBM_FLAGS := -I$(MPY_DIR)/lib    \
                 -include stdint.h   \
                 -D__int32_t=int32_t \
                 -D__uint32_t=uint32_t \
                 -D__uint8_t=uint8_t

# libm_dbl (double-precision, musl-derived) — used on x64/x86 double.
# math_shadow/math.h intercepts <math.h> to prevent glibc/__sin conflict.
# floor.c is needed by __rem_pio2_large.c.
#
_LIBM_DBL_DIR   := $(MPY_DIR)/lib/libm_dbl
_LIBM_DBL_SRCS  := $(_LIBM_DBL_DIR)/sin.c              \
                   $(_LIBM_DBL_DIR)/cos.c              \
                   $(_LIBM_DBL_DIR)/sqrt.c             \
                   $(_LIBM_DBL_DIR)/atan2.c            \
                   $(_LIBM_DBL_DIR)/atan.c             \
                   $(_LIBM_DBL_DIR)/exp.c              \
                   $(_LIBM_DBL_DIR)/pow.c              \
                   $(_LIBM_DBL_DIR)/log.c              \
                   $(_LIBM_DBL_DIR)/__sin.c            \
                   $(_LIBM_DBL_DIR)/__cos.c            \
                   $(_LIBM_DBL_DIR)/__rem_pio2.c       \
                   $(_LIBM_DBL_DIR)/__rem_pio2_large.c \
                   $(_LIBM_DBL_DIR)/__expo2.c          \
                   $(_LIBM_DBL_DIR)/scalbn.c           \
                   $(_LIBM_DBL_DIR)/frexp.c            \
                   $(_LIBM_DBL_DIR)/floor.c
_LIBM_DBL_FLAGS := -I$(MPY_DIR)/lib/libm_dbl \
                   -I src/math_shadow

# ── Math library selection ────────────────────────────────────────────────────
ifeq ($(ARCH),$(filter $(ARCH),x64 x86))
# Host: glibc libm.a uses IFUNC internals that can't be linked in a natmod.
# GCC -O2 merges sin()+cos() → sincos() (GNU ext) → math_shim.c provides it.
ifeq ($(USE_FLOAT),1)
SRC          += $(_FDLIBM_SRCS) src/math_shim.c
CFLAGS_EXTRA += $(_FDLIBM_FLAGS)
else
SRC          += $(_LIBM_DBL_SRCS) src/math_shim.c
CFLAGS_EXTRA += $(_LIBM_DBL_FLAGS)
endif
ifeq ($(ARCH),x86)
CFLAGS_EXTRA += -msse2 -mfpmath=sse
endif

else ifeq ($(ARCH),$(filter $(ARCH),rv32imc rv64imc))
# RISC-V: picolibc libc.a triggers a mpy_ld.py bug (.srodata sections not
# recognised + absolute LO12 relocation unhandled).  Use fdlibm + libgcc
# soft-float directly until the upstream fix lands.
# See micropython-natmod/RISC-V_picolibc.md for details and the patch.
SRC          += $(_FDLIBM_SRCS) src/mem_shim.c
CFLAGS_EXTRA += $(_FDLIBM_FLAGS)

else
# ARM + Xtensa: newlib libm.a is self-contained and provides both precisions.
LINK_RUNTIME  = 1
SRC          += src/mem_shim.c
endif

# ── Arch-specific cross-compiler prefix ───────────────────────────────────────
ifeq ($(ARCH),xtensawin)
CROSS = xtensa-esp32s3-elf-
else ifeq ($(ARCH),xtensa)
CROSS = xtensa-esp32-elf-
endif

# ── Common flags ──────────────────────────────────────────────────────────────
CFLAGS_EXTRA += \
    -I$(BCLIBC_ROOT)/tiny_bclibc/include \
    -Igenerated \
    -DTINY_BCLIBC_NO_THREAD_LOCAL \
    -DTINY_BCLIBC_NO_ERR_BUF \
    -DBCLIBC_BUILD_NATMOD

ifeq ($(USE_FLOAT),1)
CFLAGS_EXTRA += -DTINY_BCLIBC_USE_FLOAT
endif

include $(MPY_DIR)/py/dynruntime.mk

# ── RISC-V: add libgcc.a for soft-float builtins ─────────────────────────────
# Must be after include so CROSS and CFLAGS_ARCH are set by dynruntime.mk.
ifeq ($(ARCH),$(filter $(ARCH),rv32imc rv64imc))
MPY_LD_FLAGS += -l$(realpath $(shell $(CROSS)gcc $(CFLAGS_ARCH) --print-libgcc-file-name))
endif

# ── Version header ─────────────────────────────────────────────────────────────
$(VERSION_H): $(VERSION_IN)
	$(ECHO) "  GEN   $@"
	$(Q)mkdir -p $(dir $@)
	$(eval _TAG  := $(shell git describe --tags --always 2>/dev/null || echo v0.0.0))
	$(eval _MAJ  := $(shell printf '%s' '$(_TAG)' | sed -E 's/^v?([0-9]+)\..*/\1/;t;s/.*/0/'))
	$(eval _MIN  := $(shell printf '%s' '$(_TAG)' | sed -E 's/^v?[0-9]+\.([0-9]+)\..*/\1/;t;s/.*/0/'))
	$(eval _PAT  := $(shell printf '%s' '$(_TAG)' | sed -E 's/^v?[0-9]+\.[0-9]+\.([0-9]+).*/\1/;t;s/.*/0/'))
	$(eval _VER  := $(shell printf '%s' '$(_TAG)' | sed 's/^v//'))
	$(Q)sed -e 's/@BCLIBC_MP_VERSION_MAJOR@/$(_MAJ)/g' \
	        -e 's/@BCLIBC_MP_VERSION_MINOR@/$(_MIN)/g' \
	        -e 's/@BCLIBC_MP_VERSION_PATCH@/$(_PAT)/g' \
	        -e 's/@BCLIBC_MP_VERSION@/$(_VER)/g' \
	        $< > $@

$(BUILD)/src/bclibc_mp.o: $(VERSION_H)

# ── Convenience aliases ────────────────────────────────────────────────────────
.PHONY: x64 x64s x86 x86s rp2040 rp2350 stm32f4 stm32h7 stm32h7dp esp32s3 esp32 esp32c3 esp32c6 clean

x64:
	$(MAKE) ARCH=x64 USE_FLOAT=0

x64s:
	$(MAKE) ARCH=x64 USE_FLOAT=1

x86:
	$(MAKE) ARCH=x86 USE_FLOAT=0

x86s:
	$(MAKE) ARCH=x86 USE_FLOAT=1

rp2040:
	$(MAKE) ARCH=armv6m

rp2350 stm32f4:
	$(MAKE) ARCH=armv7emsp

stm32h7:
	$(MAKE) ARCH=armv7emdp USE_FLOAT=1

stm32h7dp:
	$(MAKE) ARCH=armv7emdp USE_FLOAT=0

esp32s3:
	$(MAKE) ARCH=xtensawin

esp32:
	$(MAKE) ARCH=xtensa

esp32c3 esp32c6:
	$(MAKE) ARCH=rv32imc

clean:
	rm -rf build-* tiny_bclibc_*.mpy generated
