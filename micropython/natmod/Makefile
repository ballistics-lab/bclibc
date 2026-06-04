# bclibc MicroPython native module (.mpy) build
#
# Requires the MicroPython source tree.  Set MPY_DIR to its root.
#
# Usage:
#   make ARCH=armv7emsp  MPY_DIR=/path/to/micropython   # STM32 Cortex-M4
#   make ARCH=armv6m     MPY_DIR=/path/to/micropython   # RP2040
#   make ARCH=xtensawin  MPY_DIR=/path/to/micropython   # ESP32
#   make ARCH=rv32imc    MPY_DIR=/path/to/micropython   # ESP32-C3
#   make clean
#
# Supported ARCH values (maps to MicroPython dynruntime architecture names):
#   armv6m       - Cortex-M0/M0+ (RP2040, no FPU; uses soft-float for double)
#   armv7m       - Cortex-M3
#   armv7emsp    - Cortex-M4F  (single-precision FPU; double is emulated)
#   armv7emdp    - Cortex-M7   (double-precision FPU)
#   xtensawin    - ESP32 Xtensa LX6
#   rv32imc      - ESP32-C3 RISC-V
#
# Note on floating-point precision:
#   bclibc uses double (64-bit) throughout.  On armv6m/armv7emsp the FPU only
#   handles float; double operations use soft-float emulation which is correct
#   but slower.  On armv7emdp the hardware FPU handles double directly.

ARCH    ?= armv7emsp
MPY_DIR ?= ../../../micropython

# MOD and MOD_NAME are both required: dynruntime.mk reads MOD for the link
# target name; we keep MOD_NAME as an alias for convenience rules below.
MOD         = bclibc
BCLIBC_ROOT = ../..
BUILD       = build-$(ARCH)

# ============================================================================
# C source (Python binding layer)
# dynruntime.mk computes SRC_O from $(filter %.c,$(SRC)) — must use SRC.
# ============================================================================
SRC = bclibc_mp.c

# Extra include dirs and defines picked up by dynruntime.mk via CFLAGS_EXTRA.
CFLAGS_EXTRA += -I$(BCLIBC_ROOT)/include -Igenerated -DBCLIBC_BUILD_NATMOD

# ============================================================================
# Pull in the MicroPython natmod build system.
# Sets CC, CFLAGS, MPY_LD, PYTHON, BUILD, and the $(MOD).mpy / .native.mpy rules.
# ============================================================================
include $(MPY_DIR)/py/dynruntime.mk

# ============================================================================
# C++ support (bclibc core + FFI + operator-new / stdlib stubs)
# ============================================================================

BCLIBC_CXX_SRCS = \
    cxx_stubs.cpp \
    $(BCLIBC_ROOT)/src/base_types.cpp \
    $(BCLIBC_ROOT)/src/engine.cpp \
    $(BCLIBC_ROOT)/src/euler.cpp \
    $(BCLIBC_ROOT)/src/interp.cpp \
    $(BCLIBC_ROOT)/src/rk4.cpp \
    $(BCLIBC_ROOT)/src/traj_data.cpp \
    $(BCLIBC_ROOT)/src/traj_filter.cpp \
    $(BCLIBC_ROOT)/src/ffi/bclibc_ffi.cpp

# Flatten all C++ object paths into $(BUILD)/ (unique filenames assumed)
BCLIBC_CXX_OBJS = $(addprefix $(BUILD)/, $(notdir $(BCLIBC_CXX_SRCS:.cpp=.o)))

# C++ compiler: always derived from CROSS_COMPILE, not from CC.
# This avoids accidentally picking up the host g++ when CC is set by the
# environment (common in CI runners).
CXX := $(CROSS_COMPILE)g++

# Derive C++ flags from the C flags dynruntime.mk built (strips -std=c99),
# then layer on C++-specific options.  CFLAGS already includes CFLAGS_EXTRA
# (and thus -DBCLIBC_BUILD_NATMOD) because dynruntime.mk does CFLAGS += $(CFLAGS_EXTRA).
CXXFLAGS = $(filter-out -std=c99 -Werror,$(CFLAGS)) -I$(BCLIBC_ROOT)/include -Igenerated \
           -fno-exceptions -fno-rtti -std=c++17

# VPATH lets the %.cpp pattern rule find sources in subdirectories.
VPATH += $(BCLIBC_ROOT)/src \
         $(BCLIBC_ROOT)/src/ffi \
         .

$(BUILD)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(Q)$(CXX) $(CXXFLAGS) -MMD -MF $(@:.o=.d) -c -o $@ $<

# cxx_stubs.cpp provides strlen/memcpy/memset/memmove/memcmp.
# -fno-builtin prevents GCC from recognising the loop patterns and emitting
# recursive calls to the same library functions.
$(BUILD)/cxx_stubs.o: cxx_stubs.cpp
	@mkdir -p $(dir $@)
	$(Q)$(CXX) $(CXXFLAGS) -fno-builtin -MMD -MF $(@:.o=.d) -c -o $@ $<

# Include generated dependency files if they exist
-include $(BCLIBC_CXX_OBJS:.o=.d)

# ============================================================================
# Partial link: bundle all C++ objects + toolchain runtime libraries into a
# single relocatable object so that mpy_ld.py sees only two input files
# (bclibc_mp.o and bclibc_rt.o) and all soft-float / math symbols are
# resolved internally.
#
# _LIBGCC: soft-float emulation (__aeabi_dadd, __adddf3, etc.)
# _LIBM:   math functions (sqrt, sin, cos, atan2, …) — included only when
#          the toolchain provides a libm.a (bare-metal arm-none-eabi and
#          xtensa-esp-elf both do; the test avoids adding a raw "libm.a"
#          sentinel returned when the file is not found).
# ============================================================================
_LIBGCC   := $(shell $(CROSS_COMPILE)gcc --print-libgcc-file-name 2>/dev/null)
_LIBM_RAW := $(shell $(CROSS_COMPILE)gcc --print-file-name=libm.a 2>/dev/null)
_LIBM     := $(if $(filter-out libm.a,$(_LIBM_RAW)),$(_LIBM_RAW),)

_RT_LIBS  := $(_LIBGCC)
ifneq ($(_LIBM),)
_RT_LIBS  += $(_LIBM)
endif

PARTIAL_OBJ := $(BUILD)/$(MOD)_rt.o

$(PARTIAL_OBJ): $(BCLIBC_CXX_OBJS)
	$(ECHO) "PARTIAL-LINK $(notdir $@)"
	$(Q)$(CROSS_COMPILE)ld -r -nostdlib \
	    --start-group $^ $(_RT_LIBS) --end-group \
	    -o $@

# Add the partial object as an extra prerequisite for the native link step.
# dynruntime.mk defines: $(BUILD)/$(MOD).native.mpy: $(SRC_O)
#   with recipe:  $(MPY_LD) ... -o $@ $^
# GNU make appends our prerequisite; $^ becomes bclibc_mp.o + bclibc_rt.o.
$(BUILD)/$(MOD).native.mpy: $(PARTIAL_OBJ)

# ============================================================================
# Convenience targets
# ============================================================================

.PHONY: all clean

all: $(MOD).mpy

clean:
	rm -rf $(BUILD) $(MOD).mpy
