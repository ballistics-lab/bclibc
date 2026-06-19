.PHONY: all build core ffi windows windows-debug linux macos clean info

# ============================================================================
# Detect Operating System
# ============================================================================
ifeq ($(OS),Windows_NT)
    DETECTED_OS := Windows
    CMAKE_GENERATOR := "Visual Studio 17 2022"
    MAKE_COMMAND := cmake --build build --config Release
    LIB_PREFIX :=
    LIB_SUFFIX := .dll
    LIB_DIR := build/bin/Release
    LIB_DIR_DEBUG := build/bin/Debug
else
    DETECTED_OS := $(shell uname -s)
    CMAKE_GENERATOR := "Unix Makefiles"
    LIB_PREFIX := lib
    LIB_DIR := build
    LIB_DIR_DEBUG := build

    ifeq ($(DETECTED_OS),Darwin)
        NPROC := $(shell sysctl -n hw.ncpu)
        LIB_SUFFIX := .dylib
    else
        NPROC := $(shell nproc)
        LIB_SUFFIX := .so
    endif

    MAKE_COMMAND := cd build && $(MAKE) -j$(NPROC)
endif

all: build

# ============================================================================
# General CMake Configuration (Release)
# ============================================================================
cmake-configure:
	mkdir -p build
	cd build && cmake -G $(CMAKE_GENERATOR) -DCMAKE_BUILD_TYPE=Release ..
	@echo "CMake configured for $(DETECTED_OS)"

build: cmake-configure
	$(MAKE_COMMAND)
	@echo "Build complete for $(DETECTED_OS)"
	@echo "Library location: $(LIB_DIR)/$(LIB_PREFIX)bclibc_ffi$(LIB_SUFFIX)"

core: cmake-configure
	cmake --build build --target bclibc_core --config Release
	@echo "Core library built"

ffi: cmake-configure
	cmake --build build --target bclibc_ffi --config Release
	@echo "FFI library built"

# ============================================================================
# Windows Specific Targets
# ============================================================================
windows: cmake-configure
	cmake --build build --config Release
	@echo "Windows build complete"
	@echo "DLL location: build/bin/Release/bclibc_ffi.dll"
	@echo "LIB location: build/lib/Release/bclibc_ffi.lib"

windows-debug:
	mkdir -p build
	cd build && cmake -G $(CMAKE_GENERATOR) -DCMAKE_BUILD_TYPE=Debug ..
	cmake --build build --config Debug
	@echo "Windows Debug build complete"
	@echo "DLL location: build/bin/Debug/bclibc_ffi.dll"

# ============================================================================
# Linux Specific Targets
# ============================================================================
linux: cmake-configure
	cd build && $(MAKE) -j$(NPROC)
	@echo "Linux build complete"

# ============================================================================
# macOS Specific Targets
# ============================================================================
macos: cmake-configure
	cd build && $(MAKE) -j$(NPROC)
	@echo "macOS build complete"

clean:
	rm -rf build
	@echo "Cleaned build directory"

# ============================================================================
# Additional Target for Inspection / Debugging
# ============================================================================
info:
	@echo "Detected OS: $(DETECTED_OS)"
	@echo "CMake Generator: $(CMAKE_GENERATOR)"
	@echo "Library prefix: $(LIB_PREFIX)"
	@echo "Library suffix: $(LIB_SUFFIX)"