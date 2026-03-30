.PHONY: all clean build ffi core windows windows-debug

# Визначаємо операційну систему
ifeq ($(OS),Windows_NT)
    DETECTED_OS := Windows
    CMAKE_GENERATOR := "Visual Studio 17 2022"
    MAKE_COMMAND := cmake --build build --config Release
    MAKE_DEBUG := cmake --build build --config Debug
    LIB_PREFIX := 
    LIB_SUFFIX := .dll
    LIB_DIR := build/bin/Release
    LIB_DIR_DEBUG := build/bin/Debug
else
    DETECTED_OS := $(shell uname -s)
    CMAKE_GENERATOR := "Unix Makefiles"
    MAKE_COMMAND := cd build && make -j$(nproc)
    MAKE_DEBUG := cd build && make -j$(nproc)
    LIB_PREFIX := lib
    LIB_SUFFIX := .so
    LIB_DIR := build
    LIB_DIR_DEBUG := build
    ifeq ($(DETECTED_OS),Darwin)
        LIB_SUFFIX := .dylib
    endif
endif

all: build

# Загальне налаштування CMake
cmake-configure:
	mkdir -p build
	cd build && cmake -G $(CMAKE_GENERATOR) -DCMAKE_BUILD_TYPE=Release ..
	@echo "CMake configured for $(DETECTED_OS)"

build: cmake-configure
	$(MAKE_COMMAND)
	@echo "Build complete for $(DETECTED_OS)"
	@echo "Library location: $(LIB_DIR)/$(LIB_PREFIX)bclibc_ffi$(LIB_SUFFIX)"

core: cmake-configure
	cd build && cmake --build . --target bclibc_core --config Release
	@echo "Core library built"

ffi: cmake-configure
	cd build && cmake --build . --target bclibc_ffi --config Release
	@echo "FFI library built"

# Windows специфічні цілі
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

# Linux специфічні цілі
linux: cmake-configure
	cd build && make -j$(nproc)
	@echo "Linux build complete"

# macOS специфічні цілі
macos: cmake-configure
	cd build && make -j$(shell sysctl -n hw.ncpu)
	@echo "macOS build complete"

clean:
	rm -rf build
	@echo "Cleaned build directory"

# Додаткова ціль для перевірки
info:
	@echo "Detected OS: $(DETECTED_OS)"
	@echo "CMake Generator: $(CMAKE_GENERATOR)"
	@echo "Library prefix: $(LIB_PREFIX)"
	@echo "Library suffix: $(LIB_SUFFIX)"