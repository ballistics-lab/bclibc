.PHONY: all headers shared static test-identity version symbols symbols-static clean info \
        cmake-configure cmake-configure-shared cmake-configure-static cmake-configure-test

# ── Detect OS ──────────────────────────────────────────────────────
ifeq ($(OS),Windows_NT)
    DETECTED_OS := Windows
    CMAKE_GENERATOR_FLAG :=
    LIB_PREFIX :=
    LIB_SUFFIX := .dll
else
    DETECTED_OS := $(shell uname -s)
    CMAKE_GENERATOR_FLAG := -G "Unix Makefiles"
    LIB_PREFIX := lib
    ifeq ($(DETECTED_OS),Darwin)
        NPROC := $(shell sysctl -n hw.ncpu)
        LIB_SUFFIX := .dylib
    else
        NPROC := $(shell nproc)
        LIB_SUFFIX := .so
    endif
endif

all: headers

# ── CMake configure ────────────────────────────────────────────────
cmake-configure:
	mkdir -p build
	cd build && cmake $(CMAKE_GENERATOR_FLAG) -DCMAKE_BUILD_TYPE=Release \
	    -DTINY_BCLIBC_INSTALL=OFF ..
	@echo "CMake configured (headers mode) for $(DETECTED_OS)"

cmake-configure-shared:
	mkdir -p build-shared
	cd build-shared && cmake $(CMAKE_GENERATOR_FLAG) -DCMAKE_BUILD_TYPE=Release \
	    -DTINY_BCLIBC_BUILD_SHARED=ON -DTINY_BCLIBC_INSTALL=OFF ..
	@echo "CMake configured (shared) for $(DETECTED_OS)"

cmake-configure-static:
	mkdir -p build-static
	cd build-static && cmake $(CMAKE_GENERATOR_FLAG) -DCMAKE_BUILD_TYPE=Release \
	    -DTINY_BCLIBC_BUILD_STATIC=ON -DTINY_BCLIBC_INSTALL=OFF ..
	@echo "CMake configured (static) for $(DETECTED_OS)"

cmake-configure-test:
	mkdir -p build-test
	cd build-test && cmake $(CMAKE_GENERATOR_FLAG) -DCMAKE_BUILD_TYPE=Debug \
	    -DTINY_BCLIBC_BUILD_IDENTITY_TEST=ON -DTINY_BCLIBC_INSTALL=OFF ..
	@echo "CMake configured (identity test) for $(DETECTED_OS)"

# ── Build targets ──────────────────────────────────────────────────
# version.h is generated automatically by configure_file during cmake-configure
headers: cmake-configure
	@echo "Header-only target — tiny_bclibc::headers is INTERFACE (nothing to compile)"
	@echo "Generated: build/generated/tiny_bclibc/version.h"

shared: cmake-configure-shared
	cmake --build build-shared --config Release
	@echo "Shared library: build-shared/$(LIB_PREFIX)tiny_bclibc$(LIB_SUFFIX)"

static: cmake-configure-static
	cmake --build build-static --config Release
	@echo "Static library: build-static/$(LIB_PREFIX)tiny_bclibc.a"

test-identity: cmake-configure-test
	cmake --build build-test -- -j$(NPROC)
	build-test/tests/test_identity
	@echo "Identity test passed"

# ── Utils ────────────────────────────────────────────────────────
version: cmake-configure
	@grep "TINY_BCLIBC_VERSION" build/generated/tiny_bclibc/version.h 2>/dev/null || \
	    echo "(version.h not generated yet — run 'make headers' first)"

# nm -D: dynamic (exported) symbols only; grep tiny_bclibc_ filters out libc noise
symbols: build-shared/$(LIB_PREFIX)tiny_bclibc$(LIB_SUFFIX)
	nm -D --defined-only build-shared/$(LIB_PREFIX)tiny_bclibc$(LIB_SUFFIX) \
	    | grep ' T ' | awk '{print $$3}' | sort

build-shared/$(LIB_PREFIX)tiny_bclibc$(LIB_SUFFIX): cmake-configure-shared
	cmake --build build-shared --config Release

symbols-static: build-static/$(LIB_PREFIX)tiny_bclibc.a
	nm --defined-only build-static/$(LIB_PREFIX)tiny_bclibc.a \
	    | grep ' T ' | awk '{print $$3}' | sort

build-static/$(LIB_PREFIX)tiny_bclibc.a: cmake-configure-static
	cmake --build build-static --config Release

clean:
	rm -rf build build-shared build-static build-test
	@echo "Cleaned"

info:
	@echo "Detected OS : $(DETECTED_OS)"
	@echo "Generator   : $(CMAKE_GENERATOR_FLAG)"
	@echo "Lib prefix  : $(LIB_PREFIX)"
	@echo "Lib suffix  : $(LIB_SUFFIX)"