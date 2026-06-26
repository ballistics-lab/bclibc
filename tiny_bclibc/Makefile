.PHONY: all headers shared static test-identity version symbols symbols-static clean info \
        cmake-configure cmake-configure-shared cmake-configure-static cmake-configure-test \
        shared-sp static-sp

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

# ── User-configurable options ─────────────────────────────────────
# Передаються в CMake. Використання:
#   make shared TINY_BCLIBC_SINGLE_PRECISION=ON
#   make shared TINY_BCLIBC_SINGLE_PRECISION=OFF (за замовчуванням)
TINY_BCLIBC_SINGLE_PRECISION ?= OFF
TINY_BCLIBC_NO_THREAD_LOCAL ?= OFF
TINY_BCLIBC_NO_ERR_BUF ?= OFF

all: headers

# ── CMake configure ────────────────────────────────────────────────
cmake-configure:
	mkdir -p build
	cd build && cmake $(CMAKE_GENERATOR_FLAG) -DCMAKE_BUILD_TYPE=Release \
	    -DTINY_BCLIBC_SINGLE_PRECISION=$(TINY_BCLIBC_SINGLE_PRECISION) \
	    -DTINY_BCLIBC_NO_THREAD_LOCAL=$(TINY_BCLIBC_NO_THREAD_LOCAL) \
	    -DTINY_BCLIBC_NO_ERR_BUF=$(TINY_BCLIBC_NO_ERR_BUF) \
	    -DTINY_BCLIBC_INSTALL=OFF ..
	@echo "CMake configured (headers mode) for $(DETECTED_OS)"
	@echo "  TINY_BCLIBC_SINGLE_PRECISION = $(TINY_BCLIBC_SINGLE_PRECISION)"
	@echo "  TINY_BCLIBC_NO_THREAD_LOCAL   = $(TINY_BCLIBC_NO_THREAD_LOCAL)"
	@echo "  TINY_BCLIBC_NO_ERR_BUF        = $(TINY_BCLIBC_NO_ERR_BUF)"

cmake-configure-shared:
	mkdir -p build-shared
	cd build-shared && cmake $(CMAKE_GENERATOR_FLAG) -DCMAKE_BUILD_TYPE=Release \
	    -DTINY_BCLIBC_BUILD_SHARED=ON \
	    -DTINY_BCLIBC_SINGLE_PRECISION=$(TINY_BCLIBC_SINGLE_PRECISION) \
	    -DTINY_BCLIBC_NO_THREAD_LOCAL=$(TINY_BCLIBC_NO_THREAD_LOCAL) \
	    -DTINY_BCLIBC_NO_ERR_BUF=$(TINY_BCLIBC_NO_ERR_BUF) \
	    -DTINY_BCLIBC_INSTALL=OFF ..
	@echo "CMake configured (shared) for $(DETECTED_OS)"
	@echo "  TINY_BCLIBC_SINGLE_PRECISION = $(TINY_BCLIBC_SINGLE_PRECISION)"

cmake-configure-static:
	mkdir -p build-static
	cd build-static && cmake $(CMAKE_GENERATOR_FLAG) -DCMAKE_BUILD_TYPE=Release \
	    -DTINY_BCLIBC_BUILD_STATIC=ON \
	    -DTINY_BCLIBC_SINGLE_PRECISION=$(TINY_BCLIBC_SINGLE_PRECISION) \
	    -DTINY_BCLIBC_NO_THREAD_LOCAL=$(TINY_BCLIBC_NO_THREAD_LOCAL) \
	    -DTINY_BCLIBC_NO_ERR_BUF=$(TINY_BCLIBC_NO_ERR_BUF) \
	    -DTINY_BCLIBC_INSTALL=OFF ..
	@echo "CMake configured (static) for $(DETECTED_OS)"
	@echo "  TINY_BCLIBC_SINGLE_PRECISION = $(TINY_BCLIBC_SINGLE_PRECISION)"

cmake-configure-test:
	mkdir -p build-test
	cd build-test && cmake $(CMAKE_GENERATOR_FLAG) -DCMAKE_BUILD_TYPE=Debug \
	    -DTINY_BCLIBC_BUILD_IDENTITY_TEST=ON \
	    -DTINY_BCLIBC_SINGLE_PRECISION=$(TINY_BCLIBC_SINGLE_PRECISION) \
	    -DTINY_BCLIBC_NO_THREAD_LOCAL=$(TINY_BCLIBC_NO_THREAD_LOCAL) \
	    -DTINY_BCLIBC_NO_ERR_BUF=$(TINY_BCLIBC_NO_ERR_BUF) \
	    -DTINY_BCLIBC_INSTALL=OFF ..
	@echo "CMake configured (identity test) for $(DETECTED_OS)"

# ── Build targets ──────────────────────────────────────────────────
headers: cmake-configure
	@echo "Header-only target — tiny_bclibc::headers is INTERFACE (nothing to compile)"
	@echo "Generated: build/generated/tiny_bclibc/version.h"

shared: cmake-configure-shared
	cmake --build build-shared --config Release -j$(NPROC)
	@echo "Shared library: build-shared/$(LIB_PREFIX)tiny_bclibc$(LIB_SUFFIX)"
	@echo "  Precision: $(if $(filter ON,$(TINY_BCLIBC_SINGLE_PRECISION)),single,double)"

# Single precision shared (окрема ціль для зручності)
shared-sp:
	$(MAKE) shared TINY_BCLIBC_SINGLE_PRECISION=ON

static: cmake-configure-static
	cmake --build build-static --config Release -j$(NPROC)
	@echo "Static library: build-static/$(LIB_PREFIX)tiny_bclibc.a"
	@echo "  Precision: $(if $(filter ON,$(TINY_BCLIBC_SINGLE_PRECISION)),single,double)"

# Single precision static (окрема ціль для зручності)
static-sp:
	$(MAKE) static TINY_BCLIBC_SINGLE_PRECISION=ON

test-identity: cmake-configure-test
	cmake --build build-test -- -j$(NPROC)
	build-test/tests/test_identity
	@echo "Identity test passed"

# ── Utils ────────────────────────────────────────────────────────
version: cmake-configure
	@grep "TINY_BCLIBC_VERSION" build/generated/tiny_bclibc/version.h 2>/dev/null || \
	    echo "(version.h not generated yet — run 'make headers' first)"

symbols: build-shared/$(LIB_PREFIX)tiny_bclibc$(LIB_SUFFIX)
	@echo "Exported symbols (shared library):"
	nm -D --defined-only build-shared/$(LIB_PREFIX)tiny_bclibc$(LIB_SUFFIX) \
	    | grep ' T ' | awk '{print $$3}' | sort

build-shared/$(LIB_PREFIX)tiny_bclibc$(LIB_SUFFIX): cmake-configure-shared
	cmake --build build-shared --config Release -j$(NPROC)

symbols-static: build-static/$(LIB_PREFIX)tiny_bclibc.a
	@echo "Exported symbols (static library):"
	nm --defined-only build-static/$(LIB_PREFIX)tiny_bclibc.a \
	    | grep ' T ' | awk '{print $$3}' | sort

build-static/$(LIB_PREFIX)tiny_bclibc.a: cmake-configure-static
	cmake --build build-static --config Release -j$(NPROC)

clean:
	rm -rf build build-shared build-static build-test
	@echo "Cleaned"

clean-all: clean
	rm -rf build-shared-sp build-static-sp
	@echo "All cleaned"

info:
	@echo "Detected OS : $(DETECTED_OS)"
	@echo "Generator   : $(CMAKE_GENERATOR_FLAG)"
	@echo "Lib prefix  : $(LIB_PREFIX)"
	@echo "Lib suffix  : $(LIB_SUFFIX)"
	@echo "NPROC       : $(NPROC)"
	@echo ""
	@echo "Available targets:"
	@echo "  headers     - Header-only mode (nothing compiled)"
	@echo "  shared      - Shared library (double precision by default)"
	@echo "  shared-sp   - Shared library (single precision)"
	@echo "  static      - Static library (double precision by default)"
	@echo "  static-sp   - Static library (single precision)"
	@echo "  test-identity - Run identity test"
	@echo "  symbols     - List exported symbols (shared)"
	@echo "  symbols-static - List exported symbols (static)"
	@echo "  version     - Show version"
	@echo "  clean       - Remove build directories"
	@echo "  info        - Show this info"
	@echo ""
	@echo "Override precision:"
	@echo "  make shared TINY_BCLIBC_SINGLE_PRECISION=ON"
	@echo "  make static TINY_BCLIBC_SINGLE_PRECISION=OFF"
