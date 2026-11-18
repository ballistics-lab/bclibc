#!/usr/bin/env bash
# The installed packages work for a consumer: bclibc and tiny_bclibc are built, installed into a scratch prefix, and a
# small project finds each with find_package() and builds, links and runs against it.
#
# This covers what an add_subdirectory() build never touches: the install rules (every header the public headers
# include), the include directories carried by the exported targets, the short target names of the README, and libm.
# The consumers get no CMAKE_BUILD_TYPE, i.e. -O0, where unused inline code is still emitted.
#
#   cmake/check_package.sh [work-dir]
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK="${1:-$(mktemp -d)}"
JOBS="$( (nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null) || echo 2)"
mkdir -p "$WORK"
echo ">>> work directory: $WORK"

fail() { echo "FAIL: $*" >&2; exit 1; }

run_consumer() { # <name> <executable> <prefix>/lib
    LD_LIBRARY_PATH="$3${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" DYLD_LIBRARY_PATH="$3${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}" \
        "$2" || fail "$1: the consumer did not run"
}

# --- bclibc: the C ABI shared library and the C++ core ---------------------------------------------------------
echo ">>> bclibc: build and install"
cmake -S "$ROOT" -B "$WORK/bclibc" -DCMAKE_BUILD_TYPE=Release > "$WORK/bclibc.log" 2>&1 || { cat "$WORK/bclibc.log"; fail "bclibc configure"; }
cmake --build "$WORK/bclibc" -j"$JOBS" >> "$WORK/bclibc.log" 2>&1 || { tail -n 40 "$WORK/bclibc.log"; fail "bclibc build"; }
cmake --install "$WORK/bclibc" --prefix "$WORK/inst/bclibc" > /dev/null || fail "bclibc install"

# every header of the tree must be installed (version.h is generated, not in the tree)
(cd "$ROOT/include" && find . -type f | sort) > "$WORK/headers.src"
(cd "$WORK/inst/bclibc/include" && find . -type f | grep -v '^./bclibc/version.h$' | sort) > "$WORK/headers.inst"
diff "$WORK/headers.src" "$WORK/headers.inst" > /dev/null || { diff "$WORK/headers.src" "$WORK/headers.inst" || true; fail "bclibc: headers missing from the install"; }

echo ">>> bclibc: find_package consumer"
cmake -S "$ROOT/tests/package" -B "$WORK/consumer-bclibc" -DCMAKE_PREFIX_PATH="$WORK/inst/bclibc" > "$WORK/consumer-bclibc.log" 2>&1 || { cat "$WORK/consumer-bclibc.log"; fail "bclibc consumer configure"; }
cmake --build "$WORK/consumer-bclibc" >> "$WORK/consumer-bclibc.log" 2>&1 || { tail -n 40 "$WORK/consumer-bclibc.log"; fail "bclibc consumer build"; }
run_consumer "bclibc ffi" "$WORK/consumer-bclibc/use_ffi" "$WORK/inst/bclibc/lib"
run_consumer "bclibc core" "$WORK/consumer-bclibc/use_core" "$WORK/inst/bclibc/lib"

# --- tiny_bclibc: each way of building it, each target name of the README ---------------------------------------
build_tiny() { # <name> <cmake options...>
    local name="$1"; shift
    cmake -S "$ROOT/tiny_bclibc" -B "$WORK/tiny-$name" -DCMAKE_BUILD_TYPE=Release "$@" > "$WORK/tiny-$name.log" 2>&1 || { cat "$WORK/tiny-$name.log"; fail "tiny_bclibc [$name] configure"; }
    cmake --build "$WORK/tiny-$name" -j"$JOBS" >> "$WORK/tiny-$name.log" 2>&1 || { tail -n 40 "$WORK/tiny-$name.log"; fail "tiny_bclibc [$name] build"; }
    cmake --install "$WORK/tiny-$name" --prefix "$WORK/inst/tiny-$name" > /dev/null || fail "tiny_bclibc [$name] install"
}

tiny_consumer() { # <install name> <target> [cmake options...]
    local name="$1" target="$2"; shift 2
    local dir="$WORK/consumer-tiny-$name-$target"
    cmake -S "$ROOT/tiny_bclibc/tests/package" -B "$dir" -DCMAKE_PREFIX_PATH="$WORK/inst/tiny-$name" -DTINY_TARGET="$target" "$@" > "$dir.log" 2>&1 || { cat "$dir.log"; fail "tiny_bclibc [$name] consumer of $target: configure"; }
    cmake --build "$dir" >> "$dir.log" 2>&1 || { tail -n 40 "$dir.log"; fail "tiny_bclibc [$name] consumer of $target: build"; }
    run_consumer "tiny_bclibc [$name] $target" "$dir/use_tiny" "$WORK/inst/tiny-$name/lib"
}

echo ">>> tiny_bclibc: build, install, find_package consumers"
build_tiny headers
tiny_consumer headers headers
tiny_consumer headers baremetal
build_tiny shared -DTINY_BCLIBC_BUILD_SHARED=ON
tiny_consumer shared shared
build_tiny static -DTINY_BCLIBC_BUILD_STATIC=ON
tiny_consumer static static
build_tiny both-sp -DTINY_BCLIBC_BUILD_SHARED=ON -DTINY_BCLIBC_BUILD_STATIC=ON -DTINY_BCLIBC_SINGLE_PRECISION=ON
tiny_consumer both-sp shared
tiny_consumer both-sp static

echo ">>> OK: the installed packages work for consumers"
