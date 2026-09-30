#!/usr/bin/env bash
# Guards the "bclibc never throws" contract natively, not only in the wasm build.
#
# Builds the core and its C ABI with -fno-exceptions -fno-rtti (so a `throw`, `try` or `catch` in our code is already a
# compile error), then fails if any object still references the C++ exception runtime or RTTI, which is what an
# indirect regression (a helper that throws, `dynamic_cast`, `typeid`) would leave behind. libstdc++'s own
# `__throw_length_error` and friends are not matched: they come from std::vector/std::function, not from our code.
#
#   cmake/check_no_exceptions.sh [build-dir]
set -euo pipefail

build="${1:-build/no-exceptions}"

cmake -S . -B "$build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-fno-exceptions -fno-rtti"
cmake --build "$build" --target bclibc_core bclibc_ffi

pattern='__cxa_throw|__cxa_allocate_exception|__cxa_begin_catch|__cxa_rethrow|_Unwind_|__dynamic_cast|typeinfo for|bad_variant_access'
found=0
while IFS= read -r object; do
    if nm -C "$object" 2>/dev/null | grep " U " | grep -E "$pattern"; then
        echo "error: $object references the exception runtime or RTTI" >&2
        found=1
    fi
done < <(find "$build" -name '*.o')

if [ "$found" -ne 0 ]; then
    exit 1
fi
echo "OK: no exception runtime or RTTI in the core"
