# Toolchain file: builds bclibc as a bare WebAssembly module with wasi-sdk (clang and a wasm32 libc, libm, libc++ with
# C++ exceptions), no Emscripten. Unlike zig, wasi-sdk has the runtime for exceptions, so the core keeps its
# `throw`/`catch` and the flat C ABI returns the same error codes as the native library.
#
#   cmake -S . -B build/wasm -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/wasi-sdk-wasm32.cmake -DBCLIBC_WASM_BARE=ON
#   cmake --build build/wasm          # -> build/wasm/bclibc_wasm.wasm
#
# wasi-sdk (https://github.com/WebAssembly/wasi-sdk/releases, version 34 tested) is taken from -DWASI_SDK_PATH=... or the
# WASI_SDK_PATH environment variable. Its libraries use the final WebAssembly exception encoding (`try_table`), so
# the module needs a host that has it: wasmtime, wasm3 (recent), Node 24+, recent Safari/iOS. The target is wasm32-wasi
# only to get a static libc, libm and libc++: nothing from WASI is called, and the build fails if the module imports
# anything.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR wasm32)
set(CMAKE_TRY_COMPILE_PLATFORM_VARIABLES WASI_SDK_PATH)  # the compiler checks read this file again

if(NOT WASI_SDK_PATH AND DEFINED ENV{WASI_SDK_PATH})
    set(WASI_SDK_PATH "$ENV{WASI_SDK_PATH}")
endif()
if(NOT WASI_SDK_PATH OR NOT EXISTS "${WASI_SDK_PATH}/bin/clang++")
    message(FATAL_ERROR "wasi-sdk not found: pass -DWASI_SDK_PATH=/path/to/wasi-sdk-XX (or set WASI_SDK_PATH)")
endif()
set(WASI_SDK_PATH "${WASI_SDK_PATH}" CACHE PATH "wasi-sdk")

set(CMAKE_C_COMPILER   "${WASI_SDK_PATH}/bin/clang")
set(CMAKE_CXX_COMPILER "${WASI_SDK_PATH}/bin/clang++")
set(CMAKE_AR           "${WASI_SDK_PATH}/bin/llvm-ar")
set(CMAKE_RANLIB       "${WASI_SDK_PATH}/bin/llvm-ranlib")
set(CMAKE_C_COMPILER_TARGET   wasm32-wasip1)
set(CMAKE_CXX_COMPILER_TARGET wasm32-wasip1)
set(CMAKE_SYSROOT "${WASI_SDK_PATH}/share/wasi-sysroot")

# The compiler check must not try to link an executable (a reactor has no `main`).
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXECUTABLE_SUFFIX ".wasm")

# This toolchain has the exception runtime: the bare build uses it (BclibcWasmBare.cmake).
set(BCLIBC_WASM_EXCEPTIONS_DEFAULT ON)
