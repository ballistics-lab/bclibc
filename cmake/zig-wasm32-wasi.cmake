# Toolchain file: builds bclibc as a bare WebAssembly module with zig (a C/C++ compiler and wasm32 libc, libm and
# libc++ in one download), no Emscripten and no WASI runtime needed to use it.
#
#   cmake -S . -B build/wasm -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/zig-wasm32-wasi.cmake -DBCLIBC_WASM_BARE=ON
#   cmake --build build/wasm          # -> build/wasm/bclibc_wasm.wasm
#
# zig is taken from, in this order: -DZIG=/path/to/zig, `zig` on PATH, or the `ziglang` package of Python
# (`pip install ziglang`). The target is wasm32-wasi only to get a static libc, libm and libc++: nothing from WASI is
# called, and the build fails if the module imports anything.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_TRY_COMPILE_PLATFORM_VARIABLES ZIG)  # the compiler checks read this file again and need to know where zig is
set(CMAKE_SYSTEM_PROCESSOR wasm32)

if(NOT ZIG)
    find_program(ZIG zig)
endif()
if(NOT ZIG)
    foreach(python python3 python)
        execute_process(
            COMMAND ${python} -c "import os, ziglang; print(os.path.join(os.path.dirname(ziglang.__file__), 'zig'))"
            OUTPUT_VARIABLE _ziglang_path
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
            RESULT_VARIABLE _ziglang_status
        )
        if(_ziglang_status EQUAL 0 AND EXISTS "${_ziglang_path}")
            # FORCE: find_program above left ZIG-NOTFOUND in the cache, which a plain `set(... CACHE ...)` keeps
            set(ZIG "${_ziglang_path}" CACHE FILEPATH "zig (from the ziglang package)" FORCE)
            break()
        endif()
    endforeach()
endif()
if(NOT ZIG)
    message(FATAL_ERROR "zig not found: put it on PATH, pass -DZIG=/path/to/zig, or `pip install ziglang`")
endif()

set(CMAKE_C_COMPILER   "${ZIG}")
set(CMAKE_C_COMPILER_ARG1   "cc")
set(CMAKE_CXX_COMPILER "${ZIG}")
set(CMAKE_CXX_COMPILER_ARG1 "c++")
set(CMAKE_C_COMPILER_TARGET   wasm32-wasi)
set(CMAKE_CXX_COMPILER_TARGET wasm32-wasi)
set(CMAKE_C_FLAGS_INIT   "-target wasm32-wasi")
set(CMAKE_CXX_FLAGS_INIT "-target wasm32-wasi")

# The compiler check must not try to link an executable (a reactor has no `main`).
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXECUTABLE_SUFFIX ".wasm")
