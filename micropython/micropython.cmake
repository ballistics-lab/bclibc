# bclibc external C module for MicroPython firmware builds.
#
# Use this when you want to compile bclibc directly into the MicroPython
# firmware image rather than loading it as a separate .mpy file.
# For most use cases the pre-built .mpy from the Releases page is simpler.
#
# Usage:
#   cmake -B build \
#     -DUSER_C_MODULES=/path/to/bclibc/micropython/micropython.cmake \
#     ...other MicroPython cmake args...

cmake_minimum_required(VERSION 3.13)

set(_BCLIBC_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

add_library(usermod_bclibc INTERFACE)

target_sources(usermod_bclibc INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/natmod/bclibc_mp.c
    ${_BCLIBC_DIR}/src/base_types.cpp
    ${_BCLIBC_DIR}/src/engine.cpp
    ${_BCLIBC_DIR}/src/euler.cpp
    ${_BCLIBC_DIR}/src/interp.cpp
    ${_BCLIBC_DIR}/src/rk4.cpp
    ${_BCLIBC_DIR}/src/traj_data.cpp
    ${_BCLIBC_DIR}/src/traj_filter.cpp
    ${_BCLIBC_DIR}/src/ffi/bclibc_ffi.cpp
)

target_include_directories(usermod_bclibc INTERFACE
    ${_BCLIBC_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/natmod/generated
)

target_compile_options(usermod_bclibc INTERFACE
    $<$<COMPILE_LANGUAGE:CXX>:
        -fno-exceptions
        -fno-rtti
        -std=c++17
    >
)

# operator new/delete backed by MicroPython's m_malloc.
# When bclibc is compiled into the firmware (not as a natmod), the
# standard operator new/delete from libstdc++ are available.  We only
# need the stubs in the natmod (.mpy) context where libstdc++ is absent.
# So we do NOT include cxx_stubs.cpp here.

target_link_libraries(usermod INTERFACE usermod_bclibc)
