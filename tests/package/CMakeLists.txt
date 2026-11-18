# A consumer of the installed bclibc package (find_package(bclibc)): the C ABI target and the C++ core target.
# Built by cmake/check_package.sh, with no CMAKE_BUILD_TYPE on purpose (-O0 keeps code an optimiser would drop).
cmake_minimum_required(VERSION 3.16)
project(bclibc_package_consumer C CXX)

find_package(bclibc REQUIRED CONFIG)

add_executable(use_ffi use_ffi.c)
target_link_libraries(use_ffi PRIVATE bclibc::bclibc_ffi)

add_executable(use_core use_core.cpp)
target_link_libraries(use_core PRIVATE bclibc::bclibc_core)
target_compile_features(use_core PRIVATE cxx_std_17)
