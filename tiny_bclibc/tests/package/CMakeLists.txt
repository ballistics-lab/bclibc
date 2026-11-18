# A consumer of the installed tiny_bclibc package (find_package(tiny_bclibc)). TINY_TARGET is one of the short names
# of the README: headers, baremetal, static, shared. Built by cmake/check_package.sh without CMAKE_BUILD_TYPE (-O0).
cmake_minimum_required(VERSION 3.13)
project(tiny_bclibc_package_consumer C)

set(TINY_TARGET headers CACHE STRING "tiny_bclibc:: target to link")
find_package(tiny_bclibc REQUIRED CONFIG)

add_executable(use_tiny use_tiny.c)
target_link_libraries(use_tiny PRIVATE tiny_bclibc::${TINY_TARGET})
