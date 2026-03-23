cmake_minimum_required(VERSION 3.13)
project(bclibc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# Dynamic Versioning from Git
# ============================================================================
find_package(Git QUIET)

# Default values for development if no git or tags found
set(BCLIBC_VERSION_MAJOR 0)
set(BCLIBC_VERSION_MINOR 1)
set(BCLIBC_VERSION_PATCH 0)
set(BCLIBC_VERSION_STR "${BCLIBC_VERSION_MAJOR}.${BCLIBC_VERSION_MINOR}.${BCLIBC_VERSION_PATCH}-dev")

if(GIT_FOUND)
    # Get describe info. If no tags, it falls back to commit hash
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --always --dirty
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_TAG
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    if(GIT_TAG)
        # Check if the tag looks like a version (v1.2.3 or 1.2.3)
        if(GIT_TAG MATCHES "^v?([0-9]+)\\.([0-9]+)\\.([0-9]+)")
            set(BCLIBC_VERSION_MAJOR ${CMAKE_MATCH_1})
            set(BCLIBC_VERSION_MINOR ${CMAKE_MATCH_2})
            set(BCLIBC_VERSION_PATCH ${CMAKE_MATCH_3})
            string(REGEX REPLACE "^v" "" BCLIBC_VERSION_STR "${GIT_TAG}")
        else()
            # If it's just a hash (no tags yet), use the dev base + hash
            set(BCLIBC_VERSION_STR "0.1.0-git-${GIT_TAG}")
        endif()
    endif()
endif()

# We use BCLIBC_VERSION for the configure_file to match version.h.in
set(BCLIBC_VERSION "${BCLIBC_VERSION_STR}")

message(STATUS "Detected Project Version: ${BCLIBC_VERSION}")

# Generate version.h from template
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/version.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/bclibc/version.h"
)

# ============================================================================
# Paths
# ============================================================================
set(BCLIBC_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(BCLIBC_SRC_DIR     "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Core only, ignore ffi directory
file(GLOB_RECURSE BCLIBC_SOURCES "${BCLIBC_SRC_DIR}/*.cpp")
list(FILTER BCLIBC_SOURCES EXCLUDE REGEX "src/ffi/.*")

# ============================================================================
# 1. Target: Pure core (Static Library)
# ============================================================================
add_library(bclibc_core STATIC ${BCLIBC_SOURCES})

target_include_directories(bclibc_core PUBLIC "${BCLIBC_INCLUDE_DIR}")
target_include_directories(bclibc_core PRIVATE "${BCLIBC_SRC_DIR}")

set_target_properties(bclibc_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ============================================================================
# 2. Target: FFI wrapper (Shared Library)
# ============================================================================
add_library(bclibc_ffi SHARED "${BCLIBC_SRC_DIR}/ffi/bclibc_ffi.cpp")

# Add generated folder to include paths for FFI
target_include_directories(bclibc_ffi PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Set version properties for the .so file
# Use numeric components for SOVERSION to ensure valid symlinks
set_target_properties(bclibc_ffi PROPERTIES 
    VERSION ${BCLIBC_VERSION_MAJOR}.${BCLIBC_VERSION_MINOR}.${BCLIBC_VERSION_PATCH}
    SOVERSION ${BCLIBC_VERSION_MAJOR}
)

# Link FFI with the core
target_link_libraries(bclibc_ffi PRIVATE bclibc_core)

# Optimizations
foreach(target bclibc_core bclibc_ffi)
    target_compile_options(${target} PRIVATE
        "$<$<NOT:$<CONFIG:Debug>>:-O3;-ffast-math>"
        "$<$<CONFIG:Debug>:-O0;-g>"
    )
endforeach()

# ============================================================================
# Symbol Export Control (Only for FFI)
# ============================================================================
if(UNIX AND NOT APPLE)
    set(VERSION_SCRIPT "${BCLIBC_SRC_DIR}/ffi/bclibc_ffi.version")
    if(NOT CMAKE_BUILD_TYPE MATCHES "Debug")
        target_link_options(bclibc_ffi PRIVATE "-Wl,--version-script=${VERSION_SCRIPT}")
    endif()
endif()