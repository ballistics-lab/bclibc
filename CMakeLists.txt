cmake_minimum_required(VERSION 3.13)
project(bclibc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# Options
# ============================================================================
option(BCLIBC_BUILD_TESTS "Build tests" OFF)

# ============================================================================
# Platform detection
# ============================================================================
if(WIN32)
    set(WINDOWS_BUILD TRUE)
elseif(APPLE)
    set(MACOS_BUILD TRUE)
else()
    set(LINUX_BUILD TRUE)
endif()

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
        COMMAND ${GIT_EXECUTABLE} describe --tags --always
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

set(BCLIBC_SOURCES
    "${BCLIBC_SRC_DIR}/base_types.cpp"
    "${BCLIBC_SRC_DIR}/engine.cpp"
    "${BCLIBC_SRC_DIR}/euler.cpp"
    "${BCLIBC_SRC_DIR}/interp.cpp"
    "${BCLIBC_SRC_DIR}/rk4.cpp"
    "${BCLIBC_SRC_DIR}/traj_data.cpp"
    "${BCLIBC_SRC_DIR}/traj_filter.cpp"
)

# ============================================================================
# 1. Target: Pure core (Static Library)
# ============================================================================
add_library(bclibc_core STATIC ${BCLIBC_SOURCES})

target_include_directories(bclibc_core PUBLIC
    "$<BUILD_INTERFACE:${BCLIBC_INCLUDE_DIR}>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)
target_include_directories(bclibc_core PRIVATE "${BCLIBC_SRC_DIR}")

set_target_properties(bclibc_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

# ============================================================================
# 2. Target: FFI wrapper (Shared Library)
# ============================================================================
add_library(bclibc_ffi SHARED "${BCLIBC_SRC_DIR}/ffi/bclibc_ffi.cpp")

# Add generated folder to include paths for FFI
target_include_directories(bclibc_ffi PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_include_directories(bclibc_ffi PRIVATE "${BCLIBC_INCLUDE_DIR}")

# Platform-specific settings for FFI library
if(WINDOWS_BUILD)
    # Windows specific settings
    set_target_properties(bclibc_ffi PROPERTIES
        PREFIX ""  # No "lib" prefix on Windows
        SUFFIX ".dll"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    )
    
    # Export symbols for Windows
    target_compile_definitions(bclibc_ffi PRIVATE
        BCLIBC_FFI_EXPORT
        _WINDLL
        NOMINMAX
    )
    
    # Disable specific MSVC warnings
    if(MSVC)
        target_compile_options(bclibc_ffi PRIVATE 
            /W4
            /wd"4251"  # Disable dll interface warning
            /wd"4275"  # Disable non dll-interface class warning
        )
    endif()
elseif(MACOS_BUILD)
    set_target_properties(bclibc_ffi PROPERTIES
        PREFIX "lib"
        SUFFIX ".dylib"
        INSTALL_NAME_DIR "@rpath"
    )
    
    # Set version properties for macOS
    set_target_properties(bclibc_ffi PROPERTIES 
        VERSION ${BCLIBC_VERSION_MAJOR}.${BCLIBC_VERSION_MINOR}.${BCLIBC_VERSION_PATCH}
        SOVERSION ${BCLIBC_VERSION_MAJOR}
    )
else()
    # Linux/Unix settings
    set_target_properties(bclibc_ffi PROPERTIES 
        PREFIX "lib"
        SUFFIX ".so"
        VERSION ${BCLIBC_VERSION_MAJOR}.${BCLIBC_VERSION_MINOR}.${BCLIBC_VERSION_PATCH}
        SOVERSION ${BCLIBC_VERSION_MAJOR}
    )
endif()

# Link FFI with the core
target_link_libraries(bclibc_ffi PRIVATE bclibc_core)

# ============================================================================
# Compiler optimizations and flags
# ============================================================================
include(CheckIPOSupported)
check_ipo_supported(RESULT IPO_SUPPORTED OUTPUT IPO_ERROR)

foreach(target bclibc_core bclibc_ffi)
    if(MSVC)
        # MSVC specific flags
        target_compile_options(${target} PRIVATE
            "$<$<CONFIG:Release>:/O2>"
            "$<$<CONFIG:Debug>:/Od>"
            "$<$<CONFIG:Debug>:/Zi>"
            /EHsc
            /std:c++17
        )
    else()
        # GCC/Clang flags
        # -ffast-math was removed: for RK4/Euler integrators it breaks
        # IEEE-754 semantics (reordering FP operations, no-NaN/Inf assumptions)
        # and introduces discrepancies with the Debug build. -ffp-contract=off
        # preserves strict IEEE arithmetic even when dealing with FMA contraction.
        target_compile_options(${target} PRIVATE
            "$<$<NOT:$<CONFIG:Debug>>:-O3;-ffunction-sections;-fdata-sections;-ffp-contract=off>"
            "$<$<CONFIG:Debug>:-O0;-g>"
        )
    endif()

    if(IPO_SUPPORTED)
        set_target_properties(${target} PROPERTIES
            INTERPROCEDURAL_OPTIMIZATION_RELEASE ON
        )
    endif()
endforeach()

# Linker: dead code elimination + strip in Release.
# Does not affect calculation results — only reduces the binary size.
if(MSVC)
    target_link_options(bclibc_ffi PRIVATE
        "$<$<CONFIG:Release>:/OPT:REF;/OPT:ICF>"
    )
elseif(MACOS_BUILD)
    target_link_options(bclibc_ffi PRIVATE
        "$<$<NOT:$<CONFIG:Debug>>:-Wl,-dead_strip;-Wl,-x>"
    )
else()
    target_link_options(bclibc_ffi PRIVATE
        "$<$<NOT:$<CONFIG:Debug>>:-Wl,--gc-sections;-Wl,-strip-all>"
    )
endif()

# ============================================================================
# Symbol Export Control (Only for FFI) - Unix/Linux only
# ============================================================================
if(LINUX_BUILD AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
    set(VERSION_SCRIPT "${BCLIBC_SRC_DIR}/ffi/bclibc_ffi.version")
    if(EXISTS ${VERSION_SCRIPT})
        target_link_options(bclibc_ffi PRIVATE "-Wl,--version-script=${VERSION_SCRIPT}")
    endif()
endif()


# ============================================================================
# Installation rules
# Only install when bclibc is the top-level project (e.g. Flatpak cmake-ninja
# standalone build). When used via add_subdirectory from a Flutter plugin the
# parent build owns the install step and bclibc's package config files are not
# needed — and running them breaks the Flutter cmake_install phase.
# ============================================================================
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  include(GNUInstallDirs)
  include(CMakePackageConfigHelpers)

  install(TARGETS bclibc_core bclibc_ffi
      EXPORT bclibc-targets
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )

  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/bclibc/version.h"
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/bclibc
  )

  install(FILES "${BCLIBC_INCLUDE_DIR}/bclibc.hpp"
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )

  install(FILES
      "${BCLIBC_INCLUDE_DIR}/bclibc/base_types.hpp"
      "${BCLIBC_INCLUDE_DIR}/bclibc/engine.hpp"
      "${BCLIBC_INCLUDE_DIR}/bclibc/euler.hpp"
      "${BCLIBC_INCLUDE_DIR}/bclibc/exceptions.hpp"
      "${BCLIBC_INCLUDE_DIR}/bclibc/interp.hpp"
      "${BCLIBC_INCLUDE_DIR}/bclibc/log.hpp"
      "${BCLIBC_INCLUDE_DIR}/bclibc/rk4.hpp"
      "${BCLIBC_INCLUDE_DIR}/bclibc/scope_guard.hpp"
      "${BCLIBC_INCLUDE_DIR}/bclibc/traj_data.hpp"
      "${BCLIBC_INCLUDE_DIR}/bclibc/traj_filter.hpp"
      "${BCLIBC_INCLUDE_DIR}/bclibc/v3d.hpp"
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/bclibc
  )

  install(FILES "${BCLIBC_INCLUDE_DIR}/bclibc/ffi/bclibc_ffi.h"
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/bclibc/ffi
  )

  install(EXPORT bclibc-targets
      FILE bclibc-targets.cmake
      NAMESPACE bclibc::
      DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/bclibc
  )

  write_basic_package_version_file(
      "${CMAKE_CURRENT_BINARY_DIR}/bclibc-config-version.cmake"
      VERSION ${BCLIBC_VERSION_MAJOR}.${BCLIBC_VERSION_MINOR}.${BCLIBC_VERSION_PATCH}
      COMPATIBILITY SameMajorVersion
  )

  configure_package_config_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/bclibc-config.cmake.in"
      "${CMAKE_CURRENT_BINARY_DIR}/bclibc-config.cmake"
      INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/bclibc
  )

  install(FILES
      "${CMAKE_CURRENT_BINARY_DIR}/bclibc-config.cmake"
      "${CMAKE_CURRENT_BINARY_DIR}/bclibc-config-version.cmake"
      DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/bclibc
  )
endif()
