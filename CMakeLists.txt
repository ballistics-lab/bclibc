cmake_minimum_required(VERSION 3.13)
project(bclibc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# Platform detection
# ============================================================================
if(WIN32)
    set(WINDOWS_BUILD TRUE)
    set(PLATFORM_LIB_PREFIX "")
    set(PLATFORM_LIB_SUFFIX ".dll")
elseif(APPLE)
    set(MACOS_BUILD TRUE)
    set(PLATFORM_LIB_PREFIX "lib")
    set(PLATFORM_LIB_SUFFIX ".dylib")
else()
    set(LINUX_BUILD TRUE)
    set(PLATFORM_LIB_PREFIX "lib")
    set(PLATFORM_LIB_SUFFIX ".so")
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
        -DBCLIBC_FFI_EXPORT
        -D_WINDLL
        -DNOMINMAX
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
foreach(target bclibc_core bclibc_ffi)
    if(MSVC)
        # MSVC specific flags
        target_compile_options(${target} PRIVATE
            "$<$<CONFIG:Release>:/O2>"
            "$<$<CONFIG:Debug>:/Od /Zi>"
            /EHsc
            /std:c++17
        )
    else()
        # GCC/Clang flags
        target_compile_options(${target} PRIVATE
            "$<$<NOT:$<CONFIG:Debug>>:-O3;-ffast-math>"
            "$<$<CONFIG:Debug>:-O0;-g>"
        )
    endif()
endforeach()

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
# Export headers and libraries for Flutter
# ============================================================================
# This makes it easier for the main CMakeLists.txt to find the library
if(WINDOWS_BUILD)
    # Export include directory
    set(BCLIBC_INCLUDE_DIRS "${BCLIBC_INCLUDE_DIR}" PARENT_SCOPE)
    set(BCLIBC_LIBRARY_DIR "${CMAKE_BINARY_DIR}/bin" PARENT_SCOPE)
    
    # Create a config file for easier finding
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/bclibc-config.cmake.in"
        "${CMAKE_BINARY_DIR}/bclibc-config.cmake"
        @ONLY
    )
endif()

# macOS specific exports for symbol checking
if(MACOS_BUILD)
    # Ensure symbols are visible for checking
    set_target_properties(bclibc_ffi PROPERTIES
        LINK_FLAGS "-Wl,-exported_symbol,_BCLIBCFFI_*"
    )
endif()