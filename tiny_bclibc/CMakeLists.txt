cmake_minimum_required(VERSION 3.13)

# ── Версія: з git-тегу або fallback ────────────────────────────────
set(TINY_BCLIBC_VERSION_MAJOR 1)
set(TINY_BCLIBC_VERSION_MINOR 0)
set(TINY_BCLIBC_VERSION_PATCH 0)
set(TINY_BCLIBC_VERSION "${TINY_BCLIBC_VERSION_MAJOR}.${TINY_BCLIBC_VERSION_MINOR}.${TINY_BCLIBC_VERSION_PATCH}-dev")

find_package(Git QUIET)
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --always
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_TAG
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    if(GIT_TAG MATCHES "^v?([0-9]+)\\.([0-9]+)\\.([0-9]+)")
        set(TINY_BCLIBC_VERSION_MAJOR ${CMAKE_MATCH_1})
        set(TINY_BCLIBC_VERSION_MINOR ${CMAKE_MATCH_2})
        set(TINY_BCLIBC_VERSION_PATCH ${CMAKE_MATCH_3})
        string(REGEX REPLACE "^v" "" TINY_BCLIBC_VERSION "${GIT_TAG}")
    elseif(GIT_TAG)
        set(TINY_BCLIBC_VERSION "0.0.0-git-${GIT_TAG}")
    endif()
endif()

project(tiny_bclibc
    VERSION ${TINY_BCLIBC_VERSION_MAJOR}.${TINY_BCLIBC_VERSION_MINOR}.${TINY_BCLIBC_VERSION_PATCH}
    LANGUAGES C)

message(STATUS "tiny_bclibc version: ${TINY_BCLIBC_VERSION}")

# ── Опції ──────────────────────────────────────────────────────────
option(TINY_BCLIBC_SINGLE_PRECISION   "Use float instead of double"        OFF)
option(TINY_BCLIBC_USE_FLOAT          "Use float instead of double (deprecated, use TINY_BCLIBC_SINGLE_PRECISION)" OFF)
option(TINY_BCLIBC_BUILD_SHARED       "Build shared library"               OFF)
option(TINY_BCLIBC_BUILD_STATIC       "Build static library"               OFF)
option(TINY_BCLIBC_INSTALL            "Install targets and headers"        ON)
option(TINY_BCLIBC_BUILD_IDENTITY_TEST "Build bclibc↔tiny_bclibc identity test" OFF)

# ── Генерація version.h ────────────────────────────────────────────
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/version.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/tiny_bclibc/version.h"
    @ONLY)

# ── Header-only INTERFACE target ────────────────────────────────────
add_library(tiny_bclibc_headers INTERFACE)
add_library(tiny_bclibc::headers ALIAS tiny_bclibc_headers)

target_include_directories(tiny_bclibc_headers INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:include>)

if(TINY_BCLIBC_SINGLE_PRECISION OR TINY_BCLIBC_USE_FLOAT)
    target_compile_definitions(tiny_bclibc_headers INTERFACE TINY_BCLIBC_SINGLE_PRECISION)
endif()

# ── Спільна функція налаштування compile-target ────────────────────
function(tiny_bclibc_configure_target tgt)
    target_include_directories(${tgt} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
        $<INSTALL_INTERFACE:include>)
    target_compile_definitions(${tgt} PRIVATE TINY_BCLIBC_BUILD_SHARED)
    if(TINY_BCLIBC_SINGLE_PRECISION OR TINY_BCLIBC_USE_FLOAT)
        target_compile_definitions(${tgt} PUBLIC TINY_BCLIBC_SINGLE_PRECISION)
    endif()
    set_target_properties(${tgt} PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED ON
        VERSION   ${TINY_BCLIBC_VERSION_MAJOR}.${TINY_BCLIBC_VERSION_MINOR}.${TINY_BCLIBC_VERSION_PATCH}
        SOVERSION ${TINY_BCLIBC_VERSION_MAJOR})
    if(NOT WIN32)
        target_link_libraries(${tgt} PRIVATE m)
        # -Os: size-first (no loop unrolling, shorter sequences) — matches embedded toolchain.
        #      TINY_BCLIBC_INTERNAL handles the critical noinline guarantee explicitly,
        #      so -Os is safe: no duplicate rk4/filter, arithmetic still well-optimized.
        # -fvisibility=hidden: only TINY_BCLIBC_FUNC symbols exported; internals hidden.
        target_compile_options(${tgt} PRIVATE -Os -fvisibility=hidden)
        # Strip unneeded symbols in Release builds
        target_link_options(${tgt} PRIVATE
            $<$<CONFIG:Release>:-Wl,--strip-all>)
    endif()
endfunction()

# ── Shared library ─────────────────────────────────────────────────
if(TINY_BCLIBC_BUILD_SHARED)
    add_library(tiny_bclibc SHARED src/tiny_bclibc_impl.c)
    add_library(tiny_bclibc::tiny_bclibc ALIAS tiny_bclibc)
    tiny_bclibc_configure_target(tiny_bclibc)
    if(WIN32)
        target_compile_definitions(tiny_bclibc PUBLIC TINY_BCLIBC_USE_SHARED)
    endif()
endif()

# ── Static library ─────────────────────────────────────────────────
if(TINY_BCLIBC_BUILD_STATIC)
    add_library(tiny_bclibc_static STATIC src/tiny_bclibc_impl.c)
    add_library(tiny_bclibc::static ALIAS tiny_bclibc_static)
    tiny_bclibc_configure_target(tiny_bclibc_static)
    set_target_properties(tiny_bclibc_static PROPERTIES OUTPUT_NAME tiny_bclibc)
endif()

# ── Bare-metal (без thread-local) ──────────────────────────────────
add_library(tiny_bclibc_baremetal INTERFACE)
add_library(tiny_bclibc::baremetal ALIAS tiny_bclibc_baremetal)
target_link_libraries(tiny_bclibc_baremetal INTERFACE tiny_bclibc_headers)
target_compile_definitions(tiny_bclibc_baremetal INTERFACE TINY_BCLIBC_NO_THREAD_LOCAL)

# ── Install ────────────────────────────────────────────────────────
if(TINY_BCLIBC_INSTALL)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)

    # Заголовки
    install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/generated/tiny_bclibc/version.h"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/tiny_bclibc)

    # Targets для export
    set(INSTALL_TARGETS tiny_bclibc_headers tiny_bclibc_baremetal)
    if(TINY_BCLIBC_BUILD_SHARED)
        list(APPEND INSTALL_TARGETS tiny_bclibc)
    endif()
    if(TINY_BCLIBC_BUILD_STATIC)
        list(APPEND INSTALL_TARGETS tiny_bclibc_static)
    endif()

    install(TARGETS ${INSTALL_TARGETS}
        EXPORT tiny_bclibc-targets
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    install(EXPORT tiny_bclibc-targets
        FILE        tiny_bclibc-targets.cmake
        NAMESPACE   tiny_bclibc::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tiny_bclibc)

    # Config + version файли
    configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/tiny_bclibc-config.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/tiny_bclibc-config.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tiny_bclibc)

    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/tiny_bclibc-config-version.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion)

    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/tiny_bclibc-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/tiny_bclibc-config-version.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tiny_bclibc)
endif()

# ── Identity test ──────────────────────────────────────────────────────────
if(TINY_BCLIBC_BUILD_IDENTITY_TEST)
    add_subdirectory(tests)
endif()
