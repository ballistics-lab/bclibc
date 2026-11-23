cmake_minimum_required(VERSION 3.13)

# The parent project is C-only; explicitly enable C++ for this sub-directory.
enable_language(CXX)

# ── Source roots ───────────────────────────────────────────────────────────
set(BCLIBC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../")
set(TINY_BCLIBC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../")

# ── bclibc static library (only the sources we need) ──────────────────────
add_library(bclibc_identity_lib STATIC
    "${BCLIBC_ROOT}/src/base_types.cpp"
    "${BCLIBC_ROOT}/src/traj_data.cpp"
    "${BCLIBC_ROOT}/src/interp.cpp"
    "${BCLIBC_ROOT}/src/rk4.cpp"
    "${BCLIBC_ROOT}/src/cash_karp.cpp"
    "${BCLIBC_ROOT}/src/tsitouras.cpp"
    "${BCLIBC_ROOT}/src/engine.cpp"
    "${BCLIBC_ROOT}/src/traj_filter.cpp"
)

target_include_directories(bclibc_identity_lib PUBLIC
    "${BCLIBC_ROOT}/include")

target_compile_features(bclibc_identity_lib PUBLIC cxx_std_17)

if(NOT WIN32)
    target_link_libraries(bclibc_identity_lib PUBLIC m)
endif()

# Suppress bclibc debug logging in test binary
target_compile_definitions(bclibc_identity_lib PUBLIC BCLIBC_LOG_LEVEL=0)

# ── tiny_bclibc headers (header-only, no TINY_BCLIBC_BUILD_SHARED) ────────────────
# Reuse the tiny_bclibc_headers INTERFACE target if we're a sub-project,
# otherwise include directly.
if(NOT TARGET tiny_bclibc_headers)
    add_library(tiny_bclibc_identity_headers INTERFACE)
    target_include_directories(tiny_bclibc_identity_headers INTERFACE
        "${TINY_BCLIBC_ROOT}/include"
        "${TINY_BCLIBC_ROOT}/generated"   # version.h (may not exist yet)
    )
    set(TINY_BCLIBC_HEADERS_TARGET tiny_bclibc_identity_headers)
else()
    set(TINY_BCLIBC_HEADERS_TARGET tiny_bclibc_headers)
endif()

# ── test_identity executable ──────────────────────────────────────────────
add_executable(test_identity test_identity.cpp)

target_include_directories(test_identity PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"   # fixture.hpp, compare.hpp
)

target_link_libraries(test_identity PRIVATE
    bclibc_identity_lib
    ${TINY_BCLIBC_HEADERS_TARGET}
)

target_compile_features(test_identity PRIVATE cxx_std_17)

# Enable CTest integration
enable_testing()
add_test(NAME identity COMMAND test_identity)

# ── test_ridder_unconverged ───────────────────────────────────────────────
# The Ridder loop of tiny_bclibc__find_zero_angle_ridders has a fixed cap (`int32_t max_iter = 50;`, it does not read
# cfg.cMaxIterations) and converges in a few iterations, so the "not converged" exit cannot be reached with the stock
# cap. This test is built against a copy of the headers whose cap is lowered to TINY_RIDDER_ITER_CAP; the library
# headers are not modified. TINY_RIDDER_HEADER_DIR may point at another tiny_bclibc include directory (e.g. an older
# tree) to check the test against it.
set(TINY_RIDDER_ITER_CAP 2)
set(TINY_RIDDER_HEADER_DIR "${TINY_BCLIBC_ROOT}/include" CACHE PATH "tiny_bclibc include dir that test_ridder_unconverged patches")
set(TINY_RIDDER_PATCHED_DIR "${CMAKE_CURRENT_BINARY_DIR}/ridder_capped_include")
file(GLOB TINY_RIDDER_HEADERS "${TINY_RIDDER_HEADER_DIR}/tiny_bclibc/*.h")
foreach(hdr IN LISTS TINY_RIDDER_HEADERS)
    get_filename_component(hdr_name "${hdr}" NAME)
    file(READ "${hdr}" hdr_text)
    if(hdr_name STREQUAL "engine.h")
        string(REPLACE "int32_t max_iter = 50;" "int32_t max_iter = ${TINY_RIDDER_ITER_CAP};" patched_text "${hdr_text}")
        if(patched_text STREQUAL hdr_text)
            message(FATAL_ERROR "test_ridder_unconverged: 'int32_t max_iter = 50;' not found in ${hdr} (the Ridder loop changed; update this test)")
        endif()
        set(hdr_text "${patched_text}")
    endif()
    file(GENERATE OUTPUT "${TINY_RIDDER_PATCHED_DIR}/tiny_bclibc/${hdr_name}" CONTENT "${hdr_text}")
endforeach()

add_executable(test_ridder_unconverged test_ridder_unconverged.cpp)
target_include_directories(test_ridder_unconverged PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${TINY_RIDDER_PATCHED_DIR}"
    "${TINY_BCLIBC_ROOT}/generated"
)
target_link_libraries(test_ridder_unconverged PRIVATE bclibc_identity_lib)
target_compile_definitions(test_ridder_unconverged PRIVATE TINY_RIDDER_ITER_CAP=${TINY_RIDDER_ITER_CAP})
target_compile_features(test_ridder_unconverged PRIVATE cxx_std_17)
add_test(NAME ridder_unconverged COMMAND test_ridder_unconverged)

message(STATUS "tiny_bclibc identity test configured")
message(STATUS "  bclibc root : ${BCLIBC_ROOT}")
message(STATUS "  tiny_bclibc root: ${TINY_BCLIBC_ROOT}")
