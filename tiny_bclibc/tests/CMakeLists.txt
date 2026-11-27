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

# ── test_continuous_root ──────────────────────────────────────────────────
# The angle-tolerance exits of the Ridder search reject a root that is a jump of the error function (class C18, as bclibc). The
# low zero is continuous, so with the real ratio (0.45) the check never fires; this test is built against a copy of the headers
# whose ratio is lowered to 1e-30, which makes every angle-tolerance exit a failure and so proves that both exits are checked.
set(TINY_CONTINUOUS_PATCHED_DIR "${CMAKE_CURRENT_BINARY_DIR}/continuous_root_include")
foreach(hdr IN LISTS TINY_RIDDER_HEADERS)
    get_filename_component(hdr_name "${hdr}" NAME)
    file(READ "${hdr}" hdr_text)
    if(hdr_name STREQUAL "engine.h")
        string(REPLACE "#define TINY_BCLIBC_CONTINUOUS_ROOT_RATIO REAL_C(0.45)" "#define TINY_BCLIBC_CONTINUOUS_ROOT_RATIO REAL_C(1e-30)" patched_text "${hdr_text}")
        if(patched_text STREQUAL hdr_text)
            message(FATAL_ERROR "test_continuous_root: the ratio define not found in ${hdr} (update this test)")
        endif()
        set(hdr_text "${patched_text}")
    endif()
    file(GENERATE OUTPUT "${TINY_CONTINUOUS_PATCHED_DIR}/tiny_bclibc/${hdr_name}" CONTENT "${hdr_text}")
endforeach()

add_executable(test_continuous_root test_continuous_root.cpp)
target_include_directories(test_continuous_root PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${TINY_CONTINUOUS_PATCHED_DIR}"
    "${TINY_BCLIBC_ROOT}/generated"
)
target_compile_features(test_continuous_root PRIVATE cxx_std_17)
if(NOT WIN32)
    target_link_libraries(test_continuous_root PRIVATE m)
endif()
add_test(NAME continuous_root COMMAND test_continuous_root)
set_tests_properties(continuous_root PROPERTIES TIMEOUT 300)

# ── test_gravity_validation ───────────────────────────────────────────────
# A gravity constant that is not a finite negative number is TINY_BCLIBC_ERR_INVALID_ARG from every entry point, not a loop
# that never returns (the bad-gravity calls never returned before the check: hence the time limit).
add_executable(test_gravity_validation test_gravity_validation.cpp)
target_include_directories(test_gravity_validation PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${TINY_BCLIBC_ROOT}/include"
    "${TINY_BCLIBC_ROOT}/generated"
)
target_compile_features(test_gravity_validation PRIVATE cxx_std_17)
if(NOT WIN32)
    target_link_libraries(test_gravity_validation PRIVATE m)
endif()
add_test(NAME gravity_validation COMMAND test_gravity_validation)
set_tests_properties(gravity_validation PROPERTIES TIMEOUT 300)

# ── test_min_velocity ─────────────────────────────────────────────────────
# The maximum-range, apex and zero searches run without the caller's cMinimumVelocity (as bclibc's), and leave the
# caller's config alone; compared with bclibc and, for the maximum range, the Python reference.
add_executable(test_min_velocity test_min_velocity.cpp)
target_include_directories(test_min_velocity PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(test_min_velocity PRIVATE bclibc_identity_lib ${TINY_BCLIBC_HEADERS_TARGET})
target_compile_features(test_min_velocity PRIVATE cxx_std_17)
add_test(NAME min_velocity COMMAND test_min_velocity)
set_tests_properties(min_velocity PROPERTIES TIMEOUT 300)

# ── test_cant_max_range_bracket ───────────────────────────────────────────
# The max-range search of a canted shot brackets only the reachable elevations asin(R) (class C38): the close zero at cant 60 deg
# and a scan of the reachable elevations as the reference of the maximum range. Headers only.
add_executable(test_cant_max_range_bracket test_cant_max_range_bracket.cpp)
target_include_directories(test_cant_max_range_bracket PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${TINY_BCLIBC_ROOT}/include"
    "${TINY_BCLIBC_ROOT}/generated"
)
target_compile_features(test_cant_max_range_bracket PRIVATE cxx_std_17)
if(NOT WIN32)
    target_link_libraries(test_cant_max_range_bracket PRIVATE m)
endif()
add_test(NAME cant_max_range_bracket COMMAND test_cant_max_range_bracket)
set_tests_properties(cant_max_range_bracket PROPERTIES TIMEOUT 600)

message(STATUS "tiny_bclibc identity test configured")
message(STATUS "  bclibc root : ${BCLIBC_ROOT}")
message(STATUS "  tiny_bclibc root: ${TINY_BCLIBC_ROOT}")
