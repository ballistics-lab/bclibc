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

message(STATUS "tiny_bclibc identity test configured")
message(STATUS "  bclibc root : ${BCLIBC_ROOT}")
message(STATUS "  tiny_bclibc root: ${TINY_BCLIBC_ROOT}")
