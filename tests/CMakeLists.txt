cmake_minimum_required(VERSION 3.13)

find_package(Threads REQUIRED)

add_executable(bclibc_tests
    test_traj_data.cpp
)

target_link_libraries(bclibc_tests PRIVATE bclibc_core)
target_compile_features(bclibc_tests PRIVATE cxx_std_17)

add_test(NAME bclibc_tests COMMAND bclibc_tests)

# Separate executable (own main()): exercises BCLIBC_CashKarpIntegrator's
# tolerance/stats API, including the concurrent-sharing pattern documented
# in cash_karp.hpp's class doc comment. Build with BCLIBC_SANITIZE_THREAD=ON
# to actually run its concurrency test under ThreadSanitizer.
add_executable(bclibc_test_rk45_integrators
    test_rk45_integrators.cpp
)

target_link_libraries(bclibc_test_rk45_integrators PRIVATE bclibc_core Threads::Threads)
target_compile_features(bclibc_test_rk45_integrators PRIVATE cxx_std_17)

add_test(NAME bclibc_test_rk45_integrators COMMAND bclibc_test_rk45_integrators)

# C ABI checks for the adaptive entry points of bclibc_ffi.h (status codes, tolerances, stats, dense output).
add_executable(bclibc_test_ffi_adaptive
    test_ffi_adaptive.cpp
)

target_link_libraries(bclibc_test_ffi_adaptive PRIVATE bclibc_ffi)
target_include_directories(bclibc_test_ffi_adaptive PRIVATE "${BCLIBC_INCLUDE_DIR}")
target_compile_features(bclibc_test_ffi_adaptive PRIVATE cxx_std_17)

add_test(NAME bclibc_test_ffi_adaptive COMMAND bclibc_test_ffi_adaptive)

# The zero search does not depend on the early-termination limits (cMaximumDrop, cMinimumAltitude): issue #45.
add_executable(bclibc_test_zero_limits
    test_zero_limits.cpp
)

target_link_libraries(bclibc_test_zero_limits PRIVATE bclibc_ffi)
target_include_directories(bclibc_test_zero_limits PRIVATE "${BCLIBC_INCLUDE_DIR}")
target_compile_features(bclibc_test_zero_limits PRIVATE cxx_std_17)

add_test(NAME bclibc_test_zero_limits COMMAND bclibc_test_zero_limits)

# C ABI checks for BCLIBCFFI_integrate_dense_shot_stats(): the fixed-step records and their step count.
add_executable(bclibc_test_ffi_fixed_stats
    test_ffi_fixed_stats.cpp
)

target_link_libraries(bclibc_test_ffi_fixed_stats PRIVATE bclibc_ffi)
target_include_directories(bclibc_test_ffi_fixed_stats PRIVATE "${BCLIBC_INCLUDE_DIR}")
target_compile_features(bclibc_test_ffi_fixed_stats PRIVATE cxx_std_17)

add_test(NAME bclibc_test_ffi_fixed_stats COMMAND bclibc_test_ffi_fixed_stats)

# The zero angle of a target closer than about twice the sight height: the shot fired with it must reach the line of
# sight (C ABI and C++ core), and the zero of every farther target is unchanged (recorded values).
# One binary holds both the C ABI layer (compiled in, not linked as the shared library) and the C++ core it wraps:
# linking bclibc_ffi next to bclibc_core would load the core twice (an ODR violation under ASan).
add_executable(bclibc_test_zero_close_target
    test_zero_close_target.cpp
    "${BCLIBC_SRC_DIR}/ffi/bclibc_ffi.cpp"
)

target_link_libraries(bclibc_test_zero_close_target PRIVATE bclibc_core)
target_include_directories(bclibc_test_zero_close_target PRIVATE "${BCLIBC_INCLUDE_DIR}" "${CMAKE_BINARY_DIR}/generated")
target_compile_features(bclibc_test_zero_close_target PRIVATE cxx_std_17)

add_test(NAME bclibc_test_zero_close_target COMMAND bclibc_test_zero_close_target)

# The angle tolerance of the Ridder zero search (1e-7 rad, as the Python reference): the lofted zero and the high-elevation
# zeros of py-ballisticcalc issue #204 hit the target, the flat 300 m zero is unchanged. One binary with the C ABI layer
# compiled in, as bclibc_test_zero_close_target.
add_executable(bclibc_test_zero_ridder_tol
    test_zero_ridder_tol.cpp
    "${BCLIBC_SRC_DIR}/ffi/bclibc_ffi.cpp"
)

target_link_libraries(bclibc_test_zero_ridder_tol PRIVATE bclibc_core)
target_include_directories(bclibc_test_zero_ridder_tol PRIVATE "${BCLIBC_INCLUDE_DIR}" "${CMAKE_BINARY_DIR}/generated")
target_compile_features(bclibc_test_zero_ridder_tol PRIVATE cxx_std_17)

add_test(NAME bclibc_test_zero_ridder_tol COMMAND bclibc_test_zero_ridder_tol)

# The zero angle (Ridder path) and the maximum range of a shot with a downhill look angle: the maximum-range search must
# follow the shot down to a line of sight that lies below the muzzle (C ABI and C++ core), and the results that did not
# depend on the defect are unchanged (recorded values).
add_executable(bclibc_test_zero_downhill
    test_zero_downhill.cpp
    "${BCLIBC_SRC_DIR}/ffi/bclibc_ffi.cpp"
)

target_link_libraries(bclibc_test_zero_downhill PRIVATE bclibc_core)
target_include_directories(bclibc_test_zero_downhill PRIVATE "${BCLIBC_INCLUDE_DIR}" "${CMAKE_BINARY_DIR}/generated")
target_compile_features(bclibc_test_zero_downhill PRIVATE cxx_std_17)

add_test(NAME bclibc_test_zero_downhill COMMAND bclibc_test_zero_downhill)

# The wind sock is rewound at the start of every integration: the trial shots of a zero or max-range solve with a wind of
# more than one segment (or one finite segment) must not start with the sock the previous trial left behind.
# C ABI compiled in with the C++ core, as test_zero_close_target (one copy of the core).
add_executable(bclibc_test_windsock_reset
    test_windsock_reset.cpp
    "${BCLIBC_SRC_DIR}/ffi/bclibc_ffi.cpp"
)

target_link_libraries(bclibc_test_windsock_reset PRIVATE bclibc_core)
target_include_directories(bclibc_test_windsock_reset PRIVATE "${BCLIBC_INCLUDE_DIR}" "${CMAKE_BINARY_DIR}/generated")
target_compile_features(bclibc_test_windsock_reset PRIVATE cxx_std_17)

add_test(NAME bclibc_test_windsock_reset COMMAND bclibc_test_windsock_reset)

# find_zero_angle() and find_zero_point() return the same angle for the same search (the bracket-width exit of the
# Ridder loop used to return the midpoint of the bracket without a point and next_angle with one); the angles of searches
# that never took that exit are unchanged (recorded values).
add_executable(bclibc_test_zero_ridder_angle
    test_zero_ridder_angle.cpp
    "${BCLIBC_SRC_DIR}/ffi/bclibc_ffi.cpp"
)

target_link_libraries(bclibc_test_zero_ridder_angle PRIVATE bclibc_core)
target_include_directories(bclibc_test_zero_ridder_angle PRIVATE "${BCLIBC_INCLUDE_DIR}" "${CMAKE_BINARY_DIR}/generated")
target_compile_features(bclibc_test_zero_ridder_angle PRIVATE cxx_std_17)

add_test(NAME bclibc_test_zero_ridder_angle COMMAND bclibc_test_zero_ridder_angle)
