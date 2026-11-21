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
