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
