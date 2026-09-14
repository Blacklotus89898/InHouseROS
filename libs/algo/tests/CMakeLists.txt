add_executable(ihrs_algo_tests
    test_math.cpp
    test_pid.cpp
)

target_link_libraries(ihrs_algo_tests
    PRIVATE
    ihrs::algo
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(ihrs_algo_tests TIMEOUT 60)
