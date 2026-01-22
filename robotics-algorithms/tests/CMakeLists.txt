# Define the test executable
add_executable(unit_tests 
    test_math.cpp
    # Add future test files here, e.g., test_planning.cpp
)

# Link against your main library AND GoogleTest
# GTest::gtest_main includes a default main() function so you don't have to write one.
target_link_libraries(unit_tests 
    PRIVATE 
    robotics_core 
    GTest::gtest_main
)

# Register the test so 'ctest' can run it
include(CTest)
add_test(NAME AllTests COMMAND unit_tests)
