#include <gtest/gtest.h>
#include "robotics_algo/common/types.hpp"

// Test Case 1: Vector Addition
TEST(MathTest, VectorAddition) {
    robotics::Vector2 v1(1.0, 2.0);
    robotics::Vector2 v2(3.0, 4.0);

    auto result = v1 + v2;

    EXPECT_DOUBLE_EQ(result.x(), 4.0);
    EXPECT_DOUBLE_EQ(result.y(), 6.0);
}

// Test Case 2: Simple Equality Check
TEST(MathTest, BasicCheck) {
    EXPECT_TRUE(true);
    EXPECT_EQ(1 + 1, 2);
}
