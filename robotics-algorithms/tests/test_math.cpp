#include <gtest/gtest.h>
#include "robotics_algo/common/types.hpp"

// Test 1: Verify Eigen integration works
TEST(MathTest, EigenBasicOperations) {
    robotics::Vector2 v1(1.0, 2.0);
    robotics::Vector2 v2(3.0, 4.0);

    auto v3 = v1 + v2;
    
    EXPECT_DOUBLE_EQ(v3.x(), 4.0);
    EXPECT_DOUBLE_EQ(v3.y(), 6.0);
    
    // Test Dot Product
    double dot = v1.dot(v2); // 1*3 + 2*4 = 3 + 8 = 11
    EXPECT_DOUBLE_EQ(dot, 11.0);
}

// Test 2: Verify Pose2D Struct
TEST(MathTest, PoseStruct) {
    robotics::Pose2D p(10.0, 5.0, 0.5);
    EXPECT_DOUBLE_EQ(p.position.x(), 10.0);
    EXPECT_DOUBLE_EQ(p.theta, 0.5);
}

// Test 3: Verify Angle Normalization (Crucial!)
TEST(MathTest, AngleNormalization) {
    // Case A: Angle is already valid
    EXPECT_NEAR(robotics::math::normalize_angle(0.5), 0.5, 1e-9);

    // Case B: Angle is slightly larger than PI (3.14159...)
    // 3.2 should wrap to approx -3.08
    EXPECT_LT(robotics::math::normalize_angle(3.2), 0.0);

    // Case C: Massive angle (720 degrees + small)
    // 4*PI + 0.1 should wrap to 0.1
    double huge_angle = 4.0 * robotics::PI + 0.1;
    EXPECT_NEAR(robotics::math::normalize_angle(huge_angle), 0.1, 1e-9);
}
