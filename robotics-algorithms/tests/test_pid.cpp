#include <gtest/gtest.h>
#include "robotics_algo/control/pid.hpp"

// Test 1: P-Controller only (Simple)
TEST(PIDTest, ProportionalLogic) {
    // Kp=2.0, Ki=0, Kd=0
    robotics::control::PID pid(2.0, 0.0, 0.0);
    
    // Target=10, Measured=8, Error=2
    // Expected Output = Kp * Error = 2.0 * 2.0 = 4.0
    double output = pid.update(10.0, 8.0, 0.1);
    
    EXPECT_DOUBLE_EQ(output, 4.0);
}

// Test 2: Integral Accumulation
TEST(PIDTest, IntegralLogic) {
    // Kp=0, Ki=1.0, Kd=0
    robotics::control::PID pid(0.0, 1.0, 0.0);
    
    // Step 1: Error=1, dt=1.0 -> Integral adds 1.0 -> Output 1.0
    double out1 = pid.update(10.0, 9.0, 1.0);
    EXPECT_DOUBLE_EQ(out1, 1.0);
    
    // Step 2: Error=1, dt=1.0 -> Integral adds 1.0 -> Total 2.0 -> Output 2.0
    double out2 = pid.update(10.0, 9.0, 1.0);
    EXPECT_DOUBLE_EQ(out2, 2.0);
}

// Test 3: Clamping Limits
TEST(PIDTest, Clamping) {
    robotics::control::PIDConfig config;
    config.kp = 10.0;
    config.max_output = 5.0; // Limit output to 5
    
    robotics::control::PID pid(config);
    
    // Error=10 -> Theoretical Output=100 -> Clamped to 5
    double output = pid.update(100.0, 90.0, 0.1);
    
    EXPECT_DOUBLE_EQ(output, 5.0);
}
