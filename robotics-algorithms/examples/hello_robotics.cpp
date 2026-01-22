#include <iostream>
#include "robotics_algo/common/types.hpp"

int main() {
    robotics::Pose2D pose;
    pose.position = robotics::Vector2(10.0, 5.0);
    pose.theta = 3.14;

    std::cout << "Robot initialized at: " 
              << pose.position.x() << ", " 
              << pose.position.y() << std::endl;
    return 0;
}
