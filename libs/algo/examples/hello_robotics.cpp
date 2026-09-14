#include <iostream>
#include "ihrs/algo/common/types.hpp"

int main() {
    ihrs::algo::Pose2D pose;
    pose.position = ihrs::algo::Vector2(10.0, 5.0);
    pose.theta = 3.14;

    std::cout << "Robot initialized at: " 
              << pose.position.x() << ", " 
              << pose.position.y() << std::endl;
    return 0;
}
