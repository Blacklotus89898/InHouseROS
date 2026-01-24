#pragma once
#include <vector>
#include <utility> // for std::pair
#include "robotics_algo/common/types.hpp"

namespace robotics::control {

    class PurePursuit {
    public:
        // lookahead_dist: How far ahead to look (e.g., 20.0 pixels)
        // max_speed: Maximum linear velocity
        PurePursuit(double lookahead_dist = 20.0, double max_speed = 4.0);

        // Inputs: Robot Pose, and the Path (list of points)
        // Returns: {linear_velocity, angular_velocity}
        std::pair<double, double> getCommand(const Pose2D& robot, const std::vector<Vector2>& path);

    private:
        double lookahead_;
        double max_speed_;
        
        // Helper to transform world point to robot frame
        Vector2 worldToRobot(Pose2D robot, Vector2 point);
    };
}
