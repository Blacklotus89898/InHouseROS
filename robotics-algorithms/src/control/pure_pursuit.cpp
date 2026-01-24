#include "robotics_algo/control/pure_pursuit.hpp"
#include <cmath>
#include <limits>

namespace robotics::control {

    PurePursuit::PurePursuit(double lookahead, double max_speed) 
        : lookahead_(lookahead), max_speed_(max_speed) {}

    Vector2 PurePursuit::worldToRobot(Pose2D robot, Vector2 point) {
        // Translate
        double dx = point.x() - robot.position.x();
        double dy = point.y() - robot.position.y();
        
        // Rotate (Inverse of robot rotation)
        double c = std::cos(-robot.theta);
        double s = std::sin(-robot.theta);
        
        return Vector2(dx * c - dy * s, dx * s + dy * c);
    }

    std::pair<double, double> PurePursuit::getCommand(const Pose2D& robot, const std::vector<Vector2>& path) {
        if (path.empty()) return {0.0, 0.0};

        // 1. Find the Lookahead Point
        // (Simplification: Iterate forward and find first point > lookahead distance)
        Vector2 target = path.back(); // Default to end
        
        // Find closest point index first (to ensure we don't look backwards)
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;
        
        for(size_t i=0; i<path.size(); ++i) {
            double d = (path[i] - robot.position).norm();
            if(d < min_dist) {
                min_dist = d;
                closest_idx = i;
            }
        }

        // Search forward from closest point
        for(size_t i = closest_idx; i < path.size(); ++i) {
            double d = (path[i] - robot.position).norm();
            if(d > lookahead_) {
                target = path[i];
                break;
            }
        }

        // 2. Stop if close to goal
        if ((path.back() - robot.position).norm() < 10.0) {
            return {0.0, 0.0};
        }

        // 3. Transform Goal to Robot Frame
        Vector2 local_target = worldToRobot(robot, target);

        // 4. Calculate Curvature (The "Pure Pursuit" Math)
        // curvature = 2 * y / L^2
        // where y is lateral offset, L is lookahead distance
        double dist_sq = local_target.norm() * local_target.norm();
        if (dist_sq < 0.001) return {0.0, 0.0};

        double curvature = (2.0 * local_target.y()) / dist_sq;

        // 5. Calculate Velocities
        // v = max_speed
        // w = v * curvature
        double v = max_speed_;
        double w = v * curvature;

        // Clamp rotation for stability
        if(w > 0.5) w = 0.5;
        if(w < -0.5) w = -0.5;

        return {v, w};
    }
}
