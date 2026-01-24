#pragma once
#include <vector>
#include <cmath>
#include "robotics_algo/common/types.hpp"

namespace robotics::perception {

    class OccupancyGrid {
    public:
        // resolution: Meters per pixel (e.g., 0.1m)
        OccupancyGrid(int width, int height, double resolution);

        // Update map based on a Lidar scan
        // robot_pose: Where the robot is (x, y, theta)
        // scan_points: List of points where the laser hit something
        void update(const Pose2D& robot_pose, const std::vector<Vector2>& scan_points);

        // Check if a world coordinate is occupied (> 50% probability)
        bool isOccupied(double x, double y) const;

        // Getters for visualization
        int getWidth() const { return width_; }
        int getHeight() const { return height_; }
        double getResolution() const { return resolution_; }
        const std::vector<double>& getData() const { return log_odds_; }

    private:
        int width_;
        int height_;
        double resolution_;
        
        // We store Log-Odds instead of raw 0.0-1.0 probability
        // LogOdds = log(p / (1-p))
        std::vector<double> log_odds_; 

        // Bresenham's Line Algorithm (to find cells between robot and hit point)
        std::vector<Vector2> traceRay(Vector2 start, Vector2 end);
    };

}
