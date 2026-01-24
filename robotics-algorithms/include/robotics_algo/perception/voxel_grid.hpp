#pragma once
#include <vector>
#include "robotics_algo/common/types.hpp"

namespace robotics::perception {

    class VoxelGrid {
    public:
        // Dimensions in grid cells
        VoxelGrid(int width, int height, int depth, double resolution);

        // Update map with 3D Lidar data
        void update(const Pose3D& robot_pose, const std::vector<Vector3>& hits);

        bool isOccupied(int x, int y, int z) const;
        
        // Get raw data for visualization
        const std::vector<double>& getData() const { return log_odds_; }
        int getWidth() const { return width_; }
        int getHeight() const { return height_; }
        int getDepth() const { return depth_; }
        double getResolution() const { return resolution_; }

    private:
        int width_, height_, depth_;
        double resolution_;
        std::vector<double> log_odds_;

        // Convert world coordinate to grid index
        // Returns false if out of bounds
        bool worldToGrid(Vector3 world, int& x, int& y, int& z) const;
    };
}
