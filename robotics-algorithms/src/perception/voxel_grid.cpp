#include "robotics_algo/perception/voxel_grid.hpp"
#include <cmath>
#include <algorithm>

namespace robotics::perception {

    VoxelGrid::VoxelGrid(int w, int h, int d, double res) 
        : width_(w), height_(h), depth_(d), resolution_(res) {
        log_odds_.resize(w * h * d, 0.0);
    }

    bool VoxelGrid::worldToGrid(Vector3 world, int& x, int& y, int& z) const {
        x = static_cast<int>(world.x() / resolution_);
        y = static_cast<int>(world.y() / resolution_);
        z = static_cast<int>(world.z() / resolution_);
        return (x >= 0 && x < width_ && y >= 0 && y < height_ && z >= 0 && z < depth_);
    }

    void VoxelGrid::update(const Pose3D& pose, const std::vector<Vector3>& hits) {
        const double LO_OCCUPIED = 0.9;
        const double LO_FREE = -0.7;
        const double CLAMP_MIN = -5.0;
        const double CLAMP_MAX = 5.0;

        for (const auto& hit : hits) {
            Vector3 start = pose.position;
            Vector3 diff = hit - start;
            double dist = diff.norm();
            Vector3 dir = diff / dist;

            // --- 3D Ray Traversal (Sampling Method) ---
            // Step size = half resolution ensures we don't skip voxels
            double step = resolution_ / 2.0; 
            
            for (double t = 0; t < dist; t += step) {
                Vector3 p = start + dir * t;
                int x, y, z;
                if (worldToGrid(p, x, y, z)) {
                    int idx = z * width_ * height_ + y * width_ + x;
                    log_odds_[idx] += LO_FREE;
                    if (log_odds_[idx] < CLAMP_MIN) log_odds_[idx] = CLAMP_MIN;
                }
            }

            // --- Mark Hit Point ---
            int hx, hy, hz;
            if (worldToGrid(hit, hx, hy, hz)) {
                int idx = hz * width_ * height_ + hy * width_ + hx;
                // Add Occupied (and cancel the last Free subtraction roughly)
                log_odds_[idx] += (LO_OCCUPIED - LO_FREE); 
                if (log_odds_[idx] > CLAMP_MAX) log_odds_[idx] = CLAMP_MAX;
            }
        }
    }

    bool VoxelGrid::isOccupied(int x, int y, int z) const {
        if (x < 0 || x >= width_ || y < 0 || y >= height_ || z < 0 || z >= depth_) return false;
        return log_odds_[z * width_ * height_ + y * width_ + x] > 0.0;
    }
}
