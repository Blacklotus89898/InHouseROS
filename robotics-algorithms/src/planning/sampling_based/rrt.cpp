#include "robotics_algo/planning/sampling_based/rrt.hpp"
#include <cmath>
#include <cstdlib> // for rand()
#include <limits>
#include <algorithm> // for reverse

namespace robotics::planning {

    RRT::RRT(double step_size, int max_iter) 
        : step_size_(step_size), max_iter_(max_iter) {}

    // Helper to calculate distance squared
    double distSq(Vector2 a, Vector2 b) {
        double dx = a.x() - b.x();
        double dy = a.y() - b.y();
        return dx*dx + dy*dy;
    }

    int RRT::getNearestNodeIndex(Vector2 point) {
        int min_idx = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < nodes_.size(); ++i) {
            double d = distSq(nodes_[i].pos, point);
            if (d < min_dist) {
                min_dist = d;
                min_idx = i;
            }
        }
        return min_idx;
    }

    bool RRT::checkCollision(Vector2 a, Vector2 b, 
                             const std::vector<Vector2>& obstacles, double radius) {
        // Simple collision: Check if endpoints or midpoint hit an obstacle
        // In pro code, you'd use line-circle intersection.
        for (const auto& obs : obstacles) {
            // Check A
            if (distSq(a, obs) < radius*radius) return true;
            // Check B
            if (distSq(b, obs) < radius*radius) return true;
        }
        return false;
    }

    std::vector<Vector2> RRT::plan(Vector2 start, Vector2 goal, 
                                   const std::vector<Vector2>& obstacles, 
                                   double obs_radius) {
        nodes_.clear();
        nodes_.push_back({start, -1}); // Root

        // Random Generator Range (Assume 800x600 screen)
        // ideally passed in, but hardcoded for demo simplicity
        int width = 800, height = 600;

        for (int i = 0; i < max_iter_; ++i) {
            // 1. Random Sampling
            // 10% Goal Bias: Sometimes pick the goal directly to speed up search
            Vector2 rand_point;
            if (rand() % 100 < 10) {
                rand_point = goal;
            } else {
                rand_point = Vector2(rand() % width, rand() % height);
            }

            // 2. Find Nearest
            int nearest_idx = getNearestNodeIndex(rand_point);
            Vector2 nearest_pos = nodes_[nearest_idx].pos;

            // 3. Steer (Move step_size towards random point)
            Vector2 direction = rand_point - nearest_pos;
            double len = direction.norm();
            
            // Normalize and scale
            if (len > step_size_) {
                direction = direction * (step_size_ / len);
            }
            Vector2 new_pos = nearest_pos + direction;

            // 4. Collision Check
            if (!checkCollision(nearest_pos, new_pos, obstacles, obs_radius)) {
                // Add Node
                nodes_.push_back({new_pos, nearest_idx});

                // 5. Check Goal
                if (distSq(new_pos, goal) < (step_size_ * step_size_)) {
                    // Goal Reached! Backtrack path
                    std::vector<Vector2> path;
                    path.push_back(goal);
                    int curr = nodes_.size() - 1;
                    while (curr != -1) {
                        path.push_back(nodes_[curr].pos);
                        curr = nodes_[curr].parent_index;
                    }
                    std::reverse(path.begin(), path.end());
                    return path;
                }
            }
        }

        return {}; // Failed
    }
    void RRT::reset(Vector2 start, Vector2 goal) {
        nodes_.clear();
        nodes_.push_back({start, -1});
        start_ = start;
        goal_ = goal;
        goal_reached_ = false;
    }

    bool RRT::step(const std::vector<Vector2>& obstacles, double obs_radius) {
        if (goal_reached_) return true;

        // 1. Random Sample
        int width = 800, height = 600; // Hardcoded map size for demo
        Vector2 rand_point;
        if (rand() % 100 < 5) { // 5% Goal Bias
            rand_point = goal_;
        } else {
            rand_point = Vector2(rand() % width, rand() % height);
        }

        // 2. Find Nearest
        int nearest_idx = getNearestNodeIndex(rand_point);
        Vector2 nearest_pos = nodes_[nearest_idx].pos;

        // 3. Steer
        Vector2 diff = rand_point - nearest_pos;
        double dist = diff.norm();
        
        Vector2 new_pos = nearest_pos;
        if (dist > step_size_) {
            new_pos = nearest_pos + (diff / dist) * step_size_;
        } else {
            new_pos = rand_point;
        }

        // 4. Collision Check
        if (!checkCollision(nearest_pos, new_pos, obstacles, obs_radius)) {
            nodes_.push_back({new_pos, nearest_idx});

            // 5. Check Goal
            if ((new_pos - goal_).norm() < step_size_) {
                goal_reached_ = true;
                return true;
            }
        }
        return false;
    }

    std::vector<Vector2> RRT::getPath() const {
        if (!goal_reached_) return {};

        std::vector<Vector2> path;
        path.push_back(goal_);
        
        int curr = nodes_.size() - 1;
        while (curr != -1) {
            path.push_back(nodes_[curr].pos);
            curr = nodes_[curr].parent_index;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
}
