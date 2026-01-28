#include "robotics_algo/planning/sampling_based/rrt_star.hpp"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <algorithm>

namespace robotics::planning {

    RRTStar::RRTStar(double step_size, int max_iter, double rewire_radius)
        : step_size_(step_size), max_iter_(max_iter), rewire_radius_(rewire_radius) {}

    double RRTStar::dist(Vector2 a, Vector2 b) {
        return (a - b).norm();
    }

    int RRTStar::getNearestNodeIndex(Vector2 point) {
        int min_idx = -1;
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& node : nodes_) {
            double d = dist(node.pos, point);
            if (d < min_dist) {
                min_dist = d;
                min_idx = node.id;
            }
        }
        return min_idx;
    }

    std::vector<int> RRTStar::getNearbyNodeIndices(Vector2 point, double radius) {
        std::vector<int> indices;
        for (const auto& node : nodes_) {
            if (dist(node.pos, point) <= radius) {
                indices.push_back(node.id);
            }
        }
        return indices;
    }

    bool RRTStar::checkCollision(Vector2 a, Vector2 b, const std::vector<Vector2>& obstacles, double radius) {
        // Check endpoints and midpoint
        for (const auto& obs : obstacles) {
            if (dist(a, obs) < radius) return true;
            if (dist(b, obs) < radius) return true;
            if (dist((a + b) * 0.5, obs) < radius) return true;
        }
        return false;
    }

    std::vector<Vector2> RRTStar::plan(Vector2 start, Vector2 goal, 
                                       const std::vector<Vector2>& obstacles, 
                                       double obs_radius) {
        nodes_.clear();
        nodes_.push_back({0, start, -1, 0.0}); // Root

        int goal_idx = -1;
        double best_goal_cost = std::numeric_limits<double>::max();
        int width = 800, height = 600; // Map bounds

        for (int i = 0; i < max_iter_; ++i) {
            // 1. Sample
            Vector2 rand_point(rand() % width, rand() % height);
            if (rand() % 100 < 5) rand_point = goal; // 5% Goal Bias

            // 2. Steer
            int nearest_idx = getNearestNodeIndex(rand_point);
            Vector2 nearest_pos = nodes_[nearest_idx].pos;
            Vector2 direction = rand_point - nearest_pos;
            double len = direction.norm();
            
            if (len > step_size_) {
                direction = direction * (step_size_ / len);
                len = step_size_;
            }
            Vector2 new_pos = nearest_pos + direction;

            if (checkCollision(nearest_pos, new_pos, obstacles, obs_radius)) continue;

            // --- RRT* STARTS HERE ---

            // 3. Choose Best Parent (Minimize Cost)
            std::vector<int> nearby_indices = getNearbyNodeIndices(new_pos, rewire_radius_);
            int best_parent_idx = nearest_idx;
            double min_cost = nodes_[nearest_idx].cost + len;

            for (int idx : nearby_indices) {
                double d = dist(nodes_[idx].pos, new_pos);
                if (nodes_[idx].cost + d < min_cost) {
                    if (!checkCollision(nodes_[idx].pos, new_pos, obstacles, obs_radius)) {
                        min_cost = nodes_[idx].cost + d;
                        best_parent_idx = idx;
                    }
                }
            }

            // Add Node
            RRTStarNode new_node;
            new_node.id = nodes_.size();
            new_node.pos = new_pos;
            new_node.parent_index = best_parent_idx;
            new_node.cost = min_cost;
            nodes_.push_back(new_node);

            // 4. Rewire (Optimize Neighbors)
            for (int idx : nearby_indices) {
                double d = dist(new_pos, nodes_[idx].pos);
                // If going through NEW node is shorter than their OLD path...
                if (new_node.cost + d < nodes_[idx].cost) {
                     if (!checkCollision(new_pos, nodes_[idx].pos, obstacles, obs_radius)) {
                        // Rewire!
                        nodes_[idx].parent_index = new_node.id;
                        nodes_[idx].cost = new_node.cost + d;
                     }
                }
            }

            // 5. Check Goal
            double d_goal = dist(new_pos, goal);
            if (d_goal < step_size_) {
                // In RRT*, we don't stop! We keep running to find better paths.
                if (new_node.cost + d_goal < best_goal_cost) {
                    best_goal_cost = new_node.cost + d_goal;
                    goal_idx = new_node.id;
                }
            }
        }

        // Reconstruct Path (from best goal node found)
        if (goal_idx == -1) return {};

        std::vector<Vector2> path;
        path.push_back(goal);
        int curr = goal_idx;
        while (curr != -1) {
            path.push_back(nodes_[curr].pos);
            curr = nodes_[curr].parent_index;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
}
