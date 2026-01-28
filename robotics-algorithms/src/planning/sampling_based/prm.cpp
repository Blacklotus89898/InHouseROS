#include "robotics_algo/planning/sampling_based/prm.hpp"
#include <cmath>
#include <cstdlib>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>

namespace robotics::planning {

    PRM::PRM(int num_samples, double connection_radius)
        : num_samples_(num_samples), connection_radius_(connection_radius) {}

    double dist(Vector2 a, Vector2 b) {
        return (a - b).norm();
    }

    bool PRM::isCollision(Vector2 a, Vector2 b, const std::vector<Vector2>& obstacles, double radius) {
        // Check midpoint and endpoints (Simplified collision)
        for (const auto& obs : obstacles) {
            if (dist(a, obs) < radius) return true;
            if (dist(b, obs) < radius) return true;
            
            // Check midpoint
            Vector2 mid = (a + b) * 0.5;
            if (dist(mid, obs) < radius) return true;
        }
        return false;
    }

    void PRM::buildRoadmap(const std::vector<Vector2>& obstacles, double obs_radius, int w, int h) {
        nodes_.clear();

        // 1. Random Sampling
        for (int i = 0; i < num_samples_; ++i) {
            Vector2 p(rand() % w, rand() % h);
            
            // Only keep collision-free samples
            bool free = true;
            for (const auto& obs : obstacles) {
                if (dist(p, obs) < obs_radius) { free = false; break; }
            }

            if (free) {
                PRMNode node;
                node.id = nodes_.size();
                node.pos = p;
                nodes_.push_back(node);
            }
        }

        // 2. Connect Neighbors (The "Road" building)
        for (size_t i = 0; i < nodes_.size(); ++i) {
            for (size_t j = i + 1; j < nodes_.size(); ++j) {
                double d = dist(nodes_[i].pos, nodes_[j].pos);
                
                if (d <= connection_radius_) {
                    if (!isCollision(nodes_[i].pos, nodes_[j].pos, obstacles, obs_radius)) {
                        // Add bidirectional edge
                        nodes_[i].neighbors.push_back(nodes_[j].id);
                        nodes_[j].neighbors.push_back(nodes_[i].id);
                    }
                }
            }
        }
    }

    int PRM::getNearestNodeId(Vector2 point) {
        int best_id = -1;
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& node : nodes_) {
            double d = dist(node.pos, point);
            if (d < min_dist) {
                min_dist = d;
                best_id = node.id;
            }
        }
        return best_id;
    }

    // A* Search on the Graph
    std::vector<Vector2> PRM::plan(Vector2 start, Vector2 goal) {
        if (nodes_.empty()) return {};

        // 1. Connect Start/Goal to the nearest roadmap nodes
        int start_id = getNearestNodeId(start);
        int goal_id = getNearestNodeId(goal);

        // Standard Dijkstra/A*
        std::unordered_map<int, int> came_from;
        std::unordered_map<int, double> cost_so_far;
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> frontier;

        frontier.push({0.0, start_id});
        cost_so_far[start_id] = 0.0;
        came_from[start_id] = start_id;

        while (!frontier.empty()) {
            int current = frontier.top().second;
            frontier.pop();

            if (current == goal_id) break;

            for (int next : nodes_[current].neighbors) {
                double new_cost = cost_so_far[current] + dist(nodes_[current].pos, nodes_[next].pos);
                
                if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]) {
                    cost_so_far[next] = new_cost;
                    // Heuristic: Euclidean distance to goal
                    double priority = new_cost + dist(nodes_[next].pos, nodes_[goal_id].pos);
                    frontier.push({priority, next});
                    came_from[next] = current;
                }
            }
        }

        // Reconstruct Path
        if (came_from.find(goal_id) == came_from.end()) return {}; // No path

        std::vector<Vector2> path;
        path.push_back(goal); // End
        int curr = goal_id;
        while (curr != start_id) {
            path.push_back(nodes_[curr].pos);
            curr = came_from[curr];
        }
        path.push_back(nodes_[start_id].pos); // Start node on graph
        path.push_back(start); // Actual Start
        
        std::reverse(path.begin(), path.end());
        return path;
    }
}
