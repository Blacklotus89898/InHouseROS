#include "robotics_algo/planning/graph_search/astar.hpp"
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>

namespace robotics::planning {

    // Helper struct for Priority Queue
    struct Node {
        GridPoint point;
        double f_score; // f = g + h

        // Priority queue orders by greatest value, so we flip operator to make it a Min-Queue
        bool operator>(const Node& other) const {
            return f_score > other.f_score;
        }
    };

    // Helper for Map Hashing (to allow GridPoint in unordered_map)
    struct GridHash {
        std::size_t operator()(const GridPoint& p) const {
            return std::hash<int>()(p.x) ^ std::hash<int>()(p.y);
        }
    };

    double AStar::heuristic(GridPoint a, GridPoint b) {
        // Manhattan distance for 4-connected grid
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    std::vector<GridPoint> AStar::plan(const GridMap& map, GridPoint start, GridPoint goal) {
        // 1. Open Set (Frontier)
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
        open_set.push({start, 0.0});

        // 2. Track where we came from (for path reconstruction)
        std::unordered_map<GridPoint, GridPoint, GridHash> came_from;
        
        // 3. Cost so far (g_score)
        std::unordered_map<GridPoint, double, GridHash> g_score;
        g_score[start] = 0.0;

        while (!open_set.empty()) {
            GridPoint current = open_set.top().point;
            open_set.pop();

            // Goal Reached?
            if (current == goal) {
                std::vector<GridPoint> path;
                while (current != start) {
                    path.push_back(current);
                    current = came_from[current];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }

            // Neighbors (Up, Down, Left, Right)
            GridPoint neighbors[4] = {
                {current.x + 1, current.y}, {current.x - 1, current.y},
                {current.x, current.y + 1}, {current.x, current.y - 1}
            };

            for (auto& next : neighbors) {
                // Check Bounds & Obstacles
                if (map.isObstacle(next.x, next.y)) continue;

                // Calculate tentative g_score
                double new_cost = g_score[current] + 1.0; // Assume cost 1 per move

                if (g_score.find(next) == g_score.end() || new_cost < g_score[next]) {
                    g_score[next] = new_cost;
                    double f_score = new_cost + heuristic(next, goal);
                    open_set.push({next, f_score});
                    came_from[next] = current;
                }
            }
        }

        return {}; // No path found
    }
}
