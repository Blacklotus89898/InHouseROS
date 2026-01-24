#pragma once
#include <vector>
#include <optional>
#include "robotics_algo/common/types.hpp"

namespace robotics::planning {

    // A node in the RRT Tree
    struct RRTNode {
        Vector2 pos;
        int parent_index; // Index in the node list (-1 for root)
    };

    class RRT {
    public:
        // step_size: How far to extend the branch in one step
        // max_iter: Safety limit to prevent infinite loops
        RRT(double step_size = 10.0, int max_iter = 5000);

        // --- Standard API (Blocking) ---
        // Returns the list of points from Start -> Goal immediately
        std::vector<Vector2> plan(Vector2 start, Vector2 goal, 
                                  const std::vector<Vector2>& obstacles, 
                                  double obstacle_radius);

        // --- Incremental API (For Animation) ---
        // 1. Set the start/goal and clear previous tree
        void reset(Vector2 start, Vector2 goal);
        
        // 2. Perform ONE expansion step. Returns true if Goal is reached.
        bool step(const std::vector<Vector2>& obstacles, double obstacle_radius);
        
        // 3. Retrieve path once goal is reached (empty otherwise)
        std::vector<Vector2> getPath() const;

        // Get the full tree (for visualization)
        const std::vector<RRTNode>& getTree() const { return nodes_; }

    private:
        int getNearestNodeIndex(Vector2 point);
        bool checkCollision(Vector2 a, Vector2 b, 
                            const std::vector<Vector2>& obstacles, double radius);

        std::vector<RRTNode> nodes_;
        double step_size_;
        int max_iter_;

        // State for Incremental Search
        Vector2 start_;
        Vector2 goal_;
        bool goal_reached_ = false;
    };
}
