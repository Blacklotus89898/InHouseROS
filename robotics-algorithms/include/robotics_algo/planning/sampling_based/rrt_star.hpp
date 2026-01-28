#pragma once
#include <vector>
#include "robotics_algo/common/types.hpp"

namespace robotics::planning {

    struct RRTStarNode {
        int id;
        Vector2 pos;
        int parent_index;
        double cost; // Total distance from Start
    };

    class RRTStar {
    public:
        // rewire_radius: How far to look for shortcuts
        RRTStar(double step_size = 15.0, int max_iter = 5000, double rewire_radius = 40.0);

        std::vector<Vector2> plan(Vector2 start, Vector2 goal, 
                                  const std::vector<Vector2>& obstacles, 
                                  double obstacle_radius);

        const std::vector<RRTStarNode>& getTree() const { return nodes_; }

    private:
        int getNearestNodeIndex(Vector2 point);
        std::vector<int> getNearbyNodeIndices(Vector2 point, double radius);
        bool checkCollision(Vector2 a, Vector2 b, const std::vector<Vector2>& obstacles, double radius);
        double dist(Vector2 a, Vector2 b);

        std::vector<RRTStarNode> nodes_;
        double step_size_;
        int max_iter_;
        double rewire_radius_;
    };
}
