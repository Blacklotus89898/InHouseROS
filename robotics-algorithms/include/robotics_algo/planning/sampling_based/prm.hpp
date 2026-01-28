#pragma once
#include <vector>
#include <list>
#include "robotics_algo/common/types.hpp"

namespace robotics::planning {

    struct PRMNode {
        int id;
        Vector2 pos;
        std::vector<int> neighbors; // IDs of connected nodes
    };

    class PRM {
    public:
        // num_samples: How many "cities" to build
        // connection_radius: Max distance to connect two cities
        PRM(int num_samples = 100, double connection_radius = 100.0);

        // Phase 1: Build the Roadmap (Slow, done once)
        void buildRoadmap(const std::vector<Vector2>& obstacles, double obs_radius, 
                          int map_width, int map_height);

        // Phase 2: Query path (Fast, done many times)
        std::vector<Vector2> plan(Vector2 start, Vector2 goal);

        // Get graph for visualization
        const std::vector<PRMNode>& getGraph() const { return nodes_; }

    private:
        bool isCollision(Vector2 a, Vector2 b, const std::vector<Vector2>& obstacles, double radius);
        int getNearestNodeId(Vector2 point);

        std::vector<PRMNode> nodes_;
        int num_samples_;
        double connection_radius_;
    };
}
