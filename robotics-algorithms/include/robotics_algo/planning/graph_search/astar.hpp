#pragma once
#include <vector>
#include "robotics_algo/planning/graph_search/grid_map.hpp"

namespace robotics::planning {

    class AStar {
    public:
        // Returns empty vector if no path found
        static std::vector<GridPoint> plan(const GridMap& map, GridPoint start, GridPoint goal);

    private:
        // Heuristic function (Manhattan Distance)
        static double heuristic(GridPoint a, GridPoint b);
    };

}
