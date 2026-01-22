#include <iostream>
#include <thread>
#include <chrono>
#include "robotics_algo/planning/graph_search/astar.hpp"

using namespace robotics::planning;

void printMap(const GridMap& map, const std::vector<GridPoint>& path) {
    // 1. Create a display buffer
    std::vector<char> display(map.width * map.height, '.');

    // 2. Draw Obstacles
    for(int y=0; y<map.height; ++y) {
        for(int x=0; x<map.width; ++x) {
            if(map.isObstacle(x, y)) display[y*map.width+x] = '#';
        }
    }

    // 3. Draw Path
    for(const auto& p : path) {
        display[p.y * map.width + p.x] = '*';
    }

    // 4. Print
    for(int y=0; y<map.height; ++y) {
        for(int x=0; x<map.width; ++x) {
            std::cout << display[y*map.width+x] << " ";
        }
        std::cout << "\n";
    }
}

int main() {
    // 1. Setup Map (10x10)
    GridMap map(10, 10);
    
    // Add a wall in the middle
    for(int y=2; y<8; ++y) map.setObstacle(5, y);

    GridPoint start = {2, 5};
    GridPoint goal = {8, 5};

    std::cout << "--- Planning Path ---" << std::endl;
    auto path = AStar::plan(map, start, goal);

    if(!path.empty()) {
        std::cout << "Path found! Length: " << path.size() << "\n\n";
        printMap(map, path);
    } else {
        std::cout << "No path found!" << std::endl;
    }

    return 0;
}
