#pragma once
#include <vector>
#include <iostream>
#include "robotics_algo/common/types.hpp"

namespace robotics::planning {

    // Simple 2D Grid Map
    struct GridMap {
        int width;
        int height;
        std::vector<int> data; // 0 = Free, 1 = Obstacle

        GridMap(int w, int h) : width(w), height(h) {
            data.resize(w * h, 0);
        }

        bool isObstacle(int x, int y) const {
            if (x < 0 || x >= width || y < 0 || y >= height) return true;
            return data[y * width + x] == 1;
        }

        void setObstacle(int x, int y) {
            if (x >= 0 && x < width && y >= 0 && y < height) {
                data[y * width + x] = 1;
            }
        }
    };

    // A coordinate on the grid
    struct GridPoint {
        int x, y;

        // Operator overloading for equality check
        bool operator==(const GridPoint& other) const {
            return x == other.x && y == other.y;
        }
        
        bool operator!=(const GridPoint& other) const {
            return !(*this == other);
        }
    };
}
