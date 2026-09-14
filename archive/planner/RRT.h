#ifndef RRT_H
#define RRT_H

#include <SDL2/SDL.h>
#include <vector>
#include "Common.h"

struct Node {
    Point point;
    int parentIndex;
};

struct Obstacle {
    int x, y, w, h;
};

class RRT {
public:
    // Constructor
    RRT(Point start, Point goal);

    // Main functions
    void step();
    void draw(SDL_Renderer* renderer);
    bool isGoalReached();

private:
    // Internal state
    std::vector<Node> nodes;
    std::vector<Obstacle> obstacles;
    Point start;
    Point goal;
    bool goalReached;

    // Helper functions (Internal use only)
    float distSq(Point p1, Point p2);
    bool isPointInObstacle(Point p);
    bool isPathClear(Point p1, Point p2);
};

#endif
