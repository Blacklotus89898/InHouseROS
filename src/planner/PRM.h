#ifndef PRM_H
#define PRM_H

#include <SDL2/SDL.h>
#include <vector>
#include <map>
#include "Common.h"

struct GraphNode {
    int id;
    Point p;
    std::vector<int> neighbors;
};

class PRM {
public:
    PRM(Point startPos, Point goalPos);

    // Unlike RRT, PRM has two distinct phases. 
    // We will use 'step()' to build the map, then solve path at the end.
    void step(); 
    void draw(SDL_Renderer* renderer);
    void reset();

private:
    std::vector<GraphNode> graph;
    std::vector<Point> path;
    
    Point start;
    Point goal;
    SDL_Rect obstacle;
    
    int targetSamples;
    bool roadmapBuilt;
    bool pathFound;

    // Parameters
    double connectionRadius;

    // Helpers
    double dist(Point p1, Point p2);
    bool isValid(Point p);
    bool lineCheck(Point p1, Point p2);
    void buildConnections();
    void findPathDijkstra();
};

#endif
