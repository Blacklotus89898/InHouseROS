#include "PRM.h"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <queue>
#include <algorithm>
#include <iostream>

PRM::PRM(Point startPos, Point goalPos) 
    : start(startPos), goal(goalPos), targetSamples(300), 
      roadmapBuilt(false), pathFound(false), connectionRadius(70.0)
{
    obstacle = {300, 200, 200, 200};
    reset();
}

void PRM::reset() {
    graph.clear();
    path.clear();
    roadmapBuilt = false;
    pathFound = false;

    // Add Start and Goal immediately so they are part of the graph
    graph.push_back({0, start, {}});
    graph.push_back({1, goal, {}});
}

void PRM::step() {
    if (pathFound) return;

    // PHASE 1: Sampling (Incremental)
    if (graph.size() < targetSamples) {
        Point p;
        p.x = rand() % 800;
        p.y = rand() % 600;

        if (isValid(p)) {
            GraphNode node;
            node.id = (int)graph.size();
            node.p = p;
            graph.push_back(node);
        }
    } 
    // PHASE 2: Connect & Solve (One-shot after sampling is done)
    else if (!roadmapBuilt) {
        buildConnections();
        findPathDijkstra();
        roadmapBuilt = true;
        pathFound = true; // Stop processing
    }
}

void PRM::buildConnections() {
    // Simple O(N^2) connection strategy
    for (size_t i = 0; i < graph.size(); ++i) {
        for (size_t j = i + 1; j < graph.size(); ++j) {
            if (dist(graph[i].p, graph[j].p) < connectionRadius) {
                if (!lineCheck(graph[i].p, graph[j].p)) {
                    graph[i].neighbors.push_back(graph[j].id);
                    graph[j].neighbors.push_back(graph[i].id);
                }
            }
        }
    }
}

void PRM::findPathDijkstra() {
    // Standard Dijkstra
    std::map<int, double> distances;
    std::map<int, int> parents;
    for(auto& n : graph) distances[n.id] = std::numeric_limits<double>::infinity();

    // Start node is index 0
    distances[0] = 0.0;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;
    pq.push({0.0, 0});

    while(!pq.empty()) {
        int u = pq.top().second;
        double d = pq.top().first;
        pq.pop();

        if (d > distances[u]) continue;
        if (u == 1) break; // Goal index is 1

        for (int v : graph[u].neighbors) {
            double weight = dist(graph[u].p, graph[v].p);
            if (distances[u] + weight < distances[v]) {
                distances[v] = distances[u] + weight;
                parents[v] = u;
                pq.push({distances[v], v});
            }
        }
    }

    // Reconstruct
    if (distances[1] != std::numeric_limits<double>::infinity()) {
        int curr = 1; // Goal
        while (curr != 0) {
            path.push_back(graph[curr].p);
            curr = parents[curr];
        }
        path.push_back(graph[0].p);
    }
}

void PRM::draw(SDL_Renderer* renderer) {
    // Obstacle
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    SDL_RenderFillRect(renderer, &obstacle);

    // Nodes (Blue dots)
    SDL_SetRenderDrawColor(renderer, 100, 200, 255, 255);
    for(const auto& n : graph) SDL_RenderDrawPoint(renderer, (int)n.p.x, (int)n.p.y);

    // Edges (Faint lines)
    SDL_SetRenderDrawColor(renderer, 200, 200, 200, 50); // Low alpha
    for(const auto& n : graph) {
        for(int neighborId : n.neighbors) {
            // Draw only one way to avoid double draw (optimization)
            if (n.id < neighborId) {
                Point p2 = graph[neighborId].p;
                SDL_RenderDrawLine(renderer, (int)n.p.x, (int)n.p.y, (int)p2.x, (int)p2.y);
            }
        }
    }

    // Start/Goal
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    SDL_Rect sR = {(int)start.x-4, (int)start.y-4, 8, 8}; SDL_RenderFillRect(renderer, &sR);
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_Rect gR = {(int)goal.x-4, (int)goal.y-4, 8, 8}; SDL_RenderFillRect(renderer, &gR);

    // Path (Thick Blue)
    if (!path.empty()) {
        SDL_SetRenderDrawColor(renderer, 50, 100, 255, 255);
        for (size_t i = 0; i < path.size() - 1; ++i) {
             SDL_RenderDrawLine(renderer, (int)path[i].x, (int)path[i].y, (int)path[i+1].x, (int)path[i+1].y);
             SDL_RenderDrawLine(renderer, (int)path[i].x+1, (int)path[i].y+1, (int)path[i+1].x+1, (int)path[i+1].y+1);
        }
    }
}

// Helpers
double PRM::dist(Point p1, Point p2) { return sqrt(pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2)); }
bool PRM::isValid(Point p) {
    if (p.x < 0 || p.x > 800 || p.y < 0 || p.y > 600) return false;
    return !(p.x > obstacle.x && p.x < obstacle.x + obstacle.w && p.y > obstacle.y && p.y < obstacle.y + obstacle.h);
}
bool PRM::lineCheck(Point p1, Point p2) {
    int steps = 10;
    for(int i=0; i<=steps; ++i) {
        double t = (double)i/steps;
        Point check = {p1.x + t*(p2.x-p1.x), p1.y + t*(p2.y-p1.y)};
        if(!isValid(check)) return true; // Collision
    }
    return false;
}
