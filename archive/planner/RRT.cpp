#include "RRT.h"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <iostream>

const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;
const int STEP_SIZE = 20;

RRT::RRT(Point s, Point g) : start(s), goal(g), goalReached(false) {
    nodes.push_back({start, -1});
    
    // Hardcoded obstacles
    obstacles.push_back({300, 100, 100, 400});
    obstacles.push_back({100, 300, 200, 50});
}

void RRT::step() {
    if (goalReached) return;

    // 1. Random Sampling (5% goal bias)
    Point randPoint;
    if (rand() % 100 < 5) {
        randPoint = goal;
    } else {
        randPoint = { (float)(rand() % SCREEN_WIDTH), (float)(rand() % SCREEN_HEIGHT) };
    }

    // 2. Find Nearest Node
    int nearestIdx = -1;
    float minDist = std::numeric_limits<float>::max();

    for (int i = 0; i < nodes.size(); i++) {
        float d = distSq(nodes[i].point, randPoint);
        if (d < minDist) {
            minDist = d;
            nearestIdx = i;
        }
    }

    // 3. Steer
    Point nearest = nodes[nearestIdx].point;
    float theta = atan2(randPoint.y - nearest.y, randPoint.x - nearest.x);
    Point newPoint = {
        nearest.x + STEP_SIZE * cos(theta),
        nearest.y + STEP_SIZE * sin(theta)
    };

    // 4. Check Bounds & Obstacles
    if (newPoint.x < 0 || newPoint.x >= SCREEN_WIDTH || newPoint.y < 0 || newPoint.y >= SCREEN_HEIGHT) return;

    if (isPathClear(nearest, newPoint)) {
        nodes.push_back({newPoint, nearestIdx});

        if (distSq(newPoint, goal) < 400) { // Reach goal threshold
            goalReached = true;
            nodes.push_back({goal, (int)nodes.size() - 1});
            std::cout << "Goal Reached!" << std::endl;
        }
    }
}

void RRT::draw(SDL_Renderer* renderer) {
    // Draw Obstacles (Grey)
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    for (const auto& obs : obstacles) {
        SDL_Rect r = {obs.x, obs.y, obs.w, obs.h};
        SDL_RenderFillRect(renderer, &r);
    }

    // Draw Tree (Green)
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    for (const auto& node : nodes) {
        if (node.parentIndex != -1) {
            Point p1 = node.point;
            Point p2 = nodes[node.parentIndex].point;
            SDL_RenderDrawLine(renderer, (int)p1.x, (int)p1.y, (int)p2.x, (int)p2.y);
        }
    }

    // Draw Start/Goal (Blue)
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
    SDL_Rect sRect = {(int)start.x - 5, (int)start.y - 5, 10, 10};
    SDL_Rect gRect = {(int)goal.x - 5, (int)goal.y - 5, 10, 10};
    SDL_RenderFillRect(renderer, &sRect);
    SDL_RenderFillRect(renderer, &gRect);
}

bool RRT::isGoalReached() {
    return goalReached;
}

// --- Helpers ---

float RRT::distSq(Point p1, Point p2) {
    return (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y);
}

bool RRT::isPointInObstacle(Point p) {
    for (const auto& obs : obstacles) {
        if (p.x > obs.x && p.x < obs.x + obs.w &&
            p.y > obs.y && p.y < obs.y + obs.h) {
            return true;
        }
    }
    return false;
}

bool RRT::isPathClear(Point p1, Point p2) {
    float dist = std::sqrt(distSq(p1, p2));
    int steps = dist / 5;
    for (int i = 0; i <= steps; i++) {
        float t = (float)i / steps;
        Point check = { p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y) };
        if (isPointInObstacle(check)) return false;
    }
    return true;
}
