#include <SDL2/SDL.h>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <limits>

// --- Constants ---
const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;
const int NODE_RADIUS = 3;
const int STEP_SIZE = 20; // How far the tree grows per step

// --- Structs ---
struct Point {
    float x, y;
};

struct Node {
    Point point;
    int parentIndex; // Index in the nodes vector (-1 for root)
};

struct Obstacle {
    int x, y, w, h;
};

// --- Helper Functions ---

// Euclidean Distance Squared (faster than sqrt)
float distSq(Point p1, Point p2) {
    return (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y);
}

// Check if a point is inside an obstacle
bool isPointInObstacle(Point p, const std::vector<Obstacle>& obstacles) {
    for (const auto& obs : obstacles) {
        if (p.x > obs.x && p.x < obs.x + obs.w &&
            p.y > obs.y && p.y < obs.y + obs.h) {
            return true;
        }
    }
    return false;
}

// Check if the line segment between p1 and p2 intersects any obstacle
// (Simple implementation: sample points along the line)
bool isPathClear(Point p1, Point p2, const std::vector<Obstacle>& obstacles) {
    float dist = std::sqrt(distSq(p1, p2));
    int steps = dist / 5; // Check every 5 pixels
    
    for (int i = 0; i <= steps; i++) {
        float t = (float)i / steps;
        Point check = {
            p1.x + t * (p2.x - p1.x),
            p1.y + t * (p2.y - p1.y)
        };
        if (isPointInObstacle(check, obstacles)) return false;
    }
    return true;
}

// --- RRT Class ---
class RRT {
public:
    std::vector<Node> nodes;
    std::vector<Obstacle> obstacles;
    Point start, goal;
    bool goalReached = false;

    RRT(Point s, Point g) : start(s), goal(g) {
        nodes.push_back({start, -1}); // Add root node
        
        // Add a big obstacle in the middle
        obstacles.push_back({300, 100, 100, 400});
        obstacles.push_back({100, 300, 200, 50});
    }

    void step() {
        if (goalReached) return;

        // 1. Random Sampling
        // 5% chance to sample the goal directly (bias)
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

        // 3. Steer (Create new point in direction of random point)
        Point nearest = nodes[nearestIdx].point;
        float theta = atan2(randPoint.y - nearest.y, randPoint.x - nearest.x);
        
        Point newPoint = {
            nearest.x + STEP_SIZE * cos(theta),
            nearest.y + STEP_SIZE * sin(theta)
        };

        // 4. Collision Check & Add
        // Check bounds
        if (newPoint.x < 0 || newPoint.x >= SCREEN_WIDTH || newPoint.y < 0 || newPoint.y >= SCREEN_HEIGHT) return;

        if (isPathClear(nearest, newPoint, obstacles)) {
            nodes.push_back({newPoint, nearestIdx});
            
            // Check if close to goal
            if (distSq(newPoint, goal) < 400) { // Within 20 pixels
                goalReached = true;
                nodes.push_back({goal, (int)nodes.size() - 1});
                std::cout << "Goal Reached! Total Nodes: " << nodes.size() << std::endl;
            }
        }
    }

    void draw(SDL_Renderer* renderer) {
        // Draw Obstacles
        SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255); // Grey
        for (const auto& obs : obstacles) {
            SDL_Rect r = {obs.x, obs.y, obs.w, obs.h};
            SDL_RenderFillRect(renderer, &r);
        }

        // Draw Tree Edges
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255); // Green Lines
        for (const auto& node : nodes) {
            if (node.parentIndex != -1) {
                Point p1 = node.point;
                Point p2 = nodes[node.parentIndex].point;
                SDL_RenderDrawLine(renderer, (int)p1.x, (int)p1.y, (int)p2.x, (int)p2.y);
            }
        }

        // Draw Path if Goal Reached
        if (goalReached) {
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // Red Path
            int curr = nodes.size() - 1;
            while (curr != -1) {
                Point p1 = nodes[curr].point;
                int parent = nodes[curr].parentIndex;
                if (parent != -1) {
                    Point p2 = nodes[parent].point;
                    // Draw thick line (cheat by drawing offsets)
                    SDL_RenderDrawLine(renderer, p1.x, p1.y, p2.x, p2.y);
                    SDL_RenderDrawLine(renderer, p1.x+1, p1.y+1, p2.x+1, p2.y+1);
                }
                curr = parent;
            }
        }
        
        // Draw Start and Goal
        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255); // Blue
        SDL_Rect sRect = {(int)start.x - 5, (int)start.y - 5, 10, 10};
        SDL_Rect gRect = {(int)goal.x - 5, (int)goal.y - 5, 10, 10};
        SDL_RenderFillRect(renderer, &sRect);
        SDL_RenderFillRect(renderer, &gRect);
    }
};

int main(int argc, char* argv[]) {
    srand(time(NULL));
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("RRT Visualization", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    RRT rrt({50, 50}, {750, 550});

    bool running = true;
    SDL_Event event;

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
        }

        // Perform one RRT expansion step per frame
        rrt.step();

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        
        rrt.draw(renderer);
        
        SDL_RenderPresent(renderer);
        SDL_Delay(10); // Slow down slightly to see the growth
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
