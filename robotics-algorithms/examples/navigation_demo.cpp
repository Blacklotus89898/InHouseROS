#include <SDL2/SDL.h>
#include <vector>
#include <iostream>
#include <cmath>
#include "robotics_algo/common/types.hpp"
#include "robotics_algo/planning/graph_search/astar.hpp" 
#include "robotics_algo/control/pure_pursuit.hpp"

using namespace robotics;
using namespace robotics::planning;
using namespace robotics::control;

// Constants
const int CELL_SIZE = 20;
const int MAP_W = 40; // 800 / 20
const int MAP_H = 30; // 600 / 20

// Helper: Convert Screen (Pixels) -> GridPoint
GridPoint toGrid(Vector2 pos) {
    int x = static_cast<int>(pos.x() / CELL_SIZE);
    int y = static_cast<int>(pos.y() / CELL_SIZE);
    if(x < 0) x = 0; if(x >= MAP_W) x = MAP_W - 1;
    if(y < 0) y = 0; if(y >= MAP_H) y = MAP_H - 1;
    return {x, y};
}

// Helper: Convert GridPoint -> Screen (Pixels)
Vector2 toWorld(GridPoint pt) {
    return Vector2(pt.x * CELL_SIZE + CELL_SIZE/2.0, pt.y * CELL_SIZE + CELL_SIZE/2.0);
}

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("Auto Nav (Click to Pathfind)", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_SHOWN);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    // 1. SETUP MAP (Fix: Use the constructor you defined in grid_map.hpp)
    GridMap map(MAP_W, MAP_H);

    // Build Walls (Using your setObstacle helper)
    for(int y=5; y<25; ++y) map.setObstacle(20, y); // Vertical Wall
    for(int x=5; x<15; ++x) map.setObstacle(x, 15); // Horizontal Wall

    // 2. Systems
    PurePursuit tracker(30.0, 3.0); // Lookahead=30px, Speed=3.0

    Pose2D robot(50, 50, 0); 
    std::vector<Vector2> current_path;

    bool running = true;
    while(running) {
        SDL_Event e;
        while(SDL_PollEvent(&e)) {
            if(e.type == SDL_QUIT) running = false;
            
            // --- PLAN PATH ON CLICK ---
            if(e.type == SDL_MOUSEBUTTONDOWN) {
                Vector2 target(e.button.x, e.button.y);
                
                GridPoint start_pt = toGrid(robot.position);
                GridPoint goal_pt = toGrid(target);

                // Run A* (Static method)
                std::vector<GridPoint> grid_path = AStar::plan(map, start_pt, goal_pt);
                
                // Convert to World Path
                current_path.clear();
                for(auto& pt : grid_path) {
                    current_path.push_back(toWorld(pt));
                }
            }
        }

        // --- CONTROL LOOP ---
        if(!current_path.empty()) {
            // 1. Get Control Command
            std::pair<double, double> cmd = tracker.getCommand(robot, current_path);
            double v = cmd.first;
            double w = cmd.second;
            
            // 2. Apply Kinematics
            robot.theta += w;
            robot.position.x() += std::cos(robot.theta) * v;
            robot.position.y() += std::sin(robot.theta) * v;
            
            // 3. Check if reached goal
            if ((robot.position - current_path.back()).norm() < 10.0) {
                current_path.clear(); 
            }
        }

        // --- RENDER ---
        SDL_SetRenderDrawColor(ren, 240, 240, 240, 255); // White BG
        SDL_RenderClear(ren);

        // Draw Map
        SDL_SetRenderDrawColor(ren, 50, 50, 50, 255);
        for(int y=0; y<MAP_H; ++y) {
            for(int x=0; x<MAP_W; ++x) {
                // Use isObstacle helper
                if(map.isObstacle(x, y)) {
                    SDL_Rect r = {x*CELL_SIZE, y*CELL_SIZE, CELL_SIZE, CELL_SIZE};
                    SDL_RenderFillRect(ren, &r);
                }
            }
        }

        // Draw Path
        if(!current_path.empty()) {
            SDL_SetRenderDrawColor(ren, 0, 200, 0, 255);
            for(size_t i=0; i<current_path.size()-1; ++i) {
                SDL_RenderDrawLine(ren, current_path[i].x(), current_path[i].y(), current_path[i+1].x(), current_path[i+1].y());
            }
        }

        // Draw Robot
        SDL_SetRenderDrawColor(ren, 0, 0, 255, 255);
        SDL_Rect r = {(int)robot.position.x()-8, (int)robot.position.y()-8, 16, 16};
        SDL_RenderFillRect(ren, &r);
        
        // Draw Heading
        int nx = robot.position.x() + std::cos(robot.theta)*15;
        int ny = robot.position.y() + std::sin(robot.theta)*15;
        SDL_RenderDrawLine(ren, robot.position.x(), robot.position.y(), nx, ny);

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
