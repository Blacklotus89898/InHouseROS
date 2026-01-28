#include <SDL2/SDL.h>
#include <vector>
#include <iostream>
#include "robotics_algo/planning/sampling_based/prm.hpp"

// FIX: Bring in the main namespace for Vector2
using namespace robotics;
using namespace robotics::planning;

// FIX: Explicitly tell C++ this is a vector of Vector2
std::vector<Vector2> obstacles = {
    {400, 300}, {200, 150}, {600, 450}, {300, 400}, {500, 200}
};
double OBSTACLE_RADIUS = 40.0;

void drawCircle(SDL_Renderer* ren, int cx, int cy, int r) {
    for (int w = 0; w < r * 2; w++) {
        for (int h = 0; h < r * 2; h++) {
            int dx = r - w;
            int dy = r - h;
            if ((dx*dx + dy*dy) <= (r * r)) {
                SDL_RenderDrawPoint(ren, cx + dx, cy + dy);
            }
        }
    }
}

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("PRM: Grey=Roadmap, Blue=Path", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_SHOWN);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    Vector2 start(50, 50);
    Vector2 goal(750, 550);
    
    // Create PRM with 200 nodes and max connection distance 150px
    PRM prm(200, 150.0);
    
    // Build roadmap ONCE
    std::cout << "Building Roadmap..." << std::endl;
    prm.buildRoadmap(obstacles, OBSTACLE_RADIUS, 800, 600);
    std::cout << "Roadmap Built with " << prm.getGraph().size() << " nodes." << std::endl;

    std::vector<Vector2> path;
    bool needs_replan = true;
    bool running = true;

    while(running) {
        SDL_Event e;
        while(SDL_PollEvent(&e)) {
            if(e.type == SDL_QUIT) running = false;
            
            if(e.type == SDL_MOUSEBUTTONDOWN) {
                if(e.button.button == SDL_BUTTON_LEFT) {
                    start = Vector2(e.button.x, e.button.y);
                } else if(e.button.button == SDL_BUTTON_RIGHT) {
                    goal = Vector2(e.button.x, e.button.y);
                }
                needs_replan = true;
            }
            
            if(e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_SPACE) {
                // Rebuild the whole map (Takes time, but new random samples)
                prm.buildRoadmap(obstacles, OBSTACLE_RADIUS, 800, 600);
                needs_replan = true;
            }
        }

        if (needs_replan) {
            path = prm.plan(start, goal);
            needs_replan = false;
        }

        // --- RENDER ---
        SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
        SDL_RenderClear(ren);

        // 1. Draw Obstacles
        SDL_SetRenderDrawColor(ren, 100, 100, 100, 255);
        for(auto& obs : obstacles) drawCircle(ren, (int)obs.x(), (int)obs.y(), (int)OBSTACLE_RADIUS);

        // 2. Draw Roadmap (The Graph) - Thin Grey Lines
        SDL_SetRenderDrawColor(ren, 60, 60, 60, 255);
        const auto& graph = prm.getGraph();
        for(const auto& node : graph) {
            // Draw nodes
            SDL_RenderDrawPoint(ren, (int)node.pos.x(), (int)node.pos.y());
            // Draw edges
            for(int neighbor_id : node.neighbors) {
                Vector2 n_pos = graph[neighbor_id].pos;
                SDL_RenderDrawLine(ren, (int)node.pos.x(), (int)node.pos.y(), 
                                        (int)n_pos.x(), (int)n_pos.y());
            }
        }

        // 3. Draw Path (Blue) - Thick Line
        if(!path.empty()) {
            SDL_SetRenderDrawColor(ren, 50, 100, 255, 255);
            for(size_t i=0; i<path.size()-1; ++i) {
                SDL_RenderDrawLine(ren, (int)path[i].x(), (int)path[i].y(), 
                                        (int)path[i+1].x(), (int)path[i+1].y());
                // Thickness
                SDL_RenderDrawLine(ren, (int)path[i].x()+1, (int)path[i].y()+1, 
                                        (int)path[i+1].x()+1, (int)path[i+1].y()+1);
            }
        }

        // 4. Start/Goal
        SDL_SetRenderDrawColor(ren, 50, 255, 50, 255);
        drawCircle(ren, (int)start.x(), (int)start.y(), 8);
        SDL_SetRenderDrawColor(ren, 255, 50, 50, 255);
        drawCircle(ren, (int)goal.x(), (int)goal.y(), 8);

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
