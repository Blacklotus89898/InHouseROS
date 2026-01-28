#include <SDL2/SDL.h>
#include <vector>
#include <iostream>
#include "robotics_algo/planning/sampling_based/rrt_star.hpp"

using namespace robotics;
using namespace robotics::planning;

std::vector<Vector2> obstacles = {
    {400, 300}, {200, 150}, {600, 450}, {300, 400}, {500, 200}
};
double OBSTACLE_RADIUS = 40.0;

void drawCircle(SDL_Renderer* ren, int cx, int cy, int r) {
    for (int w = 0; w < r * 2; w++) {
        for (int h = 0; h < r * 2; h++) {
            int dx = r - w; int dy = r - h;
            if ((dx*dx + dy*dy) <= (r * r)) SDL_RenderDrawPoint(ren, cx + dx, cy + dy);
        }
    }
}

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("RRT*: Optimizing Path (Watch branches snap!)", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_SHOWN);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    Vector2 start(50, 50);
    Vector2 goal(750, 550);
    
    // RRT* with 2000 iterations
    RRTStar planner(15.0, 2000, 40.0);
    std::vector<Vector2> path;
    bool needs_plan = true;
    bool running = true;

    while(running) {
        SDL_Event e;
        while(SDL_PollEvent(&e)) {
            if(e.type == SDL_QUIT) running = false;
            if(e.type == SDL_MOUSEBUTTONDOWN) {
                if(e.button.button == SDL_BUTTON_LEFT) start = Vector2(e.button.x, e.button.y);
                if(e.button.button == SDL_BUTTON_RIGHT) goal = Vector2(e.button.x, e.button.y);
                needs_plan = true;
            }
            if(e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_SPACE) needs_plan = true;
        }

        if(needs_plan) {
            // Plan runs fully in one frame (might pause briefly)
            path = planner.plan(start, goal, obstacles, OBSTACLE_RADIUS);
            needs_plan = false;
        }

        // --- RENDER ---
        SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
        SDL_RenderClear(ren);

        // Obstacles
        SDL_SetRenderDrawColor(ren, 100, 100, 100, 255);
        for(auto& obs : obstacles) drawCircle(ren, (int)obs.x(), (int)obs.y(), (int)OBSTACLE_RADIUS);

        // Tree (Notice the structure is much cleaner than standard RRT)
        SDL_SetRenderDrawColor(ren, 100, 100, 100, 80);
        const auto& nodes = planner.getTree();
        for(const auto& node : nodes) {
            if(node.parent_index != -1) {
                Vector2 p = nodes[node.parent_index].pos;
                SDL_RenderDrawLine(ren, (int)p.x(), (int)p.y(), (int)node.pos.x(), (int)node.pos.y());
            }
        }

        // Path (Blue)
        if(!path.empty()) {
            SDL_SetRenderDrawColor(ren, 50, 100, 255, 255);
            for(size_t i=0; i<path.size()-1; ++i) {
                SDL_RenderDrawLine(ren, (int)path[i].x(), (int)path[i].y(), 
                                        (int)path[i+1].x(), (int)path[i+1].y());
                SDL_RenderDrawLine(ren, (int)path[i].x()+1, (int)path[i].y()+1, 
                                        (int)path[i+1].x()+1, (int)path[i+1].y()+1);
            }
        }

        // Start/Goal
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
