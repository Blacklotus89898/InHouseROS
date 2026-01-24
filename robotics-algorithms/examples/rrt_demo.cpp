#include <SDL2/SDL.h>
#include <vector>
#include <iostream>
#include "robotics_algo/planning/sampling_based/rrt.hpp"

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
    SDL_Window* win = SDL_CreateWindow("RRT Animation (Space to Reset)", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_SHOWN);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    Vector2 start(50, 50);
    Vector2 goal(750, 550);
    
    RRT rrt(20.0);
    rrt.reset(start, goal);

    bool running = true;
    bool finished = false;

    while(running) {
        SDL_Event e;
        while(SDL_PollEvent(&e)) {
            if(e.type == SDL_QUIT) running = false;
            
            // Mouse to move Start/Goal
            if(e.type == SDL_MOUSEBUTTONDOWN) {
                if(e.button.button == SDL_BUTTON_LEFT) start = Vector2(e.button.x, e.button.y);
                if(e.button.button == SDL_BUTTON_RIGHT) goal = Vector2(e.button.x, e.button.y);
                rrt.reset(start, goal);
                finished = false;
            }
            // Space to Restart
            if(e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_SPACE) {
                rrt.reset(start, goal);
                finished = false;
            }
        }

        // --- ANIMATION STEP ---
        // Run 5 expansion steps per frame to make it faster (but still visible)
        if (!finished) {
            for(int i=0; i<5; ++i) {
                if(rrt.step(obstacles, OBSTACLE_RADIUS)) {
                    finished = true;
                    break;
                }
            }
        }

        // --- RENDER ---
        SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
        SDL_RenderClear(ren);

        // Obstacles
        SDL_SetRenderDrawColor(ren, 80, 80, 80, 255);
        for(auto& obs : obstacles) drawCircle(ren, (int)obs.x(), (int)obs.y(), (int)OBSTACLE_RADIUS);

        // Tree
        SDL_SetRenderDrawColor(ren, 150, 150, 150, 100);
        const auto& tree = rrt.getTree();
        for(const auto& node : tree) {
            if(node.parent_index != -1) {
                Vector2 p = tree[node.parent_index].pos;
                SDL_RenderDrawLine(ren, (int)p.x(), (int)p.y(), (int)node.pos.x(), (int)node.pos.y());
            }
        }

        // Path (Only if finished)
        if (finished) {
            std::vector<Vector2> path = rrt.getPath();
            SDL_SetRenderDrawColor(ren, 50, 100, 255, 255);
            for(size_t i=0; i<path.size()-1; ++i) {
                SDL_RenderDrawLine(ren, (int)path[i].x(), (int)path[i].y(), (int)path[i+1].x(), (int)path[i+1].y());
                SDL_RenderDrawLine(ren, (int)path[i].x()+1, (int)path[i].y()+1, (int)path[i+1].x()+1, (int)path[i+1].y()+1);
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
