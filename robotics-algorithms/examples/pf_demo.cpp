#include <SDL2/SDL.h>
#include <vector>
#include <iostream>
#include <cmath>
#include "robotics_algo/common/types.hpp"
#include "robotics_algo/planning/graph_search/grid_map.hpp" 
#include "robotics_algo/localization/particle_filter.hpp"

using namespace robotics;
using namespace robotics::planning;
using namespace robotics::localization;

const int CELL_SIZE = 20;
const int MAP_W = 40;
const int MAP_H = 30;

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("Particle Filter (Arrows=Move)", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_SHOWN);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    // 1. Setup Map
    GridMap map(MAP_W, MAP_H);
    // Box Perimeter
    for(int x=0; x<MAP_W; ++x) { map.setObstacle(x, 0); map.setObstacle(x, MAP_H-1); }
    for(int y=0; y<MAP_H; ++y) { map.setObstacle(0, y); map.setObstacle(MAP_W-1, y); }
    // Obstacles
    for(int y=10; y<20; ++y) map.setObstacle(20, y);
    for(int x=10; x<30; ++x) map.setObstacle(x, 25);

    // 2. Setup Robot & Filter
    Pose2D true_robot(100, 100, 0); 
    ParticleFilter pf(1000, map);    // 1000 Particles for dense cloud
    pf.init(100, 100, 0, 20.0);      // Spread out initially

    bool running = true;
    while(running) {
        SDL_Event e;
        while(SDL_PollEvent(&e)) if(e.type == SDL_QUIT) running = false;

        // --- CONTROLS ---
        double v = 0.0;
        double w = 0.0;
        const Uint8* keys = SDL_GetKeyboardState(NULL);
        if(keys[SDL_SCANCODE_UP]) v = 4.0;
        if(keys[SDL_SCANCODE_DOWN]) v = -4.0;
        if(keys[SDL_SCANCODE_LEFT]) w = -0.08;
        if(keys[SDL_SCANCODE_RIGHT]) w = 0.08;

        if (v != 0 || w != 0) {
            // A. COLLISION CHECK (New!)
            double nx = true_robot.position.x() + std::cos(true_robot.theta + w) * v;
            double ny = true_robot.position.y() + std::sin(true_robot.theta + w) * v;
            
            // Convert to grid coords to check map
            int gx = (int)(nx / CELL_SIZE);
            int gy = (int)(ny / CELL_SIZE);

            // Only move if valid
            if (!map.isObstacle(gx, gy)) {
                true_robot.theta += w;
                true_robot.position.x() = nx;
                true_robot.position.y() = ny;
            } else {
                // If we hit a wall, we stop moving (v=0), BUT we might still turn (w!=0)
                // For simplicity, let's just stop linear movement
                v = 0; 
                true_robot.theta += w; // You can still rotate in place
            }

            // B. Move Particles (Prediction)
            // Note: We pass the 'attempted' v even if we crashed, 
            // but in reality, odometry would report 0 if wheels don't turn.
            // Let's pass the *actual* v we managed to achieve.
            pf.predict(v, w);

            // C. Correct Particles (Update)
            pf.update(map);
            pf.resample();
        }

        // --- RENDER ---
        SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
        SDL_RenderClear(ren);

        // Draw Map
        SDL_SetRenderDrawColor(ren, 80, 80, 80, 255);
        for(int y=0; y<MAP_H; ++y) {
            for(int x=0; x<MAP_W; ++x) {
                if(map.isObstacle(x, y)) {
                    SDL_Rect r = {x*CELL_SIZE, y*CELL_SIZE, CELL_SIZE, CELL_SIZE};
                    SDL_RenderFillRect(ren, &r);
                }
            }
        }

        // Draw Particles (Red Dots)
        SDL_SetRenderDrawColor(ren, 255, 50, 50, 255);
        const auto& particles = pf.getParticles();
        for(const auto& p : particles) {
            SDL_RenderDrawPoint(ren, (int)p.x, (int)p.y);
        }

        // Draw True Robot (Blue Square)
        SDL_SetRenderDrawColor(ren, 50, 100, 255, 255);
        SDL_Rect r = {(int)true_robot.position.x()-8, (int)true_robot.position.y()-8, 16, 16};
        SDL_RenderFillRect(ren, &r);
        
        // Draw Nose (Yellow Line)
        SDL_SetRenderDrawColor(ren, 255, 255, 0, 255);
        int nose_x = true_robot.position.x() + std::cos(true_robot.theta) * 20;
        int nose_y = true_robot.position.y() + std::sin(true_robot.theta) * 20;
        SDL_RenderDrawLine(ren, true_robot.position.x(), true_robot.position.y(), nose_x, nose_y);

        // Draw Estimated Mean (Green Circle)
        Pose2D est = pf.getEstimate();
        SDL_SetRenderDrawColor(ren, 0, 255, 0, 255);
        SDL_Rect er = {(int)est.position.x()-4, (int)est.position.y()-4, 8, 8};
        SDL_RenderDrawRect(ren, &er); // Outline only

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
