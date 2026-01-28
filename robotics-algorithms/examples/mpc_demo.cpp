#include <SDL2/SDL.h>
#include <vector>
#include <iostream>
#include <cmath>
#include "robotics_algo/control/mpc/mpc_solver.hpp"

using namespace robotics::control;

// Draw a simple triangle robot
void drawRobot(SDL_Renderer* ren, State s) {
    // Robot points in local space
    double pts[3][2] = { {15, 0}, {-10, -10}, {-10, 10} };
    
    // Transform to world space
    SDL_Point sdl_pts[4];
    for(int i=0; i<3; ++i) {
        double x = pts[i][0];
        double y = pts[i][1];
        // Rotate & Translate
        sdl_pts[i].x = s.x + (x * std::cos(s.theta) - y * std::sin(s.theta));
        sdl_pts[i].y = s.y + (x * std::sin(s.theta) + y * std::cos(s.theta));
    }
    sdl_pts[3] = sdl_pts[0]; // Close loop
    
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
    SDL_RenderDrawLines(ren, sdl_pts, 4);
}

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("MPC (Green=Prediction)", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_SHOWN);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    // Setup MPC: Look 30 steps ahead (3 seconds)
    MPCSolver mpc(30, 0.1); 
    
    State robot = {100, 100, 0};
    State target = {400, 300, 0}; // Initial Target

    bool running = true;
    while(running) {
        SDL_Event e;
        while(SDL_PollEvent(&e)) {
            if(e.type == SDL_QUIT) running = false;
        }

        // Target follows mouse
        int mx, my;
        SDL_GetMouseState(&mx, &my);
        target.x = mx;
        target.y = my;

        // --- MPC STEP ---
        ControlInput u = mpc.solve(robot, target);

        // Physics Step (Apply Control)
        // Note: We use the same physics model here as the solver uses
        // In real life, reality is different from the model (Process Noise)
        robot.x += u.v * std::cos(robot.theta) * 0.1;
        robot.y += u.v * std::sin(robot.theta) * 0.1;
        robot.theta += u.w * 0.1;

        // --- RENDER ---
        SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
        SDL_RenderClear(ren);

        // 1. Draw Target
        SDL_SetRenderDrawColor(ren, 255, 50, 50, 255);
        SDL_Rect r = {(int)target.x-5, (int)target.y-5, 10, 10};
        SDL_RenderFillRect(ren, &r);

        // 2. Draw Prediction Horizon (The "Plan")
        // This shows exactly what the robot *intends* to do
        std::vector<State> horizon = mpc.getPredictedPath();
        SDL_SetRenderDrawColor(ren, 50, 255, 50, 255);
        for(size_t i=0; i<horizon.size()-1; ++i) {
            SDL_RenderDrawLine(ren, (int)horizon[i].x, (int)horizon[i].y, 
                                    (int)horizon[i+1].x, (int)horizon[i+1].y);
        }

        // 3. Draw Robot
        drawRobot(ren, robot);

        SDL_RenderPresent(ren);
        SDL_Delay(16);
    }

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
