#include <SDL2/SDL.h>
#include <iostream>
#include "robotics_algo/kinematics/two_link_arm.hpp"

using namespace robotics::kinematics;

const int WIDTH = 800;
const int HEIGHT = 600;
const int CENTER_X = WIDTH / 2;
const int CENTER_Y = HEIGHT / 2;

// Helper to draw thick lines (links)
void drawLink(SDL_Renderer* ren, int x1, int y1, int x2, int y2, int r, int g, int b) {
    SDL_SetRenderDrawColor(ren, r, g, b, 255);
    // Draw simple line for now (SDL_RenderDrawLine)
    // To make it thick, we just draw 3 offset lines
    for(int i=-2; i<=2; ++i) SDL_RenderDrawLine(ren, x1+i, y1+i, x2+i, y2+i);
}

int main() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) return 1;
    SDL_Window* window = SDL_CreateWindow("2-Link IK (Move Mouse)", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    // Setup Arm: Upper arm 150px, Forearm 100px
    TwoLinkArm arm(150.0, 100.0);
    JointState current_joints = {0.0, 0.0};

    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
        }

        // 1. Get Mouse Position (Target)
        int mx, my;
        SDL_GetMouseState(&mx, &my);
        
        // Convert Screen Space (Top-Left 0,0) to Robot Space (Center 0,0, Y-Up)
        double target_x = mx - CENTER_X;
        double target_y = CENTER_Y - my; // Invert Y

        // 2. Solve IK
        robotics::Vector2 target(target_x, target_y);
        auto solution = arm.inverseKinematics(target);

        if (solution.has_value()) {
            current_joints = solution.value();
        } 
        // If no solution (out of reach), we just keep the last valid angles

        // 3. Render
        SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
        SDL_RenderClear(renderer);

        // Calculate Joint Positions for drawing
        // P0 = Base (0,0)
        // P1 = Elbow (Computed via partial FK)
        // P2 = Hand (Computed via full FK)
        
        int p0_x = CENTER_X;
        int p0_y = CENTER_Y;

        double t1 = current_joints.theta1;
        double t2 = current_joints.theta2;

        // Elbow Position
        int p1_x = CENTER_X + (int)(arm.getL1() * std::cos(t1));
        int p1_y = CENTER_Y - (int)(arm.getL1() * std::sin(t1)); // Invert Y for screen

        // Hand Position (using the full FK function logic)
        double global_t2 = t1 + t2;
        int p2_x = p1_x + (int)(arm.getL2() * std::cos(global_t2));
        int p2_y = p1_y - (int)(arm.getL2() * std::sin(global_t2));

        // Draw Links
        drawLink(renderer, p0_x, p0_y, p1_x, p1_y, 50, 100, 255);  // Blue (Upper)
        drawLink(renderer, p1_x, p1_y, p2_x, p2_y, 255, 50, 50);   // Red (Forearm)

        // Draw Target (Green Dot)
        SDL_SetRenderDrawColor(renderer, 50, 255, 50, 255);
        SDL_Rect r = {mx - 4, my - 4, 8, 8};
        SDL_RenderFillRect(renderer, &r);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
