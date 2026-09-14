#include <SDL2/SDL.h>
#include <iostream>
#include <iomanip>
#include <algorithm> 
#include "ihrs/algo/decision/mdp/value_iteration.hpp"

using namespace ihrs::algo::decision;

const int TILE_SIZE = 80;

void drawPolicyTriangle(SDL_Renderer* ren, int x, int y, int action) {
    int cx = x * TILE_SIZE + TILE_SIZE / 2;
    int cy = y * TILE_SIZE + TILE_SIZE / 2;
    int r = 10;

    SDL_Point points[4];
    if (action == 0) { points[0] = {cx, cy - r}; points[1] = {cx - r, cy + r}; points[2] = {cx + r, cy + r}; }
    else if (action == 1) { points[0] = {cx + r, cy}; points[1] = {cx - r, cy - r}; points[2] = {cx - r, cy + r}; }
    else if (action == 2) { points[0] = {cx, cy + r}; points[1] = {cx - r, cy - r}; points[2] = {cx + r, cy - r}; }
    else if (action == 3) { points[0] = {cx - r, cy}; points[1] = {cx + r, cy - r}; points[2] = {cx + r, cy + r}; }
    points[3] = points[0];

    SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
    SDL_RenderDrawLines(ren, points, 4);
    
    // Thicken
    points[0].y += (action==0?2:(action==2?-2:0)); points[0].x += (action==1?-2:(action==3?2:0));
    SDL_RenderDrawLines(ren, points, 4);
}

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    
    int cols = 10;
    int rows = 8;
    MDPSolver mdp(cols, rows);

    // Setup Map
    mdp.setGoal(9, 0, 10.0);       
    mdp.setHazard(9, 1, -10.0);    
    mdp.setHazard(7, 0, -10.0);    
    for(int y=3; y<6; ++y) mdp.setWall(4, y);

    SDL_Window* win = SDL_CreateWindow("Left=Wall, Right=Clear, Space=Regenerate", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, cols * TILE_SIZE, rows * TILE_SIZE, SDL_WINDOW_SHOWN);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    bool running = true;
    bool mouse_down = false;
    int mouse_button = 0;

    while(running) {
        SDL_Event e;
        while(SDL_PollEvent(&e)) {
            if(e.type == SDL_QUIT) running = false;
            
            // Interaction: Space to Reset Values
            if(e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_SPACE) {
                mdp.resetValues();
            }

            // Mouse Down
            if(e.type == SDL_MOUSEBUTTONDOWN) {
                mouse_down = true;
                mouse_button = e.button.button;
            }
            if(e.type == SDL_MOUSEBUTTONUP) {
                mouse_down = false;
            }
        }

        // Handle Dragging for painting walls
        if(mouse_down) {
            int mx, my;
            SDL_GetMouseState(&mx, &my);
            int gx = mx / TILE_SIZE;
            int gy = my / TILE_SIZE;

            if (mouse_button == SDL_BUTTON_LEFT) {
                 if (!mdp.isWall(gx, gy)) mdp.setWall(gx, gy);
            }
            else if (mouse_button == SDL_BUTTON_RIGHT) {
                 mdp.clearWall(gx, gy);
            }
        }

        // Run Solver Step
        mdp.solve(0.9, 0.2);

        // --- RENDER ---
        SDL_SetRenderDrawColor(ren, 50, 50, 50, 255);
        SDL_RenderClear(ren);

        const auto& grid = mdp.getGrid();
        for(const auto& s : grid) {
            SDL_Rect r = {s.x * TILE_SIZE, s.y * TILE_SIZE, TILE_SIZE, TILE_SIZE};
            int red = 255, grn = 255, blu = 255;
            
            if (mdp.isWall(s.x, s.y)) {
                red = 40; grn = 40; blu = 40; 
            } else if (s.is_terminal) {
                if (s.reward > 0) { grn = 255; red = 0; blu=0; } 
                else { red = 255; grn = 0; blu=0; }
            } else {
                double v = std::clamp(s.value, -10.0, 10.0);
                if (v > 0) {
                    double ratio = v / 10.0;
                    red = 255 - (int)(155 * ratio);
                    blu = 255 - (int)(155 * ratio);
                    grn = 255; 
                } else {
                    double ratio = -v / 10.0;
                    grn = 255 - (int)(155 * ratio);
                    blu = 255 - (int)(155 * ratio);
                    red = 255;
                }
            }

            SDL_SetRenderDrawColor(ren, red, grn, blu, 255);
            SDL_RenderFillRect(ren, &r);
            SDL_SetRenderDrawColor(ren, 200, 200, 200, 255);
            SDL_RenderDrawRect(ren, &r);

            if (!s.is_terminal && !mdp.isWall(s.x, s.y)) {
                drawPolicyTriangle(ren, s.x, s.y, s.policy);
            }
        }

        SDL_RenderPresent(ren);
        SDL_Delay(30);
    }

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
