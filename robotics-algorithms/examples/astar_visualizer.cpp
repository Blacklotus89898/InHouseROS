#include <SDL2/SDL.h>
#include <vector>
#include <iostream>
#include "robotics_algo/planning/graph_search/astar.hpp"

// Configuration
const int TILE_SIZE = 25;
const int COLS = 32;
const int ROWS = 24;
const int SCREEN_WIDTH = COLS * TILE_SIZE;
const int SCREEN_HEIGHT = ROWS * TILE_SIZE;

using namespace robotics::planning;

void drawRect(SDL_Renderer* renderer, int x, int y, int r, int g, int b) {
    SDL_SetRenderDrawColor(renderer, r, g, b, 255);
    SDL_Rect rect = {x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE - 1, TILE_SIZE - 1}; // -1 for grid lines
    SDL_RenderFillRect(renderer, &rect);
}

int main() {
    // 1. Init SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) return 1;
    SDL_Window* window = SDL_CreateWindow("A* Interactive (Left Click: Wall, Right Click: Clear)", 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    // 2. Setup Map
    GridMap map(COLS, ROWS);
    GridPoint start = {2, ROWS / 2};
    GridPoint goal = {COLS - 3, ROWS / 2};

    bool running = true;
    bool mouseDown = false;
    int mouseButton = 0; // 1=Left, 3=Right

    while (running) {
        // --- INPUT HANDLING ---
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
            
            // Mouse Interaction
            if (event.type == SDL_MOUSEBUTTONDOWN) {
                mouseDown = true;
                mouseButton = event.button.button;
            }
            if (event.type == SDL_MOUSEBUTTONUP) {
                mouseDown = false;
            }
        }

        // Draw/Erase Walls if mouse is held
        if (mouseDown) {
            int x, y;
            SDL_GetMouseState(&x, &y);
            int gridX = x / TILE_SIZE;
            int gridY = y / TILE_SIZE;

            // Safety check bounds
            if (gridX >= 0 && gridX < COLS && gridY >= 0 && gridY < ROWS) {
                // Don't overwrite start/goal
                GridPoint p = {gridX, gridY};
                if (p != start && p != goal) {
                    if (mouseButton == SDL_BUTTON_LEFT) map.data[gridY * COLS + gridX] = 1; // Wall
                    if (mouseButton == SDL_BUTTON_RIGHT) map.data[gridY * COLS + gridX] = 0; // Eraser
                }
            }
        }

        // --- PLANNING STEP ---
        // We re-plan every frame to see updates instantly
        std::vector<GridPoint> path = AStar::plan(map, start, goal);

        // --- RENDER STEP ---
        SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255); // Dark Background
        SDL_RenderClear(renderer);

        // 1. Draw Walls
        for (int y = 0; y < ROWS; ++y) {
            for (int x = 0; x < COLS; ++x) {
                if (map.isObstacle(x, y)) {
                    drawRect(renderer, x, y, 100, 100, 100); // Grey Wall
                }
            }
        }

        // 2. Draw Path (Blue)
        // Skip first and last to keep Start/Goal visible
        for (size_t i = 0; i < path.size(); ++i) {
            GridPoint p = path[i];
            if (p != start && p != goal) {
                drawRect(renderer, p.x, p.y, 50, 100, 255); // Light Blue
            }
        }

        // 3. Draw Start (Green) & Goal (Red)
        drawRect(renderer, start.x, start.y, 50, 200, 50);
        drawRect(renderer, goal.x, goal.y, 255, 50, 50);

        SDL_RenderPresent(renderer);
        SDL_Delay(16); // ~60 FPS
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
