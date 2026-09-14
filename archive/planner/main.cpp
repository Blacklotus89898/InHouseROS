#include <SDL2/SDL.h>
#include <ctime>
#include "RRT.h"
#include "PRM.h" // Include the new PRM header
#include "Common.h"

const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;

enum Mode { MODE_RRT, MODE_PRM };

int main(int argc, char* argv[]) {
    srand(time(NULL));

    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("Path Planning Demo (Space to Switch)", 
                                          SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 
                                          SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    // Initialize both planners
    Point start = {50, 50};
    Point goal = {750, 550};
    
    // We use pointers so we can re-create them easily if we want to reset
    RRT* rrtPlanner = new RRT(start, goal);
    PRM* prmPlanner = new PRM(start, goal);

    Mode currentMode = MODE_RRT;
    bool running = true;
    SDL_Event event;

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
            
            // Handle Input
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_SPACE) {
                    // Switch Mode
                    if (currentMode == MODE_RRT) {
                        currentMode = MODE_PRM;
                        SDL_SetWindowTitle(window, "Current Mode: PRM (Probabilistic Roadmap)");
                        prmPlanner->reset(); // Restart PRM cleanly
                    } else {
                        currentMode = MODE_RRT;
                        SDL_SetWindowTitle(window, "Current Mode: RRT (Rapidly-exploring Random Tree)");
                        // Reset RRT (requires deleting and re-newing in this simple design)
                        delete rrtPlanner;
                        rrtPlanner = new RRT(start, goal);
                    }
                }
            }
        }

        // Logic & Render
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        if (currentMode == MODE_RRT) {
            rrtPlanner->step();
            rrtPlanner->draw(renderer);
        } else {
            prmPlanner->step();
            prmPlanner->draw(renderer);
        }

        SDL_RenderPresent(renderer);
        // Removed explicit Delay because we used SDL_RENDERER_PRESENTVSYNC
    }

    delete rrtPlanner;
    delete prmPlanner;
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
