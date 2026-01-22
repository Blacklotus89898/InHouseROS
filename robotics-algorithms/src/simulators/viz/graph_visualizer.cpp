#include "robotics_algo/simulators/viz/graph_visualizer.hpp"
#include <iostream>

namespace robotics::viz {

    GraphVisualizer::GraphVisualizer(const std::string& title, int w, int h) 
        : width_(w), height_(h) {
        
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            std::cerr << "SDL Error: " << SDL_GetError() << std::endl;
            running_ = false;
            return;
        }
        
        window_ = SDL_CreateWindow(title.c_str(), 
            SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width_, height_, SDL_WINDOW_SHOWN);
        renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
    }

    GraphVisualizer::~GraphVisualizer() {
        if (renderer_) SDL_DestroyRenderer(renderer_);
        if (window_) SDL_DestroyWindow(window_);
        SDL_Quit();
    }

    void GraphVisualizer::clearHistory() {
        history_.clear();
    }

    int GraphVisualizer::mapY(double val) const {
        // Map value to screen coordinates (Inverted Y)
        double ratio = (val - min_val_) / (max_val_ - min_val_);
        return height_ - (int)(ratio * height_);
    }

    void GraphVisualizer::processEvents() {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running_ = false;
            // Allow SPACE to clear graph externally if needed
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_SPACE) {
                clearHistory();
            }
        }
    }

    void GraphVisualizer::update(double target_val, double actual_val) {
        processEvents();
        if (!running_) return;

        // 1. Store Data
        history_.push_back(mapY(actual_val));
        if (history_.size() > (size_t)width_) {
            history_.erase(history_.begin());
        }

        // 2. Render
        SDL_SetRenderDrawColor(renderer_, 30, 30, 30, 255); // Dark BG
        SDL_RenderClear(renderer_);

        // Draw Target Line (Red)
        SDL_SetRenderDrawColor(renderer_, 200, 50, 50, 255);
        int ty = mapY(target_val);
        SDL_RenderDrawLine(renderer_, 0, ty, width_, ty);

        // Draw History (Green)
        SDL_SetRenderDrawColor(renderer_, 50, 200, 50, 255);
        for (size_t i = 1; i < history_.size(); ++i) {
            SDL_RenderDrawLine(renderer_, i - 1, history_[i - 1], i, history_[i]);
        }

        SDL_RenderPresent(renderer_);
        SDL_Delay(16); // ~60 FPS cap
    }

}
