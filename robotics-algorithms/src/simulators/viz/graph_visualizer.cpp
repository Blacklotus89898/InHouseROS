#include "robotics_algo/simulators/viz/graph_visualizer.hpp"
#include <iostream>
#include <algorithm>

namespace robotics::viz {

    GraphVisualizer::GraphVisualizer(const std::string& title, int w, int h) 
        : width_(w), height_(h) {
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            running_ = false; 
            return;
        }
        window_ = SDL_CreateWindow(title.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width_, height_, SDL_WINDOW_SHOWN);
        renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
    }

    GraphVisualizer::~GraphVisualizer() {
        if (renderer_) SDL_DestroyRenderer(renderer_);
        if (window_) SDL_DestroyWindow(window_);
        SDL_Quit();
    }

    void GraphVisualizer::clearHistory() {
        histories_.clear();
        min_val_ = std::numeric_limits<double>::max();
        max_val_ = std::numeric_limits<double>::lowest();
    }

    int GraphVisualizer::mapY(double val) const {
        double range = max_val_ - min_val_;
        if (range < 0.001) range = 1.0; 
        double ratio = (val - min_val_) / range;
        int padding = height_ * 0.1;
        return (height_ - padding) - (int)(ratio * (height_ - 2 * padding));
    }

    void GraphVisualizer::processEvents() {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running_ = false;
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_SPACE) clearHistory();
        }
    }

    void GraphVisualizer::update(const std::vector<double>& values, const std::vector<Color>& colors) {
        processEvents();
        if (!running_) return;

        // 1. Initialize Channels if needed
        if (histories_.size() != values.size()) {
            histories_.resize(values.size());
        }

        // 2. Process Data
        for (size_t i = 0; i < values.size(); ++i) {
            double v = values[i];
            
            // Auto-scale limits
            if (v < min_val_) min_val_ = v;
            if (v > max_val_) max_val_ = v;

            // Add to history
            histories_[i].push_back(v);
            if (histories_[i].size() > (size_t)width_) {
                histories_[i].erase(histories_[i].begin());
            }
        }

        // 3. Render
        SDL_SetRenderDrawColor(renderer_, 30, 30, 30, 255);
        SDL_RenderClear(renderer_);

        // Draw each channel
        for (size_t channel = 0; channel < histories_.size(); ++channel) {
            Color col = (channel < colors.size()) ? colors[channel] : Color{255, 255, 255};
            SDL_SetRenderDrawColor(renderer_, col.r, col.g, col.b, 255);

            const auto& line = histories_[channel];
            for (size_t i = 1; i < line.size(); ++i) {
                int y1 = mapY(line[i - 1]);
                int y2 = mapY(line[i]);
                // Draw connected lines
                SDL_RenderDrawLine(renderer_, i - 1, y1, i, y2);
            }
        }

        SDL_RenderPresent(renderer_);
        SDL_Delay(16);
    }
}
