#pragma once
#include <SDL2/SDL.h>
#include <vector>
#include <string>
#include <limits>

namespace robotics::viz {

    struct Color { uint8_t r, g, b; };

    class GraphVisualizer {
    public:
        GraphVisualizer(const std::string& title, int width = 800, int height = 600);
        ~GraphVisualizer();

        bool isRunning() const { return running_; }

        // NEW: Multi-channel update
        // Pass a list of values and a list of colors. 
        // Example: update({true_pos, meas, est}, {GREEN, RED, BLUE})
        void update(const std::vector<double>& values, const std::vector<Color>& colors);

        void clearHistory();

    private:
        void processEvents();
        int mapY(double val) const;

        int width_, height_;
        bool running_ = true;
        SDL_Window* window_ = nullptr;
        SDL_Renderer* renderer_ = nullptr;
        
        // A list of history buffers (one for each line)
        std::vector<std::vector<double>> histories_;
        
        double min_val_ = std::numeric_limits<double>::max();
        double max_val_ = std::numeric_limits<double>::lowest();
    };
}
