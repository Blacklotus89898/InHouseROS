#pragma once
#include <SDL2/SDL.h>
#include <vector>
#include <string>

namespace robotics::viz {

    class GraphVisualizer {
    public:
        GraphVisualizer(const std::string& title, int width = 800, int height = 600);
        ~GraphVisualizer();

        // Check if window was closed
        bool isRunning() const { return running_; }

        // Core Loop: Process Events -> Render -> Sleep
        void update(double target_val, double actual_val);
        
        // Reset the history (e.g. when simulation restarts)
        void clearHistory();

    private:
        void processEvents();
        int mapY(double val) const;

        int width_, height_;
        bool running_ = true;
        
        SDL_Window* window_ = nullptr;
        SDL_Renderer* renderer_ = nullptr;
        
        std::vector<int> history_;
        
        // Auto-scaling logic
        double min_val_ = 0.0;
        double max_val_ = 150.0; 
    };

}
