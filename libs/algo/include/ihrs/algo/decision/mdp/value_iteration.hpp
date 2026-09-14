#pragma once
#include <vector>
#include <string>
#include <iostream>

namespace ihrs::algo::decision {

    struct MDPState {
        int x, y;
        double reward;    // Immediate reward for being here
        bool is_terminal; // Game Over state? (Goal or Lava)
        double value;     // The calculated Utility (V*)
        int policy;       // Best action (0=Up, 1=Right, 2=Down, 3=Left)
    };

    class MDPSolver {
    public:
        // width, height: Grid size
        MDPSolver(int width, int height);

        // Add special zones
        void setGoal(int x, int y, double reward);
        void setHazard(int x, int y, double penalty);
        void setWall(int x, int y);
        void clearWall(int x, int y);
        void resetValues();

        // The Core Algo: Run until values converge
        // gamma: Discount factor (0.9 = look far ahead)
        // noise: Probability of sliding sideways (e.g., 0.2)
        void solve(double gamma = 0.9, double noise = 0.2);

        // Getters for visualization
        const std::vector<MDPState>& getGrid() const { return grid_; }
        int getWidth() const { return width_; }
        int getHeight() const { return height_; }
        bool isWall(int x, int y) const;

    private:
        int width_, height_;
        std::vector<MDPState> grid_;
        std::vector<bool> walls_;
        
        // Helper to get index
        std::size_t idx(int x, int y) const { return static_cast<std::size_t>(y * width_ + x); }
        
        // Helper to get state safely (walls block movement)
        std::size_t getNextStateIdx(int x, int y, int action);
    };

}
