#include "ihrs/algo/decision/mdp/value_iteration.hpp"
#include <algorithm>
#include <cmath>

namespace ihrs::algo::decision {

    MDPSolver::MDPSolver(int w, int h) : width_(w), height_(h) {
        grid_.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
        walls_.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), false);
        for (std::size_t i = 0; i < grid_.size(); ++i) {
            grid_[i].x = static_cast<int>(i % static_cast<std::size_t>(w));
            grid_[i].y = static_cast<int>(i / static_cast<std::size_t>(w));
            grid_[i].reward = -0.1; // Living cost
            grid_[i].value = 0.0;
            grid_[i].is_terminal = false;
        }
    }

    void MDPSolver::setGoal(int x, int y, double r) {
        const auto i = idx(x, y);
        grid_[i].reward = r;
        grid_[i].is_terminal = true;
        grid_[i].value = r; 
    }

    void MDPSolver::setHazard(int x, int y, double p) {
        const auto i = idx(x, y);
        grid_[i].reward = p;
        grid_[i].is_terminal = true;
        grid_[i].value = p;
    }

    void MDPSolver::setWall(int x, int y) {
        walls_[idx(x, y)] = true;
    }

    void MDPSolver::clearWall(int x, int y) {
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            walls_[idx(x, y)] = false;
        }
    }

    bool MDPSolver::isWall(int x, int y) const {
        if (x < 0 || x >= width_ || y < 0 || y >= height_) return true;
        return walls_[idx(x, y)];
    }

    void MDPSolver::resetValues() {
        for (auto& s : grid_) {
            if (s.is_terminal) {
                s.value = s.reward;
            } else {
                s.value = 0.0;
            }
            s.policy = 0;
        }
    }

    std::size_t MDPSolver::getNextStateIdx(int x, int y, int action) {
        int nx = x, ny = y;
        // 0=Up, 1=Right, 2=Down, 3=Left
        if (action == 0) ny--;
        else if (action == 1) nx++;
        else if (action == 2) ny++;
        else if (action == 3) nx--;

        if (isWall(nx, ny)) return idx(x, y);
        return idx(nx, ny);
    }

    void MDPSolver::solve(double gamma, double noise) {
        // CHANGED: Removed the loop 'for(iter < 1000)'
        // Now performs exactly ONE Value Iteration step per call.
        
        std::vector<MDPState> new_grid = grid_;

        for (std::size_t i = 0; i < grid_.size(); ++i) {
            if (grid_[i].is_terminal || walls_[i]) continue;

            int x = grid_[i].x;
            int y = grid_[i].y;
            double best_val = -1e9;
            int best_action = 0;

            for (int action = 0; action < 4; ++action) {
                int a_left = (action + 3) % 4;
                int a_right = (action + 1) % 4;

                double val_intended = grid_[getNextStateIdx(x, y, action)].value;
                double val_slip1 = grid_[getNextStateIdx(x, y, a_left)].value;
                double val_slip2 = grid_[getNextStateIdx(x, y, a_right)].value;

                double expected_value = (1.0 - noise) * val_intended 
                                      + (noise / 2.0) * val_slip1 
                                      + (noise / 2.0) * val_slip2;

                if (expected_value > best_val) {
                    best_val = expected_value;
                    best_action = action;
                }
            }

            new_grid[i].value = grid_[i].reward + gamma * best_val;
            new_grid[i].policy = best_action;
        }

        grid_ = new_grid;
    }
}
