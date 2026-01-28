#pragma once
#include <vector>
#include "robotics_algo/common/types.hpp"

namespace robotics::control {

    // Differential Drive State [x, y, theta]
    struct State {
        double x = 0;
        double y = 0;
        double theta = 0;
    };

    // Control Input [v, omega]
    struct ControlInput {
        double v = 0.0; // Linear Velocity
        double w = 0.0; // Angular Velocity
    };

    class MPCSolver {
    public:
        // horizon: How many steps to look ahead (e.g., 20)
        // dt: Time per step (e.g., 0.1s)
        MPCSolver(int horizon = 20, double dt = 0.1);

        // Calculate best controls to reach target
        ControlInput solve(State current, State target);

        // Get the predicted path (for visualization)
        std::vector<State> getPredictedPath() const { return predicted_path_; }

    private:
        // Forward Simulate: Given start state and inputs, where do we end up?
        std::vector<State> predictTrajectory(State start, const std::vector<ControlInput>& inputs);

        // Calculate "Badness" of a trajectory
        double calculateCost(const std::vector<State>& trajectory, State target);

        int horizon_;
        double dt_;
        
        // Memory: We keep the previous solution to "warm start" the next optimization
        std::vector<ControlInput> optimized_inputs_;
        std::vector<State> predicted_path_;
    };

}
