#include "robotics_algo/control/mpc/mpc_solver.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace robotics::control {

    MPCSolver::MPCSolver(int horizon, double dt) 
        : horizon_(horizon), dt_(dt) {
        // Initialize inputs with zeros
        optimized_inputs_.resize(horizon, {0.0, 0.0});
    }

    // Kinematic Model: The Physics
    State nextState(State s, ControlInput u, double dt) {
        State next;
        next.x = s.x + u.v * std::cos(s.theta) * dt;
        next.y = s.y + u.v * std::sin(s.theta) * dt;
        next.theta = s.theta + u.w * dt;
        return next;
    }

    std::vector<State> MPCSolver::predictTrajectory(State start, const std::vector<ControlInput>& inputs) {
        std::vector<State> path;
        path.push_back(start);
        State curr = start;
        for (const auto& u : inputs) {
            curr = nextState(curr, u, dt_);
            path.push_back(curr);
        }
        return path;
    }

    double MPCSolver::calculateCost(const std::vector<State>& traj, State target) {
        double cost = 0.0;
        for (const auto& s : traj) {
            double dx = s.x - target.x;
            double dy = s.y - target.y;
            // Weighted Cost: Position error is expensive!
            cost += (dx*dx + dy*dy) * 10.0; 
        }
        return cost;
    }

    ControlInput MPCSolver::solve(State current, State target) {
        // Optimization Parameters
        int iterations = 50;   // How hard to think per frame
        double learning_rate = 0.01; // Step size for gradient descent
        double epsilon = 0.001; // Small perturbation for derivative

        // 1. Optimization Loop
        for (int iter = 0; iter < iterations; ++iter) {
            
            // Calculate current cost
            auto traj = predictTrajectory(current, optimized_inputs_);
            double base_cost = calculateCost(traj, target);

            // Calculate Gradients for each step in the horizon
            for (int i = 0; i < horizon_; ++i) {
                
                // Wiggle V (Linear Velocity)
                optimized_inputs_[i].v += epsilon;
                double cost_v = calculateCost(predictTrajectory(current, optimized_inputs_), target);
                optimized_inputs_[i].v -= epsilon; // Undo
                double grad_v = (cost_v - base_cost) / epsilon;

                // Wiggle W (Angular Velocity)
                optimized_inputs_[i].w += epsilon;
                double cost_w = calculateCost(predictTrajectory(current, optimized_inputs_), target);
                optimized_inputs_[i].w -= epsilon; // Undo
                double grad_w = (cost_w - base_cost) / epsilon;

                // Descent Step (Update controls to reduce cost)
                optimized_inputs_[i].v -= learning_rate * grad_v;
                optimized_inputs_[i].w -= learning_rate * grad_w;

                // Clamp Inputs (Physical Limits)
                optimized_inputs_[i].v = std::clamp(optimized_inputs_[i].v, -20.0, 20.0);
                optimized_inputs_[i].w = std::clamp(optimized_inputs_[i].w, -3.0, 3.0);
            }
        }

        // 2. Shift Buffer
        // The best plan for "Now" is the previous plan for "Next Time"
        // This is "Warm Starting"
        ControlInput first_action = optimized_inputs_[0];
        
        // Shift inputs left
        for(size_t i=0; i<optimized_inputs_.size()-1; ++i) {
            optimized_inputs_[i] = optimized_inputs_[i+1];
        }
        // Duplicate last input for the new end of horizon
        optimized_inputs_.back() = optimized_inputs_[optimized_inputs_.size()-2];

        // Save path for visualization
        predicted_path_ = predictTrajectory(current, optimized_inputs_);

        return first_action;
    }
}
