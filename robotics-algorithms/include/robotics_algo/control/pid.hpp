#pragma once
#include "robotics_algo/common/types.hpp"
#include <limits> // For infinity

namespace robotics::control {

    struct PIDConfig {
        Scalar kp = 0.0;
        Scalar ki = 0.0;
        Scalar kd = 0.0;
        
        // Output limits (e.g., -10V to +10V, or -100% to 100%)
        Scalar min_output = -std::numeric_limits<Scalar>::infinity();
        Scalar max_output = std::numeric_limits<Scalar>::infinity();
    };

    class PID {
    public:
        // Constructor
        PID(const PIDConfig& config);
        PID(Scalar kp, Scalar ki, Scalar kd); // Helper constructor

        // Core Calculation
        // Returns the control signal (u) based on error and time step
        Scalar update(Scalar target, Scalar measured, Scalar dt);

        // Reset memory (integral and previous error)
        void reset();

    private:
        PIDConfig config_;
        Scalar integral_ = 0.0;
        Scalar prev_error_ = 0.0;
        bool first_run_ = true; // To handle the derivative spike on first step
    };

}
