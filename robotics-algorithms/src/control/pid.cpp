#include "robotics_algo/control/pid.hpp"
#include <algorithm> // for std::clamp
#include <iostream>

namespace robotics::control {

    PID::PID(const PIDConfig& config) : config_(config) {}

    PID::PID(Scalar kp, Scalar ki, Scalar kd) {
        config_.kp = kp;
        config_.ki = ki;
        config_.kd = kd;
    }

    Scalar PID::update(Scalar target, Scalar measured, Scalar dt) {
        // 1. Calculate Error
        Scalar error = target - measured;

        // 2. Proportional Term
        Scalar P = config_.kp * error;

        // 3. Integral Term (with simple anti-windup clamping)
        integral_ += error * dt;
        Scalar I = config_.ki * integral_;

        // 4. Derivative Term
        Scalar D = 0.0;
        if (!first_run_) {
            D = config_.kd * (error - prev_error_) / dt;
        } else {
            first_run_ = false;
        }
        prev_error_ = error;

        // 5. Total Output
        Scalar output = P + I + D;

        // 6. Output Clamping (Simulate physical limits)
        return std::clamp(output, config_.min_output, config_.max_output);
    }

    void PID::reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        first_run_ = true;
    }

}
