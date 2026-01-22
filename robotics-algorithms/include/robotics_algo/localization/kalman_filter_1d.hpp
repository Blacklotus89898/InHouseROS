#pragma once
#include "robotics_algo/common/types.hpp"

namespace robotics::localization {

    class KalmanFilter1D {
    public:
        // initial_x: Starting position
        // initial_p: Initial uncertainty (variance)
        // process_noise (Q): How much error adds up purely by moving (wheel slip)
        // sensor_noise (R): How inaccurate the sensor is
        KalmanFilter1D(Scalar initial_x, Scalar initial_p, Scalar process_noise, Scalar sensor_noise);

        // Step 1: Prediction (Physics)
        // u: Control input (e.g., velocity * dt)
        void predict(Scalar u);

        // Step 2: Update (Measurement)
        // z: The raw sensor reading
        void update(Scalar z);

        // Getters
        Scalar getState() const { return x_; }
        Scalar getUncertainty() const { return p_; }

    private:
        Scalar x_; // State Estimate
        Scalar p_; // Covariance (Estimation Error)
        
        Scalar q_; // Process Noise Covariance
        Scalar r_; // Measurement Noise Covariance
    };

}
