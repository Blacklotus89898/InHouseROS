#include "robotics_algo/localization/kalman_filter_1d.hpp"
#include <iostream>

namespace robotics::localization {

    KalmanFilter1D::KalmanFilter1D(Scalar x, Scalar p, Scalar q, Scalar r)
        : x_(x), p_(p), q_(q), r_(r) {}

    void KalmanFilter1D::predict(Scalar u) {
        // 1. Update State (Physics Model: x = x + velocity*dt)
        x_ = x_ + u;

        // 2. Increase Uncertainty (P = P + Q)
        // Because we moved, we are slightly less sure where we are now.
        p_ = p_ + q_;
    }

    void KalmanFilter1D::update(Scalar z) {
        // 1. Calculate Kalman Gain (K)
        // K = Error / (Error + MeasurementNoise)
        Scalar k = p_ / (p_ + r_);

        // 2. Update State Estimate
        // x = x + K * (measurement - x)
        // If K is high (trust sensor), we move mostly to z.
        // If K is low (trust physics), we barely move.
        x_ = x_ + k * (z - x_);

        // 3. Update Uncertainty
        // Our uncertainty always decreases after a measurement.
        p_ = (1.0 - k) * p_;
    }

}
