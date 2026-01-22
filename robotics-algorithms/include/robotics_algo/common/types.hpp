#pragma once
#include <Eigen/Dense>

namespace robotics {

    // Standardize floating point precision (easy to switch float/double later)
    using Scalar = double;

    // Use Eigen for geometry
    using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

    // A simple Pose2D struct (x, y, theta)
    struct Pose2D {
        Vector2 position;
        Scalar theta;
    };

}
