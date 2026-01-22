#pragma once

// We include Eigen here so we don't have to include it everywhere else
#include <Eigen/Dense>
#include <cmath>

namespace robotics {

    // 1. Standard Scalar (Easy to switch to float if needed for GPU/Embedded)
    using Scalar = double;

    // 2. Geometry Types (Using Eigen for high-performance math)
    using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

    // 3. Constants
    constexpr Scalar PI = 3.14159265358979323846;

    // 4. Basic Structs
    struct Pose2D {
        Vector2 position;
        Scalar theta; // Heading in radians

        Pose2D() : position(0.0, 0.0), theta(0.0) {}
        Pose2D(Scalar x, Scalar y, Scalar th) : position(x, y), theta(th) {}
    };

    // 5. Common Utility Functions Declarations
    namespace math {
        // Normalizes an angle to be within [-PI, PI]
        Scalar normalize_angle(Scalar angle);
        
        // Converts degrees to radians
        constexpr Scalar deg2rad(Scalar deg) { return deg * PI / 180.0; }
        
        // Converts radians to degrees
        constexpr Scalar rad2deg(Scalar rad) { return rad * 180.0 / PI; }
    }
}
