#pragma once
#include "robotics_algo/common/types.hpp"
#include <cmath>

namespace robotics::math {

    // Represents a 2D coordinate frame (SE2)
    struct Transform2D {
        Vector2 translation;
        Scalar rotation; // Radians

        Transform2D(Vector2 t = Vector2::Zero(), Scalar r = 0.0) 
            : translation(t), rotation(r) {}

        // Convert to 3x3 Homogeneous Matrix (Eigen)
        Matrix3 toMatrix() const {
            Matrix3 m;
            Scalar c = std::cos(rotation);
            Scalar s = std::sin(rotation);
            
            m << c, -s, translation.x(),
                 s,  c, translation.y(),
                 0,  0, 1;
            return m;
        }

        // Combine two transforms (Parent * Child)
        Transform2D operator*(const Transform2D& other) const {
            // New Rotation = Rot1 + Rot2
            Scalar new_rot = rotation + other.rotation;
            
            // New Translation = Trans1 + Rot1 * Trans2
            Scalar c = std::cos(rotation);
            Scalar s = std::sin(rotation);
            Vector2 rotated_t2(
                c * other.translation.x() - s * other.translation.y(),
                s * other.translation.x() + c * other.translation.y()
            );

            return Transform2D(translation + rotated_t2, new_rot);
        }
    };
}
