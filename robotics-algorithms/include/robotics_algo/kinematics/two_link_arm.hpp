#pragma once
#include "robotics_algo/common/math/transform.hpp"
#include <optional> // C++17 feature for "Maybe a solution"

namespace robotics::kinematics {

    struct JointState {
        Scalar theta1; // Shoulder
        Scalar theta2; // Elbow
    };

    class TwoLinkArm {
    public:
        // l1: Length of upper arm, l2: Length of forearm
        TwoLinkArm(Scalar l1, Scalar l2);

        // Forward Kinematics: Angles -> Position
        // Returns the end-effector (Hand) position
        Vector2 forwardKinematics(const JointState& joints) const;

        // Inverse Kinematics: Position -> Angles
        // Returns nullopt if the target is unreachable (too far)
        // This solves for the "Elbow Down" configuration by default
        std::optional<JointState> inverseKinematics(const Vector2& target) const;

        // Getters for visualization
        Scalar getL1() const { return l1_; }
        Scalar getL2() const { return l2_; }

    private:
        Scalar l1_;
        Scalar l2_;
    };

}
