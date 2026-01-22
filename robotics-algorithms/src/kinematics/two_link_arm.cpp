#include "robotics_algo/kinematics/two_link_arm.hpp"
#include <cmath>

namespace robotics::kinematics {

    TwoLinkArm::TwoLinkArm(Scalar l1, Scalar l2) : l1_(l1), l2_(l2) {}

    Vector2 TwoLinkArm::forwardKinematics(const JointState& q) const {
        // FK using Transformation Chains
        // T_base_hand = T_base_shoulder * T_shoulder_elbow * T_elbow_hand
        
        using namespace math;

        // Base to Shoulder (Rotation q1)
        Transform2D t1(Vector2(0, 0), q.theta1);
        
        // Shoulder to Elbow (Translation l1, Rotation q2)
        Transform2D t2(Vector2(l1_, 0), q.theta2);
        
        // Elbow to Hand (Translation l2)
        Transform2D t3(Vector2(l2_, 0), 0.0);

        // Chain them: T_global = t1 * t2 * t3
        Transform2D result = t1 * t2 * t3;

        return result.translation;
    }

    std::optional<JointState> TwoLinkArm::inverseKinematics(const Vector2& target) const {
        // Analytical Solution (Trigonometry)
        Scalar x = target.x();
        Scalar y = target.y();
        
        // 1. Check reachability
        Scalar r_sq = x*x + y*y;
        Scalar r = std::sqrt(r_sq);
        if (r > (l1_ + l2_) || r < std::abs(l1_ - l2_)) {
            return std::nullopt; // Target out of reach
        }

        // 2. Law of Cosines for Elbow Angle (Theta2)
        // c2 = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2)
        Scalar c2 = (r_sq - l1_*l1_ - l2_*l2_) / (2.0 * l1_ * l2_);
        
        // Numerical stability clamp
        if (c2 > 1.0) c2 = 1.0;
        if (c2 < -1.0) c2 = -1.0;

        Scalar theta2 = std::acos(c2); // Elbow Down solution (+val)
        // For Elbow Up, use -std::acos(c2)

        // 3. Solve Shoulder Angle (Theta1)
        // theta1 = atan2(y, x) - atan2(l2 * sin(theta2), l1 + l2 * cos(theta2))
        Scalar k1 = l1_ + l2_ * c2;
        Scalar k2 = l2_ * std::sin(theta2);
        Scalar theta1 = std::atan2(y, x) - std::atan2(k2, k1);

        return JointState{theta1, theta2};
    }

}
