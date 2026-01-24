#pragma once
#include <vector>
#include "robotics_algo/common/types.hpp"

namespace robotics::kinematics {

    class SixLinkArm {
    public:
        SixLinkArm();

        // --- FORWARD KINEMATICS ---
        // Returns the End Effector Pose (Position + Orientation Matrix)
        // input: std::vector<double> of size 6
        Matrix4 forward(const std::vector<double>& joints);

        // Get positions of all joints for rendering
        std::vector<Vector3> getSkeleton(const std::vector<double>& joints);

        // --- INVERSE KINEMATICS (Jacobian) ---
        // target_pos: Desired X,Y,Z
        // target_rot: Desired Orientation (Rotation Matrix)
        std::vector<double> solveIK(
            std::vector<double> current_q, 
            Vector3 target_pos, 
            Matrix3 target_rot
        );

    private:
        // Link Lengths
        double l1_ = 50.0;  // Base Height
        double l2_ = 100.0; // Shoulder
        double l3_ = 100.0; // Elbow
        double l4_ = 80.0;  // Wrist 1
        double l5_ = 40.0;  // Wrist 2
        double l6_ = 20.0;  // Tool Flange
    };
}
