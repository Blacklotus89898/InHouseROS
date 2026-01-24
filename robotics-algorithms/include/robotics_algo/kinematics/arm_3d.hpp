#pragma once
#include <vector>
#include "robotics_algo/common/types.hpp"

namespace robotics::kinematics {

    class Arm3D {
    public:
        // Link lengths
        Arm3D(double l1, double l2, double l3);

        // --- FORWARD KINEMATICS ---
        Vector3 forward(double q0, double q1, double q2);

        struct Skeleton { Vector3 p_base, p_shoulder, p_elbow, p_wrist; };
        Skeleton getSkeleton(double q0, double q1, double q2);

        // --- JACOBIAN IK ---
        struct Joints { double q0, q1, q2; };
        Joints solveIK(Joints current_q, Vector3 target_pos);

    private:
        double l1_, l2_, l3_;
    };
}
