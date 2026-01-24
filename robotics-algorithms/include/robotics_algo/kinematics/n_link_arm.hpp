#pragma once
#include <vector>
#include "robotics_algo/common/types.hpp"

namespace robotics::kinematics {

    class NLinkArm3D {
    public:
        // Create N links of length L, starting at base_pos
        NLinkArm3D(int num_links, double link_length, Vector3 base_pos);

        // --- 3D INVERSE KINEMATICS (CCD) ---
        // Iteratively bends the snake to touch the target
        void solveIK(Vector3 target);

        // Getters for rendering
        const std::vector<Vector3>& getJoints() const { return joints_; }

    private:
        std::vector<Vector3> joints_; // Position of each joint (0=Base, N=Tip)
        double link_len_;
    };
}
