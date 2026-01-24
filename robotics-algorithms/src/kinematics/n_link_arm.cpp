#include "robotics_algo/kinematics/n_link_arm.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace robotics::kinematics {

    NLinkArm3D::NLinkArm3D(int num_links, double link_len, Vector3 base_pos) 
        : link_len_(link_len) {
        
        // Initialize straight up (Y-axis)
        joints_.push_back(base_pos);
        for(int i=0; i<num_links; ++i) {
            Vector3 next = joints_.back();
            next.y() += link_len;
            joints_.push_back(next);
        }
    }

    void NLinkArm3D::solveIK(Vector3 target) {
        // CCD Parameters
        int iterations = 5; 
        double damp = 1.0; // 1.0 = move fully, 0.1 = slow smooth motion

        int N = joints_.size(); // Joints: 0..N-1 (Tip is N-1)

        for(int iter=0; iter<iterations; ++iter) {
            // Iterate from Second-to-Last joint (parent of tip) down to Base (0)
            // Note: Tip is joints_[N-1]. We start pivoting at N-2.
            for(int i = N - 2; i >= 0; --i) {
                
                Vector3 p_joint = joints_[i];
                Vector3 p_tip   = joints_.back();

                // 1. Vectors
                Vector3 v_cur = p_tip - p_joint;
                Vector3 v_target = target - p_joint;

                // 2. Normalize
                double len_cur = v_cur.norm();
                double len_tar = v_target.norm();
                if(len_cur < 1e-6 || len_tar < 1e-6) continue;
                
                v_cur = v_cur / len_cur;
                v_target = v_target / len_tar;

                // 3. Find Axis and Angle
                // Rotation Axis = v_cur X v_target (Perpendicular to plane)
                Vector3 axis = v_cur.cross(v_target);
                double axis_len = axis.norm();
                
                // If aligned, axis_len is 0, no rotation needed
                if(axis_len < 1e-6) continue; 
                axis = axis / axis_len;

                // Angle = acos(v_cur . v_target)
                double dot = v_cur.dot(v_target);
                dot = std::clamp(dot, -1.0, 1.0); // Safety
                double angle = std::acos(dot) * damp;

                // 4. Create Rotation Matrix (Angle-Axis)
                // Using Eigen's AngleAxis helper
                Eigen::AngleAxisd rotation(angle, axis);
                Eigen::Matrix3d rot_matrix = rotation.toRotationMatrix();

                // 5. Apply Rotation to ALL child joints (i+1 to Tip)
                // Physics: If I rotate shoulder, elbow/wrist/fingers all move.
                for(int k = i + 1; k < N; ++k) {
                    Vector3 rel = joints_[k] - p_joint; // Vector from pivot to child
                    Vector3 new_rel = rot_matrix * rel;
                    joints_[k] = p_joint + new_rel;
                }
            }
        }
    }
}
