#include "robotics_algo/kinematics/six_link_arm.hpp"
#include <cmath>
#include <iostream>
#include <algorithm> 

namespace robotics::kinematics {

    SixLinkArm::SixLinkArm() {}

    // --- MATH HELPERS ---
    Matrix4 rotY(double t) {
        Matrix4 m = Matrix4::Identity();
        m(0,0)=cos(t); m(0,2)=sin(t); m(2,0)=-sin(t); m(2,2)=cos(t);
        return m;
    }
    Matrix4 rotZ(double t) {
        Matrix4 m = Matrix4::Identity();
        m(0,0)=cos(t); m(0,1)=-sin(t); m(1,0)=sin(t); m(1,1)=cos(t);
        return m;
    }
    Matrix4 trans(double x, double y, double z) {
        Matrix4 m = Matrix4::Identity();
        m(0,3)=x; m(1,3)=y; m(2,3)=z;
        return m;
    }
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    // Helper to get partial transforms
    // Returns a vector of 6 matrices: T01, T02, T03, ... T06
    std::vector<Matrix4> getJointTransforms(const std::vector<double>& q, double l1, double l2, double l3, double l4, double l5, double l6) {
        std::vector<Matrix4> T(6);
        Matrix4 acc = Matrix4::Identity();

        // 0. Waist (Rot Y)
        acc = acc * rotY(q[0]) * trans(0, l1, 0); T[0] = acc;
        // 1. Shoulder (Rot Z)
        acc = acc * rotZ(q[1]) * trans(0, l2, 0); T[1] = acc;
        // 2. Elbow (Rot Z)
        acc = acc * rotZ(q[2]) * trans(0, l3, 0); T[2] = acc;
        // 3. Wrist 1 (Rot Y)
        acc = acc * rotY(q[3]) * trans(0, l4, 0); T[3] = acc;
        // 4. Wrist 2 (Rot Z)
        acc = acc * rotZ(q[4]) * trans(0, l5, 0); T[4] = acc;
        // 5. Wrist 3 (Rot Y)
        acc = acc * rotY(q[5]) * trans(0, l6, 0); T[5] = acc;
        
        return T;
    }

    Matrix4 SixLinkArm::forward(const std::vector<double>& q) {
        auto transforms = getJointTransforms(q, l1_, l2_, l3_, l4_, l5_, l6_);
        return transforms.back();
    }

    std::vector<Vector3> SixLinkArm::getSkeleton(const std::vector<double>& q) {
        std::vector<Vector3> joints;
        joints.push_back(Vector3(0,0,0)); // Base
        auto transforms = getJointTransforms(q, l1_, l2_, l3_, l4_, l5_, l6_);
        for(const auto& T : transforms) {
            joints.push_back(Vector3(T(0,3), T(1,3), T(2,3)));
        }
        return joints;
    }

    std::vector<double> SixLinkArm::solveIK(std::vector<double> q, Vector3 target_pos, Matrix3 target_rot) {
        // Tuned parameters for standard 6-DOF convergence
        double alpha = 0.2; 
        double max_step = 0.08;

        // 1. Get Current State
        auto transforms = getJointTransforms(q, l1_, l2_, l3_, l4_, l5_, l6_);
        Matrix4 tip_T = transforms.back();
        Vector3 current_pos(tip_T(0,3), tip_T(1,3), tip_T(2,3));
        Matrix3 current_rot = tip_T.block<3,3>(0,0);

        // 2. Calculate Error
        Vector3 pos_err = target_pos - current_pos;

        // Orientation Error (Exact formulation)
        // e_rot = 0.5 * (n x n_d + s x s_d + a x a_d)
        // Where n,s,a are the columns of the rotation matrix (current vs desired)
        Vector3 col0_cur = current_rot.col(0); Vector3 col0_tar = target_rot.col(0);
        Vector3 col1_cur = current_rot.col(1); Vector3 col1_tar = target_rot.col(1);
        Vector3 col2_cur = current_rot.col(2); Vector3 col2_tar = target_rot.col(2);

        Vector3 rot_err = 0.5 * (col0_cur.cross(col0_tar) + col1_cur.cross(col1_tar) + col2_cur.cross(col2_tar));

        Eigen::Matrix<double, 6, 1> error;
        error << pos_err.x(), pos_err.y(), pos_err.z(), 
                 rot_err.x(), rot_err.y(), rot_err.z();

        // 3. Geometric Jacobian (The Solution!)
        // Instead of numerical differentiation, we calculate exact axis vectors.
        Eigen::Matrix<double, 6, 6> J;

        // We need the rotation axis of each joint expressed in World Frame.
        // Base frame is Identity.
        // Joint 0 rotates around Y axis (0,1,0).
        // Joint 1 rotates around Z axis of Frame 0.
        // etc.
        
        // Define Local Axes for each joint (Must match forward() logic!)
        Vector3 local_axes[] = {
            Vector3(0,1,0), // Joint 0: Y
            Vector3(0,0,1), // Joint 1: Z
            Vector3(0,0,1), // Joint 2: Z
            Vector3(0,1,0), // Joint 3: Y
            Vector3(0,0,1), // Joint 4: Z
            Vector3(0,1,0)  // Joint 5: Y
        };

        Vector3 prev_pos(0,0,0); // Base position
        Matrix3 prev_rot = Matrix3::Identity(); // Base rotation

        for(int i=0; i<6; ++i) {
            // The axis of rotation for joint i is the Z (or Y) axis of the PREVIOUS frame (i-1)
            // But transformed into World Space.
            
            // Current Joint Axis in World Space
            Vector3 z_axis = prev_rot * local_axes[i];

            // Vector from Joint i to End Effector
            Vector3 p_e = current_pos - prev_pos;

            // Jacobian Column i
            // J_pos = axis x (end_effector - joint_pos)
            // J_rot = axis
            Vector3 j_pos = z_axis.cross(p_e);
            Vector3 j_rot = z_axis;

            J(0,i) = j_pos.x(); J(1,i) = j_pos.y(); J(2,i) = j_pos.z();
            J(3,i) = j_rot.x(); J(4,i) = j_rot.y(); J(5,i) = j_rot.z();

            // Advance to next frame for next iteration
            // We can grab the exact Rotation/Pos from our precomputed transforms
            // transforms[i] is the frame AFTER joint i rotation.
            Matrix4 T_next = transforms[i];
            prev_pos = Vector3(T_next(0,3), T_next(1,3), T_next(2,3));
            prev_rot = T_next.block<3,3>(0,0);
        }

        // 4. Solve (Damped Least Squares to avoid singularities)
        // J^T * (J * J^T + lambda^2 * I)^-1 * error
        // But for speed/simplicity, Transpose + Clamping is robust enough for this demo
        Eigen::Matrix<double, 6, 1> delta_q = J.transpose() * error * alpha;

        // Velocity Clamping
        for(int i=0; i<6; ++i) {
            delta_q(i) = std::clamp(delta_q(i), -max_step, max_step);
            q[i] += delta_q(i);
            q[i] = normalizeAngle(q[i]);
        }

        return q;
    }
}
