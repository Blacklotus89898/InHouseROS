#include "robotics_algo/kinematics/arm_3d.hpp"
#include <iostream>
#include <cmath>
#include <algorithm> // for clamp

namespace robotics::kinematics {

    Arm3D::Arm3D(double l1, double l2, double l3) : l1_(l1), l2_(l2), l3_(l3) {}

    // --- MATH HELPERS ---
    
    // Normalize angle to -PI to +PI
    double normalize(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    // Standard 3D Transformation Matrix Logic
    Matrix4 getTransform(double q0, double q1, double q2, double l1, double l2, double l3) {
        Matrix4 T01, T12, T23;
        double c0=cos(q0), s0=sin(q0);
        double c1=cos(q1), s1=sin(q1);
        double c2=cos(q2), s2=sin(q2);

        // Waist (Yaw)
        T01 << c0, 0, s0, 0,  0, 1, 0, l1, -s0, 0, c0, 0, 0, 0, 0, 1;
        // Shoulder (Pitch)
        T12 << c1,-s1, 0, l2, s1, c1, 0, 0,   0, 0, 1, 0,   0, 0, 0, 1;
        // Elbow (Pitch)
        T23 << c2,-s2, 0, l3, s2, c2, 0, 0,   0, 0, 1, 0,   0, 0, 0, 1;

        return T01 * T12 * T23;
    }

    Vector3 transformPoint(const Matrix4& T, const Vector3& p) {
        Vector4 p4(p.x(), p.y(), p.z(), 1.0);
        Vector4 res = T * p4;
        return Vector3(res.x(), res.y(), res.z());
    }

    // --- FORWARD KINEMATICS ---
    Vector3 Arm3D::forward(double q0, double q1, double q2) {
        Matrix4 T = getTransform(q0, q1, q2, l1_, l2_, l3_);
        return transformPoint(T, Vector3::Zero());
    }

    Arm3D::Skeleton Arm3D::getSkeleton(double q0, double q1, double q2) {
        Matrix4 T01, T12, T23;
        double c0=cos(q0), s0=sin(q0);
        double c1=cos(q1), s1=sin(q1);
        double c2=cos(q2), s2=sin(q2);

        T01 << c0, 0, s0, 0,  0, 1, 0, l1_, -s0, 0, c0, 0, 0, 0, 0, 1;
        T12 << c1,-s1, 0, l2_, s1, c1, 0, 0,   0, 0, 1, 0,   0, 0, 0, 1;
        T23 << c2,-s2, 0, l3_, s2, c2, 0, 0,   0, 0, 1, 0,   0, 0, 0, 1;

        Matrix4 BaseToShoulder = T01;
        Matrix4 BaseToElbow = T01 * T12;
        Matrix4 BaseToWrist = T01 * T12 * T23;

        return {
            Vector3::Zero(),
            transformPoint(BaseToShoulder, Vector3::Zero()),
            transformPoint(BaseToElbow, Vector3::Zero()),
            transformPoint(BaseToWrist, Vector3::Zero())
        };
    }

    // --- CCD INVERSE KINEMATICS (The Stability Fix) ---
    Arm3D::Joints Arm3D::solveIK(Joints q, Vector3 target) {
        
        // LIMITS (Prevents folding inside self)
        auto clampAngles = [](Joints& j) {
            j.q1 = std::clamp(j.q1, -M_PI/2, M_PI/2); // Shoulder Limits
            j.q2 = std::clamp(j.q2, -2.5, 2.5);       // Elbow Limits
        };

        // CCD Loop: Iterate from Last Joint -> First Joint
        // We do this loop a few times per frame for speed
        for(int iter=0; iter<5; ++iter) {
            
            // 1. ELBOW (q2)
            {
                // Get current position of end effector and elbow
                Skeleton skel = getSkeleton(q.q0, q.q1, q.q2);
                Vector3 root = skel.p_elbow;
                Vector3 cur = skel.p_wrist;
                
                // Vectors from root to end-effector (cur) and target
                Vector3 v_cur = cur - root;
                Vector3 v_target = target - root;
                
                // Project to 2D plane of the joint (XY plane of the arm)
                // Since q2 rotates Z in local frame, we need local coordinates.
                // Simplified CCD: Use the geometric angle difference.
                double cur_angle = std::atan2(v_cur.y(), v_cur.x()); // Crude approx in global
                // Actually, correct CCD uses dot product and cross product axis
                // Let's use a simpler heuristic for stability:
                
                // Vector alignment
                v_cur.normalize();
                v_target.normalize();
                
                // Angle to rotate
                double cos_theta = v_cur.dot(v_target);
                if(cos_theta < 0.9999) {
                    Vector3 cross = v_cur.cross(v_target);
                    // Determine sign based on our joint axis
                    // Elbow rotates around local Z. In global, that axis depends on q0/q1.
                    // Let's stick to Gradient Descent for just this step, it meshes well with CCD logic.
                    // Or actually, simple Geometric Heuristic:
                    
                    // The "Glitch Free" Analytic approach for the elbow:
                    // Use Law of Cosines to find ideal elbow angle based on distance to target.
                    // This is hybrid: Analytic Elbow + Geometric Shoulder/Waist.
                }
            }
        }
        
        // --- HYBRID SOLVER (The "Unbreakable" Method) ---
        // 1. Solve WAIST (q0) simply by looking at target X/Z
        q.q0 = std::atan2(target.z(), target.x()); // Yaw
        
        // 2. Solve SHOULDER & ELBOW as a 2D planar problem
        // We project the target onto the arm's plane
        double r = std::sqrt(target.x()*target.x() + target.z()*target.z()); // Horizontal dist
        double y = target.y() - l1_; // Height relative to shoulder
        
        // Target distance from shoulder
        double dist_sq = r*r + y*y;
        double dist = std::sqrt(dist_sq);
        
        // Clamp reach
        double max_reach = l2_ + l3_;
        if(dist > max_reach * 0.99) {
            dist = max_reach * 0.99; // slight buffer
        }

        // Law of Cosines for Elbow (q2)
        // c2 = (dist^2 - l2^2 - l3^2) / (2*l2*l3)
        double c2 = (dist*dist - l2_*l2_ - l3_*l3_) / (2 * l2_ * l3_);
        c2 = std::clamp(c2, -1.0, 1.0);
        q.q2 = -std::acos(c2); // Elbow down solution usually

        // Law of Cosines / Atan for Shoulder (q1)
        // Angle to target
        double alpha = std::atan2(y, r);
        // Angle of shoulder triangle
        double c1_tri = (l2_*l2_ + dist*dist - l3_*l3_) / (2 * l2_ * dist);
        c1_tri = std::clamp(c1_tri, -1.0, 1.0);
        double beta = std::acos(c1_tri);
        
        q.q1 = alpha + beta; 

        // Final Clamp for safety
        clampAngles(q);
        
        return q;
    }
}
