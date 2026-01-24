#include "robotics_algo/localization/particle_filter.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace robotics::localization {

    ParticleFilter::ParticleFilter(int num, const planning::GridMap& map) 
        : num_particles_(num) {
        gen_.seed(std::random_device{}());
    }

    void ParticleFilter::init(double start_x, double start_y, double start_theta, double spread) {
        particles_.clear();
        std::normal_distribution<double> dist_x(start_x, spread);
        std::normal_distribution<double> dist_y(start_y, spread);
        std::normal_distribution<double> dist_theta(start_theta, 0.1); 

        for(int i=0; i<num_particles_; ++i) {
            particles_.push_back({
                dist_x(gen_), 
                dist_y(gen_), 
                dist_theta(gen_), 
                1.0 / num_particles_ 
            });
        }
    }

    void ParticleFilter::predict(double d_dist, double d_theta) {
        std::normal_distribution<double> noise_d(0.0, motion_noise_dist_);
        std::normal_distribution<double> noise_th(0.0, motion_noise_theta_);

        for(auto& p : particles_) {
            // Add noise to the movement command
            double noisy_dist = d_dist + noise_d(gen_);
            double noisy_theta = d_theta + noise_th(gen_);

            // Apply kinematics
            p.theta += noisy_theta;
            p.x += std::cos(p.theta) * noisy_dist;
            p.y += std::sin(p.theta) * noisy_dist;
        }
    }

    void ParticleFilter::update(const planning::GridMap& map) {
        double total_weight = 0.0;
        
        // Helper: Convert world to grid
        // Assuming 20px cell size based on your previous map demos
        const double CELL_SIZE = 20.0; 

        for(auto& p : particles_) {
            // CHECK: Is this particle in a valid location?
            int gx = (int)(p.x / CELL_SIZE);
            int gy = (int)(p.y / CELL_SIZE);
            
            bool is_valid = true;

            // 1. Check Map Bounds
            if (gx < 0 || gx >= map.width || gy < 0 || gy >= map.height) {
                is_valid = false;
            }
            // 2. Check Obstacles
            else if (map.isObstacle(gx, gy)) {
                is_valid = false;
            }

            // ASSIGN WEIGHT
            if(is_valid) {
                p.weight = 1.0; // Perfectly fine
            } else {
                p.weight = 0.0; // Dead (Impossible location)
            }
            
            total_weight += p.weight;
        }

        // Handle "Kidnapped Robot" (All particles died)
        // If total_weight is 0, re-initialize randomly or just uniform
        if (total_weight < 1e-9) {
            for(auto& p : particles_) p.weight = 1.0 / num_particles_;
            total_weight = 1.0;
        }

        // Normalize
        for(auto& p : particles_) {
            p.weight /= total_weight;
        }
    }

    void ParticleFilter::resample() {
        std::vector<Particle> new_particles;
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        
        // "Low Variance Sampler" (Wheel Algorithm)
        double beta = 0.0;
        int index = (int)(dist(gen_) * num_particles_);
        
        double max_w = 0.0;
        for(const auto& p : particles_) if(p.weight > max_w) max_w = p.weight;

        for(int i=0; i<num_particles_; ++i) {
            beta += dist(gen_) * 2.0 * max_w;
            while(beta > particles_[index].weight) {
                beta -= particles_[index].weight;
                index = (index + 1) % num_particles_;
            }
            
            Particle p = particles_[index];
            new_particles.push_back(p);
        }
        particles_ = new_particles;
    }

    Pose2D ParticleFilter::getEstimate() const {
        double x_sum = 0, y_sum = 0;
        double sin_sum = 0, cos_sum = 0;
        
        for(const auto& p : particles_) {
            x_sum += p.x;
            y_sum += p.y;
            sin_sum += std::sin(p.theta);
            cos_sum += std::cos(p.theta);
        }
        
        return Pose2D(
            x_sum / num_particles_, 
            y_sum / num_particles_, 
            std::atan2(sin_sum, cos_sum)
        );
    }
}
