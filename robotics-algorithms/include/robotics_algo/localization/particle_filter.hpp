#pragma once
#include <vector>
#include <random>
#include "robotics_algo/common/types.hpp"
#include "robotics_algo/planning/graph_search/grid_map.hpp"

namespace robotics::localization {

    struct Particle {
        double x, y, theta;
        double weight;
    };

    class ParticleFilter {
    public:
        // num_particles: e.g., 500 or 1000
        ParticleFilter(int num_particles, const planning::GridMap& map);

        // 1. Initialization
        void init(double start_x, double start_y, double start_theta, double spread);

        // 2. Motion Update (Prediction)
        void predict(double delta_dist, double delta_theta);

        // 3. Sensor Update (Correction)
        void update(const planning::GridMap& map);

        // 4. Resample (Selection)
        void resample();

        // Getters
        const std::vector<Particle>& getParticles() const { return particles_; }
        Pose2D getEstimate() const;

    private:
        int num_particles_;
        std::vector<Particle> particles_;
        std::default_random_engine gen_;
        
        // --- TIGHTER NOISE PARAMETERS ---
        // Reduced these values to prevent the cloud from exploding
        double motion_noise_dist_ = 0.5;   // Error per step (pixels)
        double motion_noise_theta_ = 0.02; // Error per turn (radians)
    };
}
