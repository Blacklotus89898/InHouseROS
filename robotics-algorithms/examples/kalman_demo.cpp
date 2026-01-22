#include "robotics_algo/localization/kalman_filter_1d.hpp"
#include "robotics_algo/simulators/viz/graph_visualizer.hpp"
#include <random>
#include <iostream>

int main() {
    // 1. Setup Viz
    robotics::viz::GraphVisualizer viz("Kalman Filter (Blue=Est, Red=Meas, Green=True)", 1000, 600);

    // 2. Setup Filter
    // Initial Pos=0, Uncertainty=1.0
    // Process Noise (Q)=0.1, Sensor Noise (R)=5.0 (Very noisy sensor!)
    robotics::localization::KalmanFilter1D kf(0.0, 1.0, 0.1, 5.0);

    // 3. Simulation Variables
    double true_position = 0.0;
    double velocity = 0.5; // Moving constantly
    double dt = 0.1;

    // Random Noise Generator
    std::default_random_engine generator;
    std::normal_distribution<double> sensor_noise(0.0, 2.0); // Mean 0, StdDev 2.0

    while (viz.isRunning()) {
        // --- A. SIMULATION (The Real World) ---
        true_position += velocity * dt; // Robot moves smoothly

        // Generate a noisy sensor reading
        double noisy_measurement = true_position + sensor_noise(generator);

        // --- B. ROBOT LOGIC (The Filter) ---
        // 1. Predict: Robot thinks "I sent power to wheels, I should have moved"
        kf.predict(velocity * dt);

        // 2. Update: Robot sees sensor data and corrects itself
        kf.update(noisy_measurement);
        
        double estimated_position = kf.getState();

        // --- C. RENDER ---
        // We reuse GraphVisualizer. 
        // NOTE: Our simple reusable class only plots 2 lines (Target/Actual).
        // Let's cheat slightly: 
        // Target Line (Red) -> Noisy Measurement
        // History Line (Green) -> Estimated Position
        // Ideally, we'd upgrade the visualizer to support 3 lines, but this works to see the smoothing.
        
        viz.update(
        {true_position, noisy_measurement, estimated_position}, 
        { {50, 200, 50}, {255, 50, 50}, {50, 100, 255} }
    );

        // Console log to see the "Ground Truth" vs Estimate
        // std::cout << "True: " << true_position << " | Est: " << estimated_position << std::endl;
    }

    return 0;
}
