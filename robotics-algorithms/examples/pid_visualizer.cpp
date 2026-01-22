#include "robotics_algo/control/pid.hpp"
#include "robotics_algo/simulators/viz/graph_visualizer.hpp"

int main() {
    // 1. Setup Logic
    robotics::control::PID pid(0.6, 0.01, 0.15); 
    robotics::viz::GraphVisualizer viz("Reusable PID Viz");

    double target = 100.0;
    double current = 20.0;
    double velocity = 0.0;
    double dt = 0.1;

    // 2. Run Loop
    while (viz.isRunning()) {
        // Physics
        double force = pid.update(target, current, dt);
        double acceleration = force - (velocity * 0.1); 
        velocity += acceleration * dt;
        current += velocity * dt;

        // Render (One line!)
        viz.update(
        {target, current}, 
        { {200, 50, 50}, {50, 200, 50} } // Red (Target), Green (Actual)
    );
        
        // Optional: Manual Reset logic here if you want to hook into viz key events
        // But the class handles SPACE bar clearing internally now.
    }

    return 0;
}
