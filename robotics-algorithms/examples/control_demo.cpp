#include <iostream>
#include <thread>
#include <chrono>
#include "robotics_algo/control/pid.hpp"

int main() {
    std::cout << "--- PID Temperature Control Simulation ---" << std::endl;

    // Config: Standard PID
    robotics::control::PID pid(2.0, 0.5, 0.1);
    
    double target_temp = 100.0;
    double current_temp = 20.0; // Start cold
    double dt = 0.1; // 100ms time step

    for (int i = 0; i < 50; ++i) {
        // 1. Calculate control signal (e.g., Heater Power)
        double power = pid.update(target_temp, current_temp, dt);

        // 2. Physics Simulation (Simple Heating Model)
        // Temperature rises with power, falls with cooling (0.05 factor)
        current_temp += (power * 0.1) - (current_temp * 0.02);

        // 3. Log
        std::printf("Time: %.1f s | Temp: %.2f | Power: %.2f\n", i * dt, current_temp, power);

        // Optional: Slow down to see it happen
        // std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    return 0;
}
