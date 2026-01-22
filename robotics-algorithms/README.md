# 🤖 Robotics Algorithms C++

![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![License](https://img.shields.io/badge/license-MIT-blue)
![Standard](https://img.shields.io/badge/C%2B%2B-17-blue.svg)

A modern, modular C++17 library implementing core algorithms for mobile robotics and manipulator arms. This project aims to provide clean, readable, and tested implementations of standard algorithms used in Planning, Control, Localization, and Decision Making.

> **Note:** This library is designed for educational purposes and rapid prototyping. For production safety-critical systems, please refer to ROS 2 or MoveIt.

---

## 🏗 Architecture

The library is organized into five core modules following the "Sense-Think-Act" loop:

| Module | Description | Key Algorithms |
| :--- | :--- | :--- |
| **Planning** | Pathfinding & Motion Planning | Dijkstra, A*, RRT, RRT*, PRM |
| **Control** | Feedback control loops | PID, MPC, LQR |
| **Localization** | State estimation | EKF, UKF, Particle Filter (MCL), SLAM |
| **Kinematics** | Robot arm math | Forward/Inverse Kinematics, Jacobian |
| **Decision** | High-level logic | Finite State Machines (FSM), Behavior Trees |

---

## 🚀 Getting Started

### Prerequisites
* **CMake** (3.14+)
* **C++ Compiler** (GCC 9+, Clang 10+, or MSVC 2019+)
* **Eigen3** (Downloaded automatically via CMake)
* **GoogleTest** (Downloaded automatically via CMake)

### Build Instructions

```bash
# 1. Clone the repository
git clone [https://github.com/your-username/robotics-algorithms.git](https://github.com/your-username/robotics-algorithms.git)
cd robotics-algorithms

# 2. Configure the build
mkdir build && cd build
cmake ..

# 3. Build everything (Lib, Tests, Examples)
make

# 4. Run Unit Tests
ctest --output-on-failure
```

### Project Structure
```bash
robotics-algorithms/
├── CMakeLists.txt                   # Root build configuration
├── README.md                        # Documentation and build instructions
├── LICENSE                          # MIT/Apache license
│
├── include/                         # PUBLIC API (Headers users will include)
│   └── robotics_algo/               # Namespace folder (prevents file name collisions)
│       ├── common/
│       │   ├── types.hpp            # Standard typedefs (scalar_t, Time, etc.)
│       │   ├── math/
│       │   │   ├── vector2.hpp
│       │   │   ├── vector3.hpp
│       │   │   ├── matrix.hpp
│       │   │   └── transform.hpp    # SE(2) and SE(3) transforms
│       │   ├── geometry/
│       │   │   ├── point.hpp
│       │   │   ├── pose.hpp
│       │   │   └── collision_shapes.hpp
│       │   └── utils/
│       │       ├── timer.hpp
│       │       ├── logger.hpp
│       │       └── random_generator.hpp
│       │
│       ├── planning/
│       │   ├── planner_base.hpp     # Abstract base class (Strategy Pattern)
│       │   ├── graph_search/
│       │   │   ├── astar.hpp
│       │   │   ├── dijkstra.hpp
│       │   │   └── grid_graph.hpp
│       │   └── sampling_based/
│       │       ├── rrt.hpp
│       │       ├── rrt_star.hpp
│       │       └── prm.hpp
│       │
│       ├── control/
│       │   ├── controller_base.hpp
│       │   ├── pid.hpp
│       │   ├── mpc/
│       │   │   ├── mpc_solver.hpp
│       │   │   └── vehicle_model.hpp
│       │   └── lqr.hpp
│       │
│       ├── localization/
│       │   ├── kalman/
│       │   │   ├── ekf.hpp          # Extended Kalman Filter
│       │   │   └── ukf.hpp          # Unscented Kalman Filter
│       │   ├── particle_filter/
│       │   │   ├── mcl.hpp          # Monte Carlo Localization
│       │   │   └── particle.hpp
│       │   └── slam/
│       │       ├── graph_slam.hpp
│       │       └── fast_slam.hpp
│       │
│       ├── perception/
│       │   ├── occupancy_grid.hpp
│       │   └── lidar_processing.hpp # Raycasting, clustering
│       │
│       ├── kinematics/
│       │   ├── kinematic_chain.hpp  # Represents a robot arm
│       │   ├── fk_solver.hpp        # Forward Kinematics
│       │   └── ik_solver.hpp        # Inverse Kinematics (Abstract)
│       │
│       └── decision/
│           ├── fsm/
│           │   ├── state_machine.hpp
│           │   └── state.hpp
│           └── behavior_tree/
│               ├── behavior_tree.hpp
│               └── nodes.hpp
│
├── src/                             # IMPLEMENTATION (Hidden logic & .cpp files)
│   ├── common/
│   │   ├── math.cpp
│   │   ├── logger.cpp
│   │   └── CMakeLists.txt
│   │
│   ├── planning/
│   │   ├── graph_search/
│   │   │   ├── astar.cpp
│   │   │   └── dijkstra.cpp
│   │   ├── sampling_based/
│   │   │   ├── rrt.cpp
│   │   │   └── prm.cpp
│   │   └── CMakeLists.txt
│   │
│   ├── control/
│   │   ├── pid.cpp
│   │   ├── mpc.cpp
│   │   └── CMakeLists.txt
│   │
│   ├── localization/
│   │   ├── ekf.cpp
│   │   ├── mcl.cpp
│   │   └── CMakeLists.txt
│   │
│   ├── perception/
│   │   ├── occupancy_grid.cpp
│   │   └── CMakeLists.txt
│   │
│   ├── kinematics/
│   │   ├── kinematic_chain.cpp
│   │   ├── analytical_ik.cpp
│   │   └── jacobian_ik.cpp
│   │
│   └── decision/
│       ├── fsm.cpp
│       └── CMakeLists.txt
│
├── third_party/                     # External Dependencies
│   ├── Eigen/                       # Linear Algebra (Don't write your own Matrix lib!)
│   ├── json/                        # For config loading
│   └── matplotlib-cpp/              # C++ wrapper for Python plotting
│
├── simulators/                      # Simple Sandbox Environments
│   ├── CMakeLists.txt
│   ├── headless_sim/                # Physics-only logic
│   │   ├── differential_drive.hpp
│   │   └── robot_arm.hpp
│   └── viz/                         # Visualization (SDL2/OpenGL)
│       ├── renderer.hpp
│       └── window.hpp
│
├── examples/                        # "How-to-use" Demos
│   ├── CMakeLists.txt
│   ├── planning_demo.cpp            # Visual RRT demo
│   ├── control_demo.cpp             # PID plotting demo
│   ├── localization_demo.cpp        # Particle filter visualization
│   └── arm_kinematics_demo.cpp
│
├── tests/                           # Unit Tests (GoogleTest)
│   ├── CMakeLists.txt
│   ├── test_astar.cpp
│   ├── test_pid.cpp
│   ├── test_matrix_math.cpp
│   └── test_fsm.cpp
│
├── scripts/                         # Utilities
│   ├── install_deps.sh              # Setup script (apt-get install...)
│   ├── plot_logs.py                 # Python script to verify PID/MPC logs
│   └── format_code.sh               # Clang-format runner
│
└── docs/                            # Documentation
    ├── Doxyfile                     # Auto-gen API docs
    └── diagrams/
        ├── system_architecture.png
        └── class_hierarchy.png
```

### 📈 Roadmap

    [x] Project Skeleton & Build System

    [ ] Phase 1: Math Core (Vectors, Matrices, Transforms)

    [ ] Phase 2: Basic Planning (Grid Search: A*, Dijkstra)

    [ ] Phase 3: Basic Control (PID)

    [ ] Phase 4: Localization (1D Kalman Filter)

    [ ] Phase 5: Kinematics (2-Link Arm FK/IK)
