# ihrs::algo — Robotics Algorithms

![License](https://img.shields.io/badge/license-MIT-blue)
![Standard](https://img.shields.io/badge/C%2B%2B-20-blue.svg)

A modular C++20 library of core algorithms for mobile robotics and manipulator arms,
part of the [InHouseROS](../../README.md) monorepo.

> **Note:** Designed for education and rapid prototyping. For safety-critical production
> systems, use ROS 2 / MoveIt.

## Modules

| Module | Contents |
| :--- | :--- |
| `common` | Math utilities, SE(2)/SE(3) transforms |
| `planning` | A*, grid maps, RRT, RRT*, PRM |
| `control` | PID, pure pursuit, MPC |
| `localization` | 1D Kalman filter, particle filter (MCL) |
| `perception` | Occupancy grid, voxel grid mapping |
| `kinematics` | 2/3/N/6-DOF arm FK & IK |
| `decision` | FSM, MDP value iteration |
| `simulators/viz` | SDL2-based graph visualizer |

Namespaces mirror the include path: `ihrs::algo::planning`, `ihrs::algo::control`, etc.

## Layout

```
libs/algo/
├── include/ihrs/algo/   # PUBLIC API (one folder per module)
├── src/                 # Implementation
├── examples/            # Standalone demos (many with SDL2 visualization)
├── tests/               # GoogleTest unit tests
└── third_party/Eigen/   # Eigen (git submodule)
```

## Building

Built by the superproject from the repo root — see the main
[README](../../README.md):

```bash
cmake --preset debug
cmake --build --preset debug -j
ctest --preset debug
```

Standalone use: add this directory with `add_subdirectory` and link `ihrs::algo`
(needs the Eigen submodule at `third_party/Eigen`).

## Examples

`hello_robotics`, `control_demo`, `pid_visualizer`, `planning_demo`,
`astar_visualizer`, `kalman_demo`, `arm_kinematics_demo`, `state_machine_demo`,
`rrt_demo`, `rrt_star`, `prm_demo`, `voxel_demo`, `mapping_demo`,
`navigation_demo`, `pf_demo`, `arm_3d_demo`, `n_arm_demo`, `six_dof_demo`,
`mpc_demo`, `mdp_demo`.

Binaries are emitted to `build/<preset>/bin` (SDL2 visualizers require `libsdl2-dev`).

## License

MIT — see root [LICENSE](../../LICENSE).
