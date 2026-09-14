# InHouseROS

![Language](https://img.shields.io/badge/language-C%2B%2B20-blue.svg)
![Platform](https://img.shields.io/badge/platform-Linux-lightgrey.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![Build](https://img.shields.io/badge/build-CMake%20%2B%20Presets-064F8C.svg)

**A high-performance, lightweight robotics middleware and algorithm library — a ground-up
alternative to ROS with minimal dependencies and a clean, modern C++20 codebase.**

## Why

Standard ROS is powerful but heavy. InHouseROS maximizes performance (direct shared-memory
IPC without serialization overhead), minimizes dependency hell, and doubles as a learning
codebase for robotics communication and algorithms.

## Architecture

A **hybrid communication strategy** balances speed and reach:

| Scope | Technology | Library | Latency | Use case |
| :--- | :--- | :--- | :--- | :--- |
| Local node-to-node | POSIX SHM + mutex | `ihrs::ipc` | Ultra-low | High-frequency sensor data (IMU, encoders) |
| In-process pub/sub | Broker + Executor | `ihrs::core` | Low | Nodes, topics, callback scheduling |
| Network control | TCP / UDP / ZeroMQ* | `ihrs::transport` | Low | Control commands, state estimates |
| Data streaming | WebRTC* | `ihrs::transport` | Medium | Video, point clouds, telemetry |
| Fleet management | MQTT* | `ihrs::transport` | Higher | Low-frequency status, fleet coordination |

\* optional — compiled only when the dependency is found (`cppzmq`, Paho C++, libdatachannel).

`ihrs::algo` provides the robotics algorithm toolbox: planning (A*, RRT, RRT*, PRM),
control (PID, pure pursuit, MPC), localization (Kalman, particle filter), kinematics
(2–6 DOF arms), perception (occupancy/voxel grids), and decision making (FSM, MDP).

## Project layout

```
.
├── CMakeLists.txt        # Superproject: options, deps, subdirectories
├── CMakePresets.json     # debug / release / sanitize / minimal presets
├── cmake/                # Shared CMake helpers
├── libs/
│   ├── core/             # ihrs::core      Node, Executor, Broker, topics
│   ├── ipc/              # ihrs::ipc       POSIX shared-memory pub/sub
│   ├── transport/        # ihrs::transport TCP/UDP (+ optional ZMQ/MQTT/WebRTC)
│   └── algo/             # ihrs::algo      Robotics algorithms (+ Eigen, SDL2 viz)
├── apps/                 # Demo nodes, bridges and operator tools
├── tests/                # GTest unit tests (ctest)
├── third_party/          # (vendored deps land here)
├── docs/                 # Architecture and design notes
└── archive/              # Preserved experiments (not built)
```

## Building

Requirements: GCC 11+ / Clang 14+, CMake 3.21+, make, pthreads.
Optional: SDL2 (visualizers), cppzmq, Paho C++ (mqtt), libdatachannel (webrtc).

```bash
# Configure + build (pick a preset)
cmake --preset debug
cmake --build --preset debug -j

# Run the test suite
ctest --preset debug

# Other presets
cmake --preset release && cmake --build --preset release -j   # optimized
cmake --preset sanitize && cmake --build --preset sanitize -j # ASan/UBSan
cmake --preset minimal  && cmake --build --preset minimal -j  # no optional deps
```

Binaries land in `build/<preset>/bin`, libraries in `build/<preset>/lib`.

## Quick start

```bash
# In-process pub/sub (core middleware)
./build/debug/bin/odometry_node

# Shared-memory publisher / subscriber (two terminals)
./build/debug/bin/shm_publisher /demo_shm
./build/debug/bin/shm_subscriber /demo_shm

# Inspect topics
./build/debug/bin/topic_viewer odom
```

## Using the libraries

```cpp
#include <ihrs/core/publisher.hpp>
#include <ihrs/core/subscriber.hpp>
#include <ihrs/core/executor.hpp>

struct Pose2DMsg : ihrs::core::Message { double x, y, theta; };

int main() {
    using namespace ihrs::core;
    Subscriber sub("odom", [](MessagePtr msg) {
        auto p = std::static_pointer_cast<Pose2DMsg>(msg);
        // consume pose...
    });
    Publisher pub("odom");
    pub.publish(std::make_shared<Pose2DMsg>());
    Broker::instance().spinOnce();
}
```

See [`docs/`](docs/) for architecture notes and per-module documentation:

| Document | Contents |
| :--- | :--- |
| [`docs/architecture.md`](docs/architecture.md) | Full system architecture with Mermaid diagrams (context, layering, class models, sequences, memory layout, threading) |
| [`docs/architecture_evolution.md`](docs/architecture_evolution.md) | Critical architecture review + target design and staged evolution plan |
| [`docs/design_review.md`](docs/design_review.md) | Architecture & code design review: findings, resolutions, open risks, roadmap rationale |
| [`libs/core/README.md`](libs/core/README.md) | Broker / Publisher / Subscriber / Executor semantics |
| [`libs/ipc/README.md`](libs/ipc/README.md) | Shared-memory pub/sub: ownership, lifecycle, limits |
| [`libs/transport/README.md`](libs/transport/README.md) | Transport interface contract and selection guide |

## Roadmap

Prioritized; the rationale behind each item is in
[`docs/design_review.md`](docs/design_review.md).

- [x] Shared-memory pub/sub with unified wrapper API
- [x] In-process broker/executor middleware (RAII subscriptions, bounded queue, prompt-stop executor)
- [x] TCP/UDP transports behind a common `PacketSender` interface with stoppable receivers, binary-safe payloads and length-prefix framing
- [x] Robust shared-memory mutexes (`EOWNERDEAD` recovery) + CI (GitHub Actions)
- [x] Robotics algorithm library with tests
- [ ] Lock-free SHM ring buffer (SPSC, condition-variable wakeup)
- [ ] Generic `bridge_node` (SHM ↔ core ↔ transport)
- [ ] gRPC service calls, WebRTC video streaming

## License

MIT — see [LICENSE](LICENSE).
