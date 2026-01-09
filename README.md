# InHouseROS

![Language](https://img.shields.io/badge/language-C%2B%2B-blue.svg)
![Platform](https://img.shields.io/badge/platform-Linux-lightgrey.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

**A high-performance, lightweight Robot Operating System implementation aimed at removing the abstraction overhead of standard ROS.**

## 🚀 Motivation
Standard ROS is powerful but often introduces heavy dependencies and abstraction layers that can limit performance on constrained hardware. **InHouseROS** was built to:
* **Maximize Performance:** Direct memory manipulation without serialization overhead where possible.
* **Minimize Dependencies:** Avoid the "dependency hell" of massive meta-packages.
* **Enhance Learning:** A ground-up implementation of robotics communication paradigms.

---

## 🏗 Architecture
InHouseROS utilizes a **Hybrid Communication Strategy** to balance speed and reach:

| Scope | Technology | Latency | Use Case |
| :--- | :--- | :--- | :--- |
| **Local Node-to-Node** | **POSIX Shared Memory + Mutex** | Ultra-Low | High-frequency sensor data (IMU, Encoders) between processes on the same CPU. |
| **Network Control** | **ZeroMQ / gRPC** | Low | Control commands, state estimation updates between robot and controller. |
| **Data Streaming** | **WebRTC / TCP / UDP** | Medium | Video feeds, point clouds, and telemetry. |
| **Fleet Management** | **MQTT** | High | Low-frequency status updates, logging, and fleet coordination. |

---

## 📂 Project Structure
```text
.
├── bin/                  # Compiled executables
├── include/              # Header files (SHM, TCP/UDP, WebRTC wrappers)
├── src/
│   ├── comms/            # Raw TCP/UDP socket implementations
│   ├── mqtt/             # Mosquitto pub/sub implementation & Docker config
│   ├── programs/         # Entry points (main nodes, test scripts)
│   ├── pubSub/           # Shared Memory logic (Core IPC)
│   ├── webrtc/           # WebRTC Data Channels (Caller/Callee)
│   └── zmq/              # ZeroMQ implementation
└── tests/                # Unit and integration tests
```
## 🛠️ Technology Stack
Language: C++ (Core logic), C/Rust/Zig (Potential future additions)

IPC: POSIX Shared Memory, Pthreads, Mutexes

Messaging: ZeroMQ (0MQ), MQTT (Mosquitto)

Streaming: Libdatachannel (WebRTC), Raw TCP/UDP Sockets

## ⚡ Getting Started
Prerequisites
Ensure you have the following libraries installed (Ubuntu/Debian example):

```Bash

# Core build tools
sudo apt install build-essential cmake

# Communication dependencies
sudo apt install libzmq3-dev libmosquitto-dev

# WebRTC (Libdatachannel)
# Requires building from source or adding specific PPA
Building the Project
Bash

# Compile the core system
make

# Compile specific modules
make publisher
make subscriber
Running the Simulation (IPC Demo)
This demonstrates the Shared Memory Pub/Sub logic:

Bash

make test
# Output:
# [Publisher] Sent: [Pub1] Count: 0
# Sub1 received: [Pub1] Count: 0
Running Network Nodes
To run the WebRTC or TCP nodes:

Bash

# Terminal 1
./bin/receiver

# Terminal 2
./bin/sender
```
## ✅ Feature Roadmap & TODO
### Core  Infrastructure
- [x] Shared Memory Pub/Sub: Efficient local IPC.

 - [x] Wrapper Interface: Unified API to initiate publishers/subscribers.

 - [x] Multi-Node Support: Single process handling multiple Pubs/Subs.

 - [ ] Robust Mutexes: Handle crash recovery (prevent deadlocks if a node dies).

 - [ ] Lock-Free Queue: Implement a ring buffer for higher throughput.

###  Networking & Remote Control
 - [x] TCP/UDP Layer: Basic packet transmission.

 - [x] WebRTC Data Channels: Browser-compatible data streaming.

 - [x] MQTT Integration: Dockerized Mosquitto broker setup.

 - [ ] gRPC Interface: Strict typing for service calls.

 - [ ] Video Streaming: Implement webrtc video stream for camera feed.

### Tooling & Usability
 - [ ] Parameter Server: Dynamic configuration loading.

 - [ ] Process Manager: tmux script or simple daemon to manage node lifecycle.

 - [ ] Monitoring Dashboard: TUI or Web GUI to visualize active nodes and rates.