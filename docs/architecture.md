# InHouseROS Architecture

This document describes the system architecture of InHouseROS **as it is
today**: how the libraries relate, how data flows, and what the threading
and memory models are.

> For a critical assessment and where the design should go next, see
> [`architecture_evolution.md`](architecture_evolution.md).

> All diagrams are [Mermaid](https://mermaid.js.org/) and render on GitHub and in
> most Markdown viewers.

---

## 1. System context

InHouseROS sits between robot hardware (sensors, actuators) and the consumer
world (nodes, tools, dashboards). Each communication scope uses the cheapest
transport that satisfies its latency budget.

```mermaid
flowchart LR
    subgraph HW["Robot hardware"]
        SENS["Sensors\n(IMU, encoders, camera)"]
        ACT["Actuators\n(drives, grippers)"]
    end

    subgraph IHRS["InHouseROS runtime"]
        IPC["ihrs::ipc\nshared memory pub/sub"]
        CORE["ihrs::core\nBroker / Node / Executor"]
        TRANSPORT["ihrs::transport\nTCP / UDP / ZMQ / MQTT / WebRTC"]
        ALGO["ihrs::algo\nplanning, control, localization,\nkinematics, perception, decision"]
    end

    subgraph CONSUMERS["Consumers"]
        NODES["Robot nodes"]
        TOOLS["Operator tools\n(topic_viewer)"]
        FLEET["Fleet / browser clients"]
    end

    SENS -- "high-rate state" --> IPC
    IPC -- "zero-copy samples" --> CORE
    CORE --> ALGO
    ALGO -- "commands" --> ACT
    CORE <--> TRANSPORT
    TRANSPORT --> NODES
    CORE --> TOOLS
    TRANSPORT --> FLEET
```

## 2. Library layering

Dependencies point strictly downward. Nothing in `ipc` or `transport` knows
about `core`; the bridge between middleware and transports happens in
applications (today) or in a future bridge node (see roadmap).

```mermaid
flowchart TD
    APPS["apps\n(demo nodes, bridges, tools)"]

    subgraph LIBS["Libraries"]
        CORE["ihrs::core\nin-process pub/sub middleware"]
        IPC["ihrs::ipc\nPOSIX shared memory"]
        TRANSPORT["ihrs::transport\nsockets (TCP/UDP),\noptional ZMQ/MQTT/WebRTC"]
        ALGO["ihrs::algo\nrobotics algorithms\n(Eigen, optional SDL2)"]
    end

    STDLIB["C++20 standard library\npthreads, POSIX shm"]

    APPS --> CORE
    APPS --> IPC
    APPS --> TRANSPORT
    APPS --> ALGO
    CORE --> STDLIB
    IPC --> STDLIB
    TRANSPORT --> STDLIB
    ALGO --> STDLIB
```

## 3. Core middleware design

### 3.1 Class model

```mermaid
classDiagram
    class Message {
        <<abstract>>
        +~Message()
    }

    class Broker {
        <<one per Context>>
        +subscribe(topic, cb) Token
        +unsubscribe(topic, token) void
        +publish(topic, msg) void
        +spinOnce() void
        +pendingCount() size_t
        +droppedCount() size_t
        -subscribers_ : map
        -queue_ : vector of tasks
        -max_pending_ : size_t
    }

    class Publisher {
        +publish(MessagePtr) void
        +topic() string
    }

    class Subscriber {
        +Subscriber(topic, cb)
        -token_ : Token
        -unsubscribe on destruction
    }

    class Executor {
        +addNode(Node*) void
        +spin(hz) void
        +stop() void
    }

    class Node {
        <<abstract>>
        +onInit() void
        +onUpdate(dt) void
    }

    Message <|-- Pose2DMsg : user defined
    Publisher --> Broker : enqueue
    Subscriber --> Broker : register and remove
    Broker o-- Message : shared_ptr queue
    Executor --> Broker : spinOnce
    Executor o-- Node : raw pointer
```

Key decisions:

| Decision | Rationale |
| :--- | :--- |
| Broker is owned by a `Context` | `Context::global()` preserves the simple default; local contexts give isolation and DI for tests |
| RAII `Subscriber` with tokens | Destroying a subscriber can never leave a dangling callback |
| Bounded queue, drop-oldest | A slow consumer can never block a high-rate publisher; drops are observable via `droppedCount()` |
| Callbacks run in `spinOnce()` | Single-threaded execution per process — no callback races, like ROS 1 spin model |
| `Node` is a plain object | Executor stores a reference; lifetime is explicit and cheap |

### 3.2 Publish/subscribe sequence

```mermaid
sequenceDiagram
    participant P as Publisher (thread A)
    participant B as Broker (mutex-protected)
    participant Q as Pending queue (bounded)
    participant E as Executor (thread B)
    participant S as Subscriber callbacks

    P->>B: publish("odom", msg)
    B->>B: lock, find subscribers of "odom"
    B->>Q: enqueue 1 task per subscriber
    Note over Q: if full: drop oldest,<br/>droppedCount()++
    B-->>P: return (non-blocking)

    E->>B: spinOnce() [once per cycle]
    B->>B: lock, swap(queue_, local)
    B->>S: run callbacks (oldest first)
    E->>E: Node::onUpdate(dt) for each node
```

### 3.3 Executor cycle

```mermaid
flowchart LR
    A["cycle start\n(t0 = steady_clock::now)"] --> B["Broker::spinOnce()\ndispatch messages"]
    B --> C["Node::onUpdate(dt)\nfor every node"]
    C --> D{"wait_until(t0 + period)\nCV-wait, exits early on stop()"}
    D -- "running" --> A
    D -- "stop() called" --> E["return"]
```

`stop()` takes the mutex, flips an atomic flag, and notifies the condition
variable, so a 1 Hz loop stops within microseconds instead of sleeping a full
cycle. `spin()` must not be stopped from its own thread (see the class docs).

### 3.4 Typed topic API and bridges (Stage 2)

The typed layer is a thin wrapper over the broker: `Pub<T>::publish(const T&)`
wraps the payload in a type-tagged `TypedMessage<T>` and enqueues through the
same broker path; `Sub<T>` unwraps and forwards `const T&` to user code.
Payload types must be trivially copyable (auto memcpy codec) or provide a
`Codec<T>` specialization — enforced at compile time. Each topic binds one
payload type hash in the broker; conflicts throw `TypeMismatchError`.

`ihrs::bridge` composes the messaging libraries into ready-made forwarding
endpoints, so no application hand-wiring is needed:

```mermaid
flowchart LR
    subgraph PROC["one process"]
        PUB["Pub&lt;Imu&gt;
/imu"] --> BROKER["Broker"]
        BROKER --> SUB1["Sub&lt;Imu&gt;"]
        SUB1 --> SHMSINK["ShmTopicSink
latest-value"]
        SUB1 --> UDPSINK["UdpTopicSink
framed datagrams"]
        BROKER --> SUBOUT["Sub&lt;Imu&gt; (/in)"]
    end
    SHMSINK --> SHM["/dev/shm segment"]
    SHM --> SHMSRC["ShmTopicSource"] --> BROKER
    UDPSINK -->|UDP| NET["network"]
    NET --> UDPSRC["UdpTopicSource"] --> BROKER
```

The `bridge` app maps topics to endpoints from the command line over a
per-binary type registry (`--shm-out topic:segment:Type`,
`--udp-in topic:port:Type`, ...).

## 4. IPC: shared-memory pub/sub

### 4.1 Segment layout

The first publisher for a name creates the segment (`shm_open` + `ftruncate`)
and initializes a `PTHREAD_PROCESS_SHARED` mutex; it unlinks the segment on
destruction. Subscribers attach read-write and wait for the `initialized`
flag before reading.

```mermaid
flowchart LR
    subgraph SHM["POSIX shared memory segment (/dev/shm/&lt;name&gt;)"]
        MUTEX["pthread_mutex_t\nprocess-shared"]
        INIT["atomic&lt;bool&gt;\ninitialized"]
        MSG["char message[...]\nlatest payload"]
    end

    subgraph PUB["Publisher process"]
        PRUN["run():\nlock → write → unlock → sleep(1/rate)"]
    end

    subgraph SUB["Subscriber process"]
        SRUN["run():\nlock → read → unlock → callback → sleep(1/rate)"]
    end

    PRUN -- "guard" --> MUTEX
    SRUN -- "guard" --> MUTEX
    INIT -- "gate before first read" --> SRUN
    MSG --> SRUN
```

### 4.2 Typed API (`GenericSHMPublisher<T>` / `GenericSHMSubscriber<T>`)

```mermaid
sequenceDiagram
    participant P as GenericSHMPublisher<T>
    participant S as GenericSHMSubscriber<T>
    Note over P,S: same segment, created by P
    P->>P: shm_open(O_CREAT|O_EXCL), ftruncate,<br/>init process-shared mutex
    P->>P: initialized = true
    S->>S: shm_open(O_RDWR), mmap
    S->>S: wait until initialized
    loop at rate Hz until stop()
        P->>P: lock, memcpy payload, unlock
        S->>S: lock, memcpy payload, unlock, callback(msg)
    end
    P->>P: destructor: munmap + shm_unlink
    S->>S: destructor: munmap only
```

Robustness: all segments use `PTHREAD_MUTEX_ROBUST`. If an owner dies
while holding the lock, the next locker gets `EOWNERDEAD`, marks the mutex
consistent, and recovers: publishers overwrite the payload (torn data is
replaced); subscribers skip the possibly-torn sample and wait for a fresh
publish.

Remaining limitations (tracked in the roadmap): the protocol stores only
the latest message (no queueing, no liveness tracking) and `run()` polls
rather than using condition variables on the shared mutex.

## 5. Transport layer

All outbound transports implement one interface; inbound receivers expose a
blocking `run()` that returns promptly when `stop()` is called from another
thread (implemented with `shutdown()` on the socket).

```mermaid
classDiagram
    class PacketSender {
        <<interface>>
        +connect(ip, port) bool
        +send(data) bool
        +close() void
    }

    class TCPSender
    class UDPSender

    class TCPReceiver {
        +run() void
        +stop() void
    }

    class UDPReceiver {
        +run() void
        +stop() void
    }

    PacketSender <|.. TCPSender
    PacketSender <|.. UDPSender
```

Reliability model: TCP is byte-oriented (segments may be coalesced or split —
frame the payload at the application layer), UDP is datagram-oriented (each
`send()` is one datagram, silently dropped under loss).

Optional transports (ZeroMQ, MQTT, WebRTC) are compiled only when their
dependencies are found; see the main README for the dependency matrix.

## 6. Algorithm library (`ihrs::algo`)

Domain modules follow the sense-think-act loop and depend only on Eigen
(header-only) plus the standard library; the SDL2 visualizer is optional and
private to the library.

```mermaid
flowchart LR
    PERCEPTION["perception\noccupancy / voxel grids"] --> LOCALIZATION["localization\nKalman, particle filter"]
    PERCEPTION --> PLANNING["planning\nA*, RRT, RRT*, PRM"]
    LOCALIZATION --> DECISION["decision\nFSM, MDP"]
    PLANNING --> CONTROL["control\nPID, pure pursuit, MPC"]
    KINEMATICS["kinematics\nFK / IK (2-6 DOF)"] --> CONTROL
    VIZ["simulators/viz\nSDL2 plots"] -.-> PLANNING
    VIZ -.-> CONTROL
    VIZ -.-> LOCALIZATION
```

## 7. Threading model summary

| Component | Threads | Synchronization |
| :--- | :--- | :--- |
| `core::Broker` | any publisher thread, one spinning thread | one mutex; callbacks never run concurrently |
| `core::Executor` | caller of `spin()` blocks; `stop()` from anywhere | atomic flag + condition variable |
| `ipc::SharedMemory*` | one `run()` thread per process | process-shared `pthread_mutex_t` |
| `ipc::GenericSHM<T>` | one `run()` thread per instance | process-shared `pthread_mutex_t` |
| `transport::TCP/UDPReceiver` | dedicated `run()` thread | atomic flag; `shutdown()` unblocks syscalls |

## 8. Build and test infrastructure

```mermaid
flowchart TD
    PRESETS["CMakePresets.json\ndebug / release / sanitize / minimal"] --> CFG["cmake --preset &lt;name&gt;"]
    CFG --> GEN["Generate build/&lt;name&gt;/"]
    GEN --> BUILD["cmake --build --preset &lt;name&gt;"]
    BUILD --> LIBS["libihrs_core, libihrs_ipc,\nlibihrs_transport, libihrs_algo"]
    BUILD --> BINS["bin/: apps, examples, tools"]
    BUILD --> TESTS["ihrs_core_tests, ihrs_ipc_tests,\nihrs_transport_tests, ihrs_algo_tests"]
    TESTS --> CTEST["ctest --preset &lt;name&gt;\n22 tests"]
```

Warning policy: new middleware code builds with the strict profile
(`ihrs_warnings`: conversion + old-style-cast checks); the large imported
`ihrs::algo` codebase builds with `ihrs_warnings_legacy` (`-Wall -Wextra
-Wpedantic -Wshadow`) until it is modernized.
