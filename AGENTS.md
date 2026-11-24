# AGENTS.md — InHouseROS

Guide for any coding agent (Claude Code, Codex, Cursor, Copilot, Gemini…) and human contributor.

## What this is

A lightweight, ground-up alternative to ROS in modern C++20: in-process pub/sub
(`ihrs::core`), POSIX shared-memory IPC (`ihrs::ipc`), TCP/UDP (+ optional ZMQ/MQTT/WebRTC)
transports (`ihrs::transport`), a typed bridge (`ihrs::bridge`) and a robotics algorithm
library (`ihrs::algo`). Linux only. Also a learning codebase, so clarity beats cleverness.

Read first: `README.md`, `docs/architecture.md`, `docs/design_review.md` (rationale behind the
roadmap) and the current stage plan (`docs/stage3_plan.md` — lock-free SHM ring buffer).

## Commands

```bash
git submodule update --init          # Eigen (libs/algo/third_party/Eigen)
cmake --preset debug && cmake --build --preset debug -j
ctest --preset debug --output-on-failure
cmake --preset sanitize && cmake --build --preset sanitize -j && ctest --preset sanitize --output-on-failure
cmake --preset minimal  && cmake --build --preset minimal -j   # no optional deps
clang-format -i <files>              # .clang-format: LLVM-based, C++20, 100 cols, 4-space indent
```

Presets: `debug`, `release`, `sanitize` (ASan/UBSan), `minimal`. Output: `build/<preset>/{bin,lib}`.
CI (`.github/workflows/ci.yml`) builds debug/release/sanitize on Ubuntu and runs ctest.
On Windows, build inside WSL (Ubuntu-22.04): POSIX SHM and pthreads are required.

## Layout

```
libs/core/        Node, Executor, Broker, topics            → ihrs::core
libs/ipc/         POSIX shared-memory pub/sub                → ihrs::ipc
libs/transport/   TCP/UDP, optional mqtt/ zmq webrtc/        → ihrs::transport
libs/bridge/      typed SHM ↔ core ↔ UDP bridge (header-only) → ihrs::bridge
libs/algo/        algorithms (+ Eigen, SDL2 viz), own tests/  → ihrs::algo
apps/             demo nodes, bridges, operator tools (one .cpp per executable)
tests/<lib>/      GTest unit tests, mirrored per library
cmake/            shared CMake helpers
docs/             architecture, design review, stage plans
archive/          preserved experiments — NOT built, don't modify or depend on it
```

Each library has `include/ihrs/<lib>/…` (public headers) and `src/`.

## Non-negotiables

- **C++20, no new mandatory dependencies.** Optional deps must be detected in CMake and
  compiled out cleanly (the `minimal` preset must keep building).
- **Shared memory must stay crash-safe:** robust mutexes with `EOWNERDEAD` recovery; never hold
  a lock across a callback; unlink/cleanup paths tested.
- **Threads stop promptly:** executors/receivers must be stoppable (no unbounded blocking calls);
  RAII for subscriptions, sockets and SHM segments.
- **Sanitize preset is green** — no data races, leaks or UB introduced.
- Public headers under `include/ihrs/<lib>/`; no `using namespace` in headers.

## Adding a feature

1. Branch `feat/<topic>`. For a new stage or a design change, write/extend a plan in `docs/` first.
2. Put the code in the owning library (public API in `include/ihrs/<lib>/`), add GTest cases in
   `tests/<lib>/` (or `libs/algo/tests/` for algorithms), register them with ctest.
3. Demo/tool executables go in `apps/` and are added to `apps/CMakeLists.txt`.
4. New optional dependency: `find_package` guarded by an option, feature compiled out when absent.
5. `clang-format`, then debug + sanitize builds and ctest green → PR with a conventional
   title (`feat:`, `fix:`, `docs:`, `chore:`) and a short "why" in the body. Tick the README roadmap when done.
