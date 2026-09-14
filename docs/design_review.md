# Architecture & Code Design Review

Status: post-reorganization review (2026-09). Scope: all first-party libraries,
apps, build system and tests. Each finding lists its resolution; unresolved ones
feed the roadmap in the main README.

Severity scale: **high** (correctness/data race), **medium** (robustness/API
safety), **low** (style/maintainability).

---

## 1. Findings and resolutions

### core (in-process middleware)

| # | Severity | Finding | Resolution |
| :- | :- | :--- | :--- |
| C1 | **high** | `Subscriber` only registered in its constructor; destroying it left a dangling callback in the broker (use-after-free risk) | Redesigned: `Broker::subscribe()` returns a token; `Subscriber` is RAII and unsubscribes on destruction/move. Covered by `SubscriberDestructorUnsubscribes` test |
| C2 | **high** | `Executor::running_` was a plain `bool` written from another thread — data race (UB) | `std::atomic<bool>`; documented memory-ordering choice |
| C3 | medium | `Executor::stop()` did not interrupt the sleep — up to one full cycle latency | Condition-variable `wait_until`; stop is now prompt even at 1 Hz (tested: `StopIsPromptEvenAtLowRates`) |
| C4 | medium | Broker queue unbounded — a spinning-less process accumulates memory without bound | Bounded queue (default 1024) with drop-oldest policy and observable `droppedCount()` |
| C5 | low | `publish()` accepted null `MessagePtr` | Ignored explicitly, tested |
| C6 | low | Missing `[[nodiscard]]`, `explicit`, deleted copies in public APIs | Applied across the public surface; `Node` now has proper copy/move defaults |

### ipc (shared memory)

| # | Severity | Finding | Resolution |
| :- | :- | :--- | :--- |
| I1 | **high** | Signal handlers called `cleanup()` (munmap/close/shm_unlink) and `exit(0)` — none of these are async-signal-safe | Handlers now only store to a `std::atomic<bool>` flag; `run()` observes it; destructor performs all cleanup |
| I2 | **high** | Constructors called `exit(1)` on `shm_open`/`mmap` failure — a library must not terminate the host process | Both classes throw `std::runtime_error` with `strerror` context |
| I3 | medium | `run()` loops `while (true)` — no way to stop a publisher/subscriber gracefully | Added `stop()` + atomic `running_` flag in both classes |
| I4 | medium | Unchecked `ftruncate`, silent `shm_open` fallback ordering, ignored `shm_unlink` result | Checked with error propagation; `shm_unlink` failure is acceptable (segment owned by OS) |
| I5 | medium | The shared mutex is not `PTHREAD_MUTEX_ROBUST` — a SIGKILLed publisher deadlocks subscribers | **Open**: documented in README/architecture; needs a robust-mutex recovery path (EOWNERDEAD handling). Roadmap item |
| I6 | low | `shm_subscriber.hpp` used include guards *inside* the namespace block (fragile layout) | Unified on `#pragma once`, guards removed |
| I7 | low | Latest-value protocol only; readers poll at `rate` | Documented; lock-free ring buffer + condition variables remain roadmap items |

### transport (sockets)

| # | Severity | Finding | Resolution |
| :- | :- | :--- | :--- |
| T1 | medium | Receivers had no `stop()` — blocking `accept`/`recv`/`recvfrom` loops could only be killed by process exit; also untestable | Added `stop()` (atomic flag + `shutdown()` to unblock syscalls); receivers now delete-copyable, RAII-close |
| T2 | medium | TCP Receiver error paths used `exit(1)` in a constructor | Now throw `std::runtime_error` |
| T3 | low | `PacketSender` interface undocumented; `send()` partial-write semantics unclear | Contract documented (full-payload or false) |
| T4 | low | Inconsistent naming (`sock` vs `sock_`, `port` shadowing members) | Unified `snake_case_` members; shadow warning eliminated |

### apps & build

| # | Severity | Finding | Resolution |
| :- | :- | :--- | :--- |
| A1 | low | Demo binaries named generically (`sender`, `receiver`) and colliding conceptually with library concepts | Renamed (`udp_sender`, `tcp_receiver`, `shm_publisher`, ...) |
| A2 | low | Strict warnings profile (-Wconversion, -Wold-style-cast) floods on the large imported algo codebase | Two-profile policy: strict for new middleware, `ihrs_warnings_legacy` for `ihrs::algo`; documented in architecture doc |
| A3 | medium | Test presets had no per-test timeout — a hung test stalls `ctest` forever | **Fixed**: all `gtest_discover_tests` now use `TIMEOUT 60`; CI added (`.github/workflows/ci.yml`, debug/release/sanitize) |

---

## 2. Architecture assessment

### What is sound

- **Layering** (`core` / `ipc` / `transport` / `algo`, no upward dependencies)
  keeps each library independently testable and optional.
- **The ROS-1-like execution model** (publish enqueues, one spinner dispatches)
  is simple to reason about and race-free by construction for callbacks.
- **POSIX SHM with a process-shared mutex** is the right primitive for
  same-machine high-rate data; the ownership rule (creator links, others
  attach) gives sane cleanup semantics.
- **Feature-gated optional transports** keep the default build
  dependency-light and CI-friendly (the `minimal` preset proves it builds
  with nothing beyond pthreads).

### Design tensions worth knowing

1. **Singleton broker vs. testability.** `Broker::instance()` is pragmatic but
   global state. The tests tolerate it today because topics are namespaced;
   if it starts to hurt, introduce `ihrs::core::Context` (a broker owner
   passed to Publisher/Subscriber/Executor, defaulting to the global one) —
   the token-based API already supports this without breaking users.
2. **String payloads in `SharedMemoryPublisher`.** The 1 KB fixed-layout
   string segment is a demo-grade protocol. Typed `GenericSHM<T>` is the
   production path; consider deprecating the string API once a bridge node
   exists.
3. **Middleware ↔ transport seam.** Apps currently hand-wire bridges
   (SHM → UDP). The natural next step is a generic `bridge_node` in `apps`
   built on `core::Node` + both libraries; no library change is required
   because `transport` deliberately does not depend on `core`.
4. **Message copy cost.** `publish()` copies the `shared_ptr` per subscriber
   under the broker lock. For high-rate large payloads, prefer
   shared-payload messages (an inner const struct) so the copy stays cheap —
   or move large blobs through SHM anyway.

### Roadmap (priority order)

1. Robust SHM mutexes (`PTHREAD_MUTEX_ROBUST` + `EOWNERDEAD` recovery) — I5.
2. Lock-free SPSC ring buffer in SHM (drop polling; condition-variable
   wakeup via `pthread_condattr_setpshared`) — I7.
3. Per-test timeouts + a sanitize-preset CI job — A3.
4. `bridge_node` app (SHM ↔ core ↔ transport) to complete the middleware
   story — tension 3.
5. `Context`-based broker injection once multi-broker use appears — tension 1.
6. Gradually move `ihrs::algo` to the strict warning profile.

## 3. Verification

All resolutions were validated after this pass:

- `cmake --preset debug && cmake --build --preset debug`: **0 warnings**
- `ctest --preset debug`: **22/22 tests pass** (broker semantics, executor
  timing, TCP/UDP loopback, live SHM roundtrip, algo unit tests)
- release, sanitize and minimal presets configure and build clean
- `shm_publisher`, `hello_robotics` binaries smoke-tested
