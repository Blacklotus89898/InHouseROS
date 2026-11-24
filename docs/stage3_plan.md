# Stage 3 Implementation Plan — SHM v2: Lock-Free Ring Buffer

*Per [`architecture_evolution.md`](architecture_evolution.md), Stages 1–2 are
delivered. This document is the working contract for Stage 3: replacing the
polling latest-value SHM protocol with a lock-free SPSC ring buffer, futex
wakeup, sequence/timestamp metadata and publisher liveness detection.*

---

## 1. Goal

Cross-process messaging with ROS 2 "sensor data" QoS semantics:

- **Queue depth** (ring capacity), not single-slot latest-value
- **Wait-free publisher**, near-zero-latency subscriber wakeup (futex)
- **Torn-read-proof delivery** via sequence numbers (seq-lock validation)
- **Drop accounting**: subscriber sees exactly what it missed (seq gaps)
- **Liveness**: subscriber detects a dead publisher on its own

```cpp
// target API (namespace ihrs::ipc)
ShmRingPublisher<Imu> pub("/imu_ring", /*capacity_pow2=*/256);

ShmRingSubscriber<Imu> sub("/imu_ring");
switch (sub.receive(sample, 10ms)) {
    case Recv::Got:          ...; break;   // payload + timestamp valid
    case Recv::Empty:        ...; break;   // timeout, nothing new
    case Recv::Overwritten:  ...; break;   // got newest; N older samples skipped
    case Recv::Disconnected: ...; break;   // publisher heartbeat stale
}
```

## 2. Key design decisions

| Decision | Choice | Rationale |
| :--- | :--- | :--- |
| Topology | **One segment per subscriber** (SPSC pairing) | Avoids MPMC ordering problems entirely; fan-out belongs to typed topics/bridges, which exist since Stage 2 |
| Overflow policy | Publisher overwrites oldest (KEEP_LAST) | Right default for robotics; reader detects skips via sequence gap |
| Torn-read protection | Per-slot seq + **seq-lock recheck** | A slow reader may see a slot overwritten mid-read; recheck catches it and skips to newest |
| Wakeup | `futex(FUTEX_WAIT/WAKE)` on the publish counter, **polling fallback** mode | Linux-first (project scope); fallback keeps portability and makes A/B latency measurement trivial |
| Liveness | Publisher refreshes `heartbeat_ms` every tick; subscriber flags `Disconnected` when stale past a timeout | No robust-mutex equivalent exists for lock-free protocols; heartbeat is the standard pattern |
| Creation handshake | Existing **robust mutex** retained, creation only | Reuse proven code; runtime path stays lock-free |
| Segment identity | `magic + version + sizeof(T) + capacity` checked at open; mismatch → error | Prevents silently misinterpreting foreign segments |
| ABI | Same-binary only (documented) | Matches Stage 2's type-hash guarantee; file format stability is a later concern |

### Segment layout

```
SegmentHeader (cache-line aligned):
  magic, version, element_size, capacity (power of 2)
  std::atomic<uint64_t> publish_seq      // total published (monotonic)
  std::atomic<uint64_t> read_seq         // reader position (single reader)
  std::atomic<uint64_t> dropped          // reader-side skip counter
  std::atomic<uint64_t> heartbeat_ms     // refreshed by publisher
  pid_t publisher_pid
  robust pthread_mutex_t init_mutex      // creation handshake only
  std::atomic<bool> initialized

slots[capacity] (each padded to cache line):
  uint64_t seq;          // == publish_seq value when slot was filled
  uint64_t timestamp_ns; // steady_clock at publish
  T payload
```

### Memory ordering (the whole safety argument)

- Writer: fill payload → store `slot.seq = s` (release) → `publish_seq = s + 1` (release) → `futex_wake`.
- Reader: read `publish_seq` (acquire) → derive newest slot → load `slot.seq` (acquire); if `slot.seq` matches expectation, payload is complete; else a wrap overtook us → skip to newest, `dropped++`.
- Reader publishes its position to `read_seq` (relaxed) — informational only (overflow policy is writer-side overwrite; SPSC means no ABA concerns).

## 3. Implementation steps (each lands green)

| # | Step | Files | Tests / acceptance |
| :- | :- | :- | :- |
| 1 | Host-side lock-free ring core | `libs/ipc/include/ihrs/ipc/detail/spsc_ring.hpp` (pure host code, no SHM) | Hammer test: 1 writer + 1 reader thread, 1M ops, zero loss when not overflowing, seq strictly monotonic; overflow test: reader skips exactly `capacity` behind; **sanitize preset mandatory green** |
| 2 | Futex wrappers + SHM placement | `detail/futex.hpp`, `shm_ring.hpp/.cpp` (header/magic/version checks, heartbeat) | Process-visibility test: two `ShmRingSubscriber` views of one publisher in-process across threads; open-mismatch rejection test |
| 3 | Public API + liveness | `shm_ring.hpp` (`ShmRingPublisher<T>`, `ShmRingSubscriber<T>`, `Recv`, timestamps) | `Disconnected` after publisher stops heartbeating (timeout config); timestamp monotonicity; wake-latency smoke (< 1 ms wake vs poll interval) |
| 4 | Bridge + app integration | `libs/bridge/.../shm_ring_bridge.hpp` (`ShmRingTopicSink/Source`, QoS `depth` → capacity, drop counters surfaced), `--shm-ring-out/in` in `bridge` app | Integration tests mirroring Stage 2's SHM bridge trio, plus: overflow under back-pressure delivers newest with skip accounting |
| 5 | Benchmark | `apps/shm_bench` — v1 latest-value vs v2 ring: publish→receive p50/p99 latency at 1/10 kHz + throughput MB/s; results table added to `docs/architecture.md` | v2 p99 ≤ v1 p99 at equal rates (the evolution doc's "benchmark before/while building" gate, applied against our own v1 baseline; iceoryx2 comparison optional stretch) |
| 6 | Docs & roadmap | architecture.md (SHM v2 section + diagram), evolution Stage 3 → delivered, README roadmap flip, plan status | All Mermaid render |

## 4. Compatibility

- `GenericSHM<T>` (v1) stays untouched as a shim; bridge v1 endpoints keep working.
- `ShmTopicSink/Source` gain ring-backed variants; the `bridge` app exposes both (`--shm-*` legacy, `--shm-ring-*` v2).
- String `SharedMemoryPublisher` remains demo-only (per design review tension #2); last legacy apps migrate opportunistically.

## 5. Risks

| Risk | Mitigation |
| :- | :- |
| Lock-free subtleties (the classic risk) | seq-lock validation, acquire/release only, 1M-op hammer tests, sanitize preset in every CI run |
| futex is Linux-only | Polling fallback behind the same interface; project is Linux-first (documented) |
| Writer stampede over slow reader | By design (overwrite-oldest); `dropped` counter makes loss visible; QoS depth = capacity is the tuning knob |
| Segment ABI drift across builds | magic+version+sizeof(T) gate at open |

## 6. Definition of done

- [ ] Steps 1–5 merged; 47 → ~57 tests; hammer + liveness + integration green on all presets (sanitize emphasized)
- [ ] Benchmark numbers recorded in docs; v2 p99 ≤ v1 p99
- [ ] `bridge` app speaks `--shm-ring-*`; QoS depth actually bounds queueing end-to-end
- [ ] Docs updated, diagrams render, roadmap flipped
- [ ] CI green on debug/release/sanitize

## 7. What comes after (preview)

Stage 4 (discovery + `ihrs topics` introspection, latency/loss counters in tooling) builds directly on Stage 3's counters and heartbeat metadata. Hygiene backlog stays: `ihrs::algo` strict-warning migration, gRPC/WebRTC transports when their deps are installed.
