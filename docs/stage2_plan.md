# Stage 2 Implementation Plan — Unified Typed Topics

*The plan for the next step, per [`architecture_evolution.md`](architecture_evolution.md).
Stage 2 is the milestone where "publish anywhere, consume anywhere" becomes an
architectural property instead of app-level glue. This document is the working
contract for implementing it.*

---

## 1. Goal

One typed topic API in `ihrs::core`:

```cpp
using namespace ihrs::core;

Context ctx;                                  // or Context::global()
NodeHandle node = ctx.create_node("imu_node");

auto pub = node.create_publisher<ImuSample>("/imu", QoS{}.with_depth(1));
auto sub = node.create_subscriber<ImuSample>("/imu", [](const ImuSample& s) {
    // consume — called on the executor thread
}, QoS{}.with_depth(4));

pub->publish(sample);   // takes const T&
```

…where `/imu` can later be transparently backed by in-process callbacks
(Stage 2), shared memory (Stage 3), or the network — with type safety
enforced at the boundaries.

## 2. Non-goals (hard stops)

- No services, parameters, or transform tree (ROS parity creep).
- No dynamic discovery daemon (Stage 4).
- No wire-format compatibility promises between releases yet.
- No `Pub`/`Sub` replacement of the existing `Publisher`/`Subscriber` —
  the typed API is additive; both coexist (they share the same Broker).

## 3. Key design decisions

### 3.1 Type identity — `type_hash<T>()`

Compile-time hash of the mangled type name (from `std::source_location`/
`__PRETTY_FUNCTION__` trick), truncated to `uint64_t`. Registered per topic;
a `Sub<T>` on a topic published as `U` throws `TypeMismatchError` at
subscription time. Hash collisions across two *different* payload types on
the same topic are accepted as an astronomically unlikely residual risk.

### 3.2 Serialization rule — one compile-time branch

```
std::is_trivially_copyable_v<T>  ->  memcpy wire codec (zero-copy capable in SHM v2)
otherwise                        ->  Codec<T> specialization required
                                     (to_bytes/from_bytes), length-prefixed
                                     on streams via transport/framing.hpp
```

`Pub<T>`/`Sub<T>` `static_assert`s the branch at instantiation — a
non-trivially-copyable payload without a `Codec` specialization fails to
compile, never silently corrupts. No external serialization dependency
(protobuf/CDR stay out, per project ethos).

### 3.3 LocalTransport *is* the existing Broker

The typed layer is a thin wrapper over what exists:

```cpp
template <typename T>
struct TypedMessage : Message {
    std::uint64_t type_hash;
    T value;
};
```

- `publish(const T& v)` wraps `v` in `TypedMessage<T>` and enqueues through
  the existing `Broker` path → delivery semantics, RAII subscriptions,
  bounded queue, executor dispatch: all inherited, nothing new to race-test.
- `Sub<T>`'s callback unwraps and forwards `const T&` to user code.
- QoS `depth` is **accepted now** and enforced at the boundaries that have
  real queues (bridge/SHM/net); in-process depth remains the broker's
  existing global bound (documented). Per-subscription local queues are a
  stretch goal, not a Stage 2 blocker.

### 3.4 `NodeHandle` — naming

`Node` is taken (executor-driven base class). The typed API introduces
`NodeHandle`, an rclcpp-style handle bound to a `Context` that manufactures
publishers/subscribers. `Context::create_node(name)` returns it.

## 4. Implementation steps (each lands green)

> **Status: STAGE 2 COMPLETE.** Steps 1–4 delivered: typed core layer
> (`type_support.hpp`, `typed.hpp`, broker type registry, `Context::create_node`),
> `ihrs::bridge` (SHM/UDP topic bridges), config-driven `bridge` app, and the
> legacy `sender`/`receiver` demos rewritten on the typed API. 47/47 tests green
> across debug/release/sanitize (ASan/UBSan clean).

| # | Step | Files | Tests / acceptance |
| :- | :- | :- | :- |
| 1 | Type identity + codec rule | `libs/core/include/ihrs/core/type_support.hpp` | `type_hash<T>()` distinct for different types, equal for same; `static_assert` fires for `std::string` payload without `Codec` |
| 2 | Typed messaging layer | `libs/core/include/ihrs/core/typed.hpp` (`QoS`, `NodeHandle`, `Pub<T>`, `Sub<T>`, `TypeMismatchError`) | Typed roundtrip through `Executor` at 200 Hz; payload values exact; multiple subs on one topic |
| 3 | Topic type registry in Broker | `broker.hpp/.cpp` (topic → hash map, register/verify on endpoint creation) | `TypeMismatchError` on `Sub<U>` vs `Pub<T>`; same-type re-subscribe OK; registry cleared when last endpoint leaves (weak semantics: keep simple map + refcount) |
| 4 | Bridge app (SHM ↔ typed core ↔ UDP) | `apps/bridge.cpp`, config via CLI `--map topic=shm:name` / `--map topic=udp:ip:port` (trivially-copyable types only in v1) | Integration test: `Pub<T>` → bridge → `GenericSHM<T>` reader in-process; retires the hand-wired `apps/sender.cpp` pattern |
| 5 | Docs & roadmap | `architecture.md` (typed API section + class diagram), `architecture_evolution.md` (mark Stage 2), README | All Mermaid render; roadmap checkboxes updated |

Steps 1–3 are `ihrs::core`-internal and fully unit-testable; step 4 is the
first user-visible payoff and unblocks deprecating the string-SHM demo path.

## 5. API sketch (Step 2, for review before implementation)

```cpp
namespace ihrs::core {

struct QoS {
    std::size_t depth = 1;                       // KEEP_LAST bound at queueing points
    QoS& with_depth(std::size_t d) { depth = d; return *this; }
};

class TypeMismatchError : public std::runtime_error { using std::runtime_error::runtime_error; };

template <typename T> class Pub;   // publish(const T&) — non-blocking enqueue
template <typename T> class Sub;   // RAII, movable; callback: void(const T&)

class NodeHandle {
public:
    [[nodiscard]] std::string_view name() const noexcept;
    template <typename T> [[nodiscard]] std::shared_ptr<Pub<T>>
        create_publisher(std::string_view topic, const QoS& qos = {});
    template <typename T> [[nodiscard]] std::shared_ptr<Sub<T>>
        create_subscriber(std::string_view topic, std::function<void(const T&)> cb,
                          const QoS& qos = {});
private:
    friend class Context;
    NodeHandle(Context& ctx, std::string name);
    Context& context_;
    std::string name_;
};

} // namespace ihrs::core
```

Returned as `shared_ptr` because `Sub<T>` must be copy-erasable into user
containers while keeping the token; `Pub<T>` is cheap.

## 6. Risks

| Risk | Mitigation |
| :- | :- |
| Broker registry growth (topics never freed) | Refcount topic entries; drop when last endpoint destroyed; cap + warn |
| Bridge latency budget (copy through broker + SHM) | Measure in test with `steady_clock`; Stage 3 ring buffer removes the copies |
| Type hash instability across compilers/ABIs | Documented: same-binary guarantee only (matches project scope; revisit for IPC file format in Stage 3) |
| Scope creep into QoS matrices | Only `depth` exists; everything else is a documented non-goal |

## 7. Definition of done

- [x] Steps 1–4 merged with unit + integration tests green (34 → 47 tests)
- [x] 0 compiler warnings, ASan/UBSan clean on new paths
- [x] `apps/sender.cpp` / `receiver.cpp` rewritten on the typed API (hand-wired glue gone)
- [x] Docs updated, all Mermaid render, roadmap checkboxes flipped
- [ ] CI green on all three presets
