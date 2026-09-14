#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "ihrs/core/broker.hpp"
#include "ihrs/core/context.hpp"
#include "ihrs/core/node.hpp"

namespace ihrs::core {

/// Drives a fixed-rate control loop over a set of nodes and timers.
///
/// Each cycle, in order: run due timers, dispatch pending broker messages,
/// then call Node::onUpdate(dt) on every node. The loop sleeps on a
/// condition variable, so stop() interrupts it promptly.
///
/// The Executor does not own its nodes; they must outlive spin().
/// spin() runs on the calling thread and is not reentrant.
///
/// Threading model (single-threaded, ROS 1-like): timers and node
/// callbacks all run on the spin() thread and never race each other.
/// addTimer()/cancelTimer() must therefore be called before spin() starts
/// or from inside a timer/node callback — never from another thread.
class Executor {
public:
    using TimerCallback = std::function<void()>;
    using TimerId = std::uint64_t;

    Executor() : Executor(Context::global()) {}

    explicit Executor(Context& context) : broker_(context.broker()) {}

    Executor(const Executor&) = delete;
    Executor& operator=(const Executor&) = delete;

    ~Executor() { stop(); }

    /// Registers a node and calls its onInit(). Null pointers are ignored.
    void addNode(Node* node);

    /// Runs the loop at `hz` until stop() is called. `hz` must be > 0.
    void spin(double hz);

    /// Requests the loop to exit; safe to call from any thread and from
    /// inside node/timer callbacks. Returns once the loop observes the
    /// request (at most one cycle later).
    void stop();

    /// Registers a periodic callback. A timer fires at most once per
    /// executor cycle, so its resolution is bounded by the spin rate.
    /// Returns an id usable with cancelTimer().
    [[nodiscard]] TimerId addTimer(std::chrono::duration<double> period, TimerCallback cb);

    /// Removes a timer. Safe to call from inside a timer callback.
    /// Unknown ids are ignored.
    void cancelTimer(TimerId id);

private:
    struct Timer {
        TimerId id;
        std::chrono::steady_clock::duration period;
        std::chrono::steady_clock::time_point next;
        TimerCallback cb;
        bool cancelled = false;
    };

    void runTimers(const std::chrono::steady_clock::time_point& now);

    Broker& broker_;
    std::atomic<bool> running_{true};
    std::vector<Node*> nodes_;
    std::vector<Timer> timers_;
    TimerId next_timer_id_ = 1;
    std::mutex mutex_;
    std::condition_variable cv_;
};

} // namespace ihrs::core
