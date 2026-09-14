#include "ihrs/core/executor.hpp"

namespace ihrs::core {

void Executor::addNode(Node* node) {
    if (node == nullptr) {
        return;
    }
    nodes_.push_back(node);
    node->onInit();
}

Executor::TimerId Executor::addTimer(std::chrono::duration<double> period, TimerCallback cb) {
    const auto ticks = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    const TimerId id = next_timer_id_++;
    timers_.push_back(Timer{id, ticks, std::chrono::steady_clock::now() + ticks, std::move(cb), false});
    return id;
}

void Executor::cancelTimer(TimerId id) {
    for (Timer& t : timers_) {
        if (t.id == id) {
            t.cancelled = true; // compacted at the end of the cycle that runs it
            break;
        }
    }
}

void Executor::runTimers(const std::chrono::steady_clock::time_point& now) {
    for (Timer& t : timers_) {
        if (!t.cancelled && now >= t.next) {
            t.cb();
            t.next = now + t.period; // at most one fire per cycle
        }
    }
    std::erase_if(timers_, [](const Timer& t) { return t.cancelled; });
}

void Executor::spin(double hz) {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(hz > 0.0 ? 1.0 / hz : 0.0));

    while (running_.load(std::memory_order_relaxed)) {
        const auto cycle_start = clock::now();

        runTimers(cycle_start);
        broker_.spinOnce();
        for (Node* node : nodes_) {
            node->onUpdate(std::chrono::duration<double>(period).count());
        }

        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_until(lock, cycle_start + period,
                       [this] { return !running_.load(std::memory_order_relaxed); });
    }
}

void Executor::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_.store(false, std::memory_order_relaxed);
    }
    cv_.notify_all();
}

} // namespace ihrs::core
