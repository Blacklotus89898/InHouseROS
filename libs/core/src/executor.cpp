#include "ihrs/core/executor.hpp"

namespace ihrs::core {

void Executor::addNode(Node* node) {
  if (node == nullptr) {
    return;
  }
  nodes_.push_back(node);
  node->onInit();
}

void Executor::spin(double hz) {
  using clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(hz > 0.0 ? 1.0 / hz : 0.0));

  while (running_.load(std::memory_order_relaxed)) {
    const auto cycle_start = clock::now();

    Broker::instance().spinOnce();
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
