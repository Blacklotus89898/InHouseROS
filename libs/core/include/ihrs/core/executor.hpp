#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "ihrs/core/broker.hpp"
#include "ihrs/core/node.hpp"

namespace ihrs::core {

/// Drives a fixed-rate control loop over a set of nodes.
///
/// Each cycle: dispatch pending broker messages, then call
/// Node::onUpdate(dt) on every node. The loop sleeps on a condition
/// variable, so stop() interrupts it promptly (no drift, no busy wait).
///
/// The Executor does not own its nodes; they must outlive spin().
/// spin() runs on the calling thread and is not reentrant.
class Executor {
public:
  Executor() = default;

  Executor(const Executor&) = delete;
  Executor& operator=(const Executor&) = delete;

  ~Executor() { stop(); }

  /// Registers a node and calls its onInit(). Null pointers are ignored.
  void addNode(Node* node);

  /// Runs the loop at `hz` until stop() is called. `hz` must be > 0.
  /// All pending broker messages are dispatched once per cycle.
  /// Blocks the calling thread: call stop() from another thread or from
  /// inside a node callback (never from the thread running spin()).
  void spin(double hz);

  /// Requests the loop to exit; safe to call from any thread and from
  /// inside node callbacks. Returns once the loop observes the request
  /// (at most one cycle later).
  void stop();

private:
  std::atomic<bool> running_{true};
  std::vector<Node*> nodes_;
  std::mutex mutex_;
  std::condition_variable cv_;
};

} // namespace ihrs::core
