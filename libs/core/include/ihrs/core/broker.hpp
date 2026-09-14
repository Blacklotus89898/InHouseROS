#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "ihrs/core/message.hpp"

namespace ihrs::core {

/// Thread-safe in-process publish/subscribe broker.
///
/// Subscriptions are asynchronous: publish() enqueues a task per matching
/// subscriber, and spinOnce() executes all pending tasks on the calling
/// thread. This mirrors the single-threaded executor model used by ROS:
/// callbacks never run concurrently with each other inside one process.
///
/// Queue bounds: each broker bounds the number of pending tasks
/// (default kDefaultMaxPending). When the bound is reached, the oldest
/// pending task is dropped so publishers are never blocked by a slow
/// consumer. Inspect droppedCount() to monitor back-pressure.
class Broker {
public:
  using Callback = std::function<void(MessagePtr)>;
  using Token = std::uint64_t;

  /// Tokens returned by subscribe(); 0 is never a valid token.
  static constexpr Token kInvalidToken = 0;
  /// Default bound on pending (not yet dispatched) tasks.
  static constexpr std::size_t kDefaultMaxPending = 1024;

  /// Process-wide default broker (dependency injection point for tests).
  static Broker& instance();

  Broker(const Broker&) = delete;
  Broker& operator=(const Broker&) = delete;

  /// Registers a callback for a topic and returns its token.
  /// The same callback may be registered multiple times (distinct tokens).
  [[nodiscard]] Token subscribe(const std::string& topic, Callback cb);

  /// Removes a previously subscribed callback. Unknown tokens are ignored.
  void unsubscribe(const std::string& topic, Token token);

  /// Enqueues msg for delivery to every subscriber of the topic.
  /// Null messages are ignored. Non-copyable through the queue boundary.
  void publish(const std::string& topic, MessagePtr msg);

  /// Dispatches all pending tasks on the calling thread, oldest first.
  void spinOnce();

  /// Number of tasks enqueued but not yet dispatched.
  [[nodiscard]] std::size_t pendingCount() const;

  /// Total tasks dropped because the pending bound was exceeded.
  [[nodiscard]] std::size_t droppedCount() const;

  /// Changes the pending-task bound. Must not be called concurrently
  /// with publish()/spinOnce().
  void setMaxPending(std::size_t max_pending);

private:
  Broker() = default;

  struct Subscription {
    Token id;
    Callback cb;
  };

  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::vector<Subscription>> subscribers_;
  std::vector<std::function<void()>> queue_;
  Token next_token_ = 1;
  std::size_t max_pending_ = kDefaultMaxPending;
  std::size_t dropped_ = 0;
};

} // namespace ihrs::core
