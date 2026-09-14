#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ihrs/core/message.hpp"

namespace ihrs::core {

/// Thrown when an endpoint binds a payload type to a topic that is already
/// bound to a different payload type (typed API, see type_support.hpp).
class TypeMismatchError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

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

  /// --- Typed-topic registry (see type_support.hpp) --------------------

  /// Binds a payload type hash to a topic (ref-counted). Throws
  /// TypeMismatchError if the topic is bound to a different hash.
  void register_topic_type(const std::string& topic, std::uint64_t hash);

  /// Releases one reference taken by register_topic_type(); the binding
  /// is removed at zero references. Unknown pairs are ignored.
  void release_topic_type(const std::string& topic, std::uint64_t hash) noexcept;

  /// The bound hash for a topic, or nullopt if unbound.
  [[nodiscard]] std::optional<std::uint64_t> topic_type(const std::string& topic) const;

private:
  Broker() = default;
  friend class Context; // only Context may construct a broker

  struct Subscription {
    Token id;
    Callback cb;
  };

  struct TypeBinding {
    std::uint64_t hash;
    std::size_t refs;
  };

  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::vector<Subscription>> subscribers_;
  std::vector<std::function<void()>> queue_;
  std::unordered_map<std::string, TypeBinding> type_bindings_;
  Token next_token_ = 1;
  std::size_t max_pending_ = kDefaultMaxPending;
  std::size_t dropped_ = 0;
};

} // namespace ihrs::core
