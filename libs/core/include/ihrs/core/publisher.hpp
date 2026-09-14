#pragma once

#include <string>
#include <utility>

#include "ihrs/core/broker.hpp"

namespace ihrs::core {

/// Publishes messages to a single topic on the process-wide broker.
///
/// Lightweight and copyable; all copies publish to the same topic.
/// Messages are delivered asynchronously on the thread that calls
/// Broker::spinOnce() (typically via an Executor).
class Publisher {
public:
  explicit Publisher(std::string topic) : topic_(std::move(topic)) {}

  /// Enqueues msg for all subscribers of this topic. Null is a no-op.
  void publish(MessagePtr msg) { Broker::instance().publish(topic_, std::move(msg)); }

  /// The topic this publisher sends on.
  [[nodiscard]] const std::string& topic() const noexcept { return topic_; }

private:
  std::string topic_;
};

} // namespace ihrs::core
