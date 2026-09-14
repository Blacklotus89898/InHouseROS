#pragma once

#include <string>
#include <utility>

#include "ihrs/core/broker.hpp"
#include "ihrs/core/context.hpp"

namespace ihrs::core {

/// Publishes messages to a single topic on a context's broker.
///
/// Lightweight and copyable; all copies publish to the same topic.
/// Messages are delivered asynchronously on the thread that calls
/// Broker::spinOnce() (typically via an Executor).
class Publisher {
public:
    explicit Publisher(std::string topic, Context& context = Context::global())
        : topic_(std::move(topic)), broker_(context.broker()) {}

    /// Enqueues msg for all subscribers of this topic. Null is a no-op.
    void publish(MessagePtr msg) { broker_.publish(topic_, std::move(msg)); }

    /// The topic this publisher sends on.
    [[nodiscard]] const std::string& topic() const noexcept { return topic_; }

private:
    std::string topic_;
    Broker& broker_;
};

} // namespace ihrs::core
