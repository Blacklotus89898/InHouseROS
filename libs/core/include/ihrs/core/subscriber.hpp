#pragma once

#include <string>

#include "ihrs/core/broker.hpp"

namespace ihrs::core {

/// RAII handle for a topic subscription.
///
/// The callback is registered on construction and removed on destruction,
/// so a Subscriber must outlive every use of its callback (this prevents
/// the classic dangling-callback bug when subscribers are destroyed while
/// messages are still in flight).
///
/// Movable; not copyable.
class Subscriber {
public:
  Subscriber(const std::string& topic, Broker::Callback cb)
      : topic_(topic), token_(Broker::instance().subscribe(topic, std::move(cb))) {}

  ~Subscriber() {
    if (token_ != Broker::kInvalidToken) {
      Broker::instance().unsubscribe(topic_, token_);
    }
  }

  Subscriber(const Subscriber&) = delete;
  Subscriber& operator=(const Subscriber&) = delete;

  Subscriber(Subscriber&& other) noexcept
      : topic_(std::move(other.topic_)), token_(other.token_) {
    other.token_ = Broker::kInvalidToken;
  }

  Subscriber& operator=(Subscriber&& other) noexcept {
    if (this != &other) {
      if (token_ != Broker::kInvalidToken) {
        Broker::instance().unsubscribe(topic_, token_);
      }
      topic_ = std::move(other.topic_);
      token_ = other.token_;
      other.token_ = Broker::kInvalidToken;
    }
    return *this;
  }

private:
  std::string topic_;
  Broker::Token token_;
};

} // namespace ihrs::core
