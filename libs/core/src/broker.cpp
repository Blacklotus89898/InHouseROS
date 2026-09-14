#include "ihrs/core/broker.hpp"

#include "ihrs/core/context.hpp"

namespace ihrs::core {

Broker& Broker::instance() {
    return Context::global().broker();
}

Broker::Token Broker::subscribe(const std::string& topic, Callback cb) {
  std::lock_guard<std::mutex> lock(mutex_);
  const Token token = next_token_++;
  subscribers_[topic].push_back(Subscription{token, std::move(cb)});
  return token;
}

void Broker::unsubscribe(const std::string& topic, Token token) {
  if (token == kInvalidToken) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto topic_it = subscribers_.find(topic);
  if (topic_it == subscribers_.end()) {
    return;
  }
  auto& subs = topic_it->second;
  std::erase_if(subs, [token](const Subscription& s) { return s.id == token; });
  if (subs.empty()) {
    subscribers_.erase(topic_it);
  }
}

void Broker::publish(const std::string& topic, MessagePtr msg) {
  if (msg == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = subscribers_.find(topic);
  if (it == subscribers_.end()) {
    return;
  }
  for (const auto& sub : it->second) {
    Callback cb = sub.cb; // copy under lock: target may unsubscribe() in-place
    queue_.push_back([cb = std::move(cb), msg] { cb(msg); });
    if (queue_.size() > max_pending_) {
      queue_.erase(queue_.begin()); // drop oldest: never block the publisher
      ++dropped_;
    }
  }
}

void Broker::spinOnce() {
  std::vector<std::function<void()>> local;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local.swap(queue_);
  }
  for (auto& task : local) {
    task();
  }
}

std::size_t Broker::pendingCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}

std::size_t Broker::droppedCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return dropped_;
}

void Broker::setMaxPending(std::size_t max_pending) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_pending_ = max_pending;
}

void Broker::register_topic_type(const std::string& topic, std::uint64_t hash) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto [it, inserted] = type_bindings_.try_emplace(topic, TypeBinding{hash, 0});
  if (!inserted && it->second.hash != hash) {
    throw TypeMismatchError("topic '" + topic +
                            "' is already bound to a different payload type");
  }
  ++it->second.refs;
}

void Broker::release_topic_type(const std::string& topic, std::uint64_t hash) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = type_bindings_.find(topic);
  if (it == type_bindings_.end() || it->second.hash != hash) {
    return;
  }
  if (--it->second.refs == 0) {
    type_bindings_.erase(it);
  }
}

std::optional<std::uint64_t> Broker::topic_type(const std::string& topic) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = type_bindings_.find(topic);
  if (it == type_bindings_.end()) {
    return std::nullopt;
  }
  return it->second.hash;
}

} // namespace ihrs::core
