#include "ihrs/core/broker.hpp"

namespace ihrs::core {

Broker& Broker::instance() {
  static Broker b;
  return b;
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

} // namespace ihrs::core
