#include "messaging/broker.hpp"

Broker& Broker::instance() {
  static Broker b;
  return b;
}

void Broker::subscribe(const std::string& topic, Callback cb) {
  std::lock_guard<std::mutex> lock(mutex_);
  subscribers_[topic].push_back(cb);
}

void Broker::publish(const std::string& topic, MessagePtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = subscribers_.find(topic);
  if (it == subscribers_.end()) return;

  for (auto& cb : it->second) {
    queue_.push_back([=]() { cb(msg); });
  }
}

void Broker::spinOnce() {
  std::vector<std::function<void()>> local;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local.swap(queue_);
  }
  for (auto& task : local) task();
}

