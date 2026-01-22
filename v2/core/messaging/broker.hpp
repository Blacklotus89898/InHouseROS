#pragma once
#include <unordered_map>
#include <vector>
#include <functional>
#include <mutex>
#include <string>

#include "message.hpp"

class Broker {
public:
  using Callback = std::function<void(MessagePtr)>;

  static Broker& instance();

  void subscribe(const std::string& topic, Callback cb);
  void publish(const std::string& topic, MessagePtr msg);
  void spinOnce();

private:
  std::mutex mutex_;
  std::unordered_map<std::string, std::vector<Callback>> subscribers_;
  std::vector<std::function<void()>> queue_;
};

