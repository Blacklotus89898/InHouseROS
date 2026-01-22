#pragma once
#include <string>
#include "broker.hpp"

class Publisher {
public:
  explicit Publisher(const std::string& topic)
    : topic_(topic) {}

  void publish(MessagePtr msg) {
    Broker::instance().publish(topic_, msg);
  }

private:
  std::string topic_;
};

