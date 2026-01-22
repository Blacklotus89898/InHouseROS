#pragma once
#include <string>
#include "broker.hpp"

class Subscriber {
public:
  Subscriber(const std::string& topic, Broker::Callback cb) {
    Broker::instance().subscribe(topic, cb);
  }
};

