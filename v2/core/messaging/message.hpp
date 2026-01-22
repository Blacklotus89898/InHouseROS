// messaging/message.hpp
#pragma once
#include <memory>

struct Message {
  virtual ~Message() = default;
};

using MessagePtr = std::shared_ptr<Message>;

