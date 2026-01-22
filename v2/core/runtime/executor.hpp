#pragma once

#include <vector>
#include <thread>
#include <chrono>

#include "messaging/broker.hpp"

// Forward declaration (IMPORTANT)
class Node;

class Executor {
public:
  void addNode(Node* node);

  void spin(double hz);
  void stop();

private:
  bool running_ = true;
  std::vector<Node*> nodes_;
};

