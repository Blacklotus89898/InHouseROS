#include "runtime/executor.hpp"
#include "runtime/node.hpp"

void Executor::addNode(Node* node) {
  nodes_.push_back(node);
  node->onInit();
}

void Executor::spin(double hz) {
  using clock = std::chrono::steady_clock;
  auto period = std::chrono::duration<double>(1.0 / hz);

  while (running_) {
    auto start = clock::now();

    Broker::instance().spinOnce();

    for (auto* node : nodes_) {
      node->onUpdate(period.count());
    }

    std::this_thread::sleep_until(start + period);
  }
}

void Executor::stop() {
  running_ = false;
}

