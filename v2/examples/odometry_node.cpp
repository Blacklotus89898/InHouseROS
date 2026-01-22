#include <iostream>
#include "messaging/publisher.hpp"
#include "runtime/node.hpp"
#include "runtime/executor.hpp"
#include <chrono>

struct Pose2DMsg : Message {
  double x{0}, y{0}, theta{0};
};

class OdometryNode : public Node {
public:
  OdometryNode() : pub_("odom") {}


void onUpdate(double dt) override {
  static auto last = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();

  std::cout << "dt = "
            << std::chrono::duration<double>(now - last).count()
            << std::endl;

  last = now;
}

private:
  Publisher pub_;
  double x_{0}, y_{0}, theta_{0};
};

int main() {
  Executor exec;
  OdometryNode odom;

  exec.addNode(&odom);
  exec.spin(10.0);
}

