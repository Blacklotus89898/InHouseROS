#include <iostream>
#include "messaging/subscriber.hpp"
#include "runtime/executor.hpp"

struct Pose2DMsg : Message {
  double x, y, theta;
};

int main() {
  Subscriber sub2("odom", [](MessagePtr msg) {
  auto p = std::static_pointer_cast<Pose2DMsg>(msg);
  std::cout << "[SECOND] x=" << p->x << std::endl;
});

  Executor exec;
  exec.spin(100.0);
}

