#include <ihrs/core/executor.hpp>
#include <ihrs/core/node.hpp>

#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>

using namespace ihrs::core;

namespace {

class CounterNode : public Node {
public:
  void onUpdate(double /*dt*/) override { ++count; }
  std::atomic<int> count{0};
};

} // namespace

TEST(ExecutorTest, CallsNodeUpdateAtRequestedRate) {
  CounterNode node;
  Executor exec;
  exec.addNode(&node);

  std::thread spinner([&] { exec.spin(200.0); });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  exec.stop();
  spinner.join();

  EXPECT_GE(node.count.load(), 5);
  EXPECT_LE(node.count.load(), 100);
}

TEST(ExecutorTest, StopIsPromptEvenAtLowRates) {
  CounterNode node;
  Executor exec;
  exec.addNode(&node);

  std::thread spinner([&] { exec.spin(1.0); }); // 1 s cycle
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  const auto t0 = std::chrono::steady_clock::now();
  exec.stop();
  spinner.join();
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  // Without a condition-variable wait, stop() would block for a full
  // 1 s cycle. Prompt interruption must return well under that.
  EXPECT_LT(std::chrono::duration<double>(elapsed).count(), 0.5);
}

TEST(ExecutorTest, IgnoreNullNodes) {
  Executor exec;
  exec.addNode(nullptr); // must not crash

  std::thread spinner([&] { exec.spin(100.0); });
  exec.stop();
  spinner.join();
}
