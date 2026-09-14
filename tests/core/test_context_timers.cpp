#include <ihrs/core/context.hpp>
#include <ihrs/core/executor.hpp>
#include <ihrs/core/message.hpp>
#include <ihrs/core/publisher.hpp>
#include <ihrs/core/subscriber.hpp>

#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>

using namespace ihrs::core;

namespace {
struct StringMsg : Message {
  explicit StringMsg(std::string v) : value(std::move(v)) {}
  std::string value;
};
} // namespace

TEST(ContextTest, LocalContextIsIsolatedFromGlobal) {
  Context local;
  int local_calls = 0;
  int global_calls = 0;

  Subscriber local_sub("iso/topic", [&](MessagePtr) { ++local_calls; }, local);
  Subscriber global_sub("iso/topic", [&](MessagePtr) { ++global_calls; }); // global context

  Publisher local_pub("iso/topic", local);
  local_pub.publish(std::make_shared<StringMsg>("m"));
  local.broker().spinOnce();
  EXPECT_EQ(local_calls, 1);
  EXPECT_EQ(global_calls, 0);

  Publisher global_pub("iso/topic");
  global_pub.publish(std::make_shared<StringMsg>("m"));
  Broker::instance().spinOnce();
  EXPECT_EQ(local_calls, 1);
  EXPECT_EQ(global_calls, 1);
}

TEST(ContextTest, ExecutorUsesContextBroker) {
  Context local;
  int received = 0;

  Subscriber sub("ctx/topic", [&](MessagePtr) { ++received; }, local);

  Publisher pub("ctx/topic", local);
  Executor exec(local);
  pub.publish(std::make_shared<StringMsg>("m"));

  std::thread spinner([&] { exec.spin(200.0); });
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  exec.stop();
  spinner.join();

  EXPECT_EQ(received, 1);
}

TEST(ExecutorTimerTest, FiresPeriodically) {
  Context local;
  Executor exec(local);
  std::atomic<int> ticks{0};

  const auto timer_id [[maybe_unused]] =
      exec.addTimer(std::chrono::duration<double>(0.02), [&] { ++ticks; }); // 50 Hz

  std::thread spinner([&] { exec.spin(200.0); });
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  exec.stop();
  spinner.join();

  EXPECT_GE(ticks.load(), 3);
  EXPECT_LE(ticks.load(), 15);
}

TEST(ExecutorTimerTest, CancelStopsFiring) {
  Context local;
  Executor exec(local);
  std::atomic<int> ticks{0};

  const auto id = exec.addTimer(std::chrono::duration<double>(0.02), [&] { ++ticks; });

  std::thread spinner([&] { exec.spin(200.0); });
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  exec.cancelTimer(id);
  const int at_cancel = ticks.load();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  exec.stop();
  spinner.join();

  EXPECT_GE(at_cancel, 1);
  EXPECT_LE(ticks.load(), at_cancel + 2); // at most one in-flight tick after cancel
}
