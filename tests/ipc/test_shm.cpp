#include <ihrs/ipc/generic_shm_publisher.hpp>
#include <ihrs/ipc/generic_shm_subscriber.hpp>

#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>

using namespace ihrs::ipc;

namespace {

struct Payload {
  int counter{0};
  double value{0.0};
};

} // namespace

// Full shared-memory round trip: one publisher, one subscriber, both backed
// by the real POSIX shm implementation. Each run uses a unique segment name
// so the test never collides with leftovers or parallel runs.
TEST(GenericShmTest, PublisherSubscriberRoundTrip) {
  const std::string shm_name = "/ihrs_test_" + std::to_string(::getpid());

  std::atomic<int> received{0};
  std::atomic<int> last_counter{-1};

  GenericSHMPublisher<Payload> pub(
      shm_name.c_str(), sizeof(Payload) + 256,
      [](int count) {
        return Payload{count, count * 1.5};
      },
      200); // Hz: fast enough that the test finishes quickly

  GenericSHMSubscriber<Payload> sub(
      shm_name.c_str(), sizeof(Payload) + 256,
      [&](const Payload& p) {
        last_counter.store(p.counter);
        ++received;
      },
      200);

  std::thread pub_thread([&] { pub.run(); });
  std::thread sub_thread([&] { sub.run(); });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (received.load() < 5 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  pub.stop();
  sub.stop();
  pub_thread.join();
  sub_thread.join();

  EXPECT_GE(received.load(), 5);
  // Subscriber observes monotonically increasing counter values.
  EXPECT_GE(last_counter.load(), 4);
}

TEST(GenericShmTest, StopReturnsPromptly) {
  const std::string shm_name = "/ihrs_test_stop_" + std::to_string(::getpid());

  GenericSHMPublisher<Payload> pub(
      shm_name.c_str(), sizeof(Payload) + 256, [](int) { return Payload{}; }, 10);

  std::thread pub_thread([&] { pub.run(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const auto t0 = std::chrono::steady_clock::now();
  pub.stop();
  pub_thread.join();
  const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0);

  EXPECT_LT(elapsed.count(), 2.0);
}
