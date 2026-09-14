#include <ihrs/ipc/generic_shm_publisher.hpp>
#include <ihrs/ipc/generic_shm_subscriber.hpp>

#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <cerrno>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

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

// Validates the platform behavior our robust-mutex recovery relies on:
// a lock owner that dies while holding a PTHREAD_MUTEX_ROBUST yields
// EOWNERDEAD to the next locker, which must mark the mutex consistent
// before unlocking. (The IPC classes use exactly this pattern via
// detail::robust_mutex.hpp; exercising it end-to-end would require
// SIGKILLing a real process, which is not ctest-friendly.)
TEST(RobustMutexTest, OwnerDeathIsRecoverable) {
  const std::string shm_name = "/ihrs_test_robust_" + std::to_string(::getpid());
  const int fd = ::shm_open(shm_name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
  ASSERT_NE(fd, -1);
  ASSERT_EQ(::ftruncate(fd, 4096), 0);

  auto* region = static_cast<std::byte*>(::mmap(nullptr, 4096, PROT_READ | PROT_WRITE,
                                                MAP_SHARED, fd, 0));
  ASSERT_NE(region, MAP_FAILED);
  auto* mutex = new (region) pthread_mutex_t;

  pthread_mutexattr_t attr;
  pthread_mutexattr_init(&attr);
  pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
  pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
  ASSERT_EQ(pthread_mutex_init(mutex, &attr), 0);
  pthread_mutexattr_destroy(&attr);

  // Lock and die (thread exit while holding the lock).
  std::thread doomed([&] { ASSERT_EQ(pthread_mutex_lock(mutex), 0); });
  doomed.join();

  // The next lock must report owner death — not hang.
  const int rc = pthread_mutex_lock(mutex);
  EXPECT_EQ(rc, EOWNERDEAD);
  ASSERT_EQ(pthread_mutex_consistent(mutex), 0);
  EXPECT_EQ(pthread_mutex_unlock(mutex), 0);
  EXPECT_EQ(pthread_mutex_lock(mutex), 0); // fully usable again
  EXPECT_EQ(pthread_mutex_unlock(mutex), 0);

  pthread_mutex_destroy(mutex);
  ::munmap(region, 4096);
  ::close(fd);
  ::shm_unlink(shm_name.c_str());
}
