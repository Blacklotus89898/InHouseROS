#include <ihrs/transport/tcp_receiver.hpp>
#include <ihrs/transport/tcp_sender.hpp>
#include <ihrs/transport/udp_receiver.hpp>
#include <ihrs/transport/udp_sender.hpp>

#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <string>
#include <thread>

using namespace ihrs::transport;

namespace {

constexpr int kTcpPort = 44600;
constexpr int kUdpPort = 44601;

/// Waits for a value with a timeout so a regression cannot hang ctest.
template <typename T>
std::optional<T> wait_for(std::future<T>& f, std::chrono::seconds limit) {
  if (f.wait_for(limit) == std::future_status::ready) {
    return f.get();
  }
  return std::nullopt;
}

} // namespace

TEST(TcpTransportTest, LoopbackDelivery) {
  std::promise<std::string> got;
  auto received = got.get_future();

  TCPReceiver receiver(kTcpPort, [&](const std::string& s) { got.set_value(s); });
  std::thread rx([&] { receiver.run(); });

  TCPSender sender;
  ASSERT_TRUE(sender.connect("127.0.0.1", kTcpPort));
  ASSERT_TRUE(sender.send("hello-tcp"));

  const auto result = wait_for(received, std::chrono::seconds(5));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, "hello-tcp");

  sender.close();
  receiver.stop();
  rx.join();
}

TEST(UdpTransportTest, LoopbackDelivery) {
  std::promise<std::string> got;
  auto received = got.get_future();

  UDPReceiver receiver(kUdpPort, [&](const std::string& s) { got.set_value(s); });
  std::thread rx([&] { receiver.run(); });

  UDPSender sender;
  ASSERT_TRUE(sender.connect("127.0.0.1", kUdpPort));
  ASSERT_TRUE(sender.send("hello-udp"));

  const auto result = wait_for(received, std::chrono::seconds(5));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, "hello-udp");

  sender.close();
  receiver.stop();
  rx.join();
}

TEST(UdpTransportTest, StopUnblocksRunPromptly) {
  UDPReceiver receiver(kUdpPort + 1, [](const std::string&) {});

  std::thread rx([&] { receiver.run(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const auto t0 = std::chrono::steady_clock::now();
  receiver.stop();
  rx.join();
  const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0);

  EXPECT_LT(elapsed.count(), 2.0);
}
