#include <ihrs/transport/framing.hpp>
#include <ihrs/transport/tcp_receiver.hpp>
#include <ihrs/transport/tcp_sender.hpp>
#include <ihrs/transport/udp_receiver.hpp>
#include <ihrs/transport/udp_sender.hpp>

#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <vector>
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

TEST(UdpTransportTest, BinaryPayloadSurvivesLoopback) {
  std::promise<std::string> got;
  auto received = got.get_future();

  UDPReceiver receiver(kUdpPort + 2, [&](const std::string& s) { got.set_value(s); });
  std::thread rx([&] { receiver.run(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const std::string binary{char(0x00), char(0x01), char(0xFF), char(0x00), char(0x7F)};
  UDPSender sender;
  ASSERT_TRUE(sender.connect("127.0.0.1", kUdpPort + 2));
  ASSERT_TRUE(sender.send(std::span(
      reinterpret_cast<const std::byte*>(binary.data()), binary.size())));

  const auto result = wait_for(received, std::chrono::seconds(5));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, binary); // size preserved, embedded NULs intact
  EXPECT_EQ(sender.last_error(), 0);

  sender.close();
  receiver.stop();
  rx.join();
}

TEST(TcpTransportTest, FramedRoundTripOverByteStream) {
  // TCP coalesces/splits segments; framing must hide that from the app.
  std::promise<std::vector<std::string>> got;
  auto received = got.get_future();
  std::vector<std::string> frames_scratch;

  ihrs::transport::framing::FrameExtractor extractor;
  TCPReceiver receiver(kTcpPort + 1, [&](const std::string& chunk) {
    extractor.feed(std::span(reinterpret_cast<const std::byte*>(chunk.data()), chunk.size()),
                   [&](std::span<const std::byte> frame) {
                     frames_scratch.push_back(
                         std::string(reinterpret_cast<const char*>(frame.data()), frame.size()));
                     if (frames_scratch.size() == 3) {
                       got.set_value(frames_scratch);
                     }
                   });
  });
  std::thread rx([&] { receiver.run(); });

  TCPSender sender;
  ASSERT_TRUE(sender.connect("127.0.0.1", kTcpPort + 1));

  const std::string msg1{"frame-one"};
  const std::string msg2{char(0x00), char(0xAB), char(0x00)}; // binary, with NULs
  const std::string msg3{"three"};

  // Send back-to-back (likely coalesced into one segment)...
  ASSERT_TRUE(sender.send_framed(std::span(
      reinterpret_cast<const std::byte*>(msg1.data()), msg1.size())));
  ASSERT_TRUE(sender.send_framed(std::span(
      reinterpret_cast<const std::byte*>(msg2.data()), msg2.size())));
  ASSERT_TRUE(sender.send_framed(std::span(
      reinterpret_cast<const std::byte*>(msg3.data()), msg3.size())));

  // ...the extractor must still deliver exactly the three frames, in order.
  const auto result = wait_for(received, std::chrono::seconds(5));
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 3u);
  EXPECT_EQ((*result)[0], msg1);
  EXPECT_EQ((*result)[1], msg2);
  EXPECT_EQ((*result)[2], msg3);

  sender.close();
  receiver.stop();
  rx.join();
}
