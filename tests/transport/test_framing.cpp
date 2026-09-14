#include <ihrs/transport/framing.hpp>

#include <gtest/gtest.h>
#include <cstdint>
#include <string>
#include <vector>

using namespace ihrs::transport::framing;

namespace {

std::vector<std::byte> to_bytes(const std::string& s) {
  std::vector<std::byte> out(s.size());
  for (std::size_t i = 0; i < s.size(); ++i) {
    out[i] = static_cast<std::byte>(static_cast<unsigned char>(s[i]));
  }
  return out;
}

} // namespace

TEST(FramingTest, RoundTripSingleFrame) {
  std::vector<std::byte> wire;
  const auto payload = to_bytes("hello");
  write_frame(wire, payload);

  ASSERT_EQ(wire.size(), payload.size() + kHeaderSize);

  FrameExtractor fx;
  std::string got;
  ASSERT_TRUE(fx.feed(wire, [&](std::span<const std::byte> p) {
    got.assign(reinterpret_cast<const char*>(p.data()), p.size());
  }));
  EXPECT_EQ(got, "hello");
  EXPECT_EQ(fx.pending(), 0u);
}

TEST(FramingTest, MultipleFramesInOneFeed) {
  std::vector<std::byte> wire;
  write_frame(wire, to_bytes("a"));
  write_frame(wire, to_bytes("bb"));
  write_frame(wire, to_bytes("ccc"));

  FrameExtractor fx;
  std::vector<std::string> got;
  fx.feed(wire, [&](std::span<const std::byte> p) {
    got.emplace_back(reinterpret_cast<const char*>(p.data()), p.size());
  });
  ASSERT_EQ(got.size(), 3u);
  EXPECT_EQ(got[0], "a");
  EXPECT_EQ(got[1], "bb");
  EXPECT_EQ(got[2], "ccc");
}

TEST(FramingTest, ByteByByteReassembly) {
  std::vector<std::byte> wire;
  write_frame(wire, to_bytes("split"));

  FrameExtractor fx;
  std::string got;
  auto on_frame = [&](std::span<const std::byte> p) {
    got.assign(reinterpret_cast<const char*>(p.data()), p.size());
  };
  for (std::size_t i = 0; i + 1 < wire.size(); ++i) {
    ASSERT_TRUE(fx.feed(std::span(&wire[i], 1), on_frame));
    EXPECT_TRUE(got.empty());
  }
  ASSERT_TRUE(fx.feed(std::span(&wire.back(), 1), on_frame));
  EXPECT_EQ(got, "split");
}

TEST(FramingTest, BinaryPayloadWithNullBytes) {
  std::vector<std::byte> payload;
  for (int b : {0x00, 0x01, 0xFF, 0x00, 0x7F}) {
    payload.push_back(static_cast<std::byte>(b));
  }

  std::vector<std::byte> wire;
  write_frame(wire, payload);

  FrameExtractor fx;
  std::vector<std::byte> got;
  ASSERT_TRUE(fx.feed(wire, [&](std::span<const std::byte> p) {
    got.assign(p.begin(), p.end());
  }));
  EXPECT_EQ(got, payload);
}

TEST(FramingTest, OversizedFrameIsRejected) {
  std::vector<std::byte> wire{static_cast<std::byte>(0xFF), static_cast<std::byte>(0xFF),
                              static_cast<std::byte>(0xFF), static_cast<std::byte>(0x7F)};
  wire.insert(wire.end(), 16, std::byte{0x00});

  FrameExtractor fx;
  EXPECT_FALSE(fx.feed(wire, [](std::span<const std::byte>) {}));
  EXPECT_EQ(fx.pending(), 0u);
}
