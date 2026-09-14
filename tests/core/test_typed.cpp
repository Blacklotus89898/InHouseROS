#include <ihrs/core/context.hpp>
#include <ihrs/core/executor.hpp>
#include <ihrs/core/publisher.hpp>
#include <ihrs/core/typed.hpp>
#include <ihrs/core/type_support.hpp>

#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <cstring>
#include <vector>

using namespace ihrs::core;

namespace {

struct ImuSample {
  double ax{0}, ay{0}, az{0};
  int seq{0};
};

struct TextPayload {
  std::string text;
};

// A non-trivially-copyable payload WITH a user codec: must pass the gate.
// (Specializations must be declared in the Codec template's namespace.)
} // namespace

namespace ihrs::core {
template <>
struct Codec<TextPayload> {
  static std::size_t encoded_size(const TextPayload& p) { return 4 + p.text.size(); }
  static void encode(const TextPayload& p, std::byte* out) {
    const auto size = static_cast<std::uint32_t>(p.text.size());
    std::memcpy(out, &size, sizeof(size));
    std::memcpy(out + 4, p.text.data(), p.text.size());
  }
  static TextPayload decode(const std::byte* in) {
    std::uint32_t size = 0;
    std::memcpy(&size, in, sizeof(size));
    TextPayload p;
    p.text.resize(size);
    std::memcpy(p.text.data(), in + 4, size);
    return p;
  }
};
} // namespace ihrs::core

namespace {

} // namespace

TEST(TypeSupportTest, HashIsStablePerTypeAndDistinctAcrossTypes) {
  constexpr auto h1 = type_hash<ImuSample>();
  constexpr auto h1_again = type_hash<ImuSample>();
  constexpr auto h2 = type_hash<TextPayload>();
  static_assert(h1 == h1_again, "same type must hash equal");
  static_assert(h1 != h2, "different types must hash differently");
  SUCCEED();
}

TEST(TypeSupportTest, WireCompatibilityGate) {
  static_assert(is_wire_compatible_v<ImuSample>, "trivially copyable -> auto codec");
  static_assert(!is_wire_compatible_v<std::string>, "std::string without codec -> rejected");
  static_assert(is_wire_compatible_v<TextPayload>, "user Codec specialization -> accepted");
  SUCCEED();
}

TEST(TypedTest, RoundTripThroughExecutor) {
  Context ctx;
  auto node = ctx.create_node("imu_node");
  std::atomic<int> received{0};
  ImuSample last{};

  auto sub = node.create_subscriber<ImuSample>("/imu", [&](const ImuSample& s) {
    last = s;
    ++received;
  });
  auto pub = node.create_publisher<ImuSample>("/imu");

  Executor exec(ctx);
  std::thread spinner([&] { exec.spin(200.0); });

  for (int i = 1; i <= 5; ++i) {
    pub->publish(ImuSample{0.1 * i, 0.2 * i, 9.81, i});
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  exec.stop();
  spinner.join();

  ASSERT_GE(received.load(), 5);
  EXPECT_DOUBLE_EQ(last.ax, 0.5);
  EXPECT_DOUBLE_EQ(last.ay, 1.0);
  EXPECT_EQ(last.seq, 5);
}

TEST(TypedTest, MultipleSubscribersAllReceive) {
  Context ctx;
  auto node = ctx.create_node("fanout");
  std::atomic<int> a{0}, b{0};

  auto sub_a = node.create_subscriber<ImuSample>("/fan", [&](const ImuSample&) { ++a; });
  auto sub_b = node.create_subscriber<ImuSample>("/fan", [&](const ImuSample&) { ++b; });
  auto pub = node.create_publisher<ImuSample>("/fan");

  pub->publish(ImuSample{});
  ctx.broker().spinOnce();

  EXPECT_EQ(a.load(), 1);
  EXPECT_EQ(b.load(), 1);
}

TEST(TypedTest, NonTrivialPayloadWithCodecRoundTripsInProcess) {
  Context ctx;
  auto node = ctx.create_node("text");
  std::atomic<int> received{0};
  std::string last;

  auto sub = node.create_subscriber<TextPayload>("/text", [&](const TextPayload& p) {
    last = p.text;
    ++received;
  });
  auto pub = node.create_publisher<TextPayload>("/text");

  pub->publish(TextPayload{"hello typed world"});
  pub->publish(TextPayload{"second"});
  ctx.broker().spinOnce();

  EXPECT_EQ(received.load(), 2);
  EXPECT_EQ(last, "second");
}

TEST(TypedTest, TypeMismatchIsRejected) {
  Context ctx;
  auto node = ctx.create_node("typed");

  struct Meters { double v{0}; };
  struct Feet { double v{0}; };

  auto pub = node.create_publisher<Meters>("/distance");

  EXPECT_THROW((void)node.create_subscriber<Feet>("/distance", [](const Feet&) {}),
               TypeMismatchError);

  auto sub = node.create_subscriber<Meters>("/distance", [](const Meters&) {}); // same type: fine
  pub->publish(Meters{3.5});
  ctx.broker().spinOnce();
  SUCCEED();
}

TEST(TypedTest, TopicRebindableAfterLastEndpointDies) {
  Context ctx;
  auto node = ctx.create_node("rebind");
  struct A { int v{0}; };
  struct B { int v{0}; };

  {
    auto pub = node.create_publisher<A>("/re");
    ASSERT_EQ(ctx.broker().topic_type("/re").has_value(), true);
  } // last endpoint gone: binding must be released
  EXPECT_EQ(ctx.broker().topic_type("/re").has_value(), false);

  auto sub = node.create_subscriber<B>("/re", [](const B&) {}); // must not throw
  EXPECT_EQ(ctx.broker().topic_type("/re").value(), type_hash<B>());
}

TEST(TypedTest, ForeignUntypedMessagesAreIgnored) {
  Context ctx;
  auto node = ctx.create_node("mixed");
  std::atomic<int> received{0};

  auto sub = node.create_subscriber<ImuSample>("/mixed", [&](const ImuSample&) { ++received; });
  auto pub = node.create_publisher<ImuSample>("/mixed");

  // An untyped endpoint fires a foreign message onto the same topic:
  Publisher legacy("/mixed", ctx);
  legacy.publish(std::make_shared<Message>()); // not a TypedMessage<ImuSample>

  pub->publish(ImuSample{});
  ctx.broker().spinOnce();

  EXPECT_EQ(received.load(), 1); // foreign message skipped, typed one delivered
}

TEST(TypedTest, MoveKeepsBindingAlive) {
  Context ctx;
  auto node = ctx.create_node("move");
  std::atomic<int> received{0};

  auto sub = node.create_subscriber<ImuSample>("/mv", [&](const ImuSample&) { ++received; });
  auto pub = node.create_publisher<ImuSample>("/mv");

  auto pub2 = std::move(pub);
  pub2->publish(ImuSample{});
  ctx.broker().spinOnce();
  EXPECT_EQ(received.load(), 1);

  // Type binding stays registered through the move.
  EXPECT_EQ(ctx.broker().topic_type("/mv").value(), type_hash<ImuSample>());
}

TEST(QoSTest, DepthDefaultsAndChains) {
  QoS qos;
  EXPECT_EQ(qos.depth, 1u);
  qos.with_depth(8);
  EXPECT_EQ(qos.depth, 8u);
}
