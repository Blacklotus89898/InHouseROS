#include <ihrs/core/broker.hpp>
#include <ihrs/core/message.hpp>
#include <ihrs/core/publisher.hpp>
#include <ihrs/core/subscriber.hpp>

#include <gtest/gtest.h>
#include <string>

using namespace ihrs::core;

namespace {
struct StringMsg : Message {
  explicit StringMsg(std::string v) : value(std::move(v)) {}
  std::string value;
};
} // namespace

TEST(BrokerTest, DeliversMessageToSubscriber) {
  auto& broker = Broker::instance();
  std::string received;

  Subscriber sub("test/topic", [&](MessagePtr msg) {
    received = std::static_pointer_cast<StringMsg>(msg)->value;
  });

  Publisher pub("test/topic");
  pub.publish(std::make_shared<StringMsg>("hello"));
  broker.spinOnce();

  EXPECT_EQ(received, "hello");
}

TEST(BrokerTest, IgnoresUnsubscribedTopics) {
  auto& broker = Broker::instance();
  int calls = 0;

  Subscriber sub("other/topic", [&](MessagePtr) { ++calls; });

  Publisher pub("test/topic");
  pub.publish(std::make_shared<StringMsg>("hello"));
  broker.spinOnce();

  EXPECT_EQ(calls, 0);
}

TEST(BrokerTest, SupportsMultipleSubscribers) {
  auto& broker = Broker::instance();
  int calls = 0;

  Subscriber a("multi/topic", [&](MessagePtr) { ++calls; });
  Subscriber b("multi/topic", [&](MessagePtr) { ++calls; });

  Publisher pub("multi/topic");
  pub.publish(std::make_shared<StringMsg>("hello"));
  broker.spinOnce();

  EXPECT_EQ(calls, 2);
}

TEST(BrokerTest, IgnoresNullMessages) {
  auto& broker = Broker::instance();
  int calls = 0;

  Subscriber sub("null/topic", [&](MessagePtr) { ++calls; });

  Publisher pub("null/topic");
  pub.publish(nullptr);
  broker.spinOnce();

  EXPECT_EQ(calls, 0);
}

TEST(BrokerTest, SubscriberDestructorUnsubscribes) {
  auto& broker = Broker::instance();
  int calls = 0;

  {
    Subscriber sub("raii/topic", [&](MessagePtr) { ++calls; });
    Publisher pub("raii/topic");
    pub.publish(std::make_shared<StringMsg>("a"));
    broker.spinOnce();
    EXPECT_EQ(calls, 1);
  } // sub destroyed here: callback must not dangle

  Publisher pub("raii/topic");
  pub.publish(std::make_shared<StringMsg>("b"));
  broker.spinOnce();

  EXPECT_EQ(calls, 1); // no second delivery, no use-after-free
}

TEST(BrokerTest, ExplicitUnsubscribeStopsDelivery) {
  auto& broker = Broker::instance();
  int calls = 0;

  {
    Subscriber sub("tok/topic", [&](MessagePtr) { ++calls; });
    (void)sub;
    // Destroy via scope is covered above; here test that re-publishing after
    // destruction of both handles is safe.
  }
  Publisher pub("tok/topic");
  pub.publish(std::make_shared<StringMsg>("x"));
  broker.spinOnce();
  EXPECT_EQ(calls, 0);
}

TEST(BrokerTest, BoundedQueueDropsOldest) {
  auto& broker = Broker::instance();
  broker.setMaxPending(2);

  int calls = 0;
  Subscriber sub("bounded/topic", [&](MessagePtr) { ++calls; });
  Publisher pub("bounded/topic");

  // 4 publishes with no spin in between: oldest 2 must be dropped.
  for (int i = 0; i < 4; ++i) {
    pub.publish(std::make_shared<StringMsg>("m"));
  }
  EXPECT_EQ(broker.pendingCount(), 2u);
  EXPECT_EQ(broker.droppedCount(), 2u);

  broker.spinOnce();
  EXPECT_EQ(calls, 2);

  broker.setMaxPending(Broker::kDefaultMaxPending); // restore for other tests
}

TEST(BrokerTest, SubscribeUnsubscribeTokensAreIndependent) {
  auto& broker = Broker::instance();
  int a_calls = 0;
  int b_calls = 0;

  auto tok_a = broker.subscribe("token/topic", [&](MessagePtr) { ++a_calls; });
  Subscriber b("token/topic", [&](MessagePtr) { ++b_calls; });

  broker.unsubscribe("token/topic", tok_a);

  Publisher pub("token/topic");
  pub.publish(std::make_shared<StringMsg>("m"));
  broker.spinOnce();

  EXPECT_EQ(a_calls, 0);
  EXPECT_EQ(b_calls, 1);
}
