#include <ihrs/bridge/shm_bridge.hpp>
#include <ihrs/bridge/udp_bridge.hpp>
#include <ihrs/core/context.hpp>
#include <ihrs/core/executor.hpp>
#include <ihrs/ipc/generic_shm_publisher.hpp>
#include <ihrs/ipc/generic_shm_subscriber.hpp>

#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

using namespace ihrs;

namespace {

struct Sample {
    double value{0.0};
    int seq{0};
};

constexpr int kRate = 500; // Hz: fast enough for snappy tests

/// Spins an executor until stopped (RAII helper for bridge tests).
class Spinner {
public:
    explicit Spinner(core::Context& ctx, double hz = 500.0)
        : exec_(ctx), thread_([this, hz] { exec_.spin(hz); }) {}
    ~Spinner() {
        exec_.stop();
        thread_.join();
    }

    Spinner(const Spinner&) = delete;
    Spinner& operator=(const Spinner&) = delete;

private:
    core::Executor exec_;
    std::thread thread_;
};

} // namespace

// The plan's acceptance test: typed Pub<T> -> bridge -> independent raw
// GenericSHMSubscriber<T> reader, all in-process.
TEST(ShmBridgeTest, TopicToShmForwardsLatest) {
    core::Context ctx;
    auto node = ctx.create_node("t1");

    // Bridge: topic -> SHM (creates the segment).
    bridge::ShmTopicSink<Sample> sink(node, "/s", "/ihrs_test_sink_fwd", kRate);

    // Independent raw SHM reader (not part of the typed world).
    std::atomic<int> last_seq{-1};
    ipc::GenericSHMSubscriber<Sample> reader(
        "/ihrs_test_sink_fwd", sizeof(Sample) + 256,
        [&](const Sample& s) { last_seq.store(s.seq); }, kRate);
    std::thread reader_thread([&] { reader.run(); });

    Spinner spinner(ctx);
    {
        auto pub = node.create_publisher<Sample>("/s");
        for (int i = 1; i <= 5; ++i) {
            pub->publish(Sample{0.5 * i, i});
            std::this_thread::sleep_for(std::chrono::milliseconds(8));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    reader.stop();
    reader_thread.join();

    EXPECT_GE(last_seq.load(), 4); // latest-value forwarding observed

    // sink destroys last: it created and unlinks the segment.
}

TEST(ShmBridgeTest, ShmToTopicRepublishes) {
    core::Context ctx;
    auto node = ctx.create_node("t2");

    // Raw SHM publisher creates the segment first (source requires it).
    int counter = 0;
    ipc::GenericSHMPublisher<Sample> shm_pub(
        "/ihrs_test_src", sizeof(Sample) + 256,
        [&](int) { return Sample{0.25, ++counter}; }, kRate);
    std::thread shm_thread([&] { shm_pub.run(); });

    // Bridge: SHM -> topic.
    bridge::ShmTopicSource<Sample> source(node, "/s2", "/ihrs_test_src", kRate);

    std::atomic<int> received{0};
    std::atomic<int> max_seq{0};
    const auto sub = node.create_subscriber<Sample>("/s2", [&](const Sample& s) {
        ++received;
        int prev = max_seq.load();
        while (s.seq > prev && !max_seq.compare_exchange_weak(prev, s.seq)) {
        }
    });

    Spinner spinner(ctx);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));

    EXPECT_GE(received.load(), 3);
    EXPECT_GT(max_seq.load(), 0); // forward progress through the bridge

    shm_pub.stop();
    shm_thread.join();
}

TEST(UdpBridgeTest, TopicThroughUdpLoopback) {
    core::Context ctx;
    auto node = ctx.create_node("t3");

    // Two directions use two topics: wiring one topic to both sides of a
    // UDP bridge would feed back into itself.
    bridge::UdpTopicSource<Sample> source(node, "/in", 44720);
    bridge::UdpTopicSink<Sample> sink(node, "/out", "127.0.0.1", 44720);

    std::atomic<int> received{0};
    std::atomic<int> max_seq{0};
    const auto sub = node.create_subscriber<Sample>("/in", [&](const Sample& s) {
        ++received;
        int prev = max_seq.load();
        while (s.seq > prev && !max_seq.compare_exchange_weak(prev, s.seq)) {
        }
    });

    Spinner spinner(ctx);
    {
        auto pub = node.create_publisher<Sample>("/out");
        for (int i = 1; i <= 5; ++i) {
            pub->publish(Sample{2.5, i});
            std::this_thread::sleep_for(std::chrono::milliseconds(8));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    EXPECT_GE(received.load(), 3);
    EXPECT_EQ(max_seq.load(), 5); // all samples, in order, through framing
}
