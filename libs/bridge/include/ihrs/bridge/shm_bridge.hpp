#pragma once

#include <ihrs/core/typed.hpp>
#include <ihrs/ipc/generic_shm_publisher.hpp>
#include <ihrs/ipc/generic_shm_subscriber.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

namespace ihrs::bridge {

/// Forwards messages from a typed topic into a POSIX shared-memory segment
/// (latest-value semantics). Owns the SHM segment publisher and its worker
/// thread, which writes the most recent sample at `rate_hz`.
///
/// Requirements: destroy the sink while the executor is stopped (single-
/// threaded lifecycle, like all core subscriptions). The segment is created
/// here and unlinked on destruction.
template <typename T>
class ShmTopicSink final {
    static_assert(std::is_trivially_copyable_v<T>,
                  "ihrs::bridge: SHM bridge v1 supports trivially copyable payloads only");

public:
    ShmTopicSink(core::NodeHandle node, std::string topic, const std::string& segment,
                 int rate_hz = 100) {
        sub_ = node.create_subscriber<T>(topic, [this](const T& v) { store(v); });
        shm_ = std::make_unique<ipc::GenericSHMPublisher<T>>(
            segment.c_str(), sizeof(T) + 256, [this](int) { return latest(); }, rate_hz);
        worker_ = std::thread([this] { shm_->run(); });
    }

    ~ShmTopicSink() {
        if (shm_ != nullptr) {
            shm_->stop();
        }
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    ShmTopicSink(const ShmTopicSink&) = delete;
    ShmTopicSink& operator=(const ShmTopicSink&) = delete;

private:
    void store(const T& v) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = std::make_shared<const T>(v);
    }

    T latest() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_ != nullptr ? *latest_ : T{};
    }

    std::shared_ptr<core::Sub<T>> sub_;
    std::unique_ptr<ipc::GenericSHMPublisher<T>> shm_;
    std::thread worker_;
    mutable std::mutex mutex_;
    std::shared_ptr<const T> latest_;
};

/// Reads a POSIX shared-memory segment (latest-value) and republishes every
/// polled sample onto a typed topic. The segment must already exist (its
/// publisher side is the creator). Duplicates are possible — SHM v1
/// re-publishes the latest value every poll.
template <typename T>
class ShmTopicSource final {
    static_assert(std::is_trivially_copyable_v<T>,
                  "ihrs::bridge: SHM bridge v1 supports trivially copyable payloads only");

public:
    ShmTopicSource(core::NodeHandle node, std::string topic, const std::string& segment,
                   int rate_hz = 100)
        : pub_(node.create_publisher<T>(topic)) {
        shm_ = std::make_unique<ipc::GenericSHMSubscriber<T>>(
            segment.c_str(), sizeof(T) + 256,
            [this](const T& v) { pub_->publish(v); }, rate_hz);
        worker_ = std::thread([this] { shm_->run(); });
    }

    ~ShmTopicSource() {
        if (shm_ != nullptr) {
            shm_->stop();
        }
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    ShmTopicSource(const ShmTopicSource&) = delete;
    ShmTopicSource& operator=(const ShmTopicSource&) = delete;

private:
    std::shared_ptr<core::Pub<T>> pub_;
    std::unique_ptr<ipc::GenericSHMSubscriber<T>> shm_;
    std::thread worker_;
};

} // namespace ihrs::bridge
