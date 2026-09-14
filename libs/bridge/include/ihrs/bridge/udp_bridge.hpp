#pragma once

#include <ihrs/core/typed.hpp>
#include <ihrs/core/type_support.hpp>
#include <ihrs/transport/framing.hpp>
#include <ihrs/transport/udp_receiver.hpp>
#include <ihrs/transport/udp_sender.hpp>

#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

namespace ihrs::bridge {

/// Forwards messages from a typed topic to a UDP endpoint, length-prefixed
/// per transport/framing.hpp. Sends happen synchronously on the publisher's
/// thread (no queueing worker).
template <typename T>
class UdpTopicSink final {
    static_assert(std::is_trivially_copyable_v<T>,
                  "ihrs::bridge: UDP bridge v1 supports trivially copyable payloads only");

public:
    UdpTopicSink(core::NodeHandle node, std::string topic, const std::string& ip, int port) {
        sender_ = std::make_unique<transport::UDPSender>();
        if (!sender_->connect(ip, port)) {
            throw std::runtime_error("UdpTopicSink: cannot reach " + ip + ":" +
                                     std::to_string(port));
        }
        sub_ = node.create_subscriber<T>(topic, [this](const T& v) { send(v); });
    }

    ~UdpTopicSink() = default;

    UdpTopicSink(const UdpTopicSink&) = delete;
    UdpTopicSink& operator=(const UdpTopicSink&) = delete;

private:
    void send(const T& v) {
        encoded_.resize(core::Codec<T>{}.encoded_size(v));
        core::Codec<T>{}.encode(v, encoded_.data());
        frame_.clear();
        transport::framing::write_frame(frame_, std::span<const std::byte>(encoded_));
        (void)sender_->send(std::span<const std::byte>(frame_));
    }

    std::unique_ptr<transport::UDPSender> sender_;
    std::shared_ptr<core::Sub<T>> sub_;
    std::vector<std::byte> encoded_;
    std::vector<std::byte> frame_;
};

/// Receives length-prefixed datagrams on a UDP port and republishes each
/// decoded payload onto a typed topic. Owns the socket and its receive
/// thread.
template <typename T>
class UdpTopicSource final {
    static_assert(std::is_trivially_copyable_v<T>,
                  "ihrs::bridge: UDP bridge v1 supports trivially copyable payloads only");

public:
    UdpTopicSource(core::NodeHandle node, std::string topic, int port)
        : pub_(node.create_publisher<T>(topic)) {
        receiver_ = std::make_unique<transport::UDPReceiver>(
            port, [this](const std::string& chunk) { feed(chunk); });
        worker_ = std::thread([this] { receiver_->run(); });
    }

    ~UdpTopicSource() {
        if (receiver_ != nullptr) {
            receiver_->stop();
        }
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    UdpTopicSource(const UdpTopicSource&) = delete;
    UdpTopicSource& operator=(const UdpTopicSource&) = delete;

private:
    void feed(const std::string& chunk) {
        const auto bytes = std::span(reinterpret_cast<const std::byte*>(chunk.data()),
                                     chunk.size());
        extractor_.feed(bytes, [this](std::span<const std::byte> frame) {
            if (frame.size() < sizeof(T)) {
                return; // too small for the payload type: discard
            }
            pub_->publish(core::Codec<T>{}.decode(frame.data()));
        });
    }

    std::shared_ptr<core::Pub<T>> pub_;
    std::unique_ptr<transport::UDPReceiver> receiver_;
    std::thread worker_;
    transport::framing::FrameExtractor extractor_;
};

} // namespace ihrs::bridge
