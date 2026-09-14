#pragma once

#include <cstddef>
#include <netinet/in.h>
#include <span>
#include <vector>

#include <string>

#include "ihrs/transport/packet_sender.hpp"

namespace ihrs::transport {

/// PacketSender over a TCP stream. One sender talks to one endpoint;
/// connect() opens the socket, send() writes the full payload.
class TCPSender : public PacketSender {
public:
    TCPSender() = default;
    ~TCPSender() override;

    TCPSender(const TCPSender&) = delete;
    TCPSender& operator=(const TCPSender&) = delete;

    bool connect(const std::string& ip, int port) override;
    bool send(std::span<const std::byte> data) override;
    void close() override;

    using PacketSender::send; // expose string_view overload

    /// Sends payload with length-prefix framing (see framing.hpp). Pair
    /// the stream with transport::framing::FrameExtractor on the peer.
    bool send_framed(std::span<const std::byte> data);

    [[nodiscard]] int last_error() const noexcept override { return last_error_; }

private:
    int sock_ = -1;
    int last_error_ = 0;
    std::vector<std::byte> frame_scratch_;
};

} // namespace ihrs::transport
