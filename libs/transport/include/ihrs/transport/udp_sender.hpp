#pragma once

#include <cstddef>
#include <netinet/in.h>
#include <span>

#include <string>

#include "ihrs/transport/packet_sender.hpp"

namespace ihrs::transport {

/// PacketSender over UDP datagrams. connect() only records the
/// destination address; send() emits one datagram per call.
class UDPSender : public PacketSender {
public:
    UDPSender() = default;
    ~UDPSender() override;

    UDPSender(const UDPSender&) = delete;
    UDPSender& operator=(const UDPSender&) = delete;

    bool connect(const std::string& ip, int port) override;
    bool send(std::span<const std::byte> data) override;
    void close() override;

    using PacketSender::send; // expose string_view overload

    [[nodiscard]] int last_error() const noexcept override { return last_error_; }

private:
    int sock_ = -1;
    int last_error_ = 0;
    sockaddr_in addr_{};
};

} // namespace ihrs::transport
