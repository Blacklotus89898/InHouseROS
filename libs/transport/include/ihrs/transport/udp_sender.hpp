#pragma once

#include <netinet/in.h>

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
    bool send(const std::string& data) override;
    void close() override;

private:
    int sock_ = -1;
    sockaddr_in addr_{};
};

} // namespace ihrs::transport
