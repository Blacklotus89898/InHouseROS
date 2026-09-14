#pragma once

#include <netinet/in.h>

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
    bool send(const std::string& data) override;
    void close() override;

private:
    int sock_ = -1;
};

} // namespace ihrs::transport
