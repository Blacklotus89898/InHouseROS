#ifndef UDPSENDER_HPP
#define UDPSENDER_HPP

#include "PacketSender.hpp"
#include <string>
#include <arpa/inet.h>

class UDPSender : public PacketSender {
    int sock = -1;
    sockaddr_in addr{};
public:
    ~UDPSender();

    // Setup UDP "connection" (initialize socket and addr)
    bool connect(const std::string& ip, int port) override;

    // Send data via UDP
    bool send(const std::string& data) override;

    // Close socket
    void close() override;
};

#endif // UDPSENDER_HPP
