#pragma once

#include "PacketSender.hpp"
#include <string>

class TCPSender : public PacketSender {
    int sock;

public:
    TCPSender();
    ~TCPSender() override;

    bool connect(const std::string& ip, int port) override;
    bool send(const std::string& data) override;
    void close() override;
};
