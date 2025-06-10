#pragma once
#include <string>

// Essentially an interface
class PacketSender {
public:
    virtual ~PacketSender() = default;
    virtual bool connect(const std::string& ip, int port) = 0;
    virtual bool send(const std::string& data) = 0;
    virtual void close() = 0;
};
