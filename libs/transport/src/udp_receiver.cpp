#include "ihrs/transport/udp_receiver.hpp"

#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <stdexcept>

namespace ihrs::transport {

UDPReceiver::UDPReceiver(int port, Callback callback)
    : sock_(-1), port_(port), callback_(std::move(callback)) {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        throw std::runtime_error("UDP socket creation failed");
    }

    std::memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_addr.s_addr = INADDR_ANY;
    addr_.sin_port = htons(static_cast<uint16_t>(port_));

    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr_), sizeof(addr_)) < 0) {
        ::close(sock_);
        sock_ = -1;
        throw std::runtime_error("UDP bind failed");
    }
}

UDPReceiver::~UDPReceiver() {
    stop();
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

void UDPReceiver::run() {
    char buffer[1024];
    sockaddr_in sender_addr{};
    socklen_t sender_len = sizeof(sender_addr);

    std::cout << "[UDPReceiver] Listening on port " << port_ << "...\n";
    while (running_.load(std::memory_order_relaxed)) {
        const ssize_t n = recvfrom(sock_, buffer, sizeof(buffer) - 1, 0,
                                   reinterpret_cast<sockaddr*>(&sender_addr), &sender_len);
        if (n <= 0) {
            if (!running_.load(std::memory_order_relaxed)) {
                break; // stop() shut the socket down
            }
            continue;
        }
        buffer[n] = '\0';
        callback_(std::string(buffer));
    }
}

void UDPReceiver::stop() {
    running_.store(false, std::memory_order_relaxed);
    if (sock_ >= 0) {
        ::shutdown(sock_, SHUT_RDWR);
    }
}

} // namespace ihrs::transport
