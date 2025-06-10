#include "UDPReceiver.hpp"
#include <iostream>
#include <unistd.h>
#include <cstring>

UDPReceiver::UDPReceiver(int port, Callback callback)
    : sock_(-1), port_(port), callback_(callback) {

    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        perror("UDP socket failed");
        exit(1);
    }

    std::memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_addr.s_addr = INADDR_ANY;
    addr_.sin_port = htons(port_);

    if (bind(sock_, (sockaddr*)&addr_, sizeof(addr_)) < 0) {
        perror("UDP bind failed");
        close(sock_);
        exit(1);
    }
}

UDPReceiver::~UDPReceiver() {
    if (sock_ >= 0) {
        close(sock_);
    }
}

void UDPReceiver::run() {
    char buffer[1024];
    sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);

    std::cout << "[UDPReceiver] Listening on port " << port_ << "...\n";
    while (true) {
        ssize_t n = recvfrom(sock_, buffer, sizeof(buffer) - 1, 0,
                             (sockaddr*)&sender_addr, &sender_len);
        if (n <= 0) continue;
        buffer[n] = '\0';
        callback_(std::string(buffer));
    }
}
