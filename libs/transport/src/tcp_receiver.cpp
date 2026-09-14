#include "ihrs/transport/tcp_receiver.hpp"

#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <stdexcept>

namespace ihrs::transport {

TCPReceiver::TCPReceiver(int port, Callback callback)
    : listen_sock_(-1), client_sock_(-1), port_(port), callback_(std::move(callback)) {
    listen_sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock_ < 0) {
        throw std::runtime_error("TCP socket creation failed");
    }

    std::memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_addr.s_addr = INADDR_ANY;
    addr_.sin_port = htons(static_cast<uint16_t>(port_));

    if (bind(listen_sock_, reinterpret_cast<sockaddr*>(&addr_), sizeof(addr_)) < 0) {
        close();
        throw std::runtime_error("TCP bind failed");
    }

    if (listen(listen_sock_, 1) < 0) {
        close();
        throw std::runtime_error("TCP listen failed");
    }
}

TCPReceiver::~TCPReceiver() {
    stop();
    close();
}

void TCPReceiver::run() {
    std::cout << "[TCPReceiver] Waiting for connection on port " << port_ << "...\n";

    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);

    client_sock_ = accept(listen_sock_, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
    if (client_sock_ < 0) {
        if (running_.load(std::memory_order_relaxed)) {
            std::cerr << "[TCPReceiver] accept failed\n";
        }
        return; // stop() closed the socket
    }

    std::cout << "[TCPReceiver] Client connected\n";

    char buffer[1024];
    while (running_.load(std::memory_order_relaxed)) {
        const ssize_t n = recv(client_sock_, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0) {
            break; // peer closed or stop() shut the socket down
        }
        buffer[n] = '\0';
        callback_(std::string(buffer));
    }

    std::cout << "[TCPReceiver] Client disconnected\n";
    close();
}

void TCPReceiver::stop() {
    running_.store(false, std::memory_order_relaxed);
    if (listen_sock_ >= 0) {
        ::shutdown(listen_sock_, SHUT_RDWR);
    }
    if (client_sock_ >= 0) {
        ::shutdown(client_sock_, SHUT_RDWR);
    }
}

void TCPReceiver::close() {
    if (client_sock_ >= 0) {
        ::close(client_sock_);
        client_sock_ = -1;
    }
    if (listen_sock_ >= 0) {
        ::close(listen_sock_);
        listen_sock_ = -1;
    }
}

} // namespace ihrs::transport
