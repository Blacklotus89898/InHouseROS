#include "ihrs/transport/tcp_sender.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace ihrs::transport {

TCPSender::~TCPSender() {
    close();
}

bool TCPSender::connect(const std::string& ip, int port) {
    sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0) {
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);

    return ::connect(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
}

bool TCPSender::send(const std::string& data) {
    if (sock_ < 0) {
        return false;
    }
    return ::send(sock_, data.c_str(), data.size(), 0) == static_cast<ssize_t>(data.size());
}

void TCPSender::close() {
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

} // namespace ihrs::transport
