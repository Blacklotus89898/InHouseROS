#include "ihrs/transport/udp_sender.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

namespace ihrs::transport {

UDPSender::~UDPSender() {
    close();
}

bool UDPSender::connect(const std::string& ip, int port) {
    last_error_ = 0;
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        last_error_ = errno;
        return false;
    }

    std::memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(static_cast<uint16_t>(port));
    inet_pton(AF_INET, ip.c_str(), &addr_.sin_addr);
    return true;
}

bool UDPSender::send(std::span<const std::byte> data) {
    if (sock_ < 0) {
        return false;
    }
    const ssize_t rc = sendto(sock_, data.data(), data.size(), 0,
                              reinterpret_cast<sockaddr*>(&addr_), sizeof(addr_));
    if (rc < 0 || static_cast<std::size_t>(rc) != data.size()) {
        last_error_ = errno;
        return false;
    }
    return true;
}

void UDPSender::close() {
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
    last_error_ = 0;
}

} // namespace ihrs::transport
