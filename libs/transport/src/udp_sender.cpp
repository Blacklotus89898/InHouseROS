#include "ihrs/transport/udp_sender.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

namespace ihrs::transport {

UDPSender::~UDPSender() {
    close();
}

bool UDPSender::connect(const std::string& ip, int port) {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        return false;
    }

    std::memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(static_cast<uint16_t>(port));
    inet_pton(AF_INET, ip.c_str(), &addr_.sin_addr);
    return true;
}

bool UDPSender::send(const std::string& data) {
    if (sock_ < 0) {
        return false;
    }
    return sendto(sock_, data.c_str(), data.size(), 0,
                  reinterpret_cast<sockaddr*>(&addr_), sizeof(addr_)) ==
           static_cast<ssize_t>(data.size());
}

void UDPSender::close() {
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

} // namespace ihrs::transport
