#include "UDPSender.hpp"
#include <arpa/inet.h>
#include <unistd.h>

UDPSender::~UDPSender() { close(); }

bool UDPSender::connect(const std::string& ip, int port) {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return false;

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);
    return true;
}

bool UDPSender::send(const std::string& data) {
    if (sock < 0) return false;
    return sendto(sock, data.c_str(), data.size(), 0,
                  (sockaddr*)&addr, sizeof(addr)) == (ssize_t)data.size();
}

void UDPSender::close() {
    if (sock >= 0) {
        ::close(sock);
        sock = -1;
    }
}