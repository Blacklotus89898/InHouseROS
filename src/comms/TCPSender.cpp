#include "TCPSender.hpp"
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>

TCPSender::TCPSender() : sock(-1) {}

TCPSender::~TCPSender() {
    close();
}

bool TCPSender::connect(const std::string& ip, int port) {
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return false;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);

    return ::connect(sock, (sockaddr*)&addr, sizeof(addr)) == 0;
}

bool TCPSender::send(const std::string& data) {
    if (sock < 0) return false;
    return ::send(sock, data.c_str(), data.size(), 0) == (ssize_t)data.size();
}

void TCPSender::close() {
    if (sock >= 0) {
        ::close(sock);
        sock = -1;
    }
}
