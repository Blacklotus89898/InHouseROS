#include "ihrs/transport/tcp_sender.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include "ihrs/transport/framing.hpp"

namespace ihrs::transport {

TCPSender::~TCPSender() {
    close();
}

bool TCPSender::connect(const std::string& ip, int port) {
    last_error_ = 0;
    sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0) {
        last_error_ = errno;
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);

    if (::connect(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        last_error_ = errno;
        return false;
    }
    return true;
}

bool TCPSender::send(std::span<const std::byte> data) {
    if (sock_ < 0) {
        return false;
    }
    const ssize_t rc = ::send(sock_, data.data(), data.size(), 0);
    if (rc < 0 || static_cast<std::size_t>(rc) != data.size()) {
        last_error_ = errno;
        return false;
    }
    return true;
}

bool TCPSender::send_framed(std::span<const std::byte> data) {
    frame_scratch_.clear();
    framing::write_frame(frame_scratch_, data);
    return send(std::span<const std::byte>(frame_scratch_));
}

void TCPSender::close() {
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
    last_error_ = 0;
}

} // namespace ihrs::transport
