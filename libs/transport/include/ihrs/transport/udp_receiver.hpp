#pragma once

#include <atomic>
#include <functional>
#include <string>

#include <netinet/in.h>

namespace ihrs::transport {

/// Connectionless UDP receiver; forwards each datagram to the callback.
///
/// run() blocks on recvfrom() until stop() is called from another thread
/// (which unblocks the socket) — suitable for a dedicated receive thread.
class UDPReceiver {
public:
    using Callback = std::function<void(const std::string&)>;

    /// Binds to `port` on all interfaces. Throws std::runtime_error on
    /// socket/bind failure.
    UDPReceiver(int port, Callback callback);

    /// Closes the socket (safe to call repeatedly).
    ~UDPReceiver();

    UDPReceiver(const UDPReceiver&) = delete;
    UDPReceiver& operator=(const UDPReceiver&) = delete;

    /// Blocking receive loop. Returns after stop() is invoked.
    void run();

    /// Unblocks run() from another thread.
    void stop();

private:
    int sock_;
    int port_;
    sockaddr_in addr_{};
    Callback callback_;
    std::atomic<bool> running_{true};
};

} // namespace ihrs::transport
