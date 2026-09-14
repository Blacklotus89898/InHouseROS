#pragma once

#include <atomic>
#include <functional>
#include <string>

#include <netinet/in.h>

namespace ihrs::transport {

/// Blocking TCP server for a single client connection.
///
/// run() accepts one connection, then streams every received segment to
/// the callback until the peer disconnects or stop() is called. run()
/// must be called from a dedicated thread; stop() (from any thread)
/// unblocks accept()/recv() and makes run() return promptly.
class TCPReceiver {
public:
    using Callback = std::function<void(const std::string&)>;

    /// Binds and listens on `port` on all interfaces. Throws
    /// std::runtime_error on socket/bind/listen failure.
    explicit TCPReceiver(int port, Callback callback);

    /// Closes all sockets (safe to call repeatedly).
    ~TCPReceiver();

    TCPReceiver(const TCPReceiver&) = delete;
    TCPReceiver& operator=(const TCPReceiver&) = delete;

    /// Blocking accept + receive loop. Returns after the client
    /// disconnects or stop() is invoked.
    void run();

    /// Unblocks run() from another thread.
    void stop();

private:
    /// Closes all sockets; safe to call repeatedly.
    void close();

    int listen_sock_;
    int client_sock_;
    int port_;
    sockaddr_in addr_{};
    Callback callback_;
    std::atomic<bool> running_{true};
};

} // namespace ihrs::transport
