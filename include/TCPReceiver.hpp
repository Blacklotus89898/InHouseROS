#pragma once

#include <string>
#include <functional>
#include <netinet/in.h>

class TCPReceiver {
public:
    using Callback = std::function<void(const std::string&)>;

private:
    int listen_sock;
    int client_sock;
    int port;
    sockaddr_in addr{};
    Callback callback;

public:
    explicit TCPReceiver(int port, Callback callback);
    ~TCPReceiver();

    // Start the blocking receive loop
    void run();

    // Close sockets
    void close();
};
