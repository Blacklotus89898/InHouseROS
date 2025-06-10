#pragma once

#include <functional>
#include <string>
#include <netinet/in.h>

class UDPReceiver {
public:
    using Callback = std::function<void(const std::string&)>;

    UDPReceiver(int port, Callback callback);
    ~UDPReceiver();

    void run();

private:
    int sock_;
    int port_;
    sockaddr_in addr_;
    Callback callback_;
};
