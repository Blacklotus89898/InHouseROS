#include "TCPReceiver.hpp"
#include <unistd.h>
#include <arpa/inet.h>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>

TCPReceiver::TCPReceiver(int port, Callback cb)
    : listen_sock(-1), client_sock(-1), port(port), callback(std::move(cb)) {

    listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        perror("TCP socket failed");
        exit(1);
    }

    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(listen_sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("TCP bind failed");
        close();
        exit(1);
    }

    if (listen(listen_sock, 1) < 0) {
        perror("TCP listen failed");
        close();
        exit(1);
    }
}

TCPReceiver::~TCPReceiver() {
    close();
}

void TCPReceiver::run() {
    std::cout << "[TCPReceiver] Waiting for connection on port " << port << "...\n";

    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);

    client_sock = accept(listen_sock, (sockaddr*)&client_addr, &client_len);
    if (client_sock < 0) {
        perror("TCP accept failed");
        return;
    }

    std::cout << "[TCPReceiver] Client connected\n";

    char buffer[1024];
    while (true) {
        ssize_t n = recv(client_sock, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0) break;  // Connection closed or error
        buffer[n] = '\0';
        callback(std::string(buffer));
    }

    std::cout << "[TCPReceiver] Client disconnected\n";
    close();
}

void TCPReceiver::close() {
    if (client_sock >= 0) {
        ::close(client_sock);
        client_sock = -1;
    }
    if (listen_sock >= 0) {
        ::close(listen_sock);
        listen_sock = -1;
    }
}
