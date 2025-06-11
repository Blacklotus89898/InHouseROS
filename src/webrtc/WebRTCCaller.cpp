#include "WebRTCCaller.hpp"
#include <iostream>
#include <stdexcept>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <thread>

WebRTCCaller::WebRTCCaller(const std::string& serverIp, int port)
    : serverIp(serverIp), port(port), connected(false), dataChannelReady(false), offerReady(false), answerSet(false) {}

void WebRTCCaller::start() {
    rtc::InitLogger(rtc::LogLevel::Info);

    rtc::Configuration config;
    config.iceServers.emplace_back("stun:stun.l.google.com:19302");
    config.portRangeBegin = 10000;
    config.portRangeEnd = 10100;

    peerConnection = std::make_shared<rtc::PeerConnection>(config);

    setupSocket();
    setupPeerConnection();

    dataChannel = peerConnection->createDataChannel("test");
    setupDataChannel();

    peerConnection->setLocalDescription();

    signalingThread = std::thread(&WebRTCCaller::handleSignaling, this);

    waitForConnection();
}

void WebRTCCaller::sendData(const std::string& msg) {
    if (dataChannel && dataChannel->isOpen()) {
        dataChannel->send(msg);
        std::cout << "[Caller] Sent data: " << msg << std::endl;
    } else {
        std::cerr << "[Caller] Data channel not open yet!" << std::endl;
    }
}

void WebRTCCaller::close() {
    if (signalingThread.joinable()) {
        ::shutdown(sock, SHUT_RDWR);
        signalingThread.join();
    }

    if (sock != -1) {
        ::close(sock);
        sock = -1;
    }

    if (peerConnection) {
        peerConnection->close();
        peerConnection.reset();
    }

    std::cout << "[Caller] Shutdown complete." << std::endl;
}

void WebRTCCaller::setupSocket() {
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) throw std::runtime_error("Failed to create socket");

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, serverIp.c_str(), &addr.sin_addr);

    std::cout << "[Caller] Connecting to signaling server..." << std::endl;
    if (connect(sock, (sockaddr*)&addr, sizeof(addr)) < 0) throw std::runtime_error("Failed to connect");
    std::cout << "[Caller] Connected to signaling server." << std::endl;
}

void WebRTCCaller::setupPeerConnection() {
    peerConnection->onLocalCandidate([this](rtc::Candidate candidate) {
        std::string msg = "candidate:" + std::string(candidate);
        sendMessage(msg);
    });

    peerConnection->onStateChange([this](rtc::PeerConnection::State state) {
        std::cout << "[Caller] State changed: " << stateToString(state) << std::endl;
        connected = (state == rtc::PeerConnection::State::Connected);
        cv.notify_all();
    });

    peerConnection->onLocalDescription([this](rtc::Description description) {
        std::string msg = "offer:" + std::string(description);
        sendMessage(msg);
    });
}

void WebRTCCaller::setupDataChannel() {
    dataChannel->onOpen([this]() {
        std::cout << "[Caller] Data channel open." << std::endl;
        dataChannelReady = true;
        cv.notify_all();
    });

    dataChannel->onMessage([](rtc::message_variant msg) {
        std::visit([](auto&& data) {
            using T = std::decay_t<decltype(data)>;
            if constexpr (std::is_same_v<T, std::string>)
                std::cout << "[Caller] Received message: " << data << std::endl;
            else if constexpr (std::is_same_v<T, rtc::binary>)
                std::cout << "[Caller] Received binary message, size: " << data.size() << std::endl;
        }, msg);
    });
}

void WebRTCCaller::handleSignaling() {
    std::string msg;
    while (receiveMessage(msg)) {
        if (msg.rfind("answer:", 0) == 0 && !answerSet.exchange(true)) {
            std::string sdp = msg.substr(7);
            peerConnection->setRemoteDescription(rtc::Description(sdp, rtc::Description::Type::Answer));
        } else if (msg.rfind("candidate:", 0) == 0) {
            std::string candStr = msg.substr(10);
            peerConnection->addRemoteCandidate(rtc::Candidate(candStr));
        }
    }
}

void WebRTCCaller::waitForConnection() {
    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, std::chrono::seconds(30), [this] { return connected.load(); }))
        std::cerr << "[Caller] Timeout waiting for connection." << std::endl;

    if (!cv.wait_for(lock, std::chrono::seconds(10), [this] { return dataChannelReady.load(); }))
        std::cerr << "[Caller] Timeout waiting for data channel." << std::endl;
}

void WebRTCCaller::sendMessage(const std::string& msg) {
    uint32_t len = htonl(msg.size());
    send(sock, &len, sizeof(len), 0);
    send(sock, msg.data(), msg.size(), 0);
}

bool WebRTCCaller::receiveMessage(std::string& out) {
    uint32_t len = 0;
    ssize_t n = recv(sock, &len, sizeof(len), MSG_WAITALL);
    if (n == 0 || n != sizeof(len)) return false;
    len = ntohl(len);
    out.resize(len);
    return recv(sock, &out[0], len, MSG_WAITALL) == (ssize_t)len;
}

const char* WebRTCCaller::stateToString(rtc::PeerConnection::State state) const {
    switch (state) {
        case rtc::PeerConnection::State::New: return "New";
        case rtc::PeerConnection::State::Connecting: return "Connecting";
        case rtc::PeerConnection::State::Connected: return "Connected";
        case rtc::PeerConnection::State::Disconnected: return "Disconnected";
        case rtc::PeerConnection::State::Failed: return "Failed";
        case rtc::PeerConnection::State::Closed: return "Closed";
        default: return "Unknown";
    }
}
