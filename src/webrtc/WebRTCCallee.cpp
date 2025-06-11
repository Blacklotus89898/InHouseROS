#include "WebRTCCallee.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <queue>
#include <stdexcept>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

WebRTCCallee::WebRTCCallee(int signalingPort)
    : SIGNALING_PORT(signalingPort), connected(false), dataChannelReady(false), answerReady(false), server_sock(-1), sock(-1) {
    rtc::InitLogger(rtc::LogLevel::Info);

    config.iceServers.emplace_back("stun:stun.l.google.com:19302");
    config.portRangeBegin = 10000;
    config.portRangeEnd = 10100;

    pc = std::make_shared<rtc::PeerConnection>(config);
    setupPeerConnection();
}

WebRTCCallee::~WebRTCCallee() {
    close(sock);
    close(server_sock);
    if (signalingThread.joinable()) signalingThread.join();
    if (pc) pc->close();
}

void WebRTCCallee::run() {
    setupSignalingServer();
    signalingThread = std::thread([this]() { signalingLoop(); });

    {
        std::unique_lock<std::mutex> lock(mtx);
        if (!cv.wait_for(lock, std::chrono::seconds(30), [&]() { return connected.load(); })) {
            std::cerr << "[Callee] Timeout waiting for connection." << std::endl;
            return;
        }
    }

    if (connected) {
        std::cout << "[Callee] Connection established! Waiting for data channel..." << std::endl;
        {
            std::unique_lock<std::mutex> lock(mtx);
            if (!cv.wait_for(lock, std::chrono::seconds(10), [&]() { return dataChannelReady.load(); })) {
                std::cerr << "[Callee] Timeout waiting for data channel." << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(10));

        // if (dataChannelReady && dc && dc->isOpen()) {
        //     for (int i = 0; i < 5; ++i) {
        //         std::string msg = "Hello from callee #" + std::to_string(i);
        //         dc->send(msg);
        //         std::cout << "[Callee] Sent: " << msg << std::endl;
        //         std::this_thread::sleep_for(std::chrono::seconds(1));
        //     }
        // }
    }

    // std::cout << "[Callee] Shutdown complete." << std::endl;
}

void WebRTCCallee::sendData(const std::string& msg) {
    if (dc && dc->isOpen()) {
        dc->send(msg);
        std::cout << "[Callee] Sent data: " << msg << std::endl;
    } else {
        std::cerr << "[Callee] Data channel not open yet!" << std::endl;
    }
}

void WebRTCCallee::setupPeerConnection() {
    pc->onLocalCandidate([this](rtc::Candidate candidate) {
        sendMessage("candidate:" + std::string(candidate));
        std::cout << "[Callee] Sent ICE candidate to caller." << std::endl;
    });

    pc->onStateChange([this](rtc::PeerConnection::State state) {
        std::cout << "[Callee] State changed: " << stateToString(state) << std::endl;
        connected = (state == rtc::PeerConnection::State::Connected);
        cv.notify_all();
    });

    pc->onLocalDescription([this](rtc::Description description) {
        sendMessage("answer:" + std::string(description));
        std::cout << "[Callee] Sent answer SDP to caller." << std::endl;
        answerReady = true;
        cv.notify_all();
    });

    pc->onDataChannel([this](std::shared_ptr<rtc::DataChannel> channel) {
        std::cout << "[Callee] Data channel received." << std::endl;
        dc = channel;

        dc->onOpen([this]() {
            std::cout << "[Callee] Data channel open." << std::endl;
            dataChannelReady = true;
            cv.notify_all();
        });

        dc->onMessage([this](rtc::message_variant msg) {
            std::visit([this](auto&& data) {
                using T = std::decay_t<decltype(data)>;
                if constexpr (std::is_same_v<T, std::string>) {
                    {
                        std::lock_guard<std::mutex> lock(recvMutex);
                        recvQueue.push(data);
                    }
                    cv.notify_all();

                    std::cout << "[Callee] Received message: " << data << std::endl;
                    std::string response = "Callee echo: " + data;
                    if (dc && dc->isOpen()) {
                        dc->send(response);
                        std::cout << "[Callee] Sent transformed message: " << response << std::endl;
                    }
                } else if constexpr (std::is_same_v<T, rtc::binary>) {
                    std::cout << "[Callee] Received binary message, size: " << data.size() << std::endl;
                }
            }, msg);
        });
    });
}

void WebRTCCallee::setupSignalingServer() {
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) throw std::runtime_error("socket failed");

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(SIGNALING_PORT);

    if (bind(server_sock, (sockaddr*)&addr, sizeof(addr)) < 0) throw std::runtime_error("bind failed");
    if (listen(server_sock, 1) < 0) throw std::runtime_error("listen failed");

    std::cout << "[Callee] Waiting for caller connection..." << std::endl;
    sock = accept(server_sock, nullptr, nullptr);
    if (sock < 0) throw std::runtime_error("accept failed");
    std::cout << "[Callee] Caller connected." << std::endl;
}

void WebRTCCallee::signalingLoop() {
    std::string msg;
    while (receiveMessage(msg)) {
        if (msg.rfind("offer:", 0) == 0) {
            rtc::Description desc(msg.substr(6), rtc::Description::Type::Offer);
            pc->setRemoteDescription(desc);
            std::cout << "[Callee] Received and set offer SDP." << std::endl;
            pc->setLocalDescription();
        } else if (msg.rfind("candidate:", 0) == 0) {
            rtc::Candidate candidate(msg.substr(10));
            pc->addRemoteCandidate(candidate);
            std::cout << "[Callee] Received and added ICE candidate." << std::endl;
        }
    }
}

void WebRTCCallee::sendMessage(const std::string& msg) {
    uint32_t len = htonl(msg.size());
    if (send(sock, &len, sizeof(len), 0) != sizeof(len)) return;
    if (send(sock, msg.data(), msg.size(), 0) != (ssize_t)msg.size()) return;
}

bool WebRTCCallee::receiveMessage(std::string& out) {
    uint32_t len = 0;
    ssize_t n = recv(sock, &len, sizeof(len), MSG_WAITALL);
    if (n <= 0) return false;
    len = ntohl(len);
    out.resize(len);
    return recv(sock, &out[0], len, MSG_WAITALL) == (ssize_t)len;
}

const char* WebRTCCallee::stateToString(rtc::PeerConnection::State state) const {
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

bool WebRTCCallee::tryGetReceivedData(std::string& outMsg) {
    std::lock_guard<std::mutex> lock(recvMutex);
    if (recvQueue.empty()) return false;
    outMsg = std::move(recvQueue.front());
    recvQueue.pop();
    return true;
}

std::string WebRTCCallee::waitAndGetReceivedData() {
    std::unique_lock<std::mutex> lock(recvMutex);
    cv.wait(lock, [this]{ return !recvQueue.empty(); });
    std::string msg = std::move(recvQueue.front());
    recvQueue.pop();
    return msg;
}
