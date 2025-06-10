#include <rtc/rtc.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

const int SIGNALING_PORT = 9000;

const char* stateToString(rtc::PeerConnection::State state) {
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

// Simple send string over socket with length prefix
bool sendMessage(int sock, const std::string& msg) {
    uint32_t len = htonl(msg.size());
    if (send(sock, &len, sizeof(len), 0) != sizeof(len)) return false;
    if (send(sock, msg.data(), msg.size(), 0) != (ssize_t)msg.size()) return false;
    return true;
}

// Simple receive string from socket with length prefix
bool receiveMessage(int sock, std::string& out) {
    uint32_t len = 0;
    ssize_t n = recv(sock, &len, sizeof(len), MSG_WAITALL);
    if (n == 0) return false; // connection closed
    if (n != sizeof(len)) return false;
    len = ntohl(len);
    out.resize(len);
    n = recv(sock, &out[0], len, MSG_WAITALL);
    if (n != (ssize_t)len) return false;
    return true;
}

int main() {
    try {
        rtc::InitLogger(rtc::LogLevel::Info);

        rtc::Configuration config;
        config.iceServers.emplace_back("stun:stun.l.google.com:19302");
        config.portRangeBegin = 10000;
        config.portRangeEnd = 10100;

        auto caller = std::make_shared<rtc::PeerConnection>(config);

        std::mutex mtx;
        std::condition_variable cv;
        std::atomic<bool> connected{false};
        std::atomic<bool> dataChannelReady{false};
        std::atomic<bool> offerReady{false};
        std::atomic<bool> answerSet{false};

        std::queue<rtc::Candidate> candidates;
        std::mutex candidatesMtx;

        // Setup TCP socket client to signaling server (callee listens)
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            perror("socket");
            return 1;
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(SIGNALING_PORT);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

        std::cout << "[Caller] Connecting to signaling server..." << std::endl;
        if (connect(sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
            perror("connect");
            return 1;
        }
        std::cout << "[Caller] Connected to signaling server." << std::endl;

        caller->onLocalCandidate([&](rtc::Candidate candidate) {
            std::string candStr = std::string(candidate);
            std::string msg = "candidate:" + candStr;
            {
                std::lock_guard<std::mutex> lock(candidatesMtx);
                candidates.push(candidate);
            }
            sendMessage(sock, msg);
            std::cout << "[Caller] Sent ICE candidate to callee." << std::endl;
        });

        caller->onStateChange([&](rtc::PeerConnection::State state) {
            std::cout << "[Caller] State changed: " << stateToString(state) << std::endl;
            connected = (state == rtc::PeerConnection::State::Connected);
            cv.notify_all();
        });

        caller->onLocalDescription([&](rtc::Description description) {
            std::string offerSdp = std::string(description);
            std::string msg = "offer:" + offerSdp;
            sendMessage(sock, msg);
            std::cout << "[Caller] Sent offer to callee." << std::endl;
            offerReady = true;
            cv.notify_all();
        });

        // Create data channel
        auto dc = caller->createDataChannel("test");
        dc->onOpen([&]() {
            std::cout << "[Caller] Data channel open." << std::endl;
            dataChannelReady = true;
            cv.notify_all();
        });
        dc->onMessage([](rtc::message_variant msg) {
            std::visit([](auto&& data) {
                using T = std::decay_t<decltype(data)>;
                if constexpr(std::is_same_v<T, std::string>)
                    std::cout << "[Caller] Received message: " << data << std::endl;
                else if constexpr(std::is_same_v<T, rtc::binary>)
                    std::cout << "[Caller] Received binary message, size: " << data.size() << std::endl;
            }, msg);
        });

        // Create offer
        caller->setLocalDescription();

        // Thread to receive signaling messages
        std::thread signalingThread([&]() {
            std::string msg;
            std::atomic<bool> answerSet{false};

            while (receiveMessage(sock, msg)) {
                if (msg.rfind("answer:", 0) == 0) {
                    if (!answerSet.exchange(true)) {  // Only set once
                        std::string sdp = msg.substr(7);
                        rtc::Description desc(sdp, rtc::Description::Type::Answer);
                        caller->setRemoteDescription(desc);
                        std::cout << "[Caller] Received and set answer SDP." << std::endl;
                    } else {
                        std::cout << "[Caller] Duplicate answer SDP received, ignoring." << std::endl;
                    }
                } else if (msg.rfind("candidate:", 0) == 0) {
                    std::string candStr = msg.substr(10);
                    rtc::Candidate candidate(candStr);
                    caller->addRemoteCandidate(candidate);
                    std::cout << "[Caller] Received and added ICE candidate." << std::endl;
                }
            }
            
        });

        // Wait for connection
        {
            std::unique_lock<std::mutex> lock(mtx);
            if (!cv.wait_for(lock, std::chrono::seconds(30), [&]() { return connected.load(); })) {
                std::cerr << "[Caller] Timeout waiting for connection." << std::endl;
            }
        }

        if (connected) {
            std::cout << "[Caller] Connection established! Waiting for data channel..." << std::endl;
            {
                std::unique_lock<std::mutex> lock(mtx);
                if (!cv.wait_for(lock, std::chrono::seconds(10), [&]() { return dataChannelReady.load(); })) {
                    std::cerr << "[Caller] Timeout waiting for data channel." << std::endl;
                }
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(10)); 

            if (dataChannelReady) {
                // Send some test messages
                for (int i = 0; i < 5; i++) {
                    std::string msg = "Hello from caller #" + std::to_string(i);
                    dc->send(msg);
                    std::cout << "[Caller] Sent: " << msg << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }

        signalingThread.join();
        close(sock);
        caller->close();
        std::cout << "[Caller] Shutdown complete." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[Caller] Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
