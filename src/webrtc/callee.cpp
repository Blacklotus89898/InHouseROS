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

bool sendMessage(int sock, const std::string& msg) {
    uint32_t len = htonl(msg.size());
    if (send(sock, &len, sizeof(len), 0) != sizeof(len)) return false;
    if (send(sock, msg.data(), msg.size(), 0) != (ssize_t)msg.size()) return false;
    return true;
}

bool receiveMessage(int sock, std::string& out) {
    uint32_t len = 0;
    ssize_t n = recv(sock, &len, sizeof(len), MSG_WAITALL);
    if (n == 0) return false;
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

        auto callee = std::make_shared<rtc::PeerConnection>(config);

        std::mutex mtx;
        std::condition_variable cv;
        std::atomic<bool> connected{false};
        std::atomic<bool> dataChannelReady{false};
        std::atomic<bool> answerReady{false};

        int server_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (server_sock < 0) {
            perror("socket");
            return 1;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(SIGNALING_PORT);

        if (bind(server_sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
            perror("bind");
            return 1;
        }

        if (listen(server_sock, 1) < 0) {
            perror("listen");
            return 1;
        }

        std::cout << "[Callee] Waiting for caller connection..." << std::endl;
        int sock = accept(server_sock, nullptr, nullptr);
        if (sock < 0) {
            perror("accept");
            return 1;
        }
        std::cout << "[Callee] Caller connected." << std::endl;

        callee->onLocalCandidate([&](rtc::Candidate candidate) {
            std::string candStr = std::string(candidate);
            std::string msg = "candidate:" + candStr;
            sendMessage(sock, msg);
            std::cout << "[Callee] Sent ICE candidate to caller." << std::endl;
        });

        callee->onStateChange([&](rtc::PeerConnection::State state) {
            std::cout << "[Callee] State changed: " << stateToString(state) << std::endl;
            connected = (state == rtc::PeerConnection::State::Connected);
            cv.notify_all();
        });

        callee->onLocalDescription([&](rtc::Description description) {
            std::string answerSdp = std::string(description);
            std::string msg = "answer:" + answerSdp;
            sendMessage(sock, msg);
            std::cout << "[Callee] Sent answer SDP to caller." << std::endl;
            answerReady = true;
            cv.notify_all();
        });

        std::shared_ptr<rtc::DataChannel> dc;

        callee->onDataChannel([&](std::shared_ptr<rtc::DataChannel> channel) {
            std::cout << "[Callee] Data channel received." << std::endl;
            dc = channel;

            dc->onOpen([&]() {
                std::cout << "[Callee] Data channel open." << std::endl;
                dataChannelReady = true;
                cv.notify_all();
            });

            dc->onMessage([&](rtc::message_variant msg) {
                std::visit([&](auto&& data) {
                    using T = std::decay_t<decltype(data)>;
                    if constexpr(std::is_same_v<T, std::string>) {
                        std::cout << "[Callee] Received message: " << data << std::endl;
            
                        std::string response = "Callee echo: " + data;
            
                        // Send it back
                        if (dc && dc->isOpen()) {
                            dc->send(response);
                            std::cout << "[Callee] Sent transformed message: " << response << std::endl;
                        }
                    }
                        
                    else if constexpr(std::is_same_v<T, rtc::binary>)
                        std::cout << "[Callee] Received binary message, size: " << data.size() << std::endl;
                }, msg);
            });
        });

        // Thread to receive signaling messages from caller
        std::thread signalingThread([&]() {
            std::string msg;
            while (receiveMessage(sock, msg)) {
                if (msg.rfind("offer:", 0) == 0) {
                    std::string sdp = msg.substr(6);
                    rtc::Description desc(sdp, rtc::Description::Type::Offer);
                    callee->setRemoteDescription(desc);
                    std::cout << "[Callee] Received and set offer SDP." << std::endl;
                    // Now create answer
                    callee->setLocalDescription();
                } else if (msg.rfind("candidate:", 0) == 0) {
                    std::string candStr = msg.substr(10);
                    rtc::Candidate candidate(candStr);
                    callee->addRemoteCandidate(candidate);
                    std::cout << "[Callee] Received and added ICE candidate." << std::endl;
                }
            }
        });

        // Wait for connection
        {
            std::unique_lock<std::mutex> lock(mtx);
            if (!cv.wait_for(lock, std::chrono::seconds(30), [&]() { return connected.load(); })) {
                std::cerr << "[Callee] Timeout waiting for connection." << std::endl;
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

            if (dataChannelReady) {
                // Send some test messages
                for (int i = 0; i < 5; i++) {
                    if (dc && dc->isOpen()) {
                        std::string msg = "Hello from callee #" + std::to_string(i);
                        dc->send(msg);
                        std::cout << "[Callee] Sent: " << msg << std::endl;
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                }
            }
        }

        signalingThread.join();
        close(sock);
        close(server_sock);
        callee->close();
        std::cout << "[Callee] Shutdown complete." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[Callee] Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
