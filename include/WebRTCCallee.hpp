#pragma once

#include <rtc/rtc.hpp>
#include <string>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <memory>

class WebRTCCallee {
public:
    explicit WebRTCCallee(int signalingPort = 9000);
    ~WebRTCCallee();

    void run();
    void sendData(const std::string& msg);
    bool tryGetReceivedData(std::string& outMsg);
    std::string waitAndGetReceivedData();

private:
    const int SIGNALING_PORT;
    rtc::Configuration config;
    std::shared_ptr<rtc::PeerConnection> pc;
    std::shared_ptr<rtc::DataChannel> dc;

    std::atomic<bool> connected;
    std::atomic<bool> dataChannelReady;
    std::atomic<bool> answerReady;

    int server_sock;
    int sock;

    std::mutex mtx;
    std::condition_variable cv;
    std::thread signalingThread;

    std::mutex recvMutex;
    std::queue<std::string> recvQueue;

    void setupPeerConnection();
    void setupSignalingServer();
    void signalingLoop();
    void sendMessage(const std::string& msg);
    bool receiveMessage(std::string& out);
    const char* stateToString(rtc::PeerConnection::State state) const;
};
