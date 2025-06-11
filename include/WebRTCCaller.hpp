#ifndef WEBRTC_CALLER_HPP
#define WEBRTC_CALLER_HPP

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

class WebRTCCaller {
public:
    WebRTCCaller(const std::string& serverIp, int port);
    void start();                        // Start signaling and WebRTC
    void sendData(const std::string&);   // Send message over data channel
    void close();                        // Graceful shutdown

private:
    std::string serverIp;
    int port;
    int sock;
    std::shared_ptr<rtc::PeerConnection> peerConnection;
    std::shared_ptr<rtc::DataChannel> dataChannel;
    std::atomic<bool> connected;
    std::atomic<bool> dataChannelReady;
    std::atomic<bool> offerReady;
    std::atomic<bool> answerSet;
    std::mutex mtx;
    std::condition_variable cv;
    std::thread signalingThread;

    void setupSocket();
    void setupPeerConnection();
    void setupDataChannel();
    void handleSignaling();
    void waitForConnection();

    void sendMessage(const std::string&);
    bool receiveMessage(std::string&);

    const char* stateToString(rtc::PeerConnection::State) const;
};

#endif // WEBRTC_CALLER_HPP
