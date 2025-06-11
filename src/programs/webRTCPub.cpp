#include "WebRTCCallee.hpp"
#include "GenericSHMPublisher.hpp"
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>

struct MyMsg {
    int id;
    float value;
};

// Helper: parse "id:1,value:0.5" into MyMsg
bool parseMsg(const std::string& str, MyMsg& msg) {
    std::istringstream ss(str);
    std::string token;
    while (std::getline(ss, token, ',')) {
        auto pos = token.find(':');
        if (pos == std::string::npos) continue;
        std::string key = token.substr(0, pos);
        std::string val = token.substr(pos + 1);

        try {
            if (key == "id") msg.id = std::stoi(val);
            else if (key == "value") msg.value = std::stof(val);
        } catch (...) {
            return false;
        }
    }
    return true;
}

int main() {
    try {
        WebRTCCallee callee(9000);
        std::thread rtcThread([&callee]() {
            callee.run();
        });

        std::queue<MyMsg> msgQueue;
        std::mutex queueMutex;
        std::condition_variable cv;

        // Start a thread to poll WebRTC received messages from callee and push into queue
        std::atomic<bool> running{true};
        std::thread receiverThread([&]() {
            while (running) {
                std::string rawMsg;
                if (callee.tryGetReceivedData(rawMsg)) {
                    MyMsg parsedMsg{};
                    if (parseMsg(rawMsg, parsedMsg)) {
                        {
                            std::lock_guard<std::mutex> lock(queueMutex);
                            msgQueue.push(parsedMsg);
                        }
                        cv.notify_one();
                        std::cout << "[ReceiverThread] Got message id=" << parsedMsg.id 
                                  << ", value=" << parsedMsg.value << std::endl;
                    } else {
                        std::cerr << "[ReceiverThread] Failed to parse message: " << rawMsg << std::endl;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        // GenericSHMPublisher callback: waits for messages in queue and returns them
        GenericSHMPublisher<MyMsg> pub("/my_generic_shm", sizeof(MyMsg) + 128, [&](int) {
            std::unique_lock<std::mutex> lock(queueMutex);
            if (msgQueue.empty()) {
                // Wait max 100ms for new data
                cv.wait_for(lock, std::chrono::milliseconds(100), [&](){ return !msgQueue.empty(); });
            }
            if (!msgQueue.empty()) {
                MyMsg msg = msgQueue.front();
                msgQueue.pop();
                return msg;
            }
            // No data: return empty message
            return MyMsg{0, 0.0f};
        }, 10); // Publish at 10 Hz

        // Run publisher in this thread (blocking)
        pub.run();

        running = false;
        if (receiverThread.joinable()) receiverThread.join();
        if (rtcThread.joinable()) rtcThread.join();

    } catch (const std::exception& e) {
        std::cerr << "[Main] Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
