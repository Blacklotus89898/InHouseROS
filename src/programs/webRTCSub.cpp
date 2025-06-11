#include "WebRTCCallee.hpp"
#include "GenericSHMSubscriber.hpp"
#include <iostream>
#include <sstream>
#include <thread>
#include <atomic>

struct MyMsg {
    int id;
    float value;
};

// Serialize MyMsg to a string (e.g., "id:1,value:0.5")
std::string serializeMsg(const MyMsg& msg) {
    std::ostringstream oss;
    oss << "id:" << msg.id << ",value:" << msg.value;
    return oss.str();
}

int main() {
    try {
        WebRTCCallee callee(9000);
        std::thread rtcThread([&callee]() {
            callee.run();
        });

        std::atomic<bool> running = true;

        GenericSHMSubscriber<MyMsg> sub("/my_generic_shm", sizeof(MyMsg) + 128,
            [&callee](const MyMsg& msg) {
                std::cout << "[Subscriber] Got id=" << msg.id << ", value=" << msg.value << "\n";
                std::string serialized = serializeMsg(msg);
                callee.sendData(serialized);  // Send via data channel
            });

        sub.run(); // blocking

        if (rtcThread.joinable()) rtcThread.join();
    } catch (const std::exception& e) {
        std::cerr << "[Callee] Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
