#include "TCPReceiver.hpp"
#include "SharedMemoryPublisher.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <memory>

int main() {
    const char* shm_name = "/my_shm2";
    size_t shm_size = 1024;

    auto latest_tcp_message = std::make_shared<std::string>("");

    // Start the TCP receiver in a separate thread
    TCPReceiver receiver(9001, [latest_tcp_message](const std::string& msg) {
        std::cout << "[TCPReceiver] Received: " << msg << std::endl;
        *latest_tcp_message = msg;
    });

    std::thread tcp_thread([&receiver]() {
        receiver.run();  // Blocking call that accepts clients and reads messages
    });

    // Shared memory publisher callback pulls the latest TCP message
    SharedMemoryPublisher pub(shm_name, shm_size, [latest_tcp_message](int) {
        return *latest_tcp_message;
    });

    pub.run();  // Blocking call that writes repeatedly to shared memory

    tcp_thread.join();
    return 0;
}
