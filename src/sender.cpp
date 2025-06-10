#include "SharedMemorySubscriber.hpp"
#include "UDPSender.hpp"  // Replace with UDP sender
#include <memory>
#include <iostream>

int main() {
    auto sender = std::make_unique<UDPSender>();
    if (!sender->connect("127.0.0.1", 9001)) {
        std::cerr << "UDP config failed\n";
        return 1;
    }

    UDPSender* rawSender = sender.get();

    SharedMemorySubscriber subscriber(
        "/my_shm", 1024,
        [rawSender](const std::string& msg) {
            std::cout << "Shared Memory Message: " << msg << std::endl;
            if (rawSender) {
                rawSender->send(msg);  // Send it via UDP (no connect required)
            }
        }
    );

    subscriber.run();
    return 0;
}
