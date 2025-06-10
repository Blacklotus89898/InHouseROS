#include "SharedMemorySubscriber.hpp"
#include "TCPSender.hpp"
#include <memory>
#include <iostream>

int main() {
    auto sender = std::make_unique<TCPSender>();
    if (!sender->connect("127.0.0.1", 9001)) {
        std::cerr << "TCP connection failed\n";
        return 1;
    }

    TCPSender* rawSender = sender.get();

    SharedMemorySubscriber subscriber(
        "/my_shm", 1024,
        [rawSender](const std::string& msg) {
            std::cout << "Shared Memory Message: " << msg << std::endl;
            if (rawSender) {
                rawSender->send(msg);  // Send it via TCP
            }
        }
    );

    subscriber.run();  // Blocking call
    return 0;
}
