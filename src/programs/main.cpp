#include <thread>
#include <iostream>
#include "SharedMemoryPublisher.hpp"
#include "SharedMemorySubscriber.hpp"

int main() {
    const char* shm_name = "/my_shm";
    const size_t shm_size = 1024;

    // Publishers
    auto pubCallback1 = [](int count) { return "[Pub1] Count: " + std::to_string(count); };
    auto pubCallback2 = [](int count) { return "[Pub2] Msg: " + std::to_string(count * 2); };

    SharedMemoryPublisher pub1(shm_name, shm_size, pubCallback1);
    SharedMemoryPublisher pub2(shm_name, shm_size, pubCallback2);

    // Subscribers
    SharedMemorySubscriber sub1(shm_name, shm_size, [](const std::string& msg) {
        std::cout << "Sub1 received: " << msg << std::endl;
    });

    SharedMemorySubscriber sub2(shm_name, shm_size, [](const std::string& msg) {
        std::cout << "Sub2 received: " << msg << std::endl;
    });

    // Run all in threads - for simulation (risk of failure)
    std::thread t1([&]() { pub1.run(); });
    std::thread t3([&]() { sub1.run(); });
    std::thread t2([&]() { pub2.run(); });
    std::thread t4([&]() { sub2.run(); });

    // Join threads
    t1.join();
    t2.join();
    t3.join();
    t4.join();

    return 0;
}
