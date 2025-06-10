#include <iostream>
#include "SharedMemorySubscriber.hpp"
int main()
{
    const char *shm_name = "/my_shm";
    const size_t shm_size = 1024;

    // Publishers
    auto customCallback = [](const std::string &msg)
    { std::cout << "[CustomSubscriber] Received: " + msg << std::endl; };

    SharedMemorySubscriber sub(shm_name, shm_size, customCallback);

    // Run all in threads - for simulation (risk of failure)
    sub.run();

    // Join threads

    return 0;
}
