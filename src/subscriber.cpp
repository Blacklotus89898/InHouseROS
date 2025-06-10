#include <iostream>
#include "SharedMemorySubscriber.hpp"
int main()
{
    const char *shm_name = "/my_shm"; //see it as a topic name, could use threads instead of IPC, but less fault tolerant
    const size_t shm_size = 1024;

    // Publishers
    auto customCallback = [](const std::string &msg)
    { std::cout << "[CustomSubscriber] Received: " + msg << std::endl; };

    SharedMemorySubscriber sub(shm_name, shm_size, customCallback);

    sub.run();

    return 0;
}
