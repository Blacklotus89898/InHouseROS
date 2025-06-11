#include <iostream>
#include "SharedMemoryPublisher.hpp"

int main()
{
    const char *shm_name = "/my_shm";
    const size_t shm_size = 1024;

    // Publishers
    auto customCallback = [](int count)
    { return "[CustomPublisher] Count: " + std::to_string(count); };

    SharedMemoryPublisher pub(shm_name, shm_size, customCallback);

    // Run all in threads - for simulation (risk of failure)
    pub.run();

    // Join threads

    return 0;
}
