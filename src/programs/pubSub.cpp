#include <iostream>
#include <thread>
#include "GenericSHMSubscriber.hpp" 
#include "GenericSHMPublisher.hpp" 

struct MyMsg {
    int id;
    float value;
};

int main()
{
    const char *shm_name = "/my_shm";
    const size_t shm_size = 1024;
    
    GenericSHMPublisher<MyMsg> pub("/my_generic_shm", sizeof(MyMsg) + 128, [](int count) {
        return MyMsg{count, count * 0.5f};
    });
    
    GenericSHMSubscriber<MyMsg> sub("/my_generic_shm", sizeof(MyMsg) + 128, [](const MyMsg& msg) {
        std::cout << "[Subscriber] Got id=" << msg.id << ", value=" << msg.value << "\n";
    });

    // Launch threads
    std::thread t1([&]() { pub.run(); });
    std::thread t2([&]() { sub.run(); });

    // Join threads
    t1.join();
    t2.join();

    return 0;
}
