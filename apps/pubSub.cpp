#include <iostream>
#include <thread>
#include <ihrs/ipc/generic_shm_subscriber.hpp>
#include <ihrs/ipc/generic_shm_publisher.hpp> 

using namespace ihrs::ipc;

struct MyMsg {
    int id;
    float value;
};

int main()
{
        GenericSHMPublisher<MyMsg> pub("/my_generic_shm", sizeof(MyMsg) + 128, [](int count) {
        return MyMsg{count, static_cast<float>(count) * 0.5f};
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
