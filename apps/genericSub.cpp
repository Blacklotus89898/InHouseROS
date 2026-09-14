#include <ihrs/ipc/generic_shm_subscriber.hpp>

using namespace ihrs::ipc;

struct MyMsg {
    int id;
    float value;
};

int main() {
    GenericSHMSubscriber<MyMsg> sub("/my_generic_shm", sizeof(MyMsg) + 128, [](const MyMsg& msg) {
        std::cout << "[Subscriber] Got id=" << msg.id << ", value=" << msg.value << "\n";
    });

    sub.run();
    return 0;
}
