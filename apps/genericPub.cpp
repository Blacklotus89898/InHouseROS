#include <ihrs/ipc/generic_shm_publisher.hpp>

using namespace ihrs::ipc;

struct MyMsg {
    int id;
    float value;
};

int main() {
    GenericSHMPublisher<MyMsg> pub("/my_generic_shm", sizeof(MyMsg) + 128, [](int count) {
        return MyMsg{count, static_cast<float>(count) * 0.5f};
    });

    pub.run();
    return 0;
}
