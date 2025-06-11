#include "GenericSHMPublisher.hpp"

struct MyMsg {
    int id;
    float value;
};

int main() {
    GenericSHMPublisher<MyMsg> pub("/my_generic_shm", sizeof(MyMsg) + 128, [](int count) {
        return MyMsg{count, count * 0.5f};
    });

    pub.run();
    return 0;
}
