#include <iostream>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include "GenericSHMSubscriber.hpp"
#include "GenericSHMPublisher.hpp"
#include <fcntl.h>    // shm_open
#include <sys/mman.h> // shm_unlink
#include <signal.h>
#include <atomic>

std::atomic<bool> exiting{false};

void cleanup() {
    shm_unlink("/sA");
    shm_unlink("/sB");
    std::cout << "Shared memory unlinked\n";
}

// Signal handler (async-safe)
void handle_signal(int signum) {
    exiting = true;  // mark shutdown
    cleanup();
    _exit(signum);   // async-safe exit
}

void setup_signal_handlers() {
    struct sigaction sa{};
    sa.sa_handler = handle_signal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

struct MyMsg {
    int id;
    float value;
};

// Thread-safe message queue
template<typename T>
class MessageQueue {
    std::queue<T> queue;
    std::mutex mtx;
    std::condition_variable cv;

public:
    void push(const T& msg) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            queue.push(msg);
        }
        cv.notify_one();
    }

    T pop() {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [&] { return !queue.empty(); });
        T val = queue.front();
        queue.pop();
        return val;
    }
};

int main() {
    setup_signal_handlers();

    const size_t shm_size = sizeof(MyMsg) + 128;

    MessageQueue<MyMsg> pubA_queue;
    MessageQueue<MyMsg> pubB_queue;

    // Publishers
    GenericSHMPublisher<MyMsg> pubA("/sB", shm_size, [&](int) {
        return pubA_queue.pop();
    });

    GenericSHMPublisher<MyMsg> pubB("/sA", shm_size, [&](int) {
        return pubB_queue.pop();
    });

    // Seed initial data
    for (int i = 0; i < 5; ++i) {
        pubA_queue.push(MyMsg{ i, i * 1.0f });
        pubB_queue.push(MyMsg{ i + 100, i * 2.5f });
    }

    // Subscribers
    GenericSHMSubscriber<MyMsg> subA("/sA", shm_size, [&](const MyMsg& msg) {
        MyMsg transformed = { msg.id + 100, msg.value * 10.0f };
        std::cout << "[subA->pubA] id=" << msg.id << " → " << transformed.id
                  << ", value=" << msg.value << " → " << transformed.value << std::endl;
        pubA_queue.push(transformed);
    });

    GenericSHMSubscriber<MyMsg> subB("/sB", shm_size, [&](const MyMsg& msg) {
        MyMsg transformed = { msg.id * 2, msg.value + 1.0f };
        std::cout << "[subB->pubB] id=" << msg.id << " → " << transformed.id
                  << ", value=" << msg.value << " → " << transformed.value << std::endl;
        pubB_queue.push(transformed);
    });

    // Start threads
    std::thread t3([&]() { pubA.run(); });
    std::thread t4([&]() { pubB.run(); });

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // give time to publish

    std::thread t1([&]() { subA.run(); });
    std::thread t2([&]() { subB.run(); });

    // Join threads
    t1.join();
    t2.join();
    t3.join();
    t4.join();

    // cleanup(); // Final cleanup if no signal triggered

    return 0;
}
