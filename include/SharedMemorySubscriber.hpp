#ifndef SHARED_MEMORY_SUBSCRIBER_HPP
#define SHARED_MEMORY_SUBSCRIBER_HPP

#include <atomic>
#include <string>
#include <functional>
#include <pthread.h>
#include <cstddef>
#include <iostream>

class SharedMemorySubscriber {
public:
    using Callback = std::function<void(const std::string&)>;

    SharedMemorySubscriber(
        const char* shm_name = "/default_shm", 
        size_t size = 4096, 
        Callback cb = [](const std::string& msg) { std::cout << "Received: " << msg << std::endl; }, 
        int rate = 10);
    ~SharedMemorySubscriber();

    void run();

private:
    const char* shm_name_;
    size_t shm_size_;
    void* ptr_;
    int fd_;
    Callback callback_;
    int rate_;

    static SharedMemorySubscriber* instance_;

    struct SharedData {
        pthread_mutex_t mutex;
        std::atomic<bool> initialized;
        char message[1]; // Flexible array member, will cast from raw shm
    };

    void openSharedMemory();
    void mapSharedMemory();
    void waitForInitialization();
    SharedData* data();
    void cleanup();
    static void signalHandler(int);
    void setupSignalHandlers();
};

#endif // SHARED_MEMORY_SUBSCRIBER_HPP
