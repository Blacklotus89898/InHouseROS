#pragma once
#include <string>
#include <functional>

class SharedMemoryPublisher {
public:
    using Callback = std::function<std::string(int)>;

    SharedMemoryPublisher(const char* shm_name, size_t size, Callback cb);
    ~SharedMemoryPublisher();

    void run();

private:
    const char* shm_name_;
    size_t shm_size_;
    void* ptr_;
    int fd_;
    bool is_creator_;
    Callback callback_;

    static SharedMemoryPublisher* instance_;

    struct SharedData;
    SharedData* data();

    void openOrCreateSharedMemory();
    void mapSharedMemory();
    void initMutexIfCreator();
    void cleanup();
    void setupSignalHandlers();
    static void signalHandler(int signum);
};
