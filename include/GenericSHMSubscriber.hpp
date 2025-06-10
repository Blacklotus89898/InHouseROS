#pragma once

#include <functional>
#include <string>
#include <iostream>
#include <atomic>
#include <pthread.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>

template<typename T>
class GenericSHMSubscriber {
public:
    using Callback = std::function<void(const T&)>;

    GenericSHMSubscriber(const char* shm_name, size_t shm_size, Callback cb)
        : shm_name_(shm_name), shm_size_(shm_size), callback_(std::move(cb)) {
        openSharedMemory();
        mapSharedMemory();
        waitUntilInitialized();
    }

    ~GenericSHMSubscriber() {
        if (ptr_) munmap(ptr_, shm_size_);
        if (fd_ != -1) close(fd_);
    }

    void run() {
        while (true) {
            pthread_mutex_lock(&data()->mutex);

            T msg;
            std::memcpy(&msg, &data()->payload, sizeof(T));

            pthread_mutex_unlock(&data()->mutex);

            callback_(msg);
            usleep(100000);  // 10Hz polling rate
        }
    }

private:
    struct SharedData {
        pthread_mutex_t mutex;
        std::atomic<bool> initialized;
        T payload;
    };

    const char* shm_name_;
    size_t shm_size_;
    void* ptr_ = nullptr;
    int fd_ = -1;
    Callback callback_;

    void openSharedMemory() {
        fd_ = shm_open(shm_name_, O_RDWR, 0666);
        if (fd_ == -1) {
            perror("shm_open");
            throw std::runtime_error("Unable to open shared memory");
        }
    }

    void mapSharedMemory() {
        ptr_ = mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (ptr_ == MAP_FAILED) {
            perror("mmap");
            throw std::runtime_error("Failed to mmap shared memory");
        }
    }

    void waitUntilInitialized() {
        while (!data()->initialized.load()) {
            usleep(10000);
        }
    }

    SharedData* data() {
        return reinterpret_cast<SharedData*>(ptr_);
    }
};
