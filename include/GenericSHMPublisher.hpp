#pragma once

#include <functional>
#include <string>
#include <atomic>
#include <pthread.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <stdexcept>

template<typename T>
class GenericSHMPublisher {
public:
    using Callback = std::function<T(int)>;

    GenericSHMPublisher(const char* shm_name, size_t shm_size, Callback cb, int rate = 10)
        : shm_name_(shm_name), shm_size_(shm_size), callback_(std::move(cb)), rate_(rate) {
        openOrCreateSharedMemory();
        mapSharedMemory();
        initMutexIfCreator();
    }

    ~GenericSHMPublisher() {
        cleanup();
    }

    void run() {
        int count = 0;
        int delay_us = 1000000 / rate_;

        while (running_) {
            pthread_mutex_lock(&data()->mutex);

            T msg = callback_(count++);
            std::memcpy(&data()->payload, &msg, sizeof(T));

            pthread_mutex_unlock(&data()->mutex);

            std::cout << "[Publisher] Sent message of size " << sizeof(T) << std::endl;
            usleep(delay_us);
        }
    }

    void stop() {
        running_ = false;
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
    bool is_creator_ = false;
    Callback callback_;
    int rate_;
    std::atomic<bool> running_ = true;

    SharedData* data() {
        return reinterpret_cast<SharedData*>(ptr_);
    }

    void openOrCreateSharedMemory() {
        fd_ = shm_open(shm_name_, O_CREAT | O_EXCL | O_RDWR, 0666);
        if (fd_ >= 0) {
            is_creator_ = true;
            ftruncate(fd_, shm_size_);
        } else {
            fd_ = shm_open(shm_name_, O_RDWR, 0666);
            if (fd_ < 0) {
                perror("shm_open");
                throw std::runtime_error("Failed to open shared memory");
            }
        }
    }

    void mapSharedMemory() {
        ptr_ = mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (ptr_ == MAP_FAILED) {
            perror("mmap");
            throw std::runtime_error("Failed to mmap shared memory");
        }
    }

    void initMutexIfCreator() {
        if (is_creator_) {
            pthread_mutexattr_t attr;
            pthread_mutexattr_init(&attr);
            pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
            pthread_mutex_init(&data()->mutex, &attr);
            pthread_mutexattr_destroy(&attr);

            data()->initialized.store(true);
        } else {
            while (!data()->initialized.load()) {
                usleep(10000);
            }
        }
    }

    void cleanup() {
        if (ptr_) {
            munmap(ptr_, shm_size_);
            ptr_ = nullptr;
        }
        if (fd_ != -1) {
            close(fd_);
            fd_ = -1;
        }
        if (is_creator_) {
            if (shm_unlink(shm_name_) == 0) {
                std::cout << "Unlinked shared memory: " << shm_name_ << std::endl;
            } else {
                perror("shm_unlink");
            }
            is_creator_ = false;
        }
    }
};
