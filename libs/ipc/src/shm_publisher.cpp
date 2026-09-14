#include "ihrs/ipc/shm_publisher.hpp"

#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <pthread.h>
#include <sys/mman.h>
#include <unistd.h>

#include <iostream>
#include <stdexcept>

namespace ihrs::ipc {

struct SharedMemoryPublisher::SharedData {
    pthread_mutex_t mutex;
    std::atomic<bool> initialized;
    char message[kMessageCapacity];
};

std::atomic<bool> SharedMemoryPublisher::signal_stop_{false};

SharedMemoryPublisher::SharedMemoryPublisher(const char* shm_name, std::size_t size, Callback cb, int rate)
    : shm_name_(shm_name), shm_size_(size), ptr_(nullptr), fd_(-1), is_creator_(false),
      callback_(std::move(cb)), rate_(rate > 0 ? rate : 1) {
    openOrCreateSharedMemory();
    mapSharedMemory();
    initMutexIfCreator();
    setupSignalHandlers();
}

SharedMemoryPublisher::~SharedMemoryPublisher() {
    stop();
    cleanup();
}

void SharedMemoryPublisher::run() {
    int count = 0;
    const auto delta = static_cast<useconds_t>(1'000'000 / rate_);

    while (running_.load(std::memory_order_relaxed) &&
           !signal_stop_.load(std::memory_order_relaxed)) {
        pthread_mutex_lock(&data()->mutex);

        const std::string msg = callback_(count++);
        std::strncpy(data()->message, msg.c_str(), sizeof(data()->message) - 1);
        data()->message[sizeof(data()->message) - 1] = '\0';

        pthread_mutex_unlock(&data()->mutex);

        std::cout << "[Publisher] Sent: " << msg << std::endl;
        usleep(delta);
    }
}

void SharedMemoryPublisher::stop() {
    running_.store(false, std::memory_order_relaxed);
}

void SharedMemoryPublisher::openOrCreateSharedMemory() {
    fd_ = shm_open(shm_name_, O_CREAT | O_EXCL | O_RDWR, 0666);
    if (fd_ >= 0) {
        is_creator_ = true;
        if (ftruncate(fd_, static_cast<off_t>(shm_size_)) != 0) {
            throw std::runtime_error("ftruncate failed for shared memory segment");
        }
    } else if (errno == EEXIST) {
        fd_ = shm_open(shm_name_, O_RDWR, 0666);
    }
    if (fd_ < 0) {
        throw std::runtime_error(std::string("shm_open failed: ") + std::strerror(errno));
    }
}

void SharedMemoryPublisher::mapSharedMemory() {
    ptr_ = mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (ptr_ == MAP_FAILED) {
        ptr_ = nullptr;
        throw std::runtime_error(std::string("mmap failed: ") + std::strerror(errno));
    }
}

void SharedMemoryPublisher::initMutexIfCreator() {
    if (is_creator_) {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        pthread_mutex_init(&data()->mutex, &attr);
        pthread_mutexattr_destroy(&attr);

        data()->initialized.store(true, std::memory_order_release);
    } else {
        while (!data()->initialized.load(std::memory_order_acquire)) {
            usleep(10'000);
        }
    }
}

SharedMemoryPublisher::SharedData* SharedMemoryPublisher::data() {
    return static_cast<SharedData*>(ptr_);
}

void SharedMemoryPublisher::cleanup() {
    if (ptr_ != nullptr) {
        munmap(ptr_, shm_size_);
        ptr_ = nullptr;
    }
    if (fd_ != -1) {
        close(fd_);
        fd_ = -1;
    }
    if (is_creator_) {
        shm_unlink(shm_name_);
        is_creator_ = false;
    }
}

void SharedMemoryPublisher::signalHandler(int) {
    // Async-signal-safe: only an atomic store. Actual cleanup happens in
    // the destructor once run() observes the flag and returns.
    signal_stop_.store(true, std::memory_order_relaxed);
}

void SharedMemoryPublisher::setupSignalHandlers() {
    struct sigaction sa;
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

} // namespace ihrs::ipc
