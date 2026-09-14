#include "ihrs/ipc/shm_subscriber.hpp"

#include "ihrs/ipc/detail/robust_mutex.hpp"

#include <pthread.h>

#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <iostream>
#include <stdexcept>

namespace ihrs::ipc {

std::atomic<bool> SharedMemorySubscriber::signal_stop_{false};

SharedMemorySubscriber::SharedMemorySubscriber(const char* shm_name, std::size_t size, Callback cb, int rate)
    : shm_name_(shm_name), shm_size_(size), ptr_(nullptr), fd_(-1),
      callback_(std::move(cb)), rate_(rate > 0 ? rate : 1) {
    openSharedMemory();
    mapSharedMemory();
    waitForInitialization();
    setupSignalHandlers();
}

SharedMemorySubscriber::~SharedMemorySubscriber() {
    stop();
    cleanup();
}

void SharedMemorySubscriber::openSharedMemory() {
    fd_ = shm_open(shm_name_, O_RDWR, 0666);
    if (fd_ == -1) {
        throw std::runtime_error(std::string("shm_open failed: ") + std::strerror(errno));
    }
}

void SharedMemorySubscriber::mapSharedMemory() {
    ptr_ = mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (ptr_ == MAP_FAILED) {
        ptr_ = nullptr;
        throw std::runtime_error(std::string("mmap failed: ") + std::strerror(errno));
    }
}

void SharedMemorySubscriber::waitForInitialization() {
    while (!data()->initialized.load(std::memory_order_acquire)) {
        usleep(10'000);
    }
}

void SharedMemorySubscriber::run() {
    SharedData* d = data();
    const auto delta = static_cast<useconds_t>(1'000'000 / rate_);

    while (running_.load(std::memory_order_relaxed) &&
           !signal_stop_.load(std::memory_order_relaxed)) {
        const auto lock_rc = detail::lock(d->mutex);
        if (lock_rc == detail::LockStatus::Error) {
            break;
        }
        if (lock_rc == detail::LockStatus::RecoveredAfterOwnerDeath) {
            // Payload may be torn from the dead owner's interrupted write;
            // do not deliver it. Unlock and wait for a fresh publish.
            pthread_mutex_unlock(&d->mutex);
            usleep(delta);
            continue;
        }

        char buffer[1024];
        std::strncpy(buffer, d->message, sizeof(buffer) - 1);
        buffer[sizeof(buffer) - 1] = '\0';

        pthread_mutex_unlock(&d->mutex);

        callback_(std::string(buffer));
        usleep(delta);
    }
}

void SharedMemorySubscriber::stop() {
    running_.store(false, std::memory_order_relaxed);
}

SharedMemorySubscriber::SharedData* SharedMemorySubscriber::data() {
    return static_cast<SharedData*>(ptr_);
}

void SharedMemorySubscriber::cleanup() {
    if (ptr_ != nullptr) {
        munmap(ptr_, shm_size_);
        ptr_ = nullptr;
    }
    if (fd_ != -1) {
        close(fd_);
        fd_ = -1;
    }
}

void SharedMemorySubscriber::signalHandler(int) {
    // Async-signal-safe: only an atomic store. Actual cleanup happens in
    // the destructor once run() observes the flag and returns.
    signal_stop_.store(true, std::memory_order_relaxed);
}

void SharedMemorySubscriber::setupSignalHandlers() {
    struct sigaction sa;
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

} // namespace ihrs::ipc
