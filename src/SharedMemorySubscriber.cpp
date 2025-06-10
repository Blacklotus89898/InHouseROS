#include "SharedMemorySubscriber.hpp"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <signal.h>

SharedMemorySubscriber* SharedMemorySubscriber::instance_ = nullptr;

SharedMemorySubscriber::SharedMemorySubscriber(const char* shm_name, size_t shm_size, Callback cb)
    : shm_name_(shm_name), shm_size_(shm_size), ptr_(nullptr), fd_(-1), callback_(cb) {
    openSharedMemory();
    mapSharedMemory();
    waitForInitialization();
    setupSignalHandlers();
}

SharedMemorySubscriber::~SharedMemorySubscriber() {
    cleanup();
}

void SharedMemorySubscriber::openSharedMemory() {
    fd_ = shm_open(shm_name_, O_RDWR, 0666);
    if (fd_ == -1) {
        perror("shm_open");
        exit(1);
    }
}

void SharedMemorySubscriber::mapSharedMemory() {
    ptr_ = mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (ptr_ == MAP_FAILED) {
        perror("mmap");
        close(fd_);
        exit(1);
    }
}

void SharedMemorySubscriber::waitForInitialization() {
    SharedData* d = data();
    while (!d->initialized.load(std::memory_order_acquire)) {
        usleep(10000);
    }
    std::cout << "Shared memory initialized. Starting subscriber..." << std::endl;
}

void SharedMemorySubscriber::run() {
    SharedData* d = data();

    while (true) {
        if (pthread_mutex_lock(&d->mutex) != 0) {
            perror("pthread_mutex_lock");
            break;
        }

        char buffer[1024];
        strncpy(buffer, d->message, sizeof(buffer));
        buffer[sizeof(buffer) - 1] = '\0';

        if (pthread_mutex_unlock(&d->mutex) != 0) {
            perror("pthread_mutex_unlock");
            break;
        }

        callback_(std::string(buffer));
        sleep(1);
    }
}

SharedMemorySubscriber::SharedData* SharedMemorySubscriber::data() {
    return reinterpret_cast<SharedData*>(ptr_);
}

void SharedMemorySubscriber::cleanup() {
    if (ptr_) {
        munmap(ptr_, shm_size_);
        ptr_ = nullptr;
    }
    if (fd_ != -1) {
        close(fd_);
        fd_ = -1;
    }
}

void SharedMemorySubscriber::signalHandler(int signum) {
    std::cout << "\nCaught signal " << signum << ", cleaning up...\n";
    if (instance_) {
        instance_->cleanup();
    }
    exit(0);
}

void SharedMemorySubscriber::setupSignalHandlers() {
    instance_ = this;
    struct sigaction sa;
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}
