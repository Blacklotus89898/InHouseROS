// SharedMemoryPublisher.cpp
#include "SharedMemoryPublisher.hpp"
#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include <cstring>
#include <csignal>
#include <atomic>

struct SharedMemoryPublisher::SharedData {
    pthread_mutex_t mutex;
    std::atomic<bool> initialized;
    char message[1024 - sizeof(pthread_mutex_t) - sizeof(std::atomic<bool>)];
};

SharedMemoryPublisher* SharedMemoryPublisher::instance_ = nullptr;

SharedMemoryPublisher::SharedMemoryPublisher(const char* shm_name, size_t size, Callback cb, int rate)  
    : shm_name_(shm_name), shm_size_(size), ptr_(nullptr), fd_(-1), is_creator_(false), callback_(std::move(cb)), rate_(rate) {
    openOrCreateSharedMemory();
    mapSharedMemory();
    initMutexIfCreator();
    setupSignalHandlers();
}

SharedMemoryPublisher::~SharedMemoryPublisher() {
    cleanup();
}

void SharedMemoryPublisher::run() {
    int count = 0;
    int delta = 1000000 / rate_;
    while (true) {
        pthread_mutex_lock(&data()->mutex);

        std::string msg = callback_(count++);
        strncpy(data()->message, msg.c_str(), sizeof(data()->message) - 1);
        data()->message[sizeof(data()->message) - 1] = '\0';

        pthread_mutex_unlock(&data()->mutex);

        std::cout << "[Publisher] Sent: " << msg << std::endl;
        usleep(delta);
    }
}

void SharedMemoryPublisher::openOrCreateSharedMemory() {
    fd_ = shm_open(shm_name_, O_CREAT | O_EXCL | O_RDWR, 0666);
    if (fd_ >= 0) {
        is_creator_ = true;
        ftruncate(fd_, shm_size_);
    } else if (errno == EEXIST) {
        fd_ = shm_open(shm_name_, O_RDWR, 0666);
    }
}

void SharedMemoryPublisher::mapSharedMemory() {
    ptr_ = mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
}

void SharedMemoryPublisher::initMutexIfCreator() {
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

SharedMemoryPublisher::SharedData* SharedMemoryPublisher::data() {
    return reinterpret_cast<SharedData*>(ptr_);
}

void SharedMemoryPublisher::cleanup() {
    if (ptr_) munmap(ptr_, shm_size_);
    if (fd_ != -1) close(fd_);
    if (is_creator_) shm_unlink(shm_name_);
}

void SharedMemoryPublisher::signalHandler(int) {
    if (instance_) instance_->cleanup();
    exit(0);
}

void SharedMemoryPublisher::setupSignalHandlers() {
    instance_ = this;
    struct sigaction sa;
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}
