#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include <cstring>
#include <atomic>
#include <signal.h>
#include <functional> // for std::function

const char* SHM_NAME = "/my_shm";
const size_t SHM_SIZE = 1024;

struct SharedData {
    pthread_mutex_t mutex;
    std::atomic<bool> initialized;
    char message[SHM_SIZE - sizeof(pthread_mutex_t) - sizeof(std::atomic<bool>)];
};

class SharedMemorySubscriber {
public:
    using CallbackType = std::function<void(const char*)>;

    SharedMemorySubscriber(const char* shm_name, size_t shm_size, CallbackType callback = nullptr)
        : shm_name_(shm_name), shm_size_(shm_size), ptr_(nullptr), fd_(-1), running_(true), callback_(callback)
    {
        openSharedMemory();
        mapSharedMemory();
        waitForInitialization();
        setupSignalHandlers();
    }

    ~SharedMemorySubscriber() {
        cleanup();
    }

    // Can modify callback at run time
    void setCallback(CallbackType callback) {
        callback_ = callback;
    }

    void run() {
        SharedData* data = getData();

        while (running_) {
            if (pthread_mutex_lock(&data->mutex) != 0) {
                perror("pthread_mutex_lock");
                break;
            }

            char buffer[sizeof(data->message)];
            strncpy(buffer, data->message, sizeof(buffer));
            buffer[sizeof(buffer) - 1] = '\0';

            if (pthread_mutex_unlock(&data->mutex) != 0) {
                perror("pthread_mutex_unlock");
                break;
            }

            if (callback_) {
                callback_(buffer); // call user-defined callback
            } else {
                // Default action if no callback provided:
                std::cout << "[Subscriber] Received: " << buffer << std::endl;
            }

            sleep(1);
        }
        std::cout << "Subscriber exiting cleanly." << std::endl;
    }

private:
    const char* shm_name_;
    size_t shm_size_;
    void* ptr_;
    int fd_;
    volatile sig_atomic_t running_;
    CallbackType callback_;

    static SharedMemorySubscriber* instance_;

    void openSharedMemory() {
        fd_ = shm_open(shm_name_, O_RDWR, 0666);
        if (fd_ == -1) {
            perror("shm_open");
            exit(1);
        }
    }

    void mapSharedMemory() {
        ptr_ = mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (ptr_ == MAP_FAILED) {
            perror("mmap");
            close(fd_);
            exit(1);
        }
    }

    void waitForInitialization() {
        SharedData* data = getData();
        while (!data->initialized.load(std::memory_order_acquire)) {
            usleep(10000);
        }
        std::cout << "Shared memory initialized. Starting subscriber..." << std::endl;
    }

    SharedData* getData() {
        return reinterpret_cast<SharedData*>(ptr_);
    }

    void cleanup() {
        if (ptr_ != nullptr) {
            munmap(ptr_, shm_size_);
            ptr_ = nullptr;
        }
        if (fd_ != -1) {
            close(fd_);
            fd_ = -1;
        }
    }

    static void signalHandler(int) {
        if (instance_) {
            instance_->running_ = 0;
        }
    }

    void setupSignalHandlers() {
        instance_ = this;
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
    }
};

SharedMemorySubscriber* SharedMemorySubscriber::instance_ = nullptr;

int main() {

    // Modular callback function
    auto customAction = [](const char* msg) {
        std::cout << "Custom processing of message: " << msg << std::endl;
    };

    SharedMemorySubscriber subscriber(SHM_NAME, SHM_SIZE);
    subscriber.setCallback(customAction);
    subscriber.run();
    return 0;
}
