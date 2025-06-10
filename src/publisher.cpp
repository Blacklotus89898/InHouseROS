#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include <cstring>
#include <signal.h>
#include <atomic>
#include <functional>

const char* SHM_NAME = "/my_shm";
const size_t SHM_SIZE = 1024;

struct SharedData {
    pthread_mutex_t mutex;
    std::atomic<bool> initialized;
    char message[SHM_SIZE - sizeof(pthread_mutex_t) - sizeof(std::atomic<bool>)];
};

class SharedMemoryPublisher {
public:
    using PublishCallback = std::function<std::string(int)>;

    SharedMemoryPublisher(const char* shm_name, size_t size, PublishCallback cb = nullptr)
        : shm_name_(shm_name), shm_size_(size), ptr_(nullptr), fd_(-1),
          is_creator_(false), callback_(cb)
    {
        openOrCreateSharedMemory();
        mapSharedMemory();
        initMutexIfCreator();
        setupSignalHandlers();
    }

    ~SharedMemoryPublisher() {
        cleanup();
    }

    void run() {
        int count = 0;
        while (true) {
            pthread_mutex_lock(&data()->mutex);

            std::string msg;
            if (callback_) {
                msg = callback_(count);
            } else {
                msg = "Hello " + std::to_string(count);
            }
            count++;

            strncpy(data()->message, msg.c_str(), sizeof(data()->message) - 1);
            data()->message[sizeof(data()->message) - 1] = '\0';

            pthread_mutex_unlock(&data()->mutex);

            std::cout << "[Publisher] Sent: " << msg << std::endl;
            sleep(1);
        }
    }

private:
    const char* shm_name_;
    size_t shm_size_;
    void* ptr_;
    int fd_;
    bool is_creator_;
    PublishCallback callback_;

    static SharedMemoryPublisher* instance_;

    void openOrCreateSharedMemory() {
        fd_ = shm_open(shm_name_, O_CREAT | O_EXCL | O_RDWR, 0666);
        if (fd_ >= 0) {
            is_creator_ = true;
            if (ftruncate(fd_, shm_size_) == -1) {
                perror("ftruncate");
                exit(1);
            }
        } else if (errno == EEXIST) {
            fd_ = shm_open(shm_name_, O_RDWR, 0666);
            if (fd_ == -1) {
                perror("shm_open");
                exit(1);
            }
        } else {
            perror("shm_open");
            exit(1);
        }
    }

    void mapSharedMemory() {
        ptr_ = mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (ptr_ == MAP_FAILED) {
            perror("mmap");
            exit(1);
        }
    }

    void initMutexIfCreator() {
        SharedData* d = data();
        if (is_creator_) {
            pthread_mutexattr_t attr;
            pthread_mutexattr_init(&attr);
            pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
            if (pthread_mutex_init(&d->mutex, &attr) != 0) {
                perror("pthread_mutex_init");
                exit(1);
            }
            pthread_mutexattr_destroy(&attr);

            d->initialized.store(true, std::memory_order_release);
        } else {
            while (!d->initialized.load(std::memory_order_acquire)) {
                usleep(10000);
            }
            std::cout << "Memory initialized" << std::endl;
        }
    }

    SharedData* data() {
        return reinterpret_cast<SharedData*>(ptr_);
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
            shm_unlink(shm_name_);
        }
    }

    static void signalHandler(int signum) {
        std::cout << "\nCaught signal " << signum << ", cleaning up...\n";
        if (instance_) {
            instance_->cleanup();
        }
        exit(0);
    }

    void setupSignalHandlers() {
        instance_ = this;
        struct sigaction sa;
        sa.sa_handler = signalHandler;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        sigaction(SIGINT, &sa, nullptr);
        sigaction(SIGTERM, &sa, nullptr);
    }
};

SharedMemoryPublisher* SharedMemoryPublisher::instance_ = nullptr;

int main() {
    auto customCallback = [](int count) -> std::string {
        return "Custom message " + std::to_string(count * 10);
    };

    SharedMemoryPublisher publisher(SHM_NAME, SHM_SIZE, customCallback);
    publisher.run();
    return 0;
}
