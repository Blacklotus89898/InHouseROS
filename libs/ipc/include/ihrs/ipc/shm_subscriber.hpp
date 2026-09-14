#pragma once

#include <atomic>
#include <cstddef>
#include <functional>
#include <string>

namespace ihrs::ipc {

/// Reads short string messages from a POSIX shared-memory segment at a
/// fixed rate and forwards them to a callback. Attaches to a segment
/// created by a SharedMemoryPublisher and waits (polling) until the
/// segment's initialized flag is set.
///
/// run() blocks until stop() is called or SIGINT/SIGTERM is received.
class SharedMemorySubscriber {
public:
    using Callback = std::function<void(const std::string&)>;

    /// Attaches to an existing segment. Throws std::runtime_error if the
    /// segment does not exist or cannot be mapped.
    SharedMemorySubscriber(const char* shm_name = "/default_shm",
                           std::size_t size = 4096,
                           Callback cb = [](const std::string&) {},
                           int rate = 10);

    /// Unmaps the segment (never unlinks; the creator owns its lifetime).
    ~SharedMemorySubscriber();

    SharedMemorySubscriber(const SharedMemorySubscriber&) = delete;
    SharedMemorySubscriber& operator=(const SharedMemorySubscriber&) = delete;

    /// Blocking receive loop at the configured rate (Hz).
    void run();

    /// Requests run() to return. Safe from any thread or a signal handler.
    void stop();

private:
    const char* shm_name_;
    std::size_t shm_size_;
    void* ptr_;
    int fd_;
    Callback callback_;
    int rate_;
    std::atomic<bool> running_{true};

    /// Signal handlers must not touch objects; they only flip this flag.
    static std::atomic<bool> signal_stop_;

    struct SharedData {
        pthread_mutex_t mutex;
        std::atomic<bool> initialized;
        char message[1]; // variable-length payload, cast from the raw segment
    };

    void openSharedMemory();
    void mapSharedMemory();
    void waitForInitialization();
    SharedData* data();
    void cleanup();
    static void signalHandler(int signum);
    void setupSignalHandlers();
};

} // namespace ihrs::ipc
