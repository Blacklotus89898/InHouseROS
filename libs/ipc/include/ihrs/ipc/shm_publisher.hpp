#pragma once

#include <atomic>
#include <csignal>
#include <cstddef>
#include <functional>
#include <string>

namespace ihrs::ipc {

/// Publishes short string messages into a POSIX shared-memory segment at a
/// fixed rate. The first publisher for a given shm_name creates the segment
/// and its process-shared mutex, and unlinks it on destruction; later
/// publishers attach to the existing segment.
///
/// Message layout (fits in the first 1024 bytes of the segment):
///   [ pthread_mutex_t | std::atomic<bool> initialized | char message[...] ]
///
/// Concurrency: the publisher takes the shared mutex for each write.
/// run() blocks until stop() is called or SIGINT/SIGTERM is received
/// (signals only set a flag; all cleanup happens in the destructor).
class SharedMemoryPublisher {
public:
    using Callback = std::function<std::string(int)>;

    /// Opens or creates the segment and initializes the shared mutex.
    /// Throws std::runtime_error if the segment cannot be opened/mapped.
    SharedMemoryPublisher(const char* shm_name = "/default_shm",
                          std::size_t size = 4096,
                          Callback cb = [](int) { return std::string("default"); },
                          int rate = 10);

    /// Unmaps and (if creator) unlinks the segment.
    ~SharedMemoryPublisher();

    SharedMemoryPublisher(const SharedMemoryPublisher&) = delete;
    SharedMemoryPublisher& operator=(const SharedMemoryPublisher&) = delete;

    /// Blocking publish loop at the configured rate (Hz).
    void run();

    /// Requests run() to return. Safe from any thread or a signal handler.
    void stop();

private:
    static constexpr std::size_t kMessageCapacity = 1024;

    const char* shm_name_;
    std::size_t shm_size_;
    void* ptr_;
    int fd_;
    bool is_creator_;
    Callback callback_;
    int rate_;
    std::atomic<bool> running_{true};

    /// Signal handlers must not touch objects; they only flip this flag.
    static std::atomic<bool> signal_stop_;

    struct SharedData;
    SharedData* data();

    void openOrCreateSharedMemory();
    void mapSharedMemory();
    void initMutexIfCreator();
    void cleanup();
    static void signalHandler(int signum);
    void setupSignalHandlers();
};

} // namespace ihrs::ipc
