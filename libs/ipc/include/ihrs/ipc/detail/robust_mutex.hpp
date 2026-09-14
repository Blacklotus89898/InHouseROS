#pragma once

#include <pthread.h>

namespace ihrs::ipc::detail {

/// Robust-mutex support for POSIX shared memory.
///
/// A robust mutex reports EOWNERDEAD to the next locker if a process (or
/// thread) died while holding it — instead of deadlocking forever. The
/// recovering locker MUST call pthread_mutex_consistent() before unlocking;
/// the data guarded by the mutex may be torn from the interrupted write,
/// which callers handle according to their protocol (see below).

enum class LockStatus {
    Acquired,                  // normal lock
    RecoveredAfterOwnerDeath,  // EOWNERDEAD: lock acquired, marked consistent;
                               // guarded data may be torn
    Error,                     // lock failed, mutex NOT held
};

inline void set_robust_attr(pthread_mutexattr_t& attr) {
#if defined(PTHREAD_MUTEX_ROBUST)
    pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
#else
    (void)attr; // platform without robust mutexes: keep plain behavior
#endif
}

inline LockStatus lock(pthread_mutex_t& m) {
    const int rc = pthread_mutex_lock(&m);
#if defined(PTHREAD_MUTEX_ROBUST)
    if (rc == EOWNERDEAD) {
        pthread_mutex_consistent(&m);
        return LockStatus::RecoveredAfterOwnerDeath;
    }
#endif
    return rc == 0 ? LockStatus::Acquired : LockStatus::Error;
}

} // namespace ihrs::ipc::detail
