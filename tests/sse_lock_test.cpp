#include <cassert>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

// Exercise the production guard with a host mutex implementing the FreeRTOS API.
using SemaphoreHandle_t = std::mutex *;
constexpr int pdTRUE = 1;
constexpr int portMAX_DELAY = -1;
thread_local SemaphoreHandle_t ownedMutex = nullptr;
int xSemaphoreTake(SemaphoreHandle_t mutex, int wait) {
    if (wait == 0 && ownedMutex == mutex) return 0;
    if (wait == 0 && !mutex->try_lock()) return 0;
    if (wait != 0) mutex->lock();
    ownedMutex = mutex;
    return pdTRUE;
}
void xSemaphoreGive(SemaphoreHandle_t mutex) { ownedMutex = nullptr; mutex->unlock(); }
#include "../src/sse_lock.h"
#include "../src/sse_lifecycle.h"

int main() {
    std::mutex mutex;
    sseMutex = &mutex;
    std::string client = "connected";
    std::atomic<bool> started(false), finished(false), pending(false);
    std::thread worker;
    {
        SSELock lock;
        assert(lock);
        worker = std::thread([&] {
            started = true;
            SSELock lock;
            client.clear();
            finished = true;
        });
        while (!started) std::this_thread::yield();
        SSELock logger(true);
        assert(!logger);
        // Timer callback must not wait for the lifecycle lock.
        requestSSEHeartbeat(pending);
        assert(takeSSEHeartbeat(pending));
        assert(!takeSSEHeartbeat(pending));
        assert(!finished && client == "connected");
    }
    worker.join();
    assert(finished && client.empty());
    SSELock logger(true);
    assert(logger);
}
