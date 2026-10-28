#pragma once

// All subscription fields and clients are protected by this lock. Logger
// broadcasts must never wait: callers can hold the logger's own mutex.
#ifdef ESP8266
static bool sseLocked = false;
#else
static SemaphoreHandle_t sseMutex = NULL;
#endif
class SSELock
{
public:
    explicit SSELock(bool tryOnly = false)
    {
#ifdef ESP8266
        (void)tryOnly;
        acquired = !sseLocked;
        if (acquired) sseLocked = true;
#else
        acquired = sseMutex && xSemaphoreTake(sseMutex, tryOnly ? 0 : portMAX_DELAY) == pdTRUE;
#endif
    }
    ~SSELock()
    {
        if (!acquired) return;
#ifdef ESP8266
        sseLocked = false;
#else
        xSemaphoreGive(sseMutex);
#endif
    }
    explicit operator bool() const { return acquired; }
private:
    bool acquired;
};

