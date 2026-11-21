#pragma once
#include <atomic>

// A HardwareSerial instance owns a driver, locks and a task. Never copy one to
// redirect a console. Rev1 instead releases UART0 before I2C takes GPIO1/3.
class SerialConsoleState
{
    std::atomic<bool> enabled{true};
    std::atomic<bool> starting{false};

public:
    bool isEnabled() const { return enabled.load(); }
    bool isReady() const { return enabled.load() && !starting.load(); }
    void disable() { enabled.store(false); }

    template <class Port> void ensureStarted(Port &port)
    {
        if (!enabled.load() || starting.exchange(true))
            return;
        // UART begin/end can log from inside their own locks. A nested log must
        // not recursively initialize the same UART, or spin if startup fails.
        if (enabled.load() && !port)
            port.begin(115200);
        starting.store(false);
    }

    template <class Port> void releaseForI2C(Port &port)
    {
        disable(); // Must precede end(): teardown itself may emit a log.
        port.end();
    }

    template <class Port> void resumeAfterI2C(Port &port)
    {
        enabled.store(true);
        ensureStarted(port);
    }
};
