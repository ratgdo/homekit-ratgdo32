#include "gdo_uart.h"
#include <array>
#include <cassert>

struct Pin
{
    bool latch_high = true; // A previous output may leave a stale GPIO latch.
    bool uart_selected = true;
    bool uart_high = false;
    bool output_enabled = true;
    bool pullup = false;
    bool pulldown = false;

    bool active() const
    {
        if (output_enabled)
            return uart_selected ? uart_high : latch_high;
        return pullup && !pulldown;
    }
};

struct Pins
{
    std::array<Pin, 40> state;
    bool reject_rx_bias = false;
    unsigned assertions = 0;

    void observe(int pin, bool before)
    {
        if (!before && state[pin].active()) ++assertions;
    }
    void preload_low(int pin) { bool before = state[pin].active(); state[pin].latch_high = false; observe(pin, before); }
    void select_gpio(int pin) { bool before = state[pin].active(); state[pin].uart_selected = false; observe(pin, before); }
    void route_gpio(int pin) { select_gpio(pin); }
    void disable_tx_pullup(int pin) { state[pin].pullup = false; }
    void enable_tx_pulldown(int pin) { state[pin].pulldown = true; }
    void enable_output(int pin) { bool before = state[pin].active(); state[pin].output_enabled = true; observe(pin, before); }
    bool disable_rx_pulldown(int pin)
    {
        if (reject_rx_bias) return false;
        state[pin].pulldown = false;
        return true;
    }
    bool enable_rx_pullup(int pin) { state[pin].pullup = true; return true; }
    // ESP-IDF's documented gpio_reset_pin() state: pull-up, no input/output.
    void reset(int pin)
    {
        state[pin].uart_selected = false;
        state[pin].output_enabled = false;
        state[pin].pullup = true;
        state[pin].pulldown = false;
    }
};

struct Serial
{
    Pins &pins;
    int rx = -1, tx = -1;
    bool running = false, fail_begin = false, fail_fifo = false;
    unsigned fifo = 0, timeout = 0, end_calls = 0;

    void begin(unsigned long baud, uint32_t format, int rx_pin, int tx_pin, bool inverted)
    {
        assert(baud == 1200 && format == 0x8e1 && inverted);
        rx = rx_pin; tx = tx_pin;
        running = !fail_begin;
        pins.state[tx].uart_selected = true;
        pins.state[tx].uart_high = false;
        // Reproduce the driver regression behind upstream #192: begin()
        // replaces the board's RX pull-up with the wrong inverted-UART bias.
        pins.state[rx].pullup = false;
        pins.state[rx].pulldown = true;
    }
    explicit operator bool() const { return running; }
    bool setRxFIFOFull(unsigned bytes) { fifo = bytes; return !fail_fifo; }
    void setTimeout(unsigned ms) { timeout = ms; }
    void end() { ++end_calls; running = false; }
    bool receives_echo() const
    {
        return running && pins.state[rx].pullup && !pins.state[rx].pulldown;
    }
};

int main()
{
    constexpr int rx = 21, tx = 22;
    {
        Pins pins;
        Serial serial{pins};
        serial.begin(1200, 0x8e1, rx, tx, true);
        assert(!serial.receives_echo()); // Reproduce the released failure.
        assert(gdo_uart::begin_sec1(serial, pins, rx, tx, 0x8e1));
        assert(serial.receives_echo()); // RX works despite begin() clobbering pulls.
        assert(serial.fifo == 1); // A single echoed byte must reach the reader.
        assert(serial.timeout > 11.0 * 1000 / 1200 && serial.timeout < 30);
        assert(pins.assertions == 0); // Switching a stale GPIO latch never asserts TX.
    }
    {
        Pins pins;
        pins.reset(tx);
        assert(pins.state[tx].active()); // Reset's pull-up is not the idle TX state.
        gdo_uart::quiet_tx(pins, tx);
        assert(!pins.state[tx].active());
        assert(!pins.state[tx].pullup && pins.state[tx].output_enabled);
        // UART interrupts may continue after a panic. Their output is disconnected.
        pins.state[tx].uart_high = true;
        assert(!pins.state[tx].active());
        gdo_uart::quiet_tx(pins, tx); // Shutdown and panic cleanup are idempotent.
        assert(!pins.state[tx].active());
    }
    for (int failure = 0; failure < 3; ++failure)
    {
        Pins pins;
        Serial serial{pins};
        serial.fail_begin = failure == 0;
        pins.reject_rx_bias = failure == 1;
        serial.fail_fifo = failure == 2;
        assert(!gdo_uart::begin_sec1(serial, pins, rx, tx, 0x8e1));
        assert(!serial.running && serial.end_calls == 1);
        assert(!pins.state[tx].active()); // Failed initialization cannot transmit.
        pins.state[tx].uart_high = true;
        assert(!pins.state[tx].active());
    }
}
