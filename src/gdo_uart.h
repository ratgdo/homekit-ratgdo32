/****************************************************************************
 * RATGDO HomeKit
 * https://ratcloud.llc
 * https://github.com/PaulWieland/ratgdo
 *
 * Copyright (c) 2026 Eliran Sapir... https://github.com/sapireli/
 * All Rights Reserved.
 * Licensed under terms of the GPL-3.0 License.
 *
 */
#pragma once

#include <cstdint>

#ifdef ESP32
#include <driver/gpio.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_struct.h>
#include <soc/gpio_sig_map.h>

// Only the hardware adapter needs ESP32 headers. Host tests use the same
// initialization and pin sequence below with their own adapter.
// The TX operations are lock-free register operations: panic handling must
// work with the flash cache disabled and without calling the UART driver.
struct Esp32GdoUartPins
{
    __attribute__((always_inline)) inline void preload_low(int pin) { gpio_ll_set_level(&GPIO, pin, 0); }
    __attribute__((always_inline)) inline void select_gpio(int pin) { gpio_ll_func_sel(&GPIO, pin, PIN_FUNC_GPIO); }
    __attribute__((always_inline)) inline void route_gpio(int pin)
    {
        // Both the signal and its output-enable now belong to GPIO, not UART.
        REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + pin * 4, SIG_GPIO_OUT_IDX | GPIO_FUNC0_OEN_SEL);
    }
    __attribute__((always_inline)) inline void disable_tx_pullup(int pin) { gpio_ll_pullup_dis(&GPIO, pin); }
    __attribute__((always_inline)) inline void enable_tx_pulldown(int pin)
    {
        // gpio_ll_pulldown_en() uses the flash-resident GPIO_PIN_MUX_REG table
        // and may itself be outlined into flash. Use the IDF's DRAM offset table.
        REG_SET_BIT(DR_REG_IO_MUX_BASE + GPIO_PIN_MUX_REG_OFFSET[pin], FUN_PD);
    }
    __attribute__((always_inline)) inline void enable_output(int pin) { gpio_ll_output_enable(&GPIO, pin); }
    bool disable_rx_pulldown(int pin) { return gpio_pulldown_dis((gpio_num_t)pin) == ESP_OK; }
    bool enable_rx_pullup(int pin) { return gpio_pullup_en((gpio_num_t)pin) == ESP_OK; }
};
#endif

// Keep the electrical sequence independent of the ESP32 driver so the known
// inverted-UART pull regression and the shutdown pin state can be tested.
namespace gdo_uart
{
constexpr unsigned long SEC1_BAUD = 1200;
// 8E1 occupies 11 bit periods (9.17 ms). Leave scheduling margin for its echo.
constexpr unsigned long SEC1_ECHO_TIMEOUT_MS = 20;

template <class Pins>
__attribute__((always_inline)) inline void quiet_tx(Pins &pins, int tx)
{
    // Preload before switching the mux: a stale HIGH latch would assert the bus.
    pins.preload_low(tx);
    pins.select_gpio(tx);
    pins.route_gpio(tx);
    pins.disable_tx_pullup(tx);
    pins.enable_tx_pulldown(tx);
    pins.enable_output(tx);
}

template <class SerialPort, class Pins>
bool begin_sec1(SerialPort &serial, Pins &pins, int rx, int tx, uint32_t format)
{
    quiet_tx(pins, tx);
    if (serial)
    {
        // A repeated begin() with unchanged parameters may reuse the existing
        // driver without restoring its pin mux after quiet_tx().
        serial.end();
        quiet_tx(pins, tx);
    }
    serial.begin(SEC1_BAUD, format, rx, tx, true);
    // begin() may overwrite the input pulls for an inverted UART (upstream #192).
    // Apply the board's RX bias afterwards, even if the framework changes defaults.
    const bool rx_ready = pins.disable_rx_pulldown(rx) && pins.enable_rx_pullup(rx);
    if (!serial || !rx_ready || !serial.setRxFIFOFull(1))
    {
        serial.end();
        quiet_tx(pins, tx);
        return false;
    }
    serial.setTimeout(SEC1_ECHO_TIMEOUT_MS);
    return true;
}
}
