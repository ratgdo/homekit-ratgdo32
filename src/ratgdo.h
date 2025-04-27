/****************************************************************************
 * RATGDO HomeKit for ESP32
 * https://ratcloud.llc
 * https://github.com/PaulWieland/ratgdo
 *
 * Copyright (c) 2023-25 David A Kerr... https://github.com/dkerr64/
 * All Rights Reserved.
 * Licensed under terms of the GPL-3.0 License.
 *
 * Contributions acknowledged from
 * Brandon Matthews... https://github.com/thenewwazoo
 * Jonathan Stroud...  https://github.com/jgstroud
 *
 */
#pragma once

// C/C++ language includes
// none

// Arduino includes
#include <Arduino.h>

// ESP system includes
#include <driver/gpio.h>

// RATGDO project includes
#include "HomeSpan.h"
#include "../lib/ratgdo/log.h"

#define DEVICE_NAME "homekit-ratgdo"
#define MANUF_NAME "ratCloud llc"
#define SERIAL_NUMBER "0P3ND00R"
#define MODEL_NAME "ratgdo_32"
#define CHIP_FAMILY "ESP32"

/********************************** PIN DEFINITIONS *****************************************/

const gpio_num_t UART_TX_PIN = GPIO_NUM_17;
const gpio_num_t UART_RX_PIN = GPIO_NUM_21;
const gpio_num_t LED_BUILTIN = GPIO_NUM_2;
const gpio_num_t INPUT_OBST_PIN = GPIO_NUM_4;
const gpio_num_t DRY_CONTACT_OPEN_PIN = GPIO_NUM_13;  // open door contact sensor
const gpio_num_t DRY_CONTACT_CLOSE_PIN = GPIO_NUM_14; // closed door contact sensor
const gpio_num_t LIGHT_PIN = GPIO_NUM_27;             // control a light
const gpio_num_t DISCRETE_OPEN_PIN = GPIO_NUM_26;     // alternative (or in addition) to toggle, can use discrete open control
const gpio_num_t DISCRETE_CLOSE_PIN = GPIO_NUM_25;    // alternative (or in addition) to toggle, can use discrete close control

const gpio_num_t BEEPER_PIN = GPIO_NUM_33;
const gpio_num_t LASER_PIN = GPIO_NUM_23;
const gpio_num_t SENSOR_PIN = GPIO_NUM_34;

const gpio_num_t SHUTDOWN_PIN = GPIO_NUM_32;

extern uint32_t free_heap;
extern uint32_t min_heap;

/********************************** MODEL *****************************************/

enum GarageDoorCurrentState : uint8_t
{
    CURR_OPEN = Characteristic::CurrentDoorState::OPEN,
    CURR_CLOSED = Characteristic::CurrentDoorState::CLOSED,
    CURR_OPENING = Characteristic::CurrentDoorState::OPENING,
    CURR_CLOSING = Characteristic::CurrentDoorState::CLOSING,
    CURR_STOPPED = Characteristic::CurrentDoorState::STOPPED,
    UNKNOWN = 0xFF,
};

enum GarageDoorTargetState : uint8_t
{
    TGT_OPEN = Characteristic::TargetDoorState::OPEN,
    TGT_CLOSED = Characteristic::TargetDoorState::CLOSED,
};

enum LockCurrentState : uint8_t
{
    CURR_UNLOCKED = Characteristic::LockCurrentState::UNLOCKED,
    CURR_LOCKED = Characteristic::LockCurrentState::LOCKED,
    CURR_JAMMED = Characteristic::LockCurrentState::JAMMED,
    CURR_UNKNOWN = Characteristic::LockCurrentState::UNKNOWN,
};

enum LockTargetState : uint8_t
{
    TGT_UNLOCKED = Characteristic::LockTargetState::UNLOCK,
    TGT_LOCKED = Characteristic::LockTargetState::LOCK,
};

#define MOTION_TIMER_DURATION 5000 // how long to keep HomeKit motion sensor active for

struct GarageDoor
{
    bool active;
    GarageDoorCurrentState current_state;
    GarageDoorTargetState target_state;
    bool obstructed;
    bool has_motion_sensor;
    bool has_distance_sensor;
#ifndef USE_GDOLIB
    uint64_t motion_timer;
#endif
    bool motion;
    bool light;
    LockCurrentState current_lock;
    LockTargetState target_lock;
    uint16_t openingsCount;
    uint8_t batteryState;
    uint16_t openDuration;
    uint16_t closeDuration;
    uint64_t room_occupancy_timeout;
    bool room_occupied;
};

extern GarageDoor garage_door;

struct ForceRecover
{
    uint8_t push_count;
    uint64_t timeout;
};
