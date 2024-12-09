/****************************************************************************
 * RATGDO HomeKit for ESP32
 * https://ratcloud.llc
 * https://github.com/PaulWieland/ratgdo
 *
 * Copyright (c) 2023-24 David A Kerr... https://github.com/dkerr64/
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

// ESP system includes
// none

// RATGDO project includes
#include "HomeSpan.h"

void setup_homekit();
//void homekit_loop();

extern void notify_homekit_target_door_state_change();
extern void notify_homekit_current_door_state_change();
//extern void notify_homekit_active();
extern void notify_homekit_target_lock();
extern void notify_homekit_current_lock();
extern void notify_homekit_obstruction();
extern void notify_homekit_light();
extern void enable_service_homekit_motion();
extern void notify_homekit_motion();

extern void notify_homekit_vehicle_occupancy(bool vehicleDetected);
extern void notify_homekit_vehicle_arriving(bool vehicleArriving);
extern void notify_homekit_vehicle_departing(bool vehicleDeparting);

extern void homekit_unpair();
extern bool homekit_is_paired();

struct GDOEvent
{
    SpanCharacteristic *c;
    union
    {
        bool b;
        uint8_t u;
    } value;
};

struct DEV_GarageDoor : Service::GarageDoorOpener
{
    Characteristic::CurrentDoorState *current;
    Characteristic::TargetDoorState *target;
    Characteristic::ObstructionDetected *obstruction;
    Characteristic::LockCurrentState *lockCurrent;
    Characteristic::LockTargetState *lockTarget;

    QueueHandle_t event_q;

    DEV_GarageDoor();
    boolean update();
    void loop();
};

struct DEV_Light : Service::LightBulb
{
    Characteristic::On *on;

    QueueHandle_t event_q;

    DEV_Light();
    boolean update();
    void loop();
};

struct DEV_Motion : Service::MotionSensor
{
    Characteristic::MotionDetected *motion;

    QueueHandle_t event_q;
    char name[16];

    DEV_Motion(const char *name);
    void loop();
};

struct DEV_Occupancy : Service::OccupancySensor
{
    Characteristic::OccupancyDetected *occupied;

    QueueHandle_t event_q;

    DEV_Occupancy();
    void loop();
};
