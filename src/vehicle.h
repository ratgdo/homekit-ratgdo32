/****************************************************************************
 * RATGDO HomeKit for ESP32
 * https://ratcloud.llc
 * https://github.com/PaulWieland/ratgdo
 *
 * Copyright (c) 2023-25 David A Kerr... https://github.com/dkerr64/
 * All Rights Reserved.
 * Licensed under terms of the GPL-3.0 License.
 *
 */
#pragma once

// C/C++ language includes
#include <stdint.h>

#define PRESENCE_DETECT_DURATION (5 * 60 * 1000) // how long to calculate presence after door state change

extern void setup_vehicle();
extern void vehicle_loop();

extern void doorOpening();
extern void doorClosing();

extern int16_t vehicleDistance;
extern int16_t vehicleThresholdDistance;
extern char vehicleStatus[];
extern bool vehicleStatusChange;
