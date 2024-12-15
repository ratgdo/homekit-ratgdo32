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
 * T L Hagen... https://github.com/tlhagan
 *
 */
#pragma once

// C/C++ language includes

// RATGDO project includes
#include <OneButton.h>


extern void setup_drycontact();
extern void drycontact_loop();
