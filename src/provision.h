/****************************************************************************
 * RATGDO HomeKit
 * https://ratcloud.llc
 * https://github.com/PaulWieland/ratgdo
 *
 * Copyright (c) 2023-25 David A Kerr... https://github.com/dkerr64/
 * All Rights Reserved.
 * Licensed under terms of the GPL-3.0 License.
 *
 */
#pragma once

extern void setup_improv();
extern void disable_improv();
extern void improv_loop();
#ifdef GRGDO1_V1
extern void release_serial_console_for_i2c();
extern void restore_serial_console_after_i2c();
#endif
