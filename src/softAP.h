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
#include <set>
// ESP system includes
// #include <esp_wifi_types.h>

// RATGDO project includes
#include "ratgdo.h"

typedef struct
{
    String ssid;
    int32_t rssi;
    int32_t channel;
    uint8_t bssid[6];
    wifi_auth_mode_t encryptionType;
} wifiNet_t;
extern std::multiset<wifiNet_t, bool (*)(wifiNet_t, wifiNet_t)> wifiNets;

extern void start_soft_ap();
extern void soft_ap_loop();
extern void wifi_scan();

extern void handle_setssid();
extern void handle_rescan();
extern void handle_wifinets();
extern void handle_wifiap();

extern bool connect_wifi(const String &ssid, const String &password, const uint8_t *bssid);
extern bool connect_wifi(const String &ssid, const String &password);
