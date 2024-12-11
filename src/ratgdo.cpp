
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

// C/C++ language includes

// ESP system includes
#include <esp_core_dump.h>

// RATGDO project includes
#include "ratgdo.h"
#include "config.h"
#include "utilities.h"
#include "comms.h"
#include "homekit.h"
#include "web.h"
#include "softAP.h"
#include "led.h"
#include "vehicle.h"

// Logger tag
static const char *TAG = "ratgdo";

GarageDoor garage_door;

// Track our memory usage
uint32_t free_heap = (1024 * 1024);
uint32_t min_heap = (1024 * 1024);
unsigned long next_heap_check = 0;

bool status_done = false;
unsigned long status_timeout;

// support for changeing WiFi settings
unsigned long wifiConnectTimeout = 0;

void service_timer_loop();

/****************************************************************************
 * Initialize RATGDO
 */
void setup()
{
    esp_core_dump_init();
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for serial port to open

    Serial.printf("\n\n\n=== R A T G D O ===\n");

    if (esp_core_dump_image_check() == ESP_OK)
    {
        crashCount = 1;
        Serial.printf("CORE DUMP FOUND\n");
        esp_core_dump_summary_t *summary = (esp_core_dump_summary_t *)malloc(sizeof(esp_core_dump_summary_t));
        if (summary)
        {
            if (esp_core_dump_get_summary(summary) == ESP_OK)
            {
                Serial.printf("Crash in task: %s\n", summary->exc_task);
            }
        }
        free(summary);
    }

    // Beep on boot...
    tone(BEEPER_PIN, 1300, 500);
    laser.on();
    led.on();

    load_all_config_settings();

    if (softAPmode)
    {
        start_soft_ap();
        return;
    }

    if (userConfig->getWifiChanged())
    {
        wifiConnectTimeout = millis() + 30000;
    }

    setup_homekit();
}

/****************************************************************************
 * Main loop
 */
void loop()
{
    comms_loop();
    web_loop();
    soft_ap_loop();
    vehicle_loop();
    service_timer_loop();
}

/****************************************************************************
 * Service loop
 */
void service_timer_loop()
{
    unsigned long current_millis = millis();

    if ((rebootSeconds != 0) && (rebootSeconds < current_millis / 1000))
    {
        // Reboot the system if we have reached time...
        RINFO(TAG, "Rebooting system as %lu seconds expired", rebootSeconds);
        sync_and_restart();
        return;
    }

#ifdef NTP_CLIENT
    if (enableNTP && clockSet && lastRebootAt == 0)
    {
        lastRebootAt = time(NULL) - (current_millis / 1000);
        RINFO(TAG, "System boot time: %s", timeString(lastRebootAt));
    }
#endif

    // Check heap
    if (current_millis > next_heap_check)
    {
        next_heap_check = current_millis + 1000;
        free_heap = ESP.getFreeHeap();
        if (free_heap < min_heap)
        {
            min_heap = free_heap;
            RINFO(TAG, "Free heap dropped to %d", min_heap);
        }
    }

    if ((wifiConnectTimeout > 0) && (current_millis > wifiConnectTimeout))
    {
        bool connected = (WiFi.status() == WL_CONNECTED);
        RINFO(TAG, "30 seconds since WiFi settings change, connected to access point: %s", (connected) ? "true" : "false");
        // If not connected, reset to auto.
        if (!connected)
        {
            /* TODO check for WiFi Settings Change not working
            RINFO(TAG, "Reset WiFi Power to 20.5 dBm and WiFiPhyMode to: 0");
            // TODO add support for selecting WiFi PhyMode and WiFi TX Power
            userConfig->wifiPower = 20;
            userConfig->wifiPhyMode = 0;
            WiFi.setOutputPower(20.5);
            WiFi.setPhyMode((WiFiPhyMode_t)0);
            write_config_to_file();
            // Now try and reconnect...
            wifiConnectTimeout = millis() + 30000;
            WiFi.reconnect();
            return;
            */
        }
        else
        {
            RINFO(TAG, "Connected, TODO... test Gatway IP reachable");
            /* TODO Check that network reachable
            IPAddress ip;
            if (!Ping.ping(WiFi.gatewayIP(), 1))
            {
                RINFO(TAG, "Unable to ping Gateway, reset to DHCP to acquire IP address and reconnect");
                userConfig->set(cfg_staticIP, false);
                IPAddress ip;
                ip.fromString("0.0.0.0");
                WiFi.config(ip, ip, ip, ip);
                // Now try and reconnect...
                wifiConnectTimeout = millis() + 30000;
                WiFi.reconnect();
                return;
            }
            else
            {
                RINFO(TAG, "Gateway %s alive %u ms", WiFi.gatewayIP().toString().c_str(), Ping.averageTime());
            }
            */
        }
        // reset flag
        wifiConnectTimeout = 0;
        userConfig->set(cfg_wifiChanged, false);
    }
}
