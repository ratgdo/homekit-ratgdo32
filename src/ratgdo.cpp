
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

// C/C++ language includes

// ESP system includes
#include <esp_core_dump.h>
#include <ping/ping_sock.h>

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
#include "drycontact.h"
#include "provision.h"

// Logger tag
static const char *TAG = "ratgdo-main";

GarageDoor garage_door;

// Track our memory usage
uint32_t free_heap = (1024 * 1024);
uint32_t min_heap = (1024 * 1024);
uint64_t next_heap_check = 0;

bool status_done = false;
uint64_t status_timeout;

// support for changeing WiFi settings
#define WIFI_CONNECT_TIMEOUT (30 * 1000)
static uint64_t wifiConnectTimeout = 0;
static bool ping_failure = false;
static bool ping_timed_out = false;
;
static esp_ping_handle_t ping;

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
    // Beep on boot...
    tone(BEEPER_PIN, 1300, 500);
    led.on();

    esp_reset_reason_t r = esp_reset_reason();
    switch (r)
    {
    case ESP_RST_POWERON:
    case ESP_RST_PWR_GLITCH:
        RINFO(TAG, "System restart after power-on or power glitch: %d", r);
        // RTC memory does not survive power interruption. Initialize values.
        rebootTime = 0;
        crashTime = 0;
        crashCount = 0;
        break;
    default:
        RINFO(TAG, "System restart reason: %d", r);
        break;
    }

    // If there is a core dump image but no saved crash log, then set count to -1.
    if ((crashCount == 0) && (esp_core_dump_image_check() == ESP_OK))
        crashCount = -1;

    load_all_config_settings();

    if (softAPmode)
    {
        start_soft_ap();
        return;
    }

    if (userConfig->getWifiChanged())
    {
        wifiConnectTimeout = millis64() + WIFI_CONNECT_TIMEOUT;
    }

    setup_homekit();
}

/****************************************************************************
 * Main loop
 */
void loop()
{
    comms_loop();
    drycontact_loop();
    web_loop();
    soft_ap_loop();
    improv_loop();
    vehicle_loop();
    service_timer_loop();
}

/****************************************************************************
 * Functions to ping gateway to test network okay
 */
static void ping_success(esp_ping_handle_t hdl, void *args)
{
    uint8_t ttl;
    uint16_t seqno;
    uint32_t elapsed_time, recv_len;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &recv_len, sizeof(recv_len));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
    IPAddress ip_addr((uint32_t)target_addr.u_addr.ip4.addr);
    RINFO(TAG, "Ping: %d bytes from %s icmp_seq=%d ttl=%d time=%dms",
          recv_len, ip_addr.toString().c_str(), seqno, ttl, elapsed_time);
    ping_timed_out = false;
}

static void ping_timeout(esp_ping_handle_t hdl, void *args)
{
    uint16_t seqno;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    IPAddress ip_addr((uint32_t)target_addr.u_addr.ip4.addr);
    RINFO(TAG, "Ping from %s icmp_seq=%d timeout", ip_addr.toString().c_str(), seqno);
    ping_timed_out = true;
}

static void ping_end(esp_ping_handle_t hdl, void *args)
{
    ping_failure = ping_timed_out;
    RINFO(TAG, "Ping end: %s", (ping_failure) ? "failed" : "success");
}

static void ping_start()
{
    ip_addr_t addr;
    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
    WiFi.gatewayIP().to_ip_addr_t(&addr);
    RINFO(TAG, "Ping to: %s", WiFi.gatewayIP().toString().c_str());
    ping_config.target_addr = addr;
    ping_config.count = 2;

    esp_ping_callbacks_t cbs;
    cbs.on_ping_success = ping_success;
    cbs.on_ping_timeout = ping_timeout;
    cbs.on_ping_end = ping_end;
    cbs.cb_args = NULL;
    esp_ping_new_session(&ping_config, &cbs, &ping);

    ping_failure = false;
    ping_timed_out = false;
    esp_ping_start(ping);
}

static void ping_stop()
{
    esp_ping_stop(ping);
    esp_ping_delete_session(ping);
}

/****************************************************************************
 * Service loop
 */
void service_timer_loop()
{
    uint64_t current_millis = millis64();

    if ((rebootSeconds != 0) && (rebootSeconds < current_millis / 1000))
    {
        // Reboot the system if we have reached time...
        RINFO(TAG, "Rebooting system as %lu seconds expired", rebootSeconds);
        sync_and_restart();
        return;
    }

    if (enableNTP && clockSet && lastRebootAt == 0)
    {
        lastRebootAt = time(NULL) - (current_millis / 1000);
        RINFO(TAG, "Current System time: %s", timeString());
        RINFO(TAG, "System boot time:    %s", timeString(lastRebootAt));
        // Need to also set when last door open/close was
        if (userConfig->getDoorUpdateAt() != 0)
        {
            lastDoorUpdateAt = (((uint64_t)userConfig->getDoorUpdateAt() - time(NULL)) * 1000LL) + current_millis;
            RINFO(TAG, "Last door update at: %s", timeString((uint64_t)userConfig->getDoorUpdateAt()));
        }
    }

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
        if (!connected)
        {
            RERROR(TAG, "30 seconds since WiFi settings change, failed to connect");
            userConfig->set(cfg_wifiPower, WIFI_POWER_MAX);
            userConfig->set(cfg_wifiPhyMode, 0);
            // TODO support WiFi TX Power & PhyMode... set changes immediately here
            // Now try and reconnect...
            wifiConnectTimeout = millis64() + WIFI_CONNECT_TIMEOUT;
            WiFi.reconnect();
        }
        else
        {
            RINFO(TAG, "30 seconds since WiFi settings change, successfully connected to access point");
            if (userConfig->getStaticIP())
            {
                RINFO(TAG, "Connected with static IP, test gateway IP reachable");
                ping_start();
            }
            wifiConnectTimeout = 0;
        }
        userConfig->set(cfg_wifiChanged, false);
    }

    if (ping_failure)
    {
        ping_failure = false; // reset, so we only come in here once
        if (userConfig->getStaticIP())
        {
            // We timed out trying to ping gateway set by static IP, revert to DHCP
            ping_stop();
            RINFO(TAG, "Unable to ping Gateway, reset to DHCP to acquire IP address and reconnect");
            userConfig->set(cfg_staticIP, false);
            IPAddress ip;
            ip.fromString("0.0.0.0");
            WiFi.config(ip, ip, ip, ip);
            // Now try and reconnect...
            wifiConnectTimeout = millis64() + WIFI_CONNECT_TIMEOUT;
            WiFi.reconnect();
        }
    }
}
