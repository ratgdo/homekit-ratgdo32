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
#include <cstring>
#include <algorithm>

// ESP system includes
#include <esp_sntp.h>

// Arduino system includes
#include <HTTPClient.h>

// RATGDO project includes
#include "ratgdo.h"
#include "utilities.h"
#include "config.h"
#include "comms.h"
#include "led.h"

// Logger tag
static const char *TAG = "ratgdo-utils";

// What trigger motion...
motionTriggersUnion motionTriggers = {0, 0, 0, 0, 0, 0};
// Control booting into soft access point mode
bool softAPmode = false;
// Realm for MD5 credential hashing
const char www_realm[] = "RATGDO Login Required";
// automatically reboot after X seconds
uint32_t rebootSeconds = 0;

uint64_t ARDUINO_ISR_ATTR millis64()
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

bool clockSet = false;
bool enableNTP = false;
uint64_t lastRebootAt = 0;

bool get_auto_timezone()
{
    bool success = false;
    WiFiClient client;
    HTTPClient http;

    RINFO(TAG, "Get timezone automatically based on IP address");
    if (http.begin(client, "http://ip-api.com/csv/?fields=timezone"))
    {
        // start connection and send HTTP header
        int httpCode = http.GET();
        // httpCode will be negative on error
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
        {
            String tz = http.getString();
            tz.trim();
            userConfig->set(cfg_timeZone, tz.c_str());
            RINFO(TAG, "Automatic timezone set to: %s", userConfig->getTimeZone().c_str());
            success = true;
        }
        http.end();
    }
    return success;
}

void time_is_set(timeval *tv)
{
    clockSet = true;
}

char *timeString(time_t reqTime, bool syslog)
{
    static char tBuffer[32];
    time_t tTime = 0;
    tm tmTime;
    tBuffer[0] = 0;
    tTime = ((reqTime == 0) && clockSet) ? time(NULL) : reqTime;
    if (tTime != 0)
    {
        localtime_r(&tTime, &tmTime);
        if (syslog)
        {
            // syslog compatible
            strftime(tBuffer, sizeof(tBuffer), "%Y-%m-%dT%H:%M:%S.000%z", &tmTime);
            // %z returns e.g. "-400" or "+1000", we need it to be "-4:00" or "+10:00"
            // this format is REQUIRED by syslog
            int i = strlen(tBuffer);
            if (tBuffer[i - 5] == '-' || tBuffer[i - 5] == '+' ||
                tBuffer[i - 6] == '-' || tBuffer[i - 6] == '+')
            {
                tBuffer[i + 1] = 0;
                tBuffer[i] = tBuffer[i - 1];
                tBuffer[i - 1] = tBuffer[i - 2];
                tBuffer[i - 2] = ':';
            }
        }
        else
        {
            // Print format example: 27-Oct-2024 11:16:18 EDT
            strftime(tBuffer, sizeof(tBuffer), "%d-%b-%Y %H:%M:%S %Z", &tmTime);
        }
    }
    return tBuffer;
}

char *make_rfc952(char *dest, const char *src, int size)
{
    // Make device name RFC952 complient (simple, just checking for the basics)
    // RFC952 says max len of 24, [a-z][A-Z][0-9][-.] and no dash or period in last char.
    int i = 0;
    while (i <= std::min(24, size - 1) && src[i] != 0)
    {
        dest[i] = (std::isspace((unsigned char)src[i])) ? '-' : src[i];
        i++;
    }
    // remove dashes and periods from end of name
    while (i > 0 && (dest[i - 1] == '-' || dest[i - 1] == '.'))
    {
        dest[--i] = 0;
    }
    // null terminate string
    dest[std::min(i, std::min(24, size - 1))] = 0;
    return dest;
}

void load_all_config_settings()
{
    RINFO(TAG, "=== Load all config settings for %s", device_name);

    userConfig->load();
    // Set globals...
    strlcpy(device_name, userConfig->getDeviceName().c_str(), sizeof(device_name));
    make_rfc952(device_name_rfc952, device_name, sizeof(device_name_rfc952));
    led.setIdleState(userConfig->getLEDidle());
    motionTriggers.asInt = userConfig->getMotionTriggers();
    softAPmode = userConfig->getSoftAPmode();
    strlcpy(syslogIP, userConfig->getSyslogIP().c_str(), sizeof(syslogIP));
    syslogPort = userConfig->getSyslogPort();
    syslogEn = userConfig->getSyslogEn();
    rebootSeconds = userConfig->getRebootSeconds();

    // Now log what we have loaded
    RINFO(TAG, "   deviceName:          %s", userConfig->getDeviceName().c_str());
    RINFO(TAG, "   wifiChanged:         %s", userConfig->getWifiChanged() ? "true" : "false");
    RINFO(TAG, "   wifiPower:           %d", userConfig->getWifiPower());
    RINFO(TAG, "   wifiPhyMode:         %d", userConfig->getWifiPhyMode());
    RINFO(TAG, "   staticIP:            %s", userConfig->getStaticIP() ? "true" : "false");
    RINFO(TAG, "   localIP:             %s", userConfig->getLocalIP().c_str());
    RINFO(TAG, "   subnetMask:          %s", userConfig->getSubnetMask().c_str());
    RINFO(TAG, "   gatewayIP:           %s", userConfig->getGatewayIP().c_str());
    RINFO(TAG, "   nameserverIP:        %s", userConfig->getNameserverIP().c_str());
    RINFO(TAG, "   wwwPWrequired:       %s", userConfig->getPasswordRequired() ? "true" : "false");
    RINFO(TAG, "   wwwUsername:         %s", userConfig->getwwwUsername().c_str());
    RINFO(TAG, "   wwwCredentials:      %s", userConfig->getwwwCredentials().c_str());
    RINFO(TAG, "   GDOSecurityType:     %d", userConfig->getGDOSecurityType());
    RINFO(TAG, "   TTCseconds:          %d", userConfig->getTTCseconds());
    RINFO(TAG, "   rebootSeconds:       %d", userConfig->getRebootSeconds());
    RINFO(TAG, "   LEDidle:             %d", userConfig->getLEDidle());
    RINFO(TAG, "   motionTriggers:      %d", userConfig->getMotionTriggers());
    RINFO(TAG, "   enableNTP:           %s", userConfig->getEnableNTP() ? "true" : "false");
    RINFO(TAG, "   doorUpdateAt:        %d", userConfig->getDoorUpdateAt());
    RINFO(TAG, "   timeZone:            %s", userConfig->getTimeZone().c_str());
    RINFO(TAG, "   softAPmode:          %s", userConfig->getSoftAPmode() ? "true" : "false");
    RINFO(TAG, "   syslogEn:            %s", userConfig->getSyslogEn() ? "true" : "false");
    RINFO(TAG, "   syslogIP:            %s", userConfig->getSyslogIP().c_str());
    RINFO(TAG, "   syslogPort:          %d", userConfig->getSyslogPort());
    RINFO(TAG, "   vehicleThreshold:    %d", userConfig->getVehicleThreshold());
    RINFO(TAG, "   laserEnabled:        %s", userConfig->getLaserEnabled() ? "true" : "false");
    RINFO(TAG, "   laserHomeKit:        %s", userConfig->getLaserHomeKit() ? "true" : "false");
    RINFO(TAG, "   assistDuration:      %d", userConfig->getAssistDuration());
    RINFO(TAG, "RFC952 device hostname: %s", device_name_rfc952);

    // Only enable NTP client if not in soft AP mode.
    enableNTP = !softAPmode && userConfig->getEnableNTP();
    if (enableNTP)
    {
        sntp_set_time_sync_notification_cb(time_is_set);
        RINFO(TAG, "Timezone: %s", userConfig->getTimeZone().c_str());
        std::string tz = userConfig->getTimeZone();
        size_t pos = tz.find(';');
        if (pos != std::string::npos)
        {
            // semicolon may separate continent/city from posix TZ string
            // if no semicolon then no POSIX code, so use UTC
            RINFO(TAG, "Set timezone: %s", tz.substr(pos + 1).c_str());
            configTzTime(tz.substr(pos + 1).c_str(), NTP_SERVER);
        }
        else
        {
            RINFO(TAG, "Set timezone: UTC0");
            configTzTime("UTC0", NTP_SERVER);
        }
    }
}

void sync_and_restart()
{
    if (softAPmode)
    {
        // reset so next reboot will be standard station mode
        userConfig->set(cfg_softAPmode, false);
    }
    else
    {
        // In soft AP mode we never initialized garage door comms, so don't save rolling code.
        save_rolling_code();
    }

    ratgdoLogger->saveMessageLog();
    delay(100);
    ESP.restart();
}
