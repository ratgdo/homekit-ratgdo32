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
// none

// Arduino includes
#include <WebServer.h>

// RATGDO project includes
#include "ratgdo.h"
#include "config.h"
#include "WiFi.h"
#include "softAP.h"
#include "web.h"
#include "utilities.h"
#include "provision.h"

// Logger tag
static const char *TAG = "ratgdo-softAP";

static const char softAPhttpPreamble[] = "HTTP/1.1 200 OK\nContent-Type: text/html\nCache-Control: no-cache, no-store\n\n<!DOCTYPE html>\n";
// TODO enable advanced mode (AP selection), disabled below by setting display = none
static const char softAPtableHead[] = R"(
<tr style='display:none;'><td><input id='adv' name='advanced' type='checkbox' onclick='showAdvanced(this.checked)'></td><td colspan='2'>Advanced</td></tr>
<tr><th></th><th>SSID</th><th>RSSI</th><th>Chan</th><th>Hardware BSSID</th></tr>)";
static const char softAPtableRow[] = R"(
<tr %s><td><input type='radio' name='net' value='%d' %s></td><td>%s</td><td>%lddBm</td><td>%ld</td><td>&nbsp;&nbsp;%02x:%02x:%02x:%02x:%02x:%02x</td></tr>)";
static const char softAPtableLastRow[] = R"(
<tr><td><input type='radio' name='net' value='%d'></td><td colspan='2'><input type='text' name='userSSID' placeholder='SSID' value='%s'></td></tr>)";

// forward declare functions
void handle_softAPweb();
void handle_wifinets();

extern WebServer server;

#define MAX_ATTEMPTS_WIFI_CONNECTION 30
#define TXT_BUFFER_SIZE 1024

// support for scaning WiFi networks
bool wifiNetsCmp(wifiNet_t a, wifiNet_t b)
{
    // Sorts first by SSID and then by RSSI so strongest signal first.
    return (a.ssid < b.ssid) || ((a.ssid == b.ssid) && (a.rssi > b.rssi));
}
std::multiset<wifiNet_t, decltype(&wifiNetsCmp)> wifiNets(&wifiNetsCmp);

static bool softAPinitialized = false;

char *getServiceName(char *service_name, size_t max)
{
    uint8_t mac[6];
    const char *ssid_prefix = "RATGDO_";
    Network.macAddress(mac);
    snprintf(service_name, max, "%s%02X%02X%02X", ssid_prefix, mac[3], mac[4], mac[5]);
    return service_name;
}

void wifi_scan()
{
    // scan for networks
    RINFO(TAG, "Scanning for networks...");
    wifiNet_t wifiNet;
    wifiNets.clear();
    int nNets = std::min((int)WiFi.scanNetworks(), 127);
    RINFO(TAG, "Found %d networks", nNets);
    for (int i = 0; i < nNets; i++)
    {
        wifiNet.ssid = WiFi.SSID(i);
        wifiNet.channel = WiFi.channel(i);
        wifiNet.rssi = WiFi.RSSI(i);
        memcpy(wifiNet.bssid, WiFi.BSSID(i), sizeof(wifiNet.bssid));
        wifiNet.encryptionType = WiFi.encryptionType(i);
        RINFO(TAG, "Network: %s (Ch:%d, %ddBm) AP: %s, Encryption: %d",
              wifiNet.ssid.c_str(), wifiNet.channel, wifiNet.rssi, WiFi.BSSIDstr(i).c_str(), wifiNet.encryptionType);
        wifiNets.insert(wifiNet);
    }
    // delete scan from memory
    WiFi.scanDelete();
}

void start_soft_ap()
{
    softAPmode = true;
    RINFO(TAG, "Start AP mode for: %s", device_name_rfc952);
    WiFi.persistent(false);
    WiFi.setSleep(WIFI_PS_NONE); // Improves performance, at cost of power consumption
    bool apStarted = WiFi.softAP(device_name_rfc952);
    if (apStarted)
    {
        RINFO(TAG, "AP started with IP %s", WiFi.softAPIP().toString().c_str());
    }
    else
    {
        RINFO(TAG, "Error starting AP mode");
    }

    server.onNotFound(handle_softAPweb);
    server.begin();
    RINFO(TAG, "Soft AP web server started");
    softAPinitialized = true;
    // wifi_scan();
    // Allow improv WiFi provisioning when soft AP mode active.
    setup_improv();
}

void soft_ap_loop()
{
    if (!softAPinitialized)
        return;

    server.handleClient();

    if (softAPmode && (millis64() > 10 * 60 * 1000))
    {
        RINFO(TAG, "In Soft Access Point mode for over 10 minutes, reboot");
        sync_and_restart();
        return;
    }
}

void handle_softAPweb()
{
    HTTPMethod method = server.method();
    String page = server.uri();

    if ((WiFi.getMode() & WIFI_AP) == WIFI_AP)
    {
        // If we are in Soft Access Point mode
        RINFO(TAG, "WiFi Soft Access Point mode requesting: %s", page.c_str());
        if (page == "/" || page == "/wifiap")
            return handle_wifiap();
        else if (page == "/wifinets")
            return handle_wifinets();
        else if (page == "/setssid" && method == HTTP_POST)
            return handle_setssid();
        else if (page == "/reboot" && method == HTTP_POST)
            return handle_reboot();
        else if (page == "/rescan" && method == HTTP_POST)
            return handle_rescan();
        else
            return handle_notfound();
    }
}

void handle_rescan()
{
    wifi_scan();
    server.send_P(200, type_txt, "Scan complete.");
}

void handle_wifiap()
{
    return load_page("/wifiap.html");
}

void handle_wifinets()
{
    bool connected = WiFi.isConnected();
    String previousSSID = "";
    bool match = false;
    if (connected)
    {
        previousSSID = WiFi.SSID();
    }
    RINFO(TAG, "Number of WiFi networks: %d", wifiNets.size());
    String currentSSID = "";
    server.client().setNoDelay(true);
    server.sendContent(softAPhttpPreamble, strlen(softAPhttpPreamble));
    server.sendContent(softAPtableHead, strlen(softAPtableHead));
    int i = 0;
    char *txtBuffer = (char *)malloc(TXT_BUFFER_SIZE);
    for (wifiNet_t net : wifiNets)
    {
        bool hide = true;
        bool matchSSID = (previousSSID == net.ssid);
        if (matchSSID)
            match = true;
        if (currentSSID != net.ssid)
        {
            currentSSID = net.ssid;
            hide = false;
        }
        else
        {
            matchSSID = false;
        }
        snprintf(txtBuffer, TXT_BUFFER_SIZE, softAPtableRow, (hide) ? "class='adv'" : "", i, (matchSSID) ? "checked='checked'" : "",
                 net.ssid.c_str(), net.rssi, net.channel,
                 net.bssid[0], net.bssid[1], net.bssid[2], net.bssid[3], net.bssid[4], net.bssid[5]);
        server.sendContent(txtBuffer, strlen(txtBuffer));
        i++;
    }
    // user entered value
    snprintf(txtBuffer, TXT_BUFFER_SIZE, softAPtableLastRow, i, (!match) ? previousSSID.c_str() : "");
    server.sendContent(txtBuffer, strlen(txtBuffer));
    server.sendContent("\n", 1);
    server.client().stop();
    free(txtBuffer);
    return;
}

void handle_setssid()
{
    if (server.args() < 3)
    {
        RINFO(TAG, "Sending %s, for: %s as invalid number of args", response400invalid, server.uri().c_str());
        server.send_P(400, type_txt, response400invalid);
        return;
    }

    const unsigned int net = atoi(server.arg("net").c_str());
    String ssid = server.arg("userSSID");
    bool advanced = server.arg("advanced") == "on";
    wifiNet_t wifiNet;

    if (net < wifiNets.size())
    {
        // User selected network from within scanned range
        wifiNet = *std::next(wifiNets.begin(), net);
        ssid = wifiNet.ssid;
    }
    else
    {
        // Outside scanned range, do not allow locking to access point
        advanced = false;
    }

    char *txtBuffer = (char *)malloc(TXT_BUFFER_SIZE);
    if (advanced)
    {
        RINFO(TAG, "Requested WiFi SSID: %s (%d) at AP: %02x:%02x:%02x:%02x:%02x:%02x",
              ssid.c_str(), net, wifiNet.bssid[0], wifiNet.bssid[1], wifiNet.bssid[2], wifiNet.bssid[3], wifiNet.bssid[4], wifiNet.bssid[5]);
        snprintf_P(txtBuffer, TXT_BUFFER_SIZE, PSTR("Setting SSID to: %s locked to Access Point: %02x:%02x:%02x:%02x:%02x:%02x\nRATGDO rebooting.\nPlease wait 30 seconds and connect to RATGDO on new network."),
                   ssid.c_str(), wifiNet.bssid[0], wifiNet.bssid[1], wifiNet.bssid[2], wifiNet.bssid[3], wifiNet.bssid[4], wifiNet.bssid[5]);
    }
    else
    {
        RINFO(TAG, "Requested WiFi SSID: %s (%d)", ssid.c_str(), net);
        snprintf_P(txtBuffer, TXT_BUFFER_SIZE, PSTR("Setting SSID to: %s\nRATGDO rebooting.\nPlease wait 30 seconds and connect to RATGDO on new network."), ssid.c_str());
    }
    server.client().setNoDelay(true);
    server.send_P(200, type_txt, txtBuffer);
    delay(500);
    free(txtBuffer);

    const bool connected = WiFi.isConnected();
    String previousSSID;
    String previousPSK;
    String previousBSSID;
    if (connected)
    {
        previousSSID = WiFi.SSID();
        previousPSK = WiFi.psk();
        previousBSSID = WiFi.BSSIDstr();
        RINFO(TAG, "Current SSID: %s / BSSID:%s", previousSSID.c_str(), previousBSSID.c_str());
        WiFi.disconnect();
    }
    if (connect_wifi(ssid, server.arg("pw"), (advanced) ? wifiNet.bssid : NULL))
    {
        RINFO(TAG, "WiFi Successfully connects to SSID: %s", ssid);
        homeSpan.setWifiCredentials(ssid.c_str(), server.arg("pw").c_str());
        // We should reset WiFi if changing networks or were not currently connected.
        if (!connected || previousBSSID != ssid)
        {
            userConfig->set(cfg_staticIP, false);
            userConfig->set(cfg_wifiPower, WIFI_POWER_MAX);
            userConfig->set(cfg_wifiPhyMode, 0);
            userConfig->set(cfg_timeZone, "");
        }
    }
    else
    {
        RINFO(TAG, "WiFi Failed to connect to SSID: %s", ssid.c_str());
        if (connected)
        {
            RINFO(TAG, "Resetting WiFi to previous SSID: %s, removing any Access Point BSSID lock", previousSSID.c_str());
            connect_wifi(previousSSID, previousPSK);
        }
        else
        {
            // We were not connected, and we failed to connext to new SSID,
            // so best to reset any wifi settings.
            userConfig->set(cfg_staticIP, false);
            userConfig->set(cfg_wifiPower, WIFI_POWER_MAX);
            userConfig->set(cfg_wifiPhyMode, 0);
            userConfig->set(cfg_timeZone, "");
        }
    }
    sync_and_restart();
    return;
}

bool connect_wifi(const String &ssid, const String &password)
{
    return connect_wifi(ssid, password, NULL);
}

bool connect_wifi(const String &ssid, const String &password, const uint8_t *bssid)
{
    uint8_t count = 0;

    RINFO(TAG, "Attempt to connect to %s with pw %s", ssid.c_str(), password.c_str());

    WiFi.begin(ssid, password, 0, bssid);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        yield();
        if (count > MAX_ATTEMPTS_WIFI_CONNECTION)
        {
            WiFi.disconnect();
            return false;
        }
        count++;
    }
    return true;
}
