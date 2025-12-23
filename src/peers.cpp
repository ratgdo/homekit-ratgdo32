/****************************************************************************
 * RATGDO HomeKit
 * https://ratcloud.llc
 * https://github.com/PaulWieland/ratgdo
 *
 * Copyright (c) 25 Donavan Becker ... https://github.com/donavanbecker
 * All Rights Reserved.
 * Licensed under terms of the GPL-3.0 License.
 *
 * Contributions acknowledged from
 * David Kerr... https://github.com/dkerr64
 *
 */

// Even more advanced/diagnostic fields (add if supported)
// Human-readable uptime (if you have a function for it)
// Example: char uptimeHuman[16]; snprintf(uptimeHuman, sizeof(uptimeHuman), "%lud%luh", uptime/86400, (uptime%86400)/3600);
// MDNS.addServiceTxt("ratgdo", "tcp", "uptime_human", String(uptimeHuman));

// Door protocol (if tracked)
#ifdef DOOR_PROTOCOL
MDNS.addServiceTxt("ratgdo", "tcp", "doorProtocol", String(DOOR_PROTOCOL));
#endif

// Number of active web clients (if tracked)
#ifdef ACTIVE_CLIENTS
MDNS.addServiceTxt("ratgdo", "tcp", "clients", String(ACTIVE_CLIENTS));
#endif

// OTA update available (if tracked)
#ifdef OTA_AVAILABLE
MDNS.addServiceTxt("ratgdo", "tcp", "ota", String(OTA_AVAILABLE));
#endif

// Log level (if tracked)
#ifdef LOG_LEVEL
MDNS.addServiceTxt("ratgdo", "tcp", "loglevel", String(LOG_LEVEL));
#endif

// Internal temperature (if you have a sensor)
#ifdef INTERNAL_TEMP
MDNS.addServiceTxt("ratgdo", "tcp", "temp", String(INTERNAL_TEMP));
#endif

// Internal humidity (if you have a sensor)
#ifdef INTERNAL_HUMIDITY
MDNS.addServiceTxt("ratgdo", "tcp", "humidity", String(INTERNAL_HUMIDITY));
#endif

// Timezone (if set in config)
#ifdef TIMEZONE
MDNS.addServiceTxt("ratgdo", "tcp", "timezone", String(TIMEZONE));
#endif

// Country code (if set in config)
#ifdef COUNTRY_CODE
MDNS.addServiceTxt("ratgdo", "tcp", "country", String(COUNTRY_CODE));
#endif

// LAN or WiFi interface (if tracked)
#ifdef LAN_IF
MDNS.addServiceTxt("ratgdo", "tcp", "lan", String(LAN_IF));
#endif

// IPv6 address (if enabled)
#ifdef IPV6_ADDR
MDNS.addServiceTxt("ratgdo", "tcp", "ipv6", String(IPV6_ADDR));
#endif

// Last reboot reason (if tracked)
#ifdef REBOOT_REASON
MDNS.addServiceTxt("ratgdo", "tcp", "rebootReason", String(REBOOT_REASON));
#endif

// Crash count (if tracked)
#ifdef CRASH_COUNT
MDNS.addServiceTxt("ratgdo", "tcp", "crashCount", String(CRASH_COUNT));
#endif

// Free heap memory (if available)
#ifdef free_heap
MDNS.addServiceTxt("ratgdo", "tcp", "freeHeap", String(free_heap));
#endif

// Minimum heap memory since boot (if available)
#ifdef min_heap
MDNS.addServiceTxt("ratgdo", "tcp", "minHeap", String(min_heap));
#endif

// Build number or git commit hash (if defined)
#ifdef BUILD_HASH
MDNS.addServiceTxt("ratgdo", "tcp", "build", String(BUILD_HASH));
#endif
#include <algorithm>
#include <vector>
#include <cstdio>

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>
#else
#include <WiFi.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#endif
// ArduinoJson include must be outside of any namespace
#include <ArduinoJson.h>

#include "ratgdo.h"
#include "config.h"
#include "homekit.h"
#include "json.h"
#include "peers.h"
#ifndef ESP8266
#include "vehicle.h"
#endif

// Logger tag
static const char *TAG = "ratgdo-peers";

// File-scope variables and functions
bool serviceRegistered = false;
_millis_t lastQueryAt = 0;
_millis_t lastTxtUpdateAt = 0;
constexpr uint32_t kQueryIntervalMs = 15000;
constexpr uint32_t kTxtUpdateIntervalMs = 5000;
constexpr uint32_t kStaleTimeoutMs = 60000;
constexpr size_t kMaxPeers = 12;
std::vector<PeerInfo> peers;
void update_service_txt();
void query_mdns();
bool find_peer_by_identifier(const String &peerId, PeerInfo &out);

std::vector<PeerInfo> manualPeers;
void load_manual_peers(std::vector<PeerInfo> &manualPeers)
{
    manualPeers.clear();
    const char *peersJsonStr = nullptr;
    auto val = userConfig->get(cfg_peers);
    if (std::holds_alternative<configStr>(val))
    {
        peersJsonStr = std::get<configStr>(val).str;
    }
    if (!peersJsonStr || !*peersJsonStr)
        return;
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, peersJsonStr);
    if (err)
        return;
    if (!doc.is<JsonArray>())
        return;
    for (JsonObject obj : doc.as<JsonArray>())
    {
        PeerInfo info;
        info.id = obj["id"].as<String>();
        info.name = obj["name"].as<String>();
        info.host = obj["host"].as<String>();
        info.ip.fromString(obj["ip"].as<const char *>());
        info.port = obj["port"] | 80;
        info.firmware = obj["firmware"].as<String>();
        info.doorState = obj["door"].as<String>();
        info.lockState = obj["lock"].as<String>();
        info.lightOn = obj["lightOn"] | false;
        info.hasLaser = obj["hasLaser"] | false;
        info.paired = obj["paired"] | false;
        info.rssi = obj["rssi"] | 0;
        info.lastSeen = _millis();
        manualPeers.push_back(info);
    }
}

bool find_peer_by_identifier(const String &peerId, PeerInfo &out)
{
    if (!peerId.length())
        return false;

    auto match = std::find_if(peers.begin(), peers.end(), [&](const PeerInfo &peer)
                              {
            if (peer.id.length() && peer.id.equalsIgnoreCase(peerId))
                return true;
            if (peer.host.length() && peer.host.equalsIgnoreCase(peerId))
                return true;
            return peer.ip.toString() == peerId; });

    if (match == peers.end())
        return false;

    out = *match;
    return true;
}

bool string_is_true(const String &value)
{
    if (!value.length())
        return false;

    return value.equalsIgnoreCase("true") || value.equalsIgnoreCase("on") || value.equalsIgnoreCase("yes") || value == "1";
}

void update_service_txt()
{
    if (!serviceRegistered)
        return;

    const char *deviceName = userConfig->getDeviceName();
    String mac = WiFi.macAddress();
    char rssiBuf[12];
    snprintf(rssiBuf, sizeof(rssiBuf), "%d", WiFi.RSSI());
    char uptimeBuf[16];
    snprintf(uptimeBuf, sizeof(uptimeBuf), "%lu", (unsigned long)(_millis() / 1000));

    MDNS.addServiceTxt("ratgdo", "tcp", "id", mac);
    MDNS.addServiceTxt("ratgdo", "tcp", "name", String(deviceName ? deviceName : ""));
    MDNS.addServiceTxt("ratgdo", "tcp", "fw", String(AUTO_VERSION));
    MDNS.addServiceTxt("ratgdo", "tcp", "door", String(DOOR_STATE(garage_door.current_state)));
    MDNS.addServiceTxt("ratgdo", "tcp", "lock", String(REMOTES_STATE(garage_door.current_lock)));
    MDNS.addServiceTxt("ratgdo", "tcp", "light", String(garage_door.light ? "on" : "off"));
    MDNS.addServiceTxt("ratgdo", "tcp", "paired", String(homekit_is_paired() ? "1" : "0"));
    MDNS.addServiceTxt("ratgdo", "tcp", "rssi", String(rssiBuf));
    MDNS.addServiceTxt("ratgdo", "tcp", "uptime", String(uptimeBuf));
    // Additional fields for multi-door dashboard and diagnostics
    MDNS.addServiceTxt("ratgdo", "tcp", "cycle", String(garage_door.openingsCount));
    MDNS.addServiceTxt("ratgdo", "tcp", "motion", String(garage_door.motion ? "on" : "off"));
    MDNS.addServiceTxt("ratgdo", "tcp", "obstruction", String(garage_door.obstructed ? "true" : "false"));
    MDNS.addServiceTxt("ratgdo", "tcp", "battery", String(garage_door.batteryState));
    MDNS.addServiceTxt("ratgdo", "tcp", "openDuration", String(garage_door.openDuration));
    MDNS.addServiceTxt("ratgdo", "tcp", "closeDuration", String(garage_door.closeDuration));
    MDNS.addServiceTxt("ratgdo", "tcp", "ttc", String(garage_door.builtInTTC));
    MDNS.addServiceTxt("ratgdo", "tcp", "model", MODEL_NAME);
    /* If you have a require-auth config, use it here. Otherwise, default to "0". */
    MDNS.addServiceTxt("ratgdo", "tcp", "secure", "0");
    MDNS.addServiceTxt("ratgdo", "tcp", "ip", WiFi.localIP().toString());
    MDNS.addServiceTxt("ratgdo", "tcp", "fwdate", __DATE__ " " __TIME__);
    /* If you have a paired client count function, use it here. Otherwise, default to "0". */
    MDNS.addServiceTxt("ratgdo", "tcp", "pairedClients", "0");
#ifndef ESP8266
    // Values specific to the ESP32 build.
    MDNS.addServiceTxt("ratgdo", "tcp", "hasLaser", String(userConfig->getLaserEnabled() ? "1" : "0"));
    MDNS.addServiceTxt("ratgdo", "tcp", "lastchange", String(lastVehicleChangeAt / 1000));
    MDNS.addServiceTxt("ratgdo", "tcp", "vehicle", String(vehicleDistance));
#endif
}

#ifndef ESP8266
void upsert_peer(const PeerInfo &incoming)
{
    auto existing = std::find_if(peers.begin(), peers.end(), [&](const PeerInfo &peer)
                                 { return peer.id == incoming.id; });
    if (existing != peers.end())
    {
        *existing = incoming;
        return;
    }

    if (peers.size() >= kMaxPeers)
    {
        // drop oldest entry to keep memory bounded
        auto oldest = std::min_element(peers.begin(), peers.end(), [](const PeerInfo &a, const PeerInfo &b)
                                       { return a.lastSeen < b.lastSeen; });
        if (oldest != peers.end())
            peers.erase(oldest);
    }
    peers.push_back(incoming);
}

void query_mdns()
{
    const int count = MDNS.queryService("ratgdo", "tcp");
    if (count <= 0)
        return;

    const IPAddress localIp = WiFi.localIP();
    for (int i = 0; i < count; i++)
    {
        IPAddress ip = MDNS.address(i);
        if (ip == localIp)
            continue; // skip self

        PeerInfo info;
        info.ip = ip;
        info.port = MDNS.port(i);
        const String hostname = MDNS.hostname(i);
        const String txtId = MDNS.txt(i, "id");
        const String txtName = MDNS.txt(i, "name");
        const String txtFirmware = MDNS.txt(i, "fw");
        const String txtDoor = MDNS.txt(i, "door");
        const String txtLock = MDNS.txt(i, "lock");
        const String txtLight = MDNS.txt(i, "light");
        const String txtPaired = MDNS.txt(i, "paired");
        const String txtRssi = MDNS.txt(i, "rssi");
        const String txtHasLaser = MDNS.txt(i, "hasLaser");
        const String txtLaser = MDNS.txt(i, "laser");

        info.host = hostname;
        info.id = txtId.length() ? txtId : (hostname.length() ? hostname : ip.toString());
        if (txtName.length())
        {
            info.name = txtName;
        }
        else if (hostname.length())
        {
            info.name = hostname;
        }
        else
        {
            info.name = F("RATGDO");
        }
        info.firmware = txtFirmware;
        info.doorState = txtDoor;
        info.lockState = txtLock;
        info.lightOn = string_is_true(txtLight);
        // Prefer hasLaser, fallback to laser, default false
        if (txtHasLaser.length())
        {
            info.hasLaser = string_is_true(txtHasLaser);
        }
        else if (txtLaser.length())
        {
            info.hasLaser = string_is_true(txtLaser);
        }
        else
        {
            info.hasLaser = false;
        }
        info.paired = string_is_true(txtPaired);
        info.rssi = txtRssi.length() ? txtRssi.toInt() : 0;
        info.lastSeen = _millis();
        upsert_peer(info);
    }

    const _millis_t now = _millis();
    peers.erase(std::remove_if(peers.begin(), peers.end(), [&](const PeerInfo &peer)
                               { return (now - peer.lastSeen) > kStaleTimeoutMs; }),
                peers.end());
}
#endif

void peers_begin()
{
    if (serviceRegistered)
        return;

    if (!MDNS.addService("ratgdo", "tcp", 80))
    {
        ESP_LOGE(TAG, "Failed to add MDNS service for _ratgdo._tcp on port 80");
        return;
    }
    ESP_LOGI(TAG, "Added MDNS service for _ratgdo._tcp on port 80");
    serviceRegistered = true;
    update_service_txt();
}

void peers_loop()
{
    const _millis_t now = _millis();
    if (now - lastTxtUpdateAt > kTxtUpdateIntervalMs)
    {
        lastTxtUpdateAt = now;
        update_service_txt();
    }
#ifndef ESP8266
    if (now - lastQueryAt > kQueryIntervalMs)
    {
        lastQueryAt = now;
        query_mdns();
    }

    // Always reload manual peers from config
    load_manual_peers(manualPeers);
#endif
}

bool peers_available()
{
    return !peers.empty();
}

char *peers_json()
{
    char *json = status_json;
    // status_json is malloc()'d as a global during initialization of size STATUS_JSON_BUFFER_SIZE
    // Use of it needs to be serialized, this is handled in calling function in web.cpp.
    JSON_START(json);
    JSON_START_OBJ("self");
    JSON_ADD_STR("id", WiFi.macAddress().c_str());
    JSON_ADD_STR("name", userConfig->getDeviceName());
    JSON_ADD_STR("door", DOOR_STATE(garage_door.current_state));
    JSON_ADD_STR("lock", REMOTES_STATE(garage_door.current_lock));
    JSON_ADD_BOOL("lightOn", garage_door.light);
#ifdef RATGDO32_DISCO
    JSON_ADD_BOOL("hasLaser", userConfig->getLaserEnabled());
#endif
    JSON_ADD_BOOL("paired", homekit_is_paired());
    JSON_ADD_STR("firmware", (String("v") + AUTO_VERSION).c_str());
    JSON_ADD_STR("ip", WiFi.localIP().toString().c_str());
    JSON_ADD_INT("rssi", (int32_t)WiFi.RSSI());
    JSON_END_OBJ();

    if (peers.size() + manualPeers.size() == 0)
    {
        JSON_END();
        return status_json;
    }

    // We get this far only if we have peers to report.
    JSON_START_ARRAY("peers");
    for (size_t i = 0; i < peers.size(); i++)
    {
        if (i > 0)
            JSON_INSERT_COMMA_NL();
        JSON_START_OBJ(nullptr);
        JSON_ADD_STR("id", peers[i].id.c_str());
        JSON_ADD_STR("name", peers[i].name.c_str());
        JSON_ADD_STR("host", peers[i].host.c_str());
        JSON_ADD_STR("ip", peers[i].ip.toString().c_str());
        JSON_ADD_INT("port", (uint32_t)peers[i].port);
        JSON_ADD_STR("firmware", peers[i].firmware.c_str());
        JSON_ADD_STR("door", peers[i].doorState.length() ? peers[i].doorState.c_str() : "Unknown");
        JSON_ADD_STR("lock", peers[i].lockState.length() ? peers[i].lockState.c_str() : "Unknown");
        JSON_ADD_BOOL("lightOn", peers[i].lightOn);
#ifdef RATGDO32_DISCO
        JSON_ADD_BOOL("hasLaser", peers[i].hasLaser);
#endif
        JSON_ADD_BOOL("paired", peers[i].paired);
        JSON_ADD_INT("rssi", peers[i].rssi);
        JSON_ADD_INT("lastSeen", peers[i].lastSeen);
        JSON_END_OBJ();
    }

    if (peers.size() > 0 && manualPeers.size() > 0)
        JSON_INSERT_COMMA_NL();

    for (size_t i = 0; i < manualPeers.size(); i++)
    {
        if (i > 0)
            JSON_INSERT_COMMA_NL();
        JSON_START_OBJ(nullptr);
        JSON_ADD_STR("id", manualPeers[i].id.c_str());
        JSON_ADD_STR("name", manualPeers[i].name.c_str());
        JSON_ADD_STR("host", manualPeers[i].host.c_str());
        JSON_ADD_STR("ip", manualPeers[i].ip.toString().c_str());
        JSON_ADD_INT("port", (uint32_t)manualPeers[i].port);
        JSON_ADD_STR("firmware", manualPeers[i].firmware.c_str());
        JSON_ADD_STR("door", manualPeers[i].doorState.length() ? manualPeers[i].doorState.c_str() : "Unknown");
        JSON_ADD_STR("lock", manualPeers[i].lockState.length() ? manualPeers[i].lockState.c_str() : "Unknown");
        JSON_ADD_BOOL("lightOn", manualPeers[i].lightOn);
        JSON_ADD_BOOL("paired", manualPeers[i].paired);
        JSON_ADD_INT("rssi", manualPeers[i].rssi);
        JSON_ADD_INT("lastSeen", manualPeers[i].lastSeen);
        JSON_END_OBJ();
    }
    JSON_END_ARRAY();
    JSON_END();
    return status_json;
}

bool peers_send_command(const String &peerId, const String &key, const String &value, const String &authToken, int &statusCode, String &message)
{
#ifdef ESP8266
    statusCode = HTTP_CODE_NOT_IMPLEMENTED;
    message = F("Peer control not supported on ESP8266 hardware");
    return false;
#else
    if (!key.length())
    {
        statusCode = HTTP_CODE_BAD_REQUEST;
        message = F("Missing command key");
        return false;
    }

    PeerInfo target;
    if (!find_peer_by_identifier(peerId, target))
    {
        statusCode = HTTP_CODE_NOT_FOUND;
        message = F("Peer not found");
        return false;
    }

    WiFiClient client;
    HTTPClient http;
    const String url = String(F("http://")) + target.ip.toString() + ':' + String(target.port) + F("/setgdo");
    if (!http.begin(client, url))
    {
        statusCode = HTTP_CODE_BAD_GATEWAY;
        message = F("Unable to contact peer");
        return false;
    }

    http.addHeader(F("Content-Type"), F("application/x-www-form-urlencoded"));
    if (authToken.length())
    {
        String header = F("Basic ");
        header += authToken;
        http.addHeader(F("Authorization"), header);
    }

    String payload = key;
    payload += '=';
    payload += value;
    const int upstreamStatus = http.POST(payload);
    statusCode = (upstreamStatus > 0) ? upstreamStatus : HTTP_CODE_BAD_GATEWAY;
    message = http.getString();
    http.end();

    if (!message.length())
    {
        if (statusCode == HTTP_CODE_OK)
        {
            message = F("Peer acknowledged command");
        }
        else
        {
            message = String(F("Peer responded ")) + statusCode;
        }
    }
    else
    {
        message.trim();
    }

    return statusCode == HTTP_CODE_OK;
#endif
}
