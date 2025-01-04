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
#include <string>
#include <tuple>
#include <unordered_map>
#include <time.h>

// Arduino includes
#include <Ticker.h>
#include <MD5Builder.h>
#include <WebServer.h>
#include <StreamString.h>

// ESP system includes
#include "esp_core_dump.h"

// RATGDO project includes
#include "ratgdo.h"
#include "config.h"
#include "comms.h"
#include "log.h"
#include "web.h"
#include "utilities.h"
#include "homekit.h"
#include "softAP.h"
#include "json.h"
#include "led.h"
#include "vehicle.h"
#include "www/build/webcontent.h"

// Logger tag
static const char *TAG = "ratgdo-http";

// Browser cache control, time in seconds after which browser cache invalid
// This is used for CSS, JS and IMAGE file types.  Set to 30 days !!
#define CACHE_CONTROL (60 * 60 * 24 * 30)

// Forward declare the internal URI handling functions...
void handle_reset();
void handle_status();
void handle_everything();
void handle_setgdo();
void handle_logout();
void handle_auth();
void handle_subscribe();
void handle_showlog();
void handle_showrebootlog();
void handle_crashlog();
void handle_clearcrashlog();
#ifdef CRASH_DEBUG
void handle_forcecrash();
void handle_crash_oom();
void *crashptr;
char *test_str = NULL;
#endif
void handle_update();
void handle_firmware_upload();
void SSEHandler(uint8_t channel);

// Built in URI handlers
const char restEvents[] = "/rest/events/";
const std::unordered_map<std::string, std::pair<const HTTPMethod, void (*)()>> builtInUri = {
    {"/status.json", {HTTP_GET, handle_status}},
    {"/reset", {HTTP_POST, handle_reset}},
    {"/reboot", {HTTP_POST, handle_reboot}},
    {"/setgdo", {HTTP_POST, handle_setgdo}},
    {"/logout", {HTTP_GET, handle_logout}},
    {"/auth", {HTTP_GET, handle_auth}},
    {"/showlog", {HTTP_GET, handle_showlog}},
    {"/showrebootlog", {HTTP_GET, handle_showrebootlog}},
    {"/wifiap", {HTTP_POST, handle_wifiap}},
    {"/wifinets", {HTTP_GET, handle_wifinets}},
    {"/setssid", {HTTP_POST, handle_setssid}},
    {"/rescan", {HTTP_POST, handle_rescan}},
    {"/crashlog", {HTTP_GET, handle_crashlog}},
    {"/clearcrashlog", {HTTP_GET, handle_clearcrashlog}},
#ifdef CRASH_DEBUG
    {"/forcecrash", {HTTP_POST, handle_forcecrash}},
    {"/crashoom", {HTTP_POST, handle_crash_oom}},
#endif
    {"/rest/events/subscribe", {HTTP_GET, handle_subscribe}}};

WebServer server(80);

// Local copy of door status
GarageDoor last_reported_garage_door;
bool last_reported_paired = false;
bool last_reported_assist_laser = false;
uint64_t lastDoorUpdateAt = 0;
GarageDoorCurrentState lastDoorState = (GarageDoorCurrentState)0xff;

static bool web_setup_done = false;

// Implement our own firmware update so can enforce MD5 check.
// Based on ESP8266HTTPUpdateServer
std::string _updaterError;
bool _authenticatedUpdate;
char firmwareMD5[36] = "";
size_t firmwareSize = 0;

// Common HTTP responses
const char response400missing[] = "400: Bad Request, missing argument\n";
const char response400invalid[] = "400: Bad Request, invalid argument\n";
const char response404[] = "404: Not Found\n";
const char response503[] = "503: Service Unavailable.\n";
const char response200[] = "HTTP/1.1 200 OK\nContent-Type: text/plain\nConnection: close\n\n";

const char *http_methods[] = {"HTTP_ANY", "HTTP_GET", "HTTP_HEAD", "HTTP_POST", "HTTP_PUT", "HTTP_PATCH", "HTTP_DELETE", "HTTP_OPTIONS"};

// For Server Sent Events (SSE) support
// Just reloading page causes register on new channel.  So we need a reasonable number
// to accommodate "extra" until old one is detected as disconnected.
#define SSE_MAX_CHANNELS 8
struct SSESubscription
{
    IPAddress clientIP;
    WiFiClient client;
    Ticker heartbeatTimer;
    bool SSEconnected;
    int SSEfailCount;
    String clientUUID;
    bool logViewer;
};
SSESubscription subscription[SSE_MAX_CHANNELS];
// During firmware update note which subscribed client is updating
SSESubscription *firmwareUpdateSub = NULL;
uint8_t subscriptionCount = 0;

SemaphoreHandle_t jsonMutex = NULL;

#define JSON_BUFFER_SIZE 2048
char *json = NULL;

#define DOOR_STATE(s) (s == 0) ? "Open" : (s == 1) ? "Closed"  \
                                      : (s == 2)   ? "Opening" \
                                      : (s == 3)   ? "Closing" \
                                      : (s == 4)   ? "Stopped" \
                                                   : "Unknown"
#define LOCK_STATE(s) (s == 0) ? "Unsecured" : (s == 1) ? "Secured" \
                                           : (s == 2)   ? "Jammed"  \
                                                        : "Unknown"

void web_loop()
{
    if (!web_setup_done)
        return;

    uint64_t upTime = millis64();
    xSemaphoreTake(jsonMutex, portMAX_DELAY);
    START_JSON(json);
    if (garage_door.active && garage_door.current_state != lastDoorState)
    {
        RINFO(TAG, "Current Door State changing from %d to %d", lastDoorState, garage_door.current_state);
        if (enableNTP && clockSet)
        {
            if (lastDoorState == 0xff)
            {
                // initialize with saved time.
                // lastDoorUpdateAt is milliseconds relative to system reboot time.
                lastDoorUpdateAt = (userConfig->getDoorUpdateAt() != 0) ? ((userConfig->getDoorUpdateAt() - time(NULL)) * 1000) + upTime : 0;
            }
            else
            {
                // first state change after a reboot, so really is a state change.
                userConfig->set(cfg_doorUpdateAt, (int)time(NULL));
                lastDoorUpdateAt = upTime;
            }
        }
        else
        {
            lastDoorUpdateAt = (lastDoorState == 0xff) ? 0 : upTime;
        }
        // if no NTP....  lastDoorUpdateAt = (lastDoorState == 0xff) ? 0 : upTime;
        lastDoorState = garage_door.current_state;
        // We send milliseconds relative to current time... ie updated X milliseconds ago
        // First time through, zero offset from upTime, which is when we last rebooted)
        ADD_INT(json, "lastDoorUpdateAt", (upTime - lastDoorUpdateAt));
    }
    if (garage_door.has_distance_sensor)
    {
        if (vehicleStatusChange)
        {
            vehicleStatusChange = false;
            ADD_STR(json, "vehicleStatus", vehicleStatus);
        }
        ADD_BOOL_C(json, "assistLaser", laser.state(), last_reported_assist_laser);
    }
    // Conditional macros, only add if value has changed
    ADD_BOOL_C(json, "paired", homekit_is_paired(), last_reported_paired);
    ADD_STR_C(json, "garageDoorState", DOOR_STATE(garage_door.current_state), garage_door.current_state, last_reported_garage_door.current_state);
    ADD_STR_C(json, "garageLockState", LOCK_STATE(garage_door.current_lock), garage_door.current_lock, last_reported_garage_door.current_lock);
    ADD_BOOL_C(json, "garageLightOn", garage_door.light, last_reported_garage_door.light);
    ADD_BOOL_C(json, "garageMotion", garage_door.motion, last_reported_garage_door.motion);
    ADD_BOOL_C(json, "garageObstructed", garage_door.obstructed, last_reported_garage_door.obstructed);
    if (strlen(json) > 2)
    {
        // Have we added anything to the JSON string?
        ADD_INT(json, "upTime", upTime);
        END_JSON(json);
        REMOVE_NL(json);
        SSEBroadcastState(json);
    }
    xSemaphoreGive(jsonMutex);
    server.handleClient();
}

void setup_web()
{
    RINFO(TAG, "=== Starting HTTP web server ===");
    IRAM_START
    // IRAM heap is used only for allocating globals, to leave as much regular heap
    // available during operations.  We need to carefully monitor useage so as not
    // to exceed available IRAM.  We can adjust the LOG_BUFFER_SIZE (in log.h) if we
    // need to make more space available for initialization.
    json = (char *)malloc(JSON_BUFFER_SIZE);
    RINFO(TAG, "Allocated buffer for JSON, size: %d", JSON_BUFFER_SIZE);
    // We allocated json as a global block.  We are on dual core CPU.  We need to serialize access to the resource.
    jsonMutex = xSemaphoreCreateMutex();
    last_reported_paired = homekit_is_paired();

    if (motionTriggers.asInt == 0)
    {
        // maybe just initialized. If we have motion sensor then set that and write back to file
        if (garage_door.has_motion_sensor)
        {
            motionTriggers.bit.motion = 1;
            userConfig->set(cfg_motionTriggers, motionTriggers.asInt);
        }
    }
    else if (garage_door.has_motion_sensor != (bool)motionTriggers.bit.motion)
    {
        // sync up web page tracking of whether we have motion sensor or not.
        RINFO(TAG, "Motion trigger mismatch, reset to %d", (uint8_t)garage_door.has_motion_sensor);
        motionTriggers.bit.motion = (uint8_t)garage_door.has_motion_sensor;
        userConfig->set(cfg_motionTriggers, motionTriggers.asInt);
    }
    RINFO(TAG, "Motion triggers, motion : %d, obstruction: %d, light key: %d, door key: %d, lock key: %d, asInt: %d",
          motionTriggers.bit.motion,
          motionTriggers.bit.obstruction,
          motionTriggers.bit.lightKey,
          motionTriggers.bit.doorKey,
          motionTriggers.bit.lockKey,
          motionTriggers.asInt);
    lastDoorUpdateAt = 0;
    lastDoorState = (GarageDoorCurrentState)0xff;

    RINFO(TAG, "Registering URI handlers");
    server.on("/update", HTTP_POST, handle_update, handle_firmware_upload);
    server.onNotFound(handle_everything);
    // here the list of headers to be recorded
    const char *headerkeys[] = {"If-None-Match"};
    size_t headerkeyssize = sizeof(headerkeys) / sizeof(char *);
    // ask server to track these headers
    server.collectHeaders(headerkeys, headerkeyssize);
    server.begin();
    // initialize all the Server-Sent Events (SSE) slots.
    for (uint8_t i = 0; i < SSE_MAX_CHANNELS; i++)
    {
        subscription[i].SSEconnected = false;
        subscription[i].clientIP = INADDR_NONE;
        subscription[i].clientUUID.clear();
    }
    IRAM_END("HTTP server started");
    web_setup_done = true;
    return;
}

void handle_notfound()
{
    RINFO(TAG, "Sending 404 Not Found for: %s with method: %s to client: %s", server.uri().c_str(), http_methods[server.method()], server.client().remoteIP().toString().c_str());
    server.send_P(404, type_txt, response404);
    return;
}

String *ratgdoAuthenticate(HTTPAuthMethod mode, String enteredUsernameOrReq, String extraParams[])
{
    // RINFO(TAG, "Auth method: %d", mode);                // DIGEST_AUTH
    // RINFO(TAG, "User: %s", enteredUsernameOrReq);       // Username
    // RINFO(TAG, "Param 0: %s", extraParams[0].c_str());  // Realm
    // RINFO(TAG, "Param 1: %s", extraParams[1].c_str());  // URI
    String *pw = new String(nvRam->read(nvram_ratgdo_pw, "password").c_str());
    return pw;
}

#define AUTHENTICATE()                                                                 \
    if (userConfig->getPasswordRequired() && !server.authenticate(ratgdoAuthenticate)) \
        return server.requestAuthentication(DIGEST_AUTH, www_realm);

void handle_auth()
{
    AUTHENTICATE();
    server.send_P(200, type_txt, PSTR("Authenticated"));
    return;
}

void handle_reset()
{
    AUTHENTICATE();
    RINFO(TAG, "... reset requested");
    homekit_unpair();
    server.client().setNoDelay(true);
    server.send_P(200, type_txt, PSTR("Device has been un-paired from HomeKit. Rebooting...\n"));
    // Allow time to process send() before terminating web server...
    delay(500);
    server.stop();
    sync_and_restart();
    return;
}

void handle_reboot()
{
    RINFO(TAG, "System boot time:    %s", timeString(lastRebootAt));
    RINFO(TAG, "Reboot requested at: %s", timeString());
    const char *resp = "Rebooting...\n";
    server.client().setNoDelay(true);
    server.send(200, type_txt, resp);
    // Allow time to process send() before terminating web server...
    delay(500);
    server.stop();
    sync_and_restart();
    return;
}

void load_page(const char *page)
{
    if (webcontent.count(page) == 0)
        return handle_notfound();

    const char *data = (char *)webcontent.at(page).data;
    int length =  webcontent.at(page).length;
    const char *typeP = webcontent.at(page).type;
    const char *crc32 = webcontent.at(page).crc32.c_str();
    // need local copy as strcmp_P cannot take two PSTR()'s
    char type[MAX_MIME_TYPE_LEN];
    strncpy_P(type, typeP, MAX_MIME_TYPE_LEN);

    bool cache = false;
    char cacheHdr[24] = "no-cache, no-store";
    char matchHdr[8] = "";
    if ((CACHE_CONTROL > 0) &&
        (!strcmp_P(type, type_css) || !strcmp_P(type, type_js) || strstr_P(type, PSTR("image"))))
    {
        sprintf(cacheHdr, "max-age=%i", CACHE_CONTROL);
        cache = true;
    }
    if (server.hasHeader(F("If-None-Match")))
        strlcpy(matchHdr, server.header(F("If-None-Match")).c_str(), sizeof(matchHdr));

    HTTPMethod method = server.method();
    if (strcmp(crc32, matchHdr))
    {
        server.sendHeader(F("Content-Encoding"), F("gzip"));
        server.sendHeader(F("Cache-Control"), cacheHdr);
        if (cache)
            server.sendHeader(F("ETag"), crc32);
        if (method == HTTP_HEAD)
        {
            RINFO(TAG, "Client %s requesting: %s (HTTP_HEAD, type: %s)", server.client().remoteIP().toString().c_str(), page, type);
            server.send_P(200, type, "", 0);
        }
        else
        {
            RINFO(TAG, "Client %s requesting: %s (HTTP_GET, type: %s, length: %i)", server.client().remoteIP().toString().c_str(), page, type, length);
            server.send_P(200, type, data, length);
        }
    }
    else
    {
        RINFO(TAG, "Sending 304 not modified to client %s requesting: %s (method: %s, type: %s)", server.client().remoteIP().toString().c_str(), page, http_methods[method], type);
        server.send_P(304, type, "", 0);
    }
    return;
}

void handle_everything()
{
    HTTPMethod method = server.method();
    String page = server.uri();
    const char *uri = page.c_str();

    // too verbose... RINFO(TAG, "Handle everything for %s", uri);
    if (builtInUri.count(uri) > 0)
    {
        // requested page matches one of our built-in handlers
        RINFO(TAG, "Client %s requesting: %s (method: %s)", server.client().remoteIP().toString().c_str(), uri, http_methods[method]);
        if (method == builtInUri.at(uri).first)
            return builtInUri.at(uri).second();
        else
            return handle_notfound();
    }
    else if ((method == HTTP_GET) && (!strncmp_P(uri, restEvents, strlen(restEvents))))
    {
        // Request for "/rest/events/" with a channel number appended
        uri += strlen(restEvents);
        unsigned int channel = atoi(uri);
        if (channel < SSE_MAX_CHANNELS)
            return SSEHandler(channel);
        else
            return handle_notfound();
    }
    else if (method == HTTP_GET || method == HTTP_HEAD)
    {
        // HTTP_GET that does not match a built-in handler
        if (server.uri() == "/")
            return load_page("/index.html");
        else
            return load_page(uri);
    }
    // it is a HTTP_POST for unknown URI
    return handle_notfound();
}

void handle_status()
{
    uint64_t upTime = millis64();
#define clientCount 0
    // Build the JSON string
    xSemaphoreTake(jsonMutex, portMAX_DELAY);
    START_JSON(json);
    ADD_INT(json, "upTime", upTime);
    ADD_STR(json, cfg_deviceName, userConfig->getDeviceName().c_str());
    ADD_STR(json, "userName", userConfig->getwwwUsername().c_str());
    ADD_BOOL(json, "paired", homekit_is_paired());
    ADD_STR(json, "firmwareVersion", std::string(AUTO_VERSION).c_str());
    // TODO find and show HomeKit accessory ID... ADD_STR(json, "accessoryID", accessoryID);
    // TODO monitor number of HomeKit "clients" connected... ADD_INT(json, "clients", clientCount);
    ADD_STR(json, cfg_localIP, userConfig->getLocalIP().c_str());
    ADD_STR(json, cfg_subnetMask, userConfig->getSubnetMask().c_str());
    ADD_STR(json, cfg_gatewayIP, userConfig->getGatewayIP().c_str());
    ADD_STR(json, cfg_nameserverIP, userConfig->getNameserverIP().c_str());
    ADD_STR(json, "macAddress", Network.macAddress().c_str());
    ADD_STR(json, "wifiSSID", WiFi.SSID().c_str());
    ADD_STR(json, "wifiRSSI", (std::to_string(WiFi.RSSI()) + " dBm, Channel " + std::to_string(WiFi.channel())).c_str());
    ADD_STR(json, "wifiBSSID", WiFi.BSSIDstr().c_str());
    // TODO support locking to specific WiFi access point... ADD_BOOL(json, "lockedAP", wifiConf.bssid_set)
    ADD_BOOL(json, "lockedAP", false);
    ADD_INT(json, cfg_GDOSecurityType, userConfig->getGDOSecurityType());
    ADD_STR(json, "garageDoorState", garage_door.active ? DOOR_STATE(garage_door.current_state) : DOOR_STATE(255));
    ADD_STR(json, "garageLockState", LOCK_STATE(garage_door.current_lock));
    ADD_BOOL(json, "garageLightOn", garage_door.light);
    ADD_BOOL(json, "garageMotion", garage_door.motion);
    ADD_BOOL(json, "garageObstructed", garage_door.obstructed);
    ADD_BOOL(json, cfg_passwordRequired, userConfig->getPasswordRequired());
    ADD_INT(json, cfg_rebootSeconds, userConfig->getRebootSeconds());
    ADD_INT(json, "freeHeap", free_heap);
    ADD_INT(json, "minHeap", min_heap);
    // TODO monitor stack... ADD_INT(json, "minStack", 0);
    ADD_INT(json, "crashCount", abs(crashCount));
    // TODO support WiFi PhyMode... ADD_INT(json, cfg_wifiPhyMode, userConfig->getWifiPhyMode());
    // TODO support WiFi TX Power... ADD_INT(json, cfg_wifiPower, userConfig->getWifiPower());
    ADD_BOOL(json, cfg_staticIP, userConfig->getStaticIP());
    ADD_BOOL(json, cfg_syslogEn, userConfig->getSyslogEn());
    ADD_STR(json, cfg_syslogIP, userConfig->getSyslogIP().c_str());
    ADD_INT(json, cfg_syslogPort, userConfig->getSyslogPort());
    ADD_INT(json, cfg_TTCseconds, userConfig->getTTCseconds());
    ADD_INT(json, cfg_vehicleThreshold, userConfig->getVehicleThreshold());
    ADD_INT(json, cfg_motionTriggers, motionTriggers.asInt);
    ADD_INT(json, cfg_LEDidle, userConfig->getLEDidle());
    // We send milliseconds relative to current time... ie updated X milliseconds ago
    ADD_INT(json, "lastDoorUpdateAt", (upTime - lastDoorUpdateAt));
    ADD_BOOL(json, "enableNTP", enableNTP);
    if (enableNTP)
    {
        if (clockSet)
        {
            ADD_INT(json, "serverTime", time(NULL));
        }
    }
    ADD_STR(json, cfg_timeZone, userConfig->getTimeZone().c_str());
    ADD_BOOL(json, "distanceSensor", garage_door.has_distance_sensor);
    if (garage_door.has_distance_sensor)
    {
        ADD_STR(json, "vehicleStatus", vehicleStatus);
        ADD_INT(json, "vehicleDist", vehicleDistance);
        last_reported_assist_laser = laser.state();
        ADD_BOOL(json, "assistLaser", last_reported_assist_laser);
    }
    ADD_BOOL(json, cfg_laserEnabled, userConfig->getLaserEnabled());
    ADD_BOOL(json, cfg_laserHomeKit, userConfig->getLaserHomeKit());
    ADD_INT(json, cfg_assistDuration, userConfig->getAssistDuration());
    END_JSON(json);

    // send JSON straight to serial port
    Serial.printf("%s\n", json);
    last_reported_garage_door = garage_door;

    server.sendHeader(F("Cache-Control"), F("no-cache, no-store"));
    server.send_P(200, type_json, json);
    RINFO(TAG, "JSON length: %d", strlen(json));
    xSemaphoreGive(jsonMutex);
    return;
}

void handle_logout()
{
    RINFO(TAG, "Handle logout");
    return server.requestAuthentication(DIGEST_AUTH, www_realm);
}

bool helperResetDoor(const std::string &key, const std::string &value, configSetting *action)
{
    reset_door();
    return true;
}

bool helperGarageLightOn(const std::string &key, const std::string &value, configSetting *action)
{
    set_light((value == "1") ? true : false);
    return true;
}

bool helperGarageDoorState(const std::string &key, const std::string &value, configSetting *action)
{
    if (value == "1")
        open_door();
    else
        close_door();
    return true;
}

bool helperGarageLockState(const std::string &key, const std::string &value, configSetting *action)
{
    set_lock((value == "1") ? 1 : 0);
    return true;
}

bool helperCredentials(const std::string &key, const std::string &value, configSetting *action)
{
    char *newUsername = strstr(value.c_str(), "username");
    char *newCredentials = strstr(value.c_str(), "credentials");
    char *newPassword = strstr(value.c_str(), "password");
    if (!(newUsername && newCredentials && newPassword))
        return false;

    // JSON string passed in.
    // Very basic parsing, not using library functions to save memory
    // find the colon after the key string
    newUsername = strchr(newUsername, ':') + 1;
    newCredentials = strchr(newCredentials, ':') + 1;
    newPassword = strchr(newPassword, ':') + 1;
    // for strings find the double quote
    newUsername = strchr(newUsername, '"') + 1;
    newCredentials = strchr(newCredentials, '"') + 1;
    newPassword = strchr(newPassword, '"') + 1;
    // null terminate the strings (at closing quote).
    *strchr(newUsername, '"') = (char)0;
    *strchr(newCredentials, '"') = (char)0;
    *strchr(newPassword, '"') = (char)0;
    // save values...
    RINFO(TAG, "Set user credentials: %s : %s (%s)", newUsername, newPassword, newCredentials);
    userConfig->set(cfg_wwwUsername, newUsername);
    userConfig->set(cfg_wwwCredentials, newCredentials);
    nvRam->write(nvram_ratgdo_pw, newPassword);
    return true;
}

bool helperUpdateUnderway(const std::string &key, const std::string &value, configSetting *action)
{
    firmwareSize = 0;
    firmwareUpdateSub = NULL;
    char *md5 = strstr(value.c_str(), "md5");
    char *size = strstr(value.c_str(), "size");
    char *uuid = strstr(value.c_str(), "uuid");

    if (!(md5 && size && uuid))
        return false;

    // JSON string of passed in.
    // Very basic parsing, not using library functions to save memory
    // find the colon after the key string
    md5 = strchr(md5, ':') + 1;
    size = strchr(size, ':') + 1;
    uuid = strchr(uuid, ':') + 1;
    // for strings find the double quote
    md5 = strchr(md5, '"') + 1;
    uuid = strchr(uuid, '"') + 1;
    // null terminate the strings (at closing quote).
    *strchr(md5, '"') = (char)0;
    *strchr(uuid, '"') = (char)0;
    // RINFO(TAG,"MD5: %s, UUID: %s, Size: %d", md5, uuid, atoi(size));
    // save values...
    strlcpy(firmwareMD5, md5, sizeof(firmwareMD5));
    firmwareSize = atoi(size);
    for (uint8_t channel = 0; channel < SSE_MAX_CHANNELS; channel++)
    {
        if (subscription[channel].SSEconnected && subscription[channel].clientUUID == uuid && subscription[channel].client.connected())
        {
            firmwareUpdateSub = &subscription[channel];
            break;
        }
    }
    return true;
}

bool helperFactoryReset(const std::string &key, const std::string &value, configSetting *action)
{
    RINFO(TAG, "Factory reset requested");
    nvRam->erase();
    reset_door();
    homeSpan.processSerialCommand("F");
    return true;
}

bool helperAssistLaser(const std::string &key, const std::string &value, configSetting *action)
{
    if (value == "1")
        laser.on();
    else
        laser.off();
    notify_homekit_laser(value == "1");
    return true;
}

void handle_setgdo()
{
    // Build-in handlers that do not set a configuration value, or if they do they set multiple values.
    // key, {reboot, wifiChanged, value, fn to call}
    static const std::unordered_map<std::string, configSetting> setGDOhandlers = {
        {"resetDoor", {true, false, 0, helperResetDoor}},
        {"garageLightOn", {false, false, 0, helperGarageLightOn}},
        {"garageDoorState", {false, false, 0, helperGarageDoorState}},
        {"garageLockState", {false, false, 0, helperGarageLockState}},
        {"credentials", {false, false, 0, helperCredentials}}, // parse out wwwUsername and credentials
        {"updateUnderway", {false, false, 0, helperUpdateUnderway}},
        {"factoryReset", {true, false, 0, helperFactoryReset}},
        {"assistLaser", {false, false, 0, helperAssistLaser}},
    };
    bool reboot = false;
    bool error = false;
    bool wifiChanged = false;
    bool saveSettings = false;
    std::string key;
    std::string value;
    configSetting actions;

    if (!((server.args() == 1) && (server.argName(0) == cfg_timeZone)))
    {
        // We will allow setting of time zone without authentication
        AUTHENTICATE();
    }

    // Loop over all the GDO settings passed in...
    for (int i = 0; i < server.args(); i++)
    {
        key = server.argName(i).c_str();
        value = server.arg(i).c_str();

        if (setGDOhandlers.count(key))
        {
            RINFO(TAG, "Call handler for Key: %s, Value: %s", key.c_str(), value.c_str());
            actions = setGDOhandlers.at(key);
            if (actions.fn)
            {
                error = error || !actions.fn(key, value, &actions);
            }
            reboot = reboot || actions.reboot;
            wifiChanged = wifiChanged || actions.wifiChanged;
        }
        else if (userConfig->contains(key))
        {
            RINFO(TAG, "Configuration set for Key: %s, Value: %s", key.c_str(), value.c_str());
            actions = userConfig->getDetail(key);
            if (actions.fn)
            {
                // Value will be set within called function
                error = error || !actions.fn(key, value, &actions);
            }
            else
            {
                // No function to call, set value directly.
                userConfig->set(key, value);
            }
            reboot = reboot || actions.reboot;
            wifiChanged = wifiChanged || actions.wifiChanged;
            saveSettings = true;
        }
        else
        {
            ESP_LOGW(TAG, "Invalid Key: %s, Value: %s (F)", key.c_str(), value.c_str());
            error = true;
        }
        if (error)
            break;
    }

    RINFO(TAG, "SetGDO Complete");

    if (error)
    {
        // Simple error handling...
        RINFO(TAG, "Sending %s, for: %s", response400invalid, server.uri().c_str());
        server.send_P(400, type_txt, response400invalid);
        return;
    }

    if (saveSettings)
    {
        userConfig->set(cfg_wifiChanged, wifiChanged);
    }
    if (reboot)
    {
        // Some settings require reboot to take effect
        server.send_P(200, type_html, PSTR("<p>Success. Reboot.</p>"));
        RINFO(TAG, "SetGDO Restart required");
        // Allow time to process send() before terminating web server...
        delay(500);
        server.stop();
        sync_and_restart();
    }
    else
    {
        server.send_P(200, type_html, PSTR("<p>Success.</p>"));
    }
    return;
}

void SSEheartbeat(SSESubscription *s)
{
    if (!s)
        return;

    if (!(s->clientIP))
        return;

    if (!(s->SSEconnected))
    {
        if (s->SSEfailCount++ >= 5)
        {
            // 5 heartbeats have failed... assume client will not connect
            // and free up the slot
            subscriptionCount--;
            s->heartbeatTimer.detach();
            s->clientIP = INADDR_NONE;
            s->clientUUID.clear();
            s->SSEconnected = false;
            RINFO(TAG, "Client %s timeout waiting to listen, remove SSE subscription.  Total subscribed: %d", s->clientIP.toString().c_str(), subscriptionCount);
            // no need to stop client socket because it is not live yet.
        }
        else
        {
            RINFO(TAG, "Client %s not yet listening for SSE", s->clientIP.toString().c_str());
        }
        return;
    }

    if (s->client.connected())
    {
        static int8_t lastRSSI = 0;
        static int16_t lastVehicleDistance = 0;
        xSemaphoreTake(jsonMutex, portMAX_DELAY);
        START_JSON(json);
        ADD_INT(json, "upTime", millis64());
        ADD_INT(json, "freeHeap", free_heap);
        ADD_INT(json, "minHeap", min_heap);
        // TODO monitor stack... ADD_INT(json, "minStack", ESP.getFreeContStack());
        if (garage_door.has_distance_sensor && (lastVehicleDistance != vehicleDistance))
        {
            lastVehicleDistance = vehicleDistance;
            ADD_INT(json, "vehicleDist", vehicleDistance);
        }
        if (lastRSSI != WiFi.RSSI())
        {
            lastRSSI = WiFi.RSSI();
            ADD_STR(json, "wifiRSSI", (std::to_string(lastRSSI) + " dBm, Channel " + std::to_string(WiFi.channel())).c_str());
        }
        /* TODO monitor number of "clients" connected to HomeKit
        static int lastClientCount = 0;
        if (arduino_homekit_get_running_server() && arduino_homekit_get_running_server()->nfds != lastClientCount)
        {
            lastClientCount = arduino_homekit_get_running_server()->nfds;
            ADD_INT(json, "clients", lastClientCount);
        }
        */
        END_JSON(json);
        REMOVE_NL(json);
        s->client.printf("event: message\nretry: 15000\ndata: %s\n\n", json);
        xSemaphoreGive(jsonMutex);
    }
    else
    {
        subscriptionCount--;
        s->heartbeatTimer.detach();
        s->client.clear();
        s->client.stop();
        s->clientIP = INADDR_NONE;
        s->clientUUID.clear();
        s->SSEconnected = false;
        RINFO(TAG, "Client %s not listening, remove SSE subscription. Total subscribed: %d", s->clientIP.toString().c_str(), subscriptionCount);
    }
}

void SSEHandler(uint8_t channel)
{
    if (server.args() != 1)
    {
        RINFO(TAG, "Sending %s, for: %s", response400missing, server.uri().c_str());
        server.send_P(400, type_txt, response400missing);
        return;
    }
    WiFiClient client = server.client();
    SSESubscription &s = subscription[channel];
    if (s.clientUUID != server.arg(0))
    {
        RINFO(TAG, "Client %s with IP %s tries to listen for SSE but not subscribed", server.arg(0).c_str(), client.remoteIP().toString().c_str());
        return handle_notfound();
    }
    client.setNoDelay(true);
    s.client = client;                               // capture SSE server client connection
    server.setContentLength(CONTENT_LENGTH_UNKNOWN); // the payload can go on forever
    server.sendContent_P(PSTR("HTTP/1.1 200 OK\nContent-Type: text/event-stream;\nConnection: keep-alive\nCache-Control: no-cache\nAccess-Control-Allow-Origin: *\n\n"));
    s.SSEconnected = true;
    s.SSEfailCount = 0;
    s.heartbeatTimer.attach_ms(1000, [channel, &s]
                               { SSEheartbeat(&s); });
    RINFO(TAG, "Client %s listening for SSE events on channel %d", client.remoteIP().toString().c_str(), channel);
}

void handle_subscribe()
{
    uint8_t channel;
    IPAddress clientIP = server.client().remoteIP(); // get IP address of client
    std::string SSEurl = restEvents;

    if (subscriptionCount == SSE_MAX_CHANNELS)
    {
        RINFO(TAG, "Client %s SSE Subscription declined, subscription count: %d", clientIP.toString().c_str(), subscriptionCount);
        for (channel = 0; channel < SSE_MAX_CHANNELS; channel++)
        {
            RINFO(TAG, "Client %d: %s at %s", channel, subscription[channel].clientUUID.c_str(), subscription[channel].clientIP.toString().c_str());
        }
        return handle_notfound(); // We ran out of channels
    }

    if (clientIP == INADDR_NONE)
    {
        RINFO(TAG, "Sending %s, for: %s as clientIP missing", response400invalid, server.uri().c_str());
        server.send_P(400, type_txt, response400invalid);
        return;
    }

    // check we were passed at least one arguement
    if (server.args() < 1)
    {
        RINFO(TAG, "Sending %s, for: %s", response400missing, server.uri().c_str());
        server.send_P(400, type_txt, response400missing);
        return;
    }

    // find the UUID and whether client wants to receive log messages
    int id = 0;
    bool logViewer = false;
    for (int i = 0; i < server.args(); i++)
    {
        if (server.argName(i) == "id")
            id = i;
        else if (server.argName(i) == "log")
            logViewer = true;
    }

    // check if we already have a subscription for this UUID
    for (channel = 0; channel < SSE_MAX_CHANNELS; channel++)
    {
        if (subscription[channel].clientUUID == server.arg(id))
        {
            if (subscription[channel].SSEconnected)
            {
                // Already connected.  We need to close it down as client will be reconnecting
                RINFO(TAG, "SSE Subscribe - client %s with IP %s already connected on channel %d, remove subscription", server.arg(id).c_str(), clientIP.toString().c_str(), channel);
                subscription[channel].heartbeatTimer.detach();
                subscription[channel].client.clear();
                subscription[channel].client.stop();
            }
            else
            {
                // Subscribed but not connected yet, so nothing to close down.
                RINFO(TAG, "SSE Subscribe - client %s with IP %s already subscribed but not connected on channel %d", server.arg(id).c_str(), clientIP.toString().c_str(), channel);
            }
            break;
        }
    }

    if (channel == SSE_MAX_CHANNELS)
    {
        // ended loop above without finding a match, so need to allocate a free slot
        ++subscriptionCount;
        for (channel = 0; channel < SSE_MAX_CHANNELS; channel++)
            if (!subscription[channel].clientIP)
                break;
    }
    subscription[channel] = {clientIP, server.client(), Ticker(), false, 0, server.arg(id), logViewer};
    SSEurl += std::to_string(channel);
    RINFO(TAG, "SSE Subscription for client %s with IP %s: event bus location: %s, Total subscribed: %d", server.arg(id).c_str(), clientIP.toString().c_str(), SSEurl.c_str(), subscriptionCount);
    server.sendHeader(F("Cache-Control"), F("no-cache, no-store"));
    server.send_P(200, type_txt, SSEurl.c_str());
}

void handle_crashlog()
{
    RINFO(TAG, "Request to display crash log...");
    WiFiClient client = server.client();
    client.print(response200);
    ratgdoLogger->printCrashLog(client);
    client.stop();
}

void handle_showlog()
{
    WiFiClient client = server.client();
    client.print(response200);
    ratgdoLogger->printMessageLog(client);
    client.stop();
}

void handle_showrebootlog()
{
    WiFiClient client = server.client();
    client.print(response200);
    ratgdoLogger->printSavedLog(client);
    client.stop();
}

void handle_clearcrashlog()
{
    AUTHENTICATE();
    RINFO(TAG, "Clear saved crash log");
    esp_core_dump_image_erase();
    crashCount = 0;
    server.send_P(200, type_txt, PSTR("Crash log cleared\n"));
}

#ifdef CRASH_DEBUG
void handle_crash_oom()
{
    RINFO(TAG, "Attempting to use up all memory");
    server.send_P(200, type_txt, PSTR("Attempting to use up all memory\n"));
    delay(1000);
    for (int i = 0; i < 30; i++)
    {
        RINFO(TAG, "malloc(1024)");
        crashptr = malloc(1024);
    }
}

void handle_forcecrash()
{
    RINFO(TAG, "Attempting to null ptr deref");
    server.send_P(200, type_txt, PSTR("Attempting to null ptr deref\n"));
    delay(1000);
    RINFO(TAG, "Result: %s", test_str);
}
#endif // CRASH_DEBUG

void SSEBroadcastState(const char *data, BroadcastType type)
{
    if (!web_setup_done)
        return;

    // Flash LED to signal activity
    led.flash(FLASH_MS);

    // if nothing subscribed, then return
    if (subscriptionCount == 0)
        return;

    for (uint8_t i = 0; i < SSE_MAX_CHANNELS; i++)
    {
        if (subscription[i].SSEconnected && subscription[i].client.connected())
        {
            if (type == LOG_MESSAGE)
            {
                if (subscription[i].logViewer)
                {
                    subscription[i].client.printf_P(PSTR("event: logger\ndata: %s\n\n"), data);
                }
            }
            else if (type == RATGDO_STATUS)
            {
                String IPaddrstr = IPAddress(subscription[i].clientIP).toString();
                RINFO(TAG, "SSE send to client %s on channel %d, data: %s", IPaddrstr.c_str(), i, data);
                subscription[i].client.printf_P(PSTR("event: message\ndata: %s\n\n"), data);
            }
        }
    }
}

// Implement our own firmware update so can enforce MD5 check.
// Based on HTTPUpdateServer
void _setUpdaterError()
{
    StreamString str;
    Update.printError(str);
    _updaterError = str.c_str();
    RINFO(TAG, "Update error: %s", str.c_str());
}

void handle_update()
{
    bool verify = !strcmp(server.arg("action").c_str(), "verify");

    server.sendHeader(F("Access-Control-Allow-Headers"), "*");
    server.sendHeader(F("Access-Control-Allow-Origin"), "*");
    AUTHENTICATE();

    server.client().setNoDelay(true);
    if (!verify && Update.hasError())
    {
        // Error logged in _setUpdaterError
        // TODO how to handle firmware upload failurem was... eboot_command_clear();
        RERROR(TAG, "Firmware upload error. Aborting update, not rebooting");
        server.send(400, "text/plain", _updaterError.c_str());
        return;
    }

    if (server.args() > 0)
    {
        // Don't reboot, user/client must explicity request reboot.
        server.send_P(200, type_txt, PSTR("Upload Success.\n"));
    }
    else
    {
        // Legacy... no query string args, so automatically reboot...
        server.send_P(200, type_txt, PSTR("Upload Success. Rebooting...\n"));
        // Allow time to process send() before terminating web server...
        delay(500);
        server.stop();
        sync_and_restart();
    }
}

void handle_firmware_upload()
{
    // handler for the file upload, gets the sketch bytes, and writes
    // them through the Update object
    static size_t uploadProgress;
    static unsigned int nextPrintPercent;
    HTTPUpload &upload = server.upload();
    static bool verify = false;
    static size_t size = 0;
    static const char *md5 = NULL;

    if (upload.status == UPLOAD_FILE_START)
    {
        _updaterError.clear();

        _authenticatedUpdate = !userConfig->getPasswordRequired() || server.authenticate(ratgdoAuthenticate);
        if (!_authenticatedUpdate)
        {
            RINFO(TAG, "Unauthenticated Update");
            return;
        }
        RINFO(TAG, "Update: %s", upload.filename.c_str());
        verify = !strcmp(server.arg("action").c_str(), "verify");
        size = atoi(server.arg("size").c_str());
        md5 = server.arg("md5").c_str();

        // We are updating.  If size and MD5 provided, save them
        firmwareSize = size;
        if (strlen(md5) > 0)
            strlcpy(firmwareMD5, md5, sizeof(firmwareMD5));

        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        RINFO(TAG, "Available space for upload: %lu", maxSketchSpace);
        RINFO(TAG, "Firmware size: %s", (firmwareSize > 0) ? std::to_string(firmwareSize).c_str() : "Unknown");
        RINFO(TAG, "Flash chip speed %d MHz", ESP.getFlashChipSpeed() / 1000000);
        // struct eboot_command ebootCmd;
        // eboot_command_read(&ebootCmd);
        // RINFO(TAG, "eboot_command: 0x%08X 0x%08X [0x%08X 0x%08X 0x%08X (%d)]", ebootCmd.magic, ebootCmd.action, ebootCmd.args[0], ebootCmd.args[1], ebootCmd.args[2], ebootCmd.args[2]);
        if (!verify)
        {
            // Close HomeKit server so we don't have to handle HomeKit network traffic during update
            // Only if not verifying as either will have been shutdown on immediately prior upload, or we
            // just want to verify without disrupting operation of the HomeKit service.
            // TODO close HomeKit server during OTA update... arduino_homekit_close();
            // IRAM_START
            // IRAM_END("HomeKit Server Closed");
        }
        if (!verify && !Update.begin((firmwareSize > 0) ? firmwareSize : maxSketchSpace, U_FLASH))
        {
            _setUpdaterError();
        }
        else if (strlen(firmwareMD5) > 0)
        {
            // uncomment for testing...
            // char firmwareMD5[] = "675cbfa11d83a792293fdc3beb199cXX";
            RINFO(TAG, "Expected MD5: %s", firmwareMD5);
            Update.setMD5(firmwareMD5);
            if (firmwareSize > 0)
            {
                uploadProgress = 0;
                nextPrintPercent = 10;
                RINFO(TAG, "%s progress: 00%%", verify ? "Verify" : "Update");
            }
        }
    }
    else if (_authenticatedUpdate && upload.status == UPLOAD_FILE_WRITE && !_updaterError.length())
    {
        // Progress dot dot dot
        Serial.printf(".");
        if (firmwareSize > 0)
        {
            uploadProgress += upload.currentSize;
            unsigned int uploadPercent = (uploadProgress * 100) / firmwareSize;
            if (uploadPercent >= nextPrintPercent)
            {
                Serial.printf("\n"); // newline after the dot dot dots
                RINFO(TAG, "%s progress: %i%%", verify ? "Verify" : "Update", uploadPercent);
                SSEheartbeat(firmwareUpdateSub); // keep SSE connection alive.
                nextPrintPercent += 10;
                // Report percentage to browser client if it is listening
                if (firmwareUpdateSub && firmwareUpdateSub->client.connected())
                {
                    xSemaphoreTake(jsonMutex, portMAX_DELAY);
                    START_JSON(json);
                    ADD_INT(json, "uploadPercent", uploadPercent);
                    END_JSON(json);
                    REMOVE_NL(json);
                    firmwareUpdateSub->client.printf_P(PSTR("event: uploadStatus\ndata: %s\n\n"), json);
                    xSemaphoreGive(jsonMutex);
                }
            }
        }
        if (!verify)
        {
            // Don't write if verifying... we will just check MD5 of the flash at the end.
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
                _setUpdaterError();
        }
    }
    else if (_authenticatedUpdate && upload.status == UPLOAD_FILE_END && !_updaterError.length())
    {
        Serial.printf("\n"); // newline after last of the dot dot dots
        if (!verify)
        {
            if (Update.end(true))
            {
                RINFO(TAG, "Upload size: %zu", upload.totalSize);
            }
            else
            {
                _setUpdaterError();
            }
        }
    }
    else if (_authenticatedUpdate && upload.status == UPLOAD_FILE_ABORTED)
    {
        if (!verify)
            Update.end();
        RINFO(TAG, "%s was aborted", verify ? "Verify" : "Update");
    }
    delay(0);
}
