/****************************************************************************
 * RATGDO HomeKit
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
 * Mitchell Solomon... https://github.com/mitchjs
 * Haglerd...          https://github.com/Haglerd  (2025-26: forceClose /setgdo POST handler, auto-close timer)
 *
 */

// C/C++ language includes
#include <string>
#include <tuple>
#include <unordered_map>
#include <time.h>

// ESP system includes
#include <Ticker.h>
#include <MD5Builder.h>
#include <StreamString.h>
#ifdef ESP8266
#include <arduino_homekit_server.h>
#include <eboot_command.h>
#include <ESP8266mDNS.h>
#else
#include "esp_core_dump.h"
#include <ESPmDNS.h>
#ifndef ESP8266
// v24: setsockopt(SO_SNDTIMEO) on SSE TCP sockets to bound write times.
// esp_timer_get_time for clientWrite slow-write instrumentation.
#include <sys/socket.h>
#include <sys/time.h>
#include <esp_timer.h>
#endif
#endif

// RATGDO project includes
#ifdef USE_GDOLIB
#include "gdo.h"
#endif
#include "ratgdo.h"
#include "config.h"
#include "comms.h"
#include "web.h"
#include "homekit.h"
#include "softAP.h"
#include "json.h"
#include "led.h"
#ifdef ESP8266
#include "wifi_8266.h"
#endif
#ifdef RATGDO32_DISCO
#include "vehicle.h"
#endif
// built by "build_web_content.py"
#include "webcontent.h"

// Logger tag
static const char *TAG = "ratgdo-http";

// Browser cache control, time in seconds after which browser cache invalid
// This is used for CSS, JS and IMAGE file types.  Set to 30 days !!
#define CACHE_CONTROL (60 * 60 * 24 * 30)

// Forward declare the internal URI handling functions...
void handle_reset();
void handle_reconnect_homekit();
void handle_refresh_mdns();
void handle_dump_homekit_state();
void handle_status();
void handle_everything();
void handle_setgdo();
void handle_logout();
void handle_auth();
void handle_subscribe();
void handle_unsubscribe(); // v27: best-effort beacon cleanup, see handle_unsubscribe()
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
void SSEHandler(uint32_t channel);
void add_static_mdns();
void add_dynamic_mdns();

// Built in URI handlers
const char restEvents[] = "/rest/events/";
const std::unordered_map<std::string, std::pair<const HTTPMethod, void (*)()>> builtInUri = {
    {"/status.json", {HTTP_GET, handle_status}},
    {"/reset", {HTTP_POST, handle_reset}},
    {"/reboot", {HTTP_POST, handle_reboot}},
    {"/reconnectHomeKit", {HTTP_POST, handle_reconnect_homekit}},
    {"/refreshHomeKitMDNS", {HTTP_POST, handle_refresh_mdns}},
    {"/dumpHomeKitState", {HTTP_POST, handle_dump_homekit_state}},
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
    {"/rest/events/subscribe", {HTTP_GET, handle_subscribe}},
    // v27: paired endpoint for navigator.sendBeacon() on beforeunload
    {"/rest/events/unsubscribe", {HTTP_POST, handle_unsubscribe}}};

// Declare web server on HTTP port 80.
#ifdef ESP8266
ESP8266WebServer server(80);
#else
WebServer server(80);
#endif

// Local copy of door status
GarageDoor last_reported_garage_door;
bool last_reported_paired = false;
bool last_reported_assist_laser = false;
_millis_t lastDoorUpdateAt;
_millis_t lastDoorOpenAt;
_millis_t lastDoorCloseAt;
GarageDoorCurrentState lastDoorState = (GarageDoorCurrentState)0xff;
static bool new_ipv4_address = false;
static bool new_ipv6_address = false;

bool web_setup_done = false;

// Implement our own firmware update so can enforce MD5 check.
// Based on ESP8266HTTPUpdateServer
std::string _updaterError;
bool _authenticatedUpdate;
char firmwareMD5[36] = "";
size_t firmwareSize = 0;

// Common HTTP responses
constexpr char response400missing[] = "400: Bad Request, missing argument\n";
constexpr char response400invalid[] = "400: Bad Request, invalid argument\n";
constexpr char response404[] = "404: Not Found\n";
constexpr char response503[] = "503: Service Unavailable.\n";
constexpr char response200[] = "HTTP/1.1 200 OK\nContent-Type: text/plain\nConnection: close\n\n";

const char *http_methods[] = {"HTTP_ANY", "HTTP_GET", "HTTP_HEAD", "HTTP_POST", "HTTP_PUT", "HTTP_PATCH", "HTTP_DELETE", "HTTP_OPTIONS"};

// All this is to support a 303 redirect to js.map files when debugging, so we don't have to embed in our firmware !!!!
#ifndef STRINGIFY
#define STRINGIFY_HELPER(x) #x
#define STRINGIFY(x) STRINGIFY_HELPER(x)
#endif
// If not building in main github repo, then add -D GITUSER=your_userid to the compile line (no quotes, STRINGIFY adds that here)
#ifndef GITUSER
#define _GITUSER "ratgdo"
#else
#define _GITUSER STRINGIFY(GITUSER)
#endif
#ifndef GITREPO
#ifdef ESP8266
#define _GITREPO "homekit-ratgdo"
#else
#define _GITREPO "homekit-ratgdo32"
#endif
#else
#define _GITREPO STRINGIFY(GITREPO)
#endif
#ifndef GITBRANCH
#define _GITBRANCH "main"
#else
#define _GITBRANCH STRINGIFY(GITBRANCH)
#endif
constexpr char gitUser[] = _GITUSER;
constexpr char gitRepo[] = _GITREPO;
constexpr char gitRawURL[] = "https://raw.githubusercontent.com/" _GITUSER "/" _GITREPO "/refs/heads/" _GITBRANCH;
constexpr char gitTaggedURL[] = "https://raw.githubusercontent.com/" _GITUSER "/" _GITREPO "/refs/tags/v" AUTO_VERSION;

// For Server Sent Events (SSE) support
// Just reloading page causes register on new channel.  So we need a reasonable number
// to accommodate "extra" until old one is detected as disconnected.
#define SSE_MAX_CHANNELS 8
// v27: orphan-slot sweep timeouts. Run from service_timer_loop, independent
// of the per-slot heartbeat Ticker (which doesn't fire when heartbeat=0).
//   PREHANDSHAKE: slot has been subscribed but the EventSource never came
//                 back to /events/N. Browser was closed mid-flight or the
//                 GET hung. 15s is well past any realistic round-trip.
//   IDLE        : slot is fully connected but hasn't received any
//                 broadcast/heartbeat traffic in 5min. Belt-and-suspenders
//                 for a wedged TCP socket whose client.connected() still
//                 returns true. 5min > 60s max heartbeat so a healthy
//                 client never trips this.
// v28: was 15000UL — tightened to 5000UL after observing real-world
// reconnect-storm where 8 slots filled (e.g. browser EventSource auto-
// retrying after a 503 storm) faster than the sweep could drain them.
// 5s is well above the typical EventSource handshake (<500ms even on
// cellular), so a slow legitimate client just hits a fresh subscribe
// retry — same recovery as 15s, but storms can't outpace the sweep.
// v29: SSE_IDLE_TIMEOUT_MS raised 120000 → 300000 (2min → 5min) as
// defense-in-depth on top of the BUFFER_FULL stamping fix in clientWriteEx.
// The primary fix is in clientWriteEx — callers now stamp lastActivity
// on OK or BUFFER_FULL (both = "broadcast loop reached this slot and
// tried"). 5min idle is past most NAT-binding refresh intervals; a
// genuinely-wedged slot still gets caught well before users notice
// stale data, and class 5b (client.connected() == false once lwIP
// KeepAlive fires, typically <60s) remains the actual safety net for
// dead sockets with stale lwIP cached state.
#define SSE_PREHANDSHAKE_TIMEOUT_MS  5000UL
#define SSE_IDLE_TIMEOUT_MS         300000UL
struct SSESubscription
{
    IPAddress clientIP;
    WiFiClient client;
    Ticker heartbeatTimer;
    uint32_t heartbeatInterval;
    bool SSEconnected;
    int SSEfailCount;
    String clientUUID;
    bool logViewer;
    // v22: deferred-cleanup flag to break the heartbeatTimer self-detach
    // crash. SSEheartbeat() runs IN the Ticker callback context — calling
    // heartbeatTimer.detach() from there ends up in vTaskDelete on the
    // Ticker's own task, which corrupts the FreeRTOS task list and
    // panics in uxListRemove. Instead we set this flag and let
    // service_timer_loop() (main loop context) do the actual remove.
    volatile bool pendingRemove;
    // v27: timestamps for the orphan-slot sweep that runs independent
    // of the heartbeat Ticker. Pre-v27 the only liveness driver was
    // SSEheartbeat which only ran if heartbeatInterval > 0 — clients
    // that subscribed with heartbeat=0 (like logs.html does) leaked
    // slots forever once the page navigated away. Now the orphan sweep
    // in service_timer_loop catches three classes of leaks: (1)
    // subscribed-but-never-connected to /events/N within 15s, (2)
    // SSEconnected with client.connected()==false, (3) idle for over
    // 120s with no broadcast traffic (heartbeat=0 belt+suspenders).
    // v28: int64_t was an int64_t-tearing risk — read by sweep
    // (main loop) racing writers in Ticker / SSEBroadcastState.
    // Truncated uint32_t millis is safe: 32-bit aligned writes are
    // atomic on ESP32, subtraction is wrap-safe modulo 2^32, and
    // the intervals we compare against (15000/120000 ms) fit
    // comfortably. Even the ~49.7-day rollover causes at most one
    // bad delta on the wrap tick — recoverable next tick.
    volatile uint32_t subscribedAt;
    volatile uint32_t lastActivity;
};
SSESubscription subscription[SSE_MAX_CHANNELS];
// During firmware update note which subscribed client is updating
SSESubscription *firmwareUpdateSub = NULL;
uint32_t subscriptionCount = 0;

// Performance management - removed redundant connection tracking
#define MIN_REQUEST_INTERVAL_MS 100

// Performance monitoring
static uint32_t request_count = 0;
static uint32_t max_response_time = 0;

#ifdef ESP8266
// ESP8266 is single core / single threaded, no mutex's.
#define TAKE_MUTEX()
#define GIVE_MUTEX()
#else
// ESP32 is multi-core, need to serialize access to JSON buffers
static SemaphoreHandle_t jsonMutex = NULL;
#define TAKE_MUTEX() \
    if (jsonMutex)   \
    xSemaphoreTake(jsonMutex, portMAX_DELAY)
#define GIVE_MUTEX() \
    if (jsonMutex)   \
    xSemaphoreGive(jsonMutex)
#endif

// v29: tri-state return from clientWrite. Pre-v29 was bool, which
// conflated "TCP buffer full this instant" (flow control) with "write
// failed" (real wedge). The 120s idle reap (sweep class 5c) treated
// both as "no activity" → reaped healthy slots on chronically-slow
// tunnels (Tailscale, mobile data with marginal signal).
enum class SseWriteResult : uint8_t { OK, BUFFER_FULL, FAILED };

// mDNS update management... re-announcing every 2 minutes.
#define MDNS_ANNOUNCE_TIMEOUT (2 * 60 * 1000)
// But not more often than every 10 seconds if pending updates.
#define MDNS_UPDATE_INTERVAL (10 * 1000)
static _millis_t lastMDNSupdate = 0;
static bool mdnsUpdatePending = false;

// Connection throttling
#define MAX_CONCURRENT_REQUESTS 8
#define REQUEST_TIMEOUT_MS 2000
struct ActiveRequest
{
    IPAddress clientIP;
    _millis_t startTime;
    bool inUse;
};
ActiveRequest activeRequests[MAX_CONCURRENT_REQUESTS];
int activeRequestCount = 0;

#define CLIENT_WRITE_TIMEOUT 500
// v24: maximum time a single SSE clientWrite is allowed to spend in
// client.write before we give up + report the slow channel. Was
// effectively unbounded — Arduino-ESP32 WiFiClient::setTimeout only
// affects READS, not writes, so a wedged subscriber would block the
// caller indefinitely. Combined with the lwIP SO_SNDTIMEO set in
// SSEHandler this caps the write to ~200ms.
#define CLIENT_SLOW_WRITE_MS 200
static char writeBuffer[512];
// v24: bump per-broadcast slow-write counter when a single channel
// exceeds CLIENT_SLOW_WRITE_MS. homekit_health_log reads + zeros this
// every 180s. Climbing values pre-freeze identify a wedged subscriber.
volatile uint32_t sseSlowWrites = 0;
// v29: count of clientWrite calls skipped because the lwIP send buffer
// couldn't accept the full payload (flow control). Distinguishes
// chronically-slow connections (Tailscale, congested links) from
// genuinely-wedged sockets — drives whether v30 needs per-slot
// adaptive idle timeouts.
volatile uint32_t sseBufferFullSkips = 0;
// v27: live count of allocated SSE slots (refreshed by sweep_sse_orphans
// every service tick) and a windowed counter of slots reaped by the
// sweep. homekit_health_log reads both every 180s and zeros sseOrphansReaped
// (sseSlotsAlloc is a snapshot, not a counter). Lets us see at a glance
// whether the slot leak that caused the v26 25s-post-boot wedge is back.
volatile uint32_t sseSlotsAlloc = 0;
volatile uint32_t sseOrphansReaped = 0;
// v29: tri-state version of clientWrite. Distinguishes "lwIP send buffer
// can't accept the full payload right now" (BUFFER_FULL — peer is alive
// but slow / link is congested, e.g. Tailscale tunnel) from "client.write
// returned 0 after lwIP accepted bytes for delivery" (FAILED — real wedge,
// socket already stopped). Callers stamp lastActivity on OK or BUFFER_FULL
// so the orphan-sweep idle check (5c) doesn't misclassify a chronically-
// flow-controlled slot as idle. Only FAILED skips the stamp.
SseWriteResult clientWriteEx(WiFiClient client, const char *data)
{
    size_t len = strlen(data);
    size_t written = 0;
#ifdef ESP8266
    client.flush(); // make sure previous data all sent.
#endif
    // v24 fast-path: if the TCP send buffer can't accept the full
    // payload right now, the subscriber isn't draining and we'd block
    // in client.write below. Skip + report; v29: this is BUFFER_FULL
    // (flow control), NOT a wedge — caller still stamps lastActivity.
    int avail = client.availableForWrite();
    if (avail >= 0 && (size_t)avail < len)
    {
        sseBufferFullSkips++;
        ESP_LOGD(TAG, "SSE clientWrite skipped — buffer full (need %u, have %d)", (unsigned)len, avail);
        return SseWriteResult::BUFFER_FULL;
    }
#ifndef ESP8266
    uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000ULL);
#else
    uint32_t t0 = millis();
#endif
    written = client.write(data, len);
#ifndef ESP8266
    uint32_t dt = (uint32_t)(esp_timer_get_time() / 1000ULL) - t0;
#else
    uint32_t dt = millis() - t0;
#endif
    if (dt > CLIENT_SLOW_WRITE_MS)
    {
        sseSlowWrites++;
        ESP_LOGW(TAG, "SSE clientWrite slow: %ums for %u bytes (subscriber may be wedged)", dt, (unsigned)len);
    }
    if (written == 0)
    {
        YIELD();
        client.stop();
        ESP_LOGW(TAG, "Failed writing to WiFi Client (%d of %d), connection closed.", written, len);
        return SseWriteResult::FAILED;
    }
    return SseWriteResult::OK;
}

// v29: thin bool wrapper for any callers we don't migrate to the tri-state.
// Returns true on OK or BUFFER_FULL (both = "broadcast loop reached this
// slot and tried"; peer is alive enough to keep), false only on FAILED.
bool clientWrite(WiFiClient client, const char *data)
{
    return clientWriteEx(client, data) != SseWriteResult::FAILED;
}

// Helper functions for connection throttling
bool registerRequest()
{
    IPAddress clientIP = server.client().remoteIP();
    _millis_t now = _millis();

    // Clean up timed-out requests
    for (int i = 0; i < MAX_CONCURRENT_REQUESTS; i++)
    {
        if (activeRequests[i].inUse && (now - activeRequests[i].startTime > REQUEST_TIMEOUT_MS))
        {
            ESP_LOGD(TAG, "Request timeout for client %s", activeRequests[i].clientIP.toString().c_str());
            activeRequests[i].inUse = false;
            activeRequestCount--;
        }
    }

    // Check if we're at capacity
    if (activeRequestCount >= MAX_CONCURRENT_REQUESTS)
    {
        ESP_LOGE(TAG, "Max concurrent requests reached, rejecting %s", clientIP.toString().c_str());
        return false;
    }

    // Find a free slot
    for (int i = 0; i < MAX_CONCURRENT_REQUESTS; i++)
    {
        if (!activeRequests[i].inUse)
        {
            activeRequests[i].clientIP = clientIP;
            activeRequests[i].startTime = now;
            activeRequests[i].inUse = true;
            activeRequestCount++;
            return true;
        }
    }

    return false;
}

void unregisterRequest()
{
    IPAddress clientIP = server.client().remoteIP();

    for (int i = 0; i < MAX_CONCURRENT_REQUESTS; i++)
    {
        if (activeRequests[i].inUse && activeRequests[i].clientIP == clientIP)
        {
            activeRequests[i].inUse = false;
            if (activeRequestCount > 0)
                activeRequestCount--; // Prevent negative count
            break;
        }
    }
}

void notify_new_ipv4_address()
{
    TAKE_MUTEX();
    new_ipv4_address = true;
    GIVE_MUTEX();
};

#ifndef ESP8266
void notify_new_ipv6_address()
{
    TAKE_MUTEX();
    new_ipv6_address = true;
    GIVE_MUTEX();
};
#endif

void web_loop()
{
    if (!web_setup_done)
        return;

    static char *json = status_json;
    _millis_t upTime = _millis();
    static _millis_t last_request_time = 0;

    // manage frequency of mDNS updates
    if (mdnsUpdatePending)
    {
        if (upTime - lastMDNSupdate > MDNS_UPDATE_INTERVAL)
        {
            // This function also resets mdnsUpdatePending and lastMDNSupdate.
            add_dynamic_mdns();
        }
    }
    else if (upTime - lastMDNSupdate > MDNS_ANNOUNCE_TIMEOUT)
    {
        // if it has been more than MDNS_ANNOUNCE_TIMEOUT since last update, re-announce
        add_dynamic_mdns();
    }

    TAKE_MUTEX();
    JSON_START(json);
    if (garage_door.active && garage_door.current_state != lastDoorState)
    {
        ESP_LOGD(TAG, "Current Door State changing from %s to %s", DOOR_STATE(lastDoorState), DOOR_STATE(garage_door.current_state));
        if (enableNTP && clockSet)
        {
            time_t timeNow = time(NULL);
            if (lastDoorState == 0xff)
            {
                // initialize with saved time.
                // lastDoorUpdateAt is milliseconds relative to system reboot time.
                lastDoorUpdateAt = (userConfig->getDoorUpdateAt() != 0) ? ((userConfig->getDoorUpdateAt() - timeNow) * 1000) + upTime : 0;
                lastDoorOpenAt = (userConfig->getDoorOpenAt() != 0) ? ((userConfig->getDoorOpenAt() - timeNow) * 1000) + upTime : 0;
                lastDoorCloseAt = (userConfig->getDoorCloseAt() != 0) ? ((userConfig->getDoorCloseAt() - timeNow) * 1000) + upTime : 0;
            }
            else
            {
                // first state change after a reboot, so really is a state change.
                lastDoorUpdateAt = upTime;
                userConfig->set(cfg_doorUpdateAt, (int)timeNow);
                if (garage_door.current_state == GarageDoorCurrentState::CURR_OPEN)
                {
                    lastDoorOpenAt = upTime;
                    userConfig->set(cfg_doorOpenAt, (int)timeNow);
                }
                if (garage_door.current_state == GarageDoorCurrentState::CURR_CLOSED)
                {
                    lastDoorCloseAt = upTime;
                    userConfig->set(cfg_doorCloseAt, (int)timeNow);
                }
                ESP8266_SAVE_CONFIG();
            }
        }
        else
        {
            // No realtime set, use upTime.
            lastDoorUpdateAt = (lastDoorState == 0xff) ? 0 : upTime;
            if (garage_door.current_state == GarageDoorCurrentState::CURR_OPEN)
                lastDoorOpenAt = lastDoorUpdateAt;
            if (garage_door.current_state == GarageDoorCurrentState::CURR_CLOSED)
                lastDoorCloseAt = lastDoorUpdateAt;
        }
        lastDoorState = garage_door.current_state;
        // We send milliseconds relative to current time... ie updated X milliseconds ago
        // First time through, zero offset from upTime, which is when we last rebooted)
        JSON_ADD_INT(cfg_doorUpdateAt, (upTime - lastDoorUpdateAt));
        JSON_ADD_INT(cfg_doorOpenAt, (upTime - lastDoorOpenAt));
        JSON_ADD_INT(cfg_doorCloseAt, (upTime - lastDoorCloseAt));
    }
#ifdef RATGDO32_DISCO
    // Feature not available on ESP8266
    if (garage_door.has_distance_sensor)
    {
        if (vehicleStatusChange)
        {
            vehicleStatusChange = false;
            JSON_ADD_STR("vehicleStatus", vehicleStatus);
        }
        JSON_ADD_BOOL_C("assistLaser", laser.state(), last_reported_assist_laser);
    }
#endif
    // Conditional macros, only add if value has changed
    JSON_ADD_BOOL_C("paired", homekit_is_paired(), last_reported_paired);
    JSON_ADD_STR_C("garageDoorState", DOOR_STATE(garage_door.current_state), garage_door.current_state, last_reported_garage_door.current_state);
    JSON_ADD_STR_C("garageLockState", REMOTES_STATE(garage_door.current_lock), garage_door.current_lock, last_reported_garage_door.current_lock);
    JSON_ADD_BOOL_C("garageLightOn", garage_door.light, last_reported_garage_door.light);
    JSON_ADD_BOOL_C("garageMotion", garage_door.motion, last_reported_garage_door.motion);
    JSON_ADD_BOOL_C("pinBasedObst", garage_door.pinModeObstructionSensor, last_reported_garage_door.pinModeObstructionSensor);
    JSON_ADD_BOOL_C("garageObstructed", garage_door.obstructed, last_reported_garage_door.obstructed);
    JSON_ADD_BOOL_C("garageSec1Emulated", garage_door.wallPanelEmulated, last_reported_garage_door.wallPanelEmulated);
    if (doorControlType == 2)
    {
        JSON_ADD_INT_C("batteryState", garage_door.batteryState, last_reported_garage_door.batteryState);
        JSON_ADD_INT_C("openingsCount", garage_door.openingsCount, last_reported_garage_door.openingsCount);
        JSON_ADD_INT_C(cfg_builtInTTC, garage_door.builtInTTC, last_reported_garage_door.builtInTTC);
        JSON_ADD_INT_C("builtInTTCremaining", garage_door.builtInTTCremaining, last_reported_garage_door.builtInTTCremaining);
        JSON_ADD_BOOL_C("builtInTTChold", garage_door.builtInTTChold, last_reported_garage_door.builtInTTChold);
    }
    JSON_ADD_INT_C("openDuration", garage_door.openDuration, last_reported_garage_door.openDuration);
    JSON_ADD_INT_C("closeDuration", garage_door.closeDuration, last_reported_garage_door.closeDuration);
    JSON_ADD_INT_C("ttcActive", is_ttc_active(), last_reported_garage_door.ttcActive);
    if (new_ipv4_address)
    {
        JSON_ADD_STR(cfg_localIP, userConfig->getLocalIP());
        JSON_ADD_STR(cfg_subnetMask, userConfig->getSubnetMask());
        JSON_ADD_STR(cfg_gatewayIP, userConfig->getGatewayIP());
        JSON_ADD_STR(cfg_nameserverIP, userConfig->getNameserverIP());
        new_ipv4_address = false;
    }
#ifndef ESP8266
    if (new_ipv6_address)
    {
        JSON_ADD_STR("ipv6Addresses", ipv6_addresses);
        new_ipv6_address = false;
    }
#endif
    // got any json?
    if (strlen(json) > 2)
    {
        // Have we added anything to the JSON string?
        JSON_ADD_INT("upTime", upTime);
        JSON_END();
        if (strlen(json) > STATUS_JSON_BUFFER_SIZE * 8 / 10)
        {
            ESP_LOGW(TAG, "WARNING web_loop JSON length: %d is over 80%% of available buffer", strlen(json));
        }
        JSON_REMOVE_NL(json);
        // v24: copy + release-mutex before the broadcast. Same audit
        // pattern as log.cpp / handle_status / SSEheartbeat — the
        // broadcast walks every SSE subscriber writing TCP, which can
        // block on a wedged client. The mutex isn't needed for the
        // write itself, only the buffer construction.
        bool doFanout = !firmwareUpdateSub;
        char localJson[STATUS_JSON_BUFFER_SIZE];
        if (doFanout)
        {
            strncpy(localJson, json, sizeof(localJson) - 1);
            localJson[sizeof(localJson) - 1] = '\0';
        }
        mdnsUpdatePending = true;
        GIVE_MUTEX();

        if (doFanout)
            SSEBroadcastState(localJson);
    }
    else
    {
        GIVE_MUTEX();
    }
    static time_t mdnsDoorUpdateAt = 0;
    if (lastDoorUpdateAt && !mdnsDoorUpdateAt)
    {
        // First time setting it... subsequent changes handled above.
        mdnsDoorUpdateAt = lastDoorUpdateAt;
        mdnsUpdatePending = true;
    }
    // Rate limiting - minimum interval between requests
    _millis_t current_time = _millis();
    if (current_time - last_request_time < MIN_REQUEST_INTERVAL_MS)
    {
        return; // Skip this cycle to enforce rate limit
    }

    server.handleClient();
    // Update last request time after handling client
    last_request_time = current_time;
}

void setup_web()
{
    if (web_setup_done)
        return;

    ESP_LOGI(TAG, "=== Starting HTTP web server ===");
#ifndef USE_GDOLIB
    if (!garage_door.active)
    {
        // Garage door should be active by now (will have set door state, etc.)
        // If for some reason it is not, send a get status command (Sec+ 2.0 doors only)
        ESP_LOGI(TAG, "Garage door comms not active yet, sending a getStatus to recover");
        send_get_status();
    }
#endif
    IRAM_START(TAG);
    // IRAM heap is used only for allocating globals, to leave as much regular heap
    // available during operations.  We need to carefully monitor useage so as not
    // to exceed available IRAM.  We can adjust the LOG_BUFFER_SIZE (in log.h) if we
    // need to make more space available for initialization.
#ifndef ESP8266
    // We allocated json as a global block.  We are on dual core CPU.  We need to serialize access to the resource.
    jsonMutex = xSemaphoreCreateMutex();
#endif
    last_reported_paired = homekit_is_paired();

    if (!garage_door.has_motion_sensor && (bool)motionTriggers.bit.motion)
    {
        // If we do not have a motion sensor, disable motion sensing setting
        motionTriggers.bit.motion = 0;
        userConfig->set(cfg_motionTriggers, motionTriggers.asInt);
        ESP8266_SAVE_CONFIG();
    }

    ESP_LOGI(TAG, "Has motion sensor %s, Triggers... motion %d, obstruction %d, light key %d, door key %d, lock key %d (asInt: %d)",
             garage_door.has_motion_sensor ? "true" : "false",
             motionTriggers.bit.motion,
             motionTriggers.bit.obstruction,
             motionTriggers.bit.lightKey,
             motionTriggers.bit.doorKey,
             motionTriggers.bit.lockKey,
             motionTriggers.asInt);
    lastDoorUpdateAt = 0;
    lastDoorOpenAt = 0;
    lastDoorCloseAt = 0;
    lastDoorState = (GarageDoorCurrentState)0xff;

    ESP_LOGI(TAG, "Registering URI handlers");
    server.on("/update", HTTP_POST, handle_update, handle_firmware_upload);
    server.onNotFound(handle_everything);
    // here the list of headers to be recorded
    // Origin/Referer/Host are needed for CSRF guard on /setgdo
    const char *headerkeys[] = {"If-None-Match", "Origin", "Referer", "Host"};
    size_t headerkeyssize = sizeof(headerkeys) / sizeof(char *);
    // ask server to track these headers
    server.collectHeaders(headerkeys, headerkeyssize);
    server.begin();
    // initialize all the Server-Sent Events (SSE) slots.
    for (uint32_t i = 0; i < SSE_MAX_CHANNELS; i++)
    {
        subscription[i].SSEconnected = false;
        subscription[i].clientIP = INADDR_NONE;
        subscription[i].clientUUID.clear();
        // v27: explicit init for the orphan-sweep + deferred-remove fields.
        // Without this the struct is left with whatever pattern BSS happened
        // to land on (zero in practice, but the sweep is too sensitive to
        // assume — a non-zero subscribedAt + zero clientIP would still get
        // skipped by the FREE check, but leaving it implicit is brittle).
        subscription[i].pendingRemove = false;
        subscription[i].subscribedAt = 0;
        subscription[i].lastActivity = 0;
    }

    // Initialize connection tracking
    for (int i = 0; i < MAX_CONCURRENT_REQUESTS; i++)
    {
        activeRequests[i].inUse = false;
    }
    activeRequestCount = 0;

    IRAM_END(TAG);

    if (MDNS.addService("http", "tcp", 80))
    {
        ESP_LOGI(TAG, "Added MDNS service for _http._tcp on port 80");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to add MDNS service for _http._tcp on port 80");
    }

    if (MDNS.addService("ratgdo", "tcp", 80))
    {
        ESP_LOGI(TAG, "Added MDNS service for _ratgdo._tcp on port 80");
        add_static_mdns();
        add_dynamic_mdns();
    }
    else
    {
        ESP_LOGE(TAG, "Failed to add MDNS service for _ratgdo._tcp on port 80");
    }

    web_setup_done = true;
    return;
}

void handle_notfound()
{
    ESP_LOGD(TAG, "Sending 404 Not Found for: %s with method: %s to client: %s", server.uri().c_str(), http_methods[server.method()], server.client().remoteIP().toString().c_str());
    server.send_P(404, type_txt, response404);
    return;
}

#ifdef ESP8266
#define AUTHENTICATE()                                                                                                                  \
    if (userConfig->getPasswordRequired() && !server.authenticateDigest(userConfig->getwwwUsername(), userConfig->getwwwCredentials())) \
        return server.requestAuthentication(DIGEST_AUTH, www_realm);
#else
String *ratgdoAuthenticate(HTTPAuthMethod mode, String enteredUsernameOrReq, String extraParams[])
{
    // ESP_LOGI(TAG, "Auth method: %d", mode);                // DIGEST_AUTH
    // ESP_LOGI(TAG, "User: %s", enteredUsernameOrReq);       // Username
    // ESP_LOGI(TAG, "Param 0: %s", extraParams[0].c_str());  // Realm
    // ESP_LOGI(TAG, "Param 1: %s", extraParams[1].c_str());  // URI
    String *pw = new String(read_door_str(nvram_ratgdo_pw, "password").c_str());
    return pw;
}

#define AUTHENTICATE()                                                                 \
    if (userConfig->getPasswordRequired() && !server.authenticate(ratgdoAuthenticate)) \
        return server.requestAuthentication(DIGEST_AUTH, www_realm);
#endif

// v23: same-origin / CSRF guard for state-changing endpoints. If
// Origin or Referer is present and the host part doesn't match our
// own Host header, reject with 403. Missing headers are accepted (some
// legacy tools and direct curl POSTs don't send them, and the user
// already authenticated through AUTHENTICATE() above) so this doesn't
// lock anyone out — it only hard-fails the obvious cross-site case.
//
// Pulled out of handle_setgdo (where it lived inline) so the new
// /reconnectHomeKit, /refreshHomeKitMDNS, /dumpHomeKitState endpoints
// can reuse the same logic. Returns true if the request should
// proceed; returns false (and sends 403) if it should be rejected.
static bool enforce_same_origin(const char *uriForLog)
{
    auto headerHas = [](const String &h, const char *needle) -> bool {
        return h.length() > 0 && h.indexOf(needle) >= 0;
    };
    String origin  = server.hasHeader("Origin")  ? server.header("Origin")  : String();
    String referer = server.hasHeader("Referer") ? server.header("Referer") : String();
    String myHost  = server.hasHeader("Host")    ? server.header("Host")    : String();
    if ((origin.length() > 0 || referer.length() > 0) && myHost.length() > 0)
    {
        String hostOnly = myHost;
        int colon = hostOnly.indexOf(':');
        if (colon >= 0) hostOnly = hostOnly.substring(0, colon);
        bool sameOrigin =
            (origin.length() > 0  && headerHas(origin,  hostOnly.c_str())) ||
            (referer.length() > 0 && headerHas(referer, hostOnly.c_str()));
        if (!sameOrigin)
        {
            ESP_LOGW(TAG, "CSRF: rejecting %s — Origin=%s Referer=%s Host=%s",
                     uriForLog, origin.c_str(), referer.c_str(), myHost.c_str());
            server.send_P(403, type_txt, PSTR("Forbidden: cross-origin"));
            return false;
        }
    }
    return true;
}

void handle_auth()
{
    AUTHENTICATE();
    server.send_P(200, type_txt, PSTR("Authenticated"));
    return;
}

void handle_reset()
{
    AUTHENTICATE();
    // v28: same-origin guard added — consistent with /setgdo and the
    // other state-changing v23+ POSTs. Pre-v28 a cross-origin page on
    // the same LAN could un-pair the device with a single beacon.
    if (!enforce_same_origin("/reset")) return;
    ESP_LOGI(TAG, "... reset requested");
#ifdef ESP8266
    homekit_storage_reset();
#else
    homekit_unpair();
#endif
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
    // v28: AUTHENTICATE + same-origin guard added. Pre-v28 /reboot was
    // entirely unguarded — a cross-origin page on the same LAN could
    // reboot the device. AUTHENTICATE is a no-op when the user hasn't
    // set a www password, preserving the legacy default-no-password UX;
    // for password-protected installs it now enforces the password,
    // matching /reset and /setgdo behaviour.
    AUTHENTICATE();
    if (!enforce_same_origin("/reboot")) return;
    const char *resp = "Rebooting...\n";
    server.client().setNoDelay(true);
    server.send(200, type_txt, resp);
    // Allow time to process send() before terminating web server...
    delay(500);
    server.stop();
    sync_and_restart();
    return;
}

// User-triggered HomeKit recovery — much less disruptive than /reboot
// when the device is otherwise healthy but the HomeKit hub thinks it's
// "No Response" (stale HAP TCP, mDNS gone stale, controller cache).
// Just cycles WiFi; HomeSpan auto-reattaches on link-up.
//
// Returns 200 immediately with a short text response. The actual cycle
// happens after the response is flushed so the client sees the OK and
// the user gets feedback in the UI before HTTP becomes briefly
// unreachable. Logged via ESP_LOGW with "via web UI" tag for syslog
// visibility.
void handle_reconnect_homekit()
{
    AUTHENTICATE();
    if (!enforce_same_origin("/reconnectHomeKit")) return;
    const char *resp = "HomeKit reconnect triggered. WiFi will cycle in ~1s; expect a brief HTTP outage.\n";
    server.client().setNoDelay(true);
    server.send(200, type_txt, resp);
    // v24: route through the deferred-flag drain in main loop instead
    // of a one-shot Ticker. v23's deferred Ticker still ran in
    // esp_timer task context where the ~750ms WiFi cycle would stall
    // every other Ticker callback (SSE heartbeats, health log).
    homekit_request_reconnect("via web UI /reconnectHomeKit");
    return;
}

// Lighter-touch HomeKit recovery — re-broadcast mDNS without dropping
// WiFi. First thing to try when iOS says "No Response" but the syslog
// shows the device is otherwise healthy. No HTTP outage — just a quick
// HomeSpan database update + mDNS re-advert.
void handle_refresh_mdns()
{
    AUTHENTICATE();
    if (!enforce_same_origin("/refreshHomeKitMDNS")) return;
    const char *resp = "HomeKit mDNS refresh triggered.\n";
    server.client().setNoDelay(true);
    server.send(200, type_txt, resp);
    homekit_refresh_mdns("via web UI /refreshHomeKitMDNS");
    return;
}

// Dumps HomeSpan's full diagnostic CLI output (status + accessory DB +
// operational diagnostics) to the system log / syslog. Read-only —
// useful for debugging "No Response" against device-side state without
// needing a USB serial cable.
//
// v23: gated on the homespanCLI setting. The HomeSpan dump exposes
// pairing controller count and HAP IIDs to syslog (and to anyone who
// can read it), which is information disclosure that the user never
// opted into. Tying it to the existing homespanCLI toggle means this
// only works when the user has explicitly enabled HomeSpan diagnostic
// access for the device.
void handle_dump_homekit_state()
{
    AUTHENTICATE();
    if (!enforce_same_origin("/dumpHomeKitState")) return;
    if (!userConfig->getEnableHomeSpanCLI())
    {
        server.send_P(403, type_txt, PSTR("Forbidden: enable 'HomeSpan CLI' in Settings to use the state dump."));
        return;
    }
    const char *resp = "HomeSpan state dump triggered. Check the System Log / HomeKit tab for output.\n";
    server.client().setNoDelay(true);
    server.send(200, type_txt, resp);
    homekit_dump_state("via web UI /dumpHomeKitState");
    return;
}

void load_page(const char *page)
{
    IPAddress clientIP = server.client().remoteIP();

    if ((strlen(page) > 6) && !strcmp(&page[strlen(page) - 6], "js.map"))
    {
        // js.map files, also known as JavaScript source maps, are files that provide a mapping between a minified, transpiled,
        // or bundled JavaScript file and its original, uncompressed source code. The browser only requests this if console/debugger
        // is opened. We do not store these locally (as large) and will redirect the browser to load from our GitHub repo.
        if (!strcmp(gitUser, "ratgdo"))
        {
            // If we are building on ratgdo (for published release) then use tagged URL to make sure map file matches the one embedded in the firmware
            strlcpy(writeBuffer, gitTaggedURL, sizeof(writeBuffer));
        }
        else
        {
            // else we are building for our test purposes, point to the raw URL
            strlcpy(writeBuffer, gitRawURL, sizeof(writeBuffer));
        }
        strlcat(writeBuffer, "/src/www", sizeof(writeBuffer));
        strlcat(writeBuffer, page, sizeof(writeBuffer));
        ESP_LOGD(TAG, "Sending 303 redirect to client %s for: %s", clientIP.toString().c_str(), writeBuffer);
        server.sendHeader(F("Location"), writeBuffer);
        server.send_P(303, type_txt, "", 0);
        return;
    }
    else if (webcontent.count(page) == 0)
        return handle_notfound();

    const unsigned char *data = webcontent.at(page).data;
    int length = webcontent.at(page).length;
    const char *typeP = webcontent.at(page).type;
    const char *crc32 = webcontent.at(page).crc32.c_str();
    // need local copy as strcmp_P cannot take two PSTR()'s
    char type[MAX_MIME_TYPE_LEN];
    strncpy_P(type, typeP, MAX_MIME_TYPE_LEN);

    bool cache = false;
    char cacheHdr[24] = "no-cache, no-store";
    char matchHdr[8] = "";
    if ((CACHE_CONTROL > 0) &&
        (!strcmp_P(type, type_css) || !strcmp_P(type, type_html) || !strcmp_P(type, type_js) || strstr_P(type, PSTR("image"))))
    {
        snprintf_P(cacheHdr, sizeof(cacheHdr), PSTR("max-age=%d"), CACHE_CONTROL);
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
            ESP_LOGD(TAG, "Client %s requesting: %s (HTTP_HEAD, type: %s)", clientIP.toString().c_str(), page, type);
            server.send_P(200, type, "", 0);
        }
        else
        {
            ESP_LOGD(TAG, "Client %s requesting: %s (HTTP_GET, type: %s, length: %d)", clientIP.toString().c_str(), page, type, length);
            server.send_P(200, type, reinterpret_cast<const char *>(data), length);
        }
    }
    else
    {
        ESP_LOGD(TAG, "Sending 304 not modified to client %s requesting: %s (method: %s, type: %s)", clientIP.toString().c_str(), page, http_methods[method], type);
        server.send_P(304, type, "", 0);
    }
    return;
}

void handle_everything()
{
    // Connection throttling
    if (!registerRequest())
    {
        server.send(503, type_txt, response503);
        ESP_LOGW(TAG, "Reject request, server too busy (handle_everything)");
        return;
    }

    HTTPMethod method = server.method();
    String page = server.uri();
    const char *uri = page.c_str();

    // too verbose... ESP_LOGI(TAG, "Handle everything for %s", uri);
    if (builtInUri.count(uri) > 0)
    {
        // requested page matches one of our built-in handlers
        ESP_LOGD(TAG, "Client %s requesting: %s (method: %s)", server.client().remoteIP().toString().c_str(), uri, http_methods[method]);
        if (method == builtInUri.at(uri).first)
        {
            builtInUri.at(uri).second();
        }
        else
        {
            handle_notfound();
        }
        unregisterRequest();
        return;
    }
    else if ((method == HTTP_GET) && (!strncmp_P(uri, restEvents, strlen(restEvents))))
    {
        // Request for "/rest/events/" with a channel number appended
        uri += strlen(restEvents);
        uint32_t channel = atoi(uri);
        if (channel < SSE_MAX_CHANNELS)
        {
            SSEHandler(channel);
        }
        else
        {
            handle_notfound();
        }
        unregisterRequest();
        return;
    }
    else if (method == HTTP_GET || method == HTTP_HEAD)
    {
        // HTTP_GET that does not match a built-in handler
        if (page.equals("/"))
        {
            load_page("/index.html");
        }
        else
        {
            load_page(uri);
        }
        unregisterRequest();
        return;
    }
    // it is a HTTP_POST for unknown URI
    handle_notfound();
    unregisterRequest();
    return;
}

void build_status_json(char *json)
{
    // Build the JSON string
    _millis_t upTime = _millis();
    JSON_START(json);
    JSON_ADD_STR("gitUser", gitUser);
    JSON_ADD_STR("gitRepo", gitRepo);
    // Auto-close (fork addition) — exposes the four config values so the
    // plugin and web UI can show current state. autoClose toggles the
    // feature; the rest define when/how it fires.
    JSON_ADD_BOOL("autoClose", userConfig->getAutoClose());
    JSON_ADD_INT("autoCloseMinutes", userConfig->getAutoCloseMinutes());
    JSON_ADD_INT("autoCloseStartMinutes", userConfig->getAutoCloseStartMinutes());
    JSON_ADD_INT("autoCloseEndMinutes", userConfig->getAutoCloseEndMinutes());
    JSON_ADD_BOOL("autoCloseIgnoreWindow", userConfig->getAutoCloseIgnoreWindow());
    // HomeKit watchdog (fork addition) — toggle + thresholds drive the
    // periodic homekit_health_log diagnostics and (if enabled) recovery.
    JSON_ADD_BOOL("hkAutoRecover", userConfig->getHKAutoRecover());
    JSON_ADD_INT("hkAutoRecoverSecs", userConfig->getHKAutoRecoverSecs());
    JSON_ADD_INT("hkHintQuietSecs", userConfig->getHKHintQuietSecs());
    JSON_ADD_INT("hkHintStaleSecs", userConfig->getHKHintStaleSecs());
    JSON_ADD_INT("hkHintLikelyNRSecs", userConfig->getHKHintLikelyNRSecs());
    JSON_ADD_INT("upTime", upTime);
    JSON_ADD_STR(cfg_deviceName, userConfig->getDeviceName());
    JSON_ADD_STR("userName", userConfig->getwwwUsername());
    JSON_ADD_BOOL("paired", homekit_is_paired());
    JSON_ADD_STR("firmwareVersion", std::string(AUTO_VERSION).c_str());
    JSON_ADD_STR(cfg_localIP, userConfig->getLocalIP());
    JSON_ADD_STR(cfg_subnetMask, userConfig->getSubnetMask());
    JSON_ADD_STR(cfg_gatewayIP, userConfig->getGatewayIP());
    JSON_ADD_STR(cfg_nameserverIP, userConfig->getNameserverIP());
    new_ipv4_address = false;
    JSON_ADD_STR("macAddress", WiFi.macAddress().c_str());
    JSON_ADD_STR("wifiSSID", WiFi.SSID().c_str());
    JSON_ADD_STR("wifiRSSI", (std::to_string(WiFi.RSSI()) + " dBm, Channel " + std::to_string(WiFi.channel())).c_str());
    JSON_ADD_STR("wifiBSSID", WiFi.BSSIDstr().c_str());
#ifdef ESP8266
    JSON_ADD_BOOL("lockedAP", wifiConf.bssid_set);
#else
    JSON_ADD_BOOL("lockedAP", false);
#endif
    JSON_ADD_INT("wifiPower", userConfig->getWifiPower());
    JSON_ADD_INT(cfg_GDOSecurityType, (uint32_t)userConfig->getGDOSecurityType());
    JSON_ADD_BOOL("garageSec1Emulated", garage_door.wallPanelEmulated);
    JSON_ADD_STR("garageDoorState", garage_door.active ? DOOR_STATE(garage_door.current_state) : DOOR_STATE(255));
    JSON_ADD_STR("garageLockState", REMOTES_STATE(garage_door.current_lock));
    JSON_ADD_BOOL("garageLightOn", garage_door.light);
    JSON_ADD_BOOL("garageMotion", garage_door.motion);
    JSON_ADD_BOOL("garageObstructed", garage_door.obstructed);
    JSON_ADD_BOOL("pinBasedObst", garage_door.pinModeObstructionSensor);
    JSON_ADD_BOOL(cfg_passwordRequired, userConfig->getPasswordRequired());
    JSON_ADD_INT(cfg_rebootSeconds, (uint32_t)userConfig->getRebootSeconds());
    JSON_ADD_INT("freeHeap", free_heap);
    JSON_ADD_INT("minHeap", min_heap);
    JSON_ADD_INT("crashCount", abs(crashCount));
    JSON_ADD_BOOL(cfg_staticIP, userConfig->getStaticIP());
    JSON_ADD_BOOL(cfg_syslogEn, userConfig->getSyslogEn());
    JSON_ADD_STR(cfg_syslogIP, userConfig->getSyslogIP());
    JSON_ADD_INT(cfg_syslogPort, userConfig->getSyslogPort());
    JSON_ADD_INT(cfg_syslogFacility, userConfig->getSyslogFacility());
    JSON_ADD_INT(cfg_logLevel, userConfig->getLogLevel());
    JSON_ADD_INT(cfg_TTCseconds, userConfig->getTTCseconds());
    JSON_ADD_BOOL(cfg_TTClight, userConfig->getTTClight());
    JSON_ADD_INT(cfg_motionTriggers, (uint32_t)motionTriggers.asInt);
    JSON_ADD_INT(cfg_LEDidle, userConfig->getLEDidle());
    // We send milliseconds relative to current time... ie updated X milliseconds ago
    JSON_ADD_INT(cfg_doorUpdateAt, (upTime - lastDoorUpdateAt));
    JSON_ADD_INT(cfg_doorOpenAt, (upTime - lastDoorOpenAt));
    JSON_ADD_INT(cfg_doorCloseAt, (upTime - lastDoorCloseAt));
    JSON_ADD_BOOL("enableNTP", enableNTP);
    if (enableNTP && (bool)clockSet)
    {
        JSON_ADD_INT("serverTime", time(NULL));
    }
    JSON_ADD_STR(cfg_ntpServer, userConfig->getNTPServer());
    JSON_ADD_STR(cfg_timeZone, userConfig->getTimeZone());
    JSON_ADD_BOOL(cfg_dcOpenClose, userConfig->getDCOpenClose());
    JSON_ADD_BOOL(cfg_dcBypassTTC, userConfig->getDCBypassTTC());
    JSON_ADD_BOOL(cfg_obstFromStatus, userConfig->getObstFromStatus());
    JSON_ADD_INT(cfg_dcDebounceDuration, userConfig->getDCDebounceDuration());
    JSON_ADD_STR("qrPayload", qrPayload);
    if (doorControlType == 2)
    {
        JSON_ADD_INT("batteryState", garage_door.batteryState);
        JSON_ADD_INT("openingsCount", garage_door.openingsCount);
        JSON_ADD_INT(cfg_builtInTTC, userConfig->getBuiltInTTC());
        JSON_ADD_INT("builtInTTCremaining", garage_door.builtInTTCremaining);
        JSON_ADD_BOOL("builtInTTChold", garage_door.builtInTTChold);
        JSON_ADD_BOOL(cfg_useToggle, userConfig->getUseToggle());
    }
    if (garage_door.openDuration)
    {
        JSON_ADD_INT("openDuration", garage_door.openDuration);
        snprintf_P(writeBuffer, sizeof(writeBuffer), PSTR("{ \"max\": %d, \"count\": %d, \"duration\": [ %d, %d, %d, %d, %d, %d ] }"),
                   openHistory.max, openHistory.count,
                   openHistory(1), openHistory(2), openHistory(3), openHistory(4), openHistory(5), openHistory(6));
        JSON_ADD_RAW("openHistory", writeBuffer);
    }
    if (garage_door.closeDuration)
    {
        JSON_ADD_INT("closeDuration", garage_door.closeDuration);
        snprintf_P(writeBuffer, sizeof(writeBuffer), PSTR("{ \"max\": %d, \"count\": %d, \"duration\": [ %d, %d, %d, %d, %d, %d ] }"),
                   closeHistory.max, closeHistory.count,
                   closeHistory(1), closeHistory(2), closeHistory(3), closeHistory(4), closeHistory(5), closeHistory(6));
        JSON_ADD_RAW("closeHistory", writeBuffer);
    }
#ifdef ESP8266
#define accessoryID arduino_homekit_get_running_server() ? arduino_homekit_get_running_server()->accessory_id : "Inactive"
#define clientCount arduino_homekit_get_running_server() ? arduino_homekit_get_running_server()->nfds : 0
    JSON_ADD_STR("accessoryID", accessoryID);
    JSON_ADD_INT("clients", clientCount);
    JSON_ADD_BOOL("lockedAP", wifiConf.bssid_set);
    JSON_ADD_INT("wifiPhyMode", userConfig->getWifiPhyMode());
    JSON_ADD_INT("minStack", ESP.getFreeContStack());
#else
    JSON_ADD_INT(cfg_occupancyDuration, userConfig->getOccupancyDuration());
    JSON_ADD_BOOL(cfg_enableIPv6, userConfig->getEnableIPv6());
    JSON_ADD_STR("ipv6Addresses", ipv6_addresses);
    new_ipv6_address = false;
#ifdef USE_GDOLIB
    JSON_ADD_BOOL(cfg_useSWserial, userConfig->getUseSWserial());
#endif
#ifdef RATGDO32_DISCO
    JSON_ADD_BOOL("distanceSensor", garage_door.has_distance_sensor);
    if (garage_door.has_distance_sensor)
    {
        JSON_ADD_STR("vehicleStatus", vehicleStatus);
        JSON_ADD_INT("vehicleDist", (uint32_t)vehicleDistance);
        last_reported_assist_laser = laser.state();
        JSON_ADD_BOOL("assistLaser", last_reported_assist_laser);
    }
    JSON_ADD_BOOL(cfg_vehicleHomeKit, userConfig->getVehicleHomeKit());
    JSON_ADD_BOOL(cfg_vehicleOccupancyHomeKit, userConfig->getVehicleOccupancyHomeKit());
    JSON_ADD_BOOL(cfg_vehicleArrivingHomeKit, userConfig->getVehicleArrivingHomeKit());
    JSON_ADD_BOOL(cfg_vehicleDepartingHomeKit, userConfig->getVehicleDepartingHomeKit());
    JSON_ADD_INT(cfg_vehicleThreshold, userConfig->getVehicleThreshold());
    JSON_ADD_BOOL(cfg_laserEnabled, userConfig->getLaserEnabled());
    JSON_ADD_BOOL(cfg_laserHomeKit, userConfig->getLaserHomeKit());
    JSON_ADD_INT(cfg_assistDuration, userConfig->getAssistDuration());
#endif
    JSON_ADD_BOOL(cfg_homespanCLI, userConfig->getEnableHomeSpanCLI());
    JSON_ADD_BOOL(cfg_lightHomeKit, userConfig->getLightHomeKit());
    JSON_ADD_BOOL(cfg_motionHomeKit, userConfig->getMotionHomeKit());
#endif
    JSON_ADD_INT("webRequests", request_count);
    JSON_ADD_INT("webMaxResponseTime", max_response_time);
    JSON_ADD_INT("ttcActive", is_ttc_active());
    JSON_END();
}

void add_static_mdns()
{
    // Values that do not change during runtime
    ESP_LOGD(TAG, "Adding static mDNS TXT records");
    MDNS.addServiceTxt("ratgdo", "tcp", "model", MODEL_NAME);
    MDNS.addServiceTxt("ratgdo", "tcp", "firmwareVersion", AUTO_VERSION);
    MDNS.addServiceTxt("ratgdo", "tcp", "firmwareDate", __DATE__ " " __TIME__);
    MDNS.addServiceTxt("ratgdo", "tcp", cfg_deviceName, userConfig->getDeviceName());
    MDNS.addServiceTxt("ratgdo", "tcp", "gitRepo", gitRepo);
    MDNS.addServiceTxt("ratgdo", "tcp", "macAddress", WiFi.macAddress().c_str());
    MDNS.addServiceTxt("ratgdo", "tcp", "wifiSSID", WiFi.SSID().c_str());
#ifdef RATGDO32_DISCO
    MDNS.addServiceTxt("ratgdo", "tcp", "distanceSensor", garage_door.has_distance_sensor ? "true" : "false");
#endif
}

void add_dynamic_mdns()
{
    // Values that may change during runtime
    ESP_LOGD(TAG, "Updating dynamic mDNS TXT records");
    _millis_t upTime = _millis();
    MDNS.addServiceTxt("ratgdo", "tcp", "upTime", std::to_string(upTime).c_str());
    MDNS.addServiceTxt("ratgdo", "tcp", "wifiRSSI", std::to_string(WiFi.RSSI()).c_str());
    MDNS.addServiceTxt("ratgdo", "tcp", "wifiChannel", std::to_string(WiFi.channel()).c_str());
    MDNS.addServiceTxt("ratgdo", "tcp", "wifiBSSID", WiFi.BSSIDstr().c_str());
    MDNS.addServiceTxt("ratgdo", "tcp", "paired", homekit_is_paired() ? "true" : "false");
    MDNS.addServiceTxt("ratgdo", "tcp", "garageDoorState", DOOR_STATE(garage_door.current_state));
    MDNS.addServiceTxt("ratgdo", "tcp", "garageLockState", REMOTES_STATE(garage_door.current_lock));
    MDNS.addServiceTxt("ratgdo", "tcp", "garageLightOn", garage_door.light ? "true" : "false");
    MDNS.addServiceTxt("ratgdo", "tcp", "garageMotion", garage_door.motion ? "true" : "false");
    MDNS.addServiceTxt("ratgdo", "tcp", "garageObstructed", garage_door.obstructed ? "true" : "false");
    if (doorControlType == 2)
    {
        MDNS.addServiceTxt("ratgdo", "tcp", "batteryState", std::to_string(garage_door.batteryState).c_str());
        MDNS.addServiceTxt("ratgdo", "tcp", "openingsCount", std::to_string(garage_door.openingsCount).c_str());
        MDNS.addServiceTxt("ratgdo", "tcp", cfg_builtInTTC, std::to_string(userConfig->getBuiltInTTC()).c_str());
    }
    MDNS.addServiceTxt("ratgdo", "tcp", cfg_TTCseconds, std::to_string(userConfig->getTTCseconds()).c_str());
    MDNS.addServiceTxt("ratgdo", "tcp", "openDuration", std::to_string(garage_door.openDuration).c_str());
    MDNS.addServiceTxt("ratgdo", "tcp", "closeDuration", std::to_string(garage_door.closeDuration).c_str());
    MDNS.addServiceTxt("ratgdo", "tcp", cfg_passwordRequired, userConfig->getPasswordRequired() ? "true" : "false");
    // We send milliseconds relative to current time... ie updated X milliseconds ago
    MDNS.addServiceTxt("ratgdo", "tcp", cfg_doorUpdateAt, std::to_string(upTime - lastDoorUpdateAt).c_str());
    MDNS.addServiceTxt("ratgdo", "tcp", cfg_doorOpenAt, std::to_string(upTime - lastDoorOpenAt).c_str());
    MDNS.addServiceTxt("ratgdo", "tcp", cfg_doorCloseAt, std::to_string(upTime - lastDoorCloseAt).c_str());
#ifdef RATGDO32_DISCO
    if (garage_door.has_distance_sensor)
    {
        MDNS.addServiceTxt("ratgdo", "tcp", "vehicleStatus", (const char *)vehicleStatus);
        MDNS.addServiceTxt("ratgdo", "tcp", "vehicleDist", std::to_string((uint32_t)vehicleDistance).c_str());
    }
#endif
    if (enableNTP && (bool)clockSet)
    {
        MDNS.addServiceTxt("ratgdo", "tcp", "serverTime", std::to_string(time(NULL)).c_str());
        MDNS.addServiceTxt("ratgdo", "tcp", "serverTimeStr", (const char *)timeString());
        MDNS.addServiceTxt("ratgdo", "tcp", cfg_timeZone, userConfig->getTimeZone());
    }
#ifdef ESP8266
    MDNS.announce();
#else
    MDNS.setInstanceName(device_name);
#endif
    mdnsUpdatePending = false;
    lastMDNSupdate = _millis();
}

void handle_status()
{
    _millis_t startTime = _millis();
    uint32_t response_time;
    uint32_t build_time;
    static char *json = status_json;

    TAKE_MUTEX();
    request_count++;
    build_status_json(json);
    build_time = (uint32_t)(_millis() - startTime);
    last_reported_garage_door = garage_door;
    // v24: release the JSON mutex BEFORE the synchronous server.send_P
    // TCP write. Same audit-flagged pattern as log.cpp — holding the
    // mutex across a blocking write means a slow Homebridge poll could
    // stall every other JSON consumer (web_loop SSE broadcasts,
    // SSEheartbeat). The status JSON is a static buffer; once the
    // build completes the mutex's job is done.
    GIVE_MUTEX();

    server.sendHeader(F("Cache-Control"), F("no-cache, no-store"));
    server.send_P(200, type_json, json);
    response_time = _millis() - startTime;
    max_response_time = std::max(max_response_time, response_time);
    if (strlen(json) > STATUS_JSON_BUFFER_SIZE * 95 / 100)
    {
        ESP_LOGW(TAG, "WARNING JSON status: %d is over 95%% of available buffer (%d), build time %lums, response time: %lums", strlen(json), STATUS_JSON_BUFFER_SIZE, build_time, response_time);
    }
    else
    {
        // Demoted from INFO → DEBUG in v22. The Homebridge plugin polls
        // /status.json every 3s by default to drive the GarageDoorOpener
        // tile + Obstruction/Motion sensors, so this line was firing ~20
        // times/min and burying every other log message. The 95%-buffer
        // WARNING above still fires at WARN level — that's the actionable
        // signal. To see these again, set log level to DEBUG in settings.
        ESP_LOGD(TAG, "JSON status: %d (%d%%), build time %lums, response time: %lums", strlen(json), strlen(json) * 100 / STATUS_JSON_BUFFER_SIZE, build_time, response_time);
    }
    return;
}

void handle_logout()
{
    ESP_LOGI(TAG, "Handle logout");
    return server.requestAuthentication(DIGEST_AUTH, www_realm);
}

bool helperResetDoor(const std::string &key, const char *value, configSetting *action)
{
    reset_door();
    return true;
}

bool helperGarageLightOn(const std::string &key, const char *value, configSetting *action)
{
    set_light((atoi(value) == 1) ? true : false);
    return true;
}

bool helperGarageDoorState(const std::string &key, const char *value, configSetting *action)
{
    if (atoi(value) == 1)
        open_door();
    else
        close_door();
    return true;
}

// forceClose=<hold_ms>: simulate a wall-button hold-to-close override.
// Useful when the photo-eye is false-tripping (sun glare) or otherwise
// preventing a normal close — the wall-button hold pattern is the only
// way to override the GDO motor's safety beam check.
//
// SAFETY: only intended for cases where the user has visually verified
// nothing is blocking the door. The plugin/UI calling this is expected
// to surface that warning to the user.
//
// Sec+1.0 only. On Sec+2.0 doorControlType, this falls back to a normal
// close (no protocol-level hold pattern exists).
bool helperForceClose(const std::string &key, const char *value, configSetting *action)
{
    int hold_ms = atoi(value);
    if (hold_ms <= 0) hold_ms = 3500; // default
    door_command_force_close((uint32_t)hold_ms);
    return true;
}

bool helperGarageLockState(const std::string &key, const char *value, configSetting *action)
{
    set_lock((atoi(value) == 1) ? 1 : 0);
    return true;
}

bool helperCredentials(const std::string &key, const char *value, configSetting *action)
{
    const char *newUsername = strstr(value, "username");
    const char *newCredentials = strstr(value, "credentials");
    const char *newPassword = strstr(value, "password");
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
    ESP_LOGI(TAG, "Set credentials for user: %s", newUsername);
    userConfig->set(cfg_wwwUsername, newUsername);
    userConfig->set(cfg_wwwCredentials, newCredentials);
#ifndef ESP8266
    // We only need to save password (distinct from credentials) on ESP32
    write_door_str(nvram_ratgdo_pw, newPassword);
#endif
    ESP8266_SAVE_CONFIG();
    return true;
}

bool helperUpdateUnderway(const std::string &key, const char *value, configSetting *action)
{
    firmwareSize = 0;
    firmwareUpdateSub = NULL;
    const char *md5 = strstr(value, "md5");
    const char *size = strstr(value, "size");
    const char *uuid = strstr(value, "uuid");

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
    // ESP_LOGI(TAG,"MD5: %s, UUID: %s, Size: %d", md5, uuid, atoi(size));
    // save values...
    strlcpy(firmwareMD5, md5, sizeof(firmwareMD5));
    firmwareSize = atoi(size);
    for (uint32_t channel = 0; channel < SSE_MAX_CHANNELS; channel++)
    {
        if (subscription[channel].SSEconnected && subscription[channel].clientUUID == uuid && subscription[channel].client.connected())
        {
            firmwareUpdateSub = &subscription[channel];
            break;
        }
    }
    return true;
}

bool helperFactoryReset(const std::string &key, const char *value, configSetting *action)
{
#ifdef ESP8266
    userConfig->erase();
    reset_door();
    WiFi.disconnect();
    ESP.eraseConfig();
    sync_and_restart();
#else
    ESP_LOGI(TAG, "System boot time: %s", timeString(lastRebootAt));
    ESP_LOGI(TAG, "Factory reset at: %s", timeString());
    erase_door_data();
    reset_door();
    homeSpan.processSerialCommand("F");
#endif
    return true;
}

void factoryReset()
{
    helperFactoryReset("", "", nullptr);
}

#ifdef RATGDO32_DISCO
bool helperAssistLaser(const std::string &key, const char *value, configSetting *action)
{
    if (atoi(value) == 1)
        laser.on();
    else
        laser.off();
    notify_homekit_laser(atoi(value) == 1);
    return true;
}
#endif

void handle_setgdo()
{
    // Build-in handlers that do not set a configuration value, or if they do they set multiple values.
    // key, {reboot, wifiChanged, value, fn to call}
    static const std::unordered_map<std::string, configSetting> setGDOhandlers = {
        {PSTR("resetDoor"), {true, false, 0, helperResetDoor}},
        {PSTR("garageLightOn"), {false, false, 0, helperGarageLightOn}},
        {PSTR("garageDoorState"), {false, false, 0, helperGarageDoorState}},
        {PSTR("forceClose"), {false, false, 0, helperForceClose}},
        {PSTR("garageLockState"), {false, false, 0, helperGarageLockState}},
        {PSTR("credentials"), {false, false, 0, helperCredentials}}, // parse out wwwUsername and credentials
        {PSTR("updateUnderway"), {false, false, 0, helperUpdateUnderway}},
        {PSTR("factoryReset"), {true, false, 0, helperFactoryReset}},
#ifdef RATGDO32_DISCO
        {PSTR("assistLaser"), {false, false, 0, helperAssistLaser}},
#endif
    };
    bool reboot = false;
    bool error = false;
    bool wifiChanged = false;
    bool saveSettings = false;
    configSetting actions;

    if (!((server.args() == 1) && (server.argName(0) == cfg_timeZone)))
    {
        // We will allow setting of time zone without authentication
        AUTHENTICATE();
    }

    // v23: CSRF guard pulled into enforce_same_origin() (above) so the
    // 4 state-changing endpoints share one implementation.
    if (!enforce_same_origin("/setgdo")) return;

    // Loop over all the GDO settings passed in...
    for (int i = 0; i < server.args(); i++)
    {
        std::string key(server.argName(i).c_str());
        std::string value(server.arg(i).c_str());

        // Clamp known integer-ranged auto-close keys server-side. The web UI
        // already enforces these bounds, but a hand-crafted POST can bypass
        // them; defending here keeps NVRAM from accepting nonsense like
        // autoCloseMinutes=99999999 or autoCloseStartMinutes=-1.
        if (key == "autoCloseMinutes" ||
            key == "autoCloseStartMinutes" ||
            key == "autoCloseEndMinutes")
        {
            long n = strtol(value.c_str(), nullptr, 10);
            long lo = (key == "autoCloseMinutes") ? 1 : 0;
            long hi = (key == "autoCloseMinutes") ? 720 : 1439;
            if (n < lo) n = lo;
            if (n > hi) n = hi;
            value = std::to_string(n);
        }
        // v23: same defensive clamp for HomeKit watchdog timer keys.
        // Without this, a hand-crafted POST hkAutoRecoverSecs=0 (or any
        // value < 60) would make the watchdog auto-fire on every health
        // tick → WiFi cycles every 3 minutes forever → device unreachable
        // by HomeKit until manual settings rescue. Range [60, 7200] matches
        // the form bounds in src/www/index.html.
        if (key == "hkAutoRecoverSecs" ||
            key == "hkHintQuietSecs" ||
            key == "hkHintStaleSecs" ||
            key == "hkHintLikelyNRSecs")
        {
            long n = strtol(value.c_str(), nullptr, 10);
            if (n < 60) n = 60;
            if (n > 7200) n = 7200;
            value = std::to_string(n);
        }

        if (setGDOhandlers.count(key))
        {
            if (key == "credentials")
                ESP_LOGI(TAG, "Call SetGDO handler for Key: %s", key.c_str());
            else
                ESP_LOGI(TAG, "Call SetGDO handler for Key: %s, Value: %s", key.c_str(), value.c_str());
            actions = setGDOhandlers.at(key);
            if (actions.fn)
            {
                error = error || !actions.fn(key, value.c_str(), &actions);
            }
            reboot = reboot || actions.reboot;
            wifiChanged = wifiChanged || actions.wifiChanged;
        }
        else if (userConfig->contains(key))
        {
            ESP_LOGI(TAG, "Configuration set for Key: %s, Value: %s", key.c_str(), value.c_str());
            actions = userConfig->getDetail(key);
            if (actions.fn)
            {
                // Value will be set within called function
                error = error || !actions.fn(key, value.c_str(), &actions);
            }
            else
            {
                // No function to call, set value directly.
                userConfig->set(key, value.c_str());
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
        YIELD(); // Yield while looping over all received settings, just-in-case!
        if (error)
            break;
    }

    ESP_LOGV(TAG, "SetGDO Complete");

    if (error)
    {
        // Simple error handling...
        ESP_LOGE(TAG, "Sending %s, for: %s", response400invalid, server.uri().c_str());
        server.send_P(400, type_txt, response400invalid);
        return;
    }

    if (saveSettings)
    {
        userConfig->set(cfg_wifiChanged, wifiChanged);
        ESP8266_SAVE_CONFIG();
        // v22 hooks — after settings save, re-arm anything that caches
        // a config value at boot. Cheap (refreshing a handful of static
        // vars / setting a flag) and only runs when the user actually
        // changes settings. v23 routes the auto-close reschedule via
        // the deferred-flag drain so it happens on the main loop instead
        // of racing the Ticker callback's own self-reschedule.
        homekit_refresh_watchdog_config();
        request_auto_close_reschedule();
    }
    if (reboot)
    {
        // Some settings require reboot to take effect
        server.send_P(200, type_html, PSTR("<p>Success. Reboot.</p>"));
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

void removeSSEsubscription(SSESubscription *s)
{
    if (subscriptionCount > 0)
        subscriptionCount--; // Prevent negative count
    s->heartbeatTimer.detach();
    ESP_LOGD(TAG, "Remove SSE subscription. Total subscribed: %d", subscriptionCount);
    s->client.stop();
    s->clientIP = INADDR_NONE;
    s->clientUUID.clear();
    s->SSEconnected = false;
    s->pendingRemove = false;
    // v27: reset orphan-sweep timestamps so a freshly freed slot doesn't
    // get reaped on its next allocation if the new client subscribes
    // faster than _millis ticks. Both are cheap writes.
    s->subscribedAt = 0;
    s->lastActivity = 0;
}

// v22: drain SSESubscription entries flagged pendingRemove during a
// Ticker callback (where calling Ticker.detach() on the still-running
// Ticker would crash in vTaskDelete → uxListRemove). Called every main
// loop tick from service_timer_loop in ratgdo.cpp. Cheap when nothing
// is pending — just an array scan.
void process_sse_pending_removes()
{
    for (uint32_t i = 0; i < SSE_MAX_CHANNELS; i++)
    {
        if (subscription[i].pendingRemove)
        {
            removeSSEsubscription(&subscription[i]);
        }
    }
}

// v27: orphan-slot sweep. Runs from service_timer_loop BEFORE
// process_sse_pending_removes so any slot we flag here is reaped
// the same tick. Three classes of orphans:
//   5a) Subscribed but EventSource never opened (pre-handshake leak).
//       Pre-v27 this was the dominant leak — logs.html with heartbeat=0
//       had no Ticker to fire SSEheartbeat, so the SSEfailCount
//       fast-path never ran. After SSE_PREHANDSHAKE_TIMEOUT_MS the
//       browser is presumed gone.
//   5b) Connected but TCP socket has dropped without us noticing.
//       client.connected() now returns false but no broadcast/heartbeat
//       has noticed yet (e.g. client subscribed but never sees broadcast
//       traffic during a quiet period).
//   5c) Connected and TCP still up, but truly idle for SSE_IDLE_TIMEOUT_MS.
//       Belt-and-suspenders for a wedged-but-not-yet-RST socket. A
//       healthy heartbeat>=30s client never trips this; logs.html with
//       v27 heartbeat=10 also won't trip (broadcast+heartbeat keep
//       lastActivity fresh).
// Also reconciles subscriptionCount: if it desyncs from the actual slot
// count (which it has been doing because handle_subscribe used to
// pre-increment before all rejection paths exhausted), log + correct.
void sweep_sse_orphans()
{
    // v28: truncate now to uint32_t to match the timestamp field width
    // (changed from _millis_t int64_t in v28 to avoid tearing risk).
    // Subtraction is wrap-safe modulo 2^32.
    uint32_t now = (uint32_t)_millis();
    uint32_t currentlyAlloc = 0;
    uint32_t reapedThisTick = 0;
    for (uint32_t i = 0; i < SSE_MAX_CHANNELS; i++)
    {
        SSESubscription &s = subscription[i];
        // FREE — nothing to do.
        if (s.clientIP == IPAddress(INADDR_NONE))
            continue;
        // Already flagged — process_sse_pending_removes will pick it up.
        if (s.pendingRemove)
        {
            currentlyAlloc++;
            continue;
        }
        currentlyAlloc++;
        // 5a) pre-handshake abandoned
        if (!s.SSEconnected && s.subscribedAt != 0 &&
            (now - s.subscribedAt) > SSE_PREHANDSHAKE_TIMEOUT_MS)
        {
            ESP_LOGW(TAG, "SSE orphan (pre-handshake) channel=%u uuid=%s ip=%s age=%ums — reaping",
                     (unsigned)i, s.clientUUID.c_str(),
                     s.clientIP.toString().c_str(),
                     (unsigned)(now - s.subscribedAt));
            s.pendingRemove = true;
            sseOrphansReaped++;
            reapedThisTick++;
            continue;
        }
        // 5b) connected but TCP gone
        if (s.SSEconnected && !s.client.connected())
        {
            ESP_LOGW(TAG, "SSE orphan (TCP dropped) channel=%u uuid=%s ip=%s — reaping",
                     (unsigned)i, s.clientUUID.c_str(),
                     s.clientIP.toString().c_str());
            s.pendingRemove = true;
            sseOrphansReaped++;
            reapedThisTick++;
            continue;
        }
        // 5c) idle past the watchdog
        if (s.SSEconnected && s.lastActivity != 0 &&
            (now - s.lastActivity) > SSE_IDLE_TIMEOUT_MS)
        {
            ESP_LOGI(TAG, "SSE orphan (idle) channel=%u uuid=%s ip=%s idle=%ums — reaping",
                     (unsigned)i, s.clientUUID.c_str(),
                     s.clientIP.toString().c_str(),
                     (unsigned)(now - s.lastActivity));
            s.pendingRemove = true;
            sseOrphansReaped++;
            reapedThisTick++;
            continue;
        }
    }
    sseSlotsAlloc = currentlyAlloc;
    // F3 reconciliation: subscriptionCount used to drift on the rejection
    // paths in handle_subscribe (counter incremented before all rejection
    // checks ran). If we see a mismatch here, the counter is wrong, the
    // slots are right — assign + warn.
    if (currentlyAlloc != subscriptionCount)
    {
        ESP_LOGW(TAG, "SSE subscriptionCount desync: counter=%u actual=%u — reconciling",
                 (unsigned)subscriptionCount, (unsigned)currentlyAlloc);
        subscriptionCount = currentlyAlloc;
    }
    (void)reapedThisTick;
}

void SSEheartbeat(SSESubscription *s)
{
    if (!s)
        return;

    // v28: was `if (!(s->clientIP))` — same INADDR_NONE-vs-zero
    // mismatch as the subscribe scan. Compare to the canonical free
    // marker so a heartbeat scheduled before removeSSEsubscription
    // ran doesn't accidentally fire on a slot that's logically free.
    if (s->clientIP == IPAddress(INADDR_NONE))
        return;

    if (!(s->SSEconnected))
    {
        if (s->SSEfailCount++ >= 5)
        {
            // v22: defer the remove to main-loop context — calling
            // removeSSEsubscription here would Ticker.detach() the very
            // Ticker that's running this callback, crashing in uxListRemove.
            ESP_LOGD(TAG, "Client %s (%s) >5 heartbeat fails, marking for deferred remove", s->clientIP.toString().c_str(), s->clientUUID.c_str());
            s->pendingRemove = true;
        }
        else
        {
            ESP_LOGE(TAG, "Client %s (%s) not yet listening for SSE", s->clientIP.toString().c_str(), s->clientUUID.c_str());
        }
        return;
    }

    if (s->client.connected())
    {
        static int8_t lastRSSI = 0;
        static char *json = status_json;
        TAKE_MUTEX();
        JSON_START(json);
        JSON_ADD_INT("upTime", _millis());
        JSON_ADD_INT("freeHeap", free_heap);
        JSON_ADD_INT("minHeap", min_heap);
        // TODO monitor stack... JSON_ADD_INT("minStack", ESP.getFreeContStack());
#ifdef RATGDO32_DISCO
        static int32_t lastVehicleDistance = 0;
        if (garage_door.has_distance_sensor && (lastVehicleDistance != vehicleDistance))
        {
            lastVehicleDistance = vehicleDistance;
            JSON_ADD_INT("vehicleDist", (uint32_t)vehicleDistance);
        }
#endif
        if (lastRSSI != WiFi.RSSI())
        {
            lastRSSI = WiFi.RSSI();
            JSON_ADD_STR("wifiRSSI", (std::to_string(lastRSSI) + " dBm, Channel " + std::to_string(WiFi.channel())).c_str());
        }
#ifdef ESP8266
        static int lastClientCount = 0;
        if (arduino_homekit_get_running_server() && arduino_homekit_get_running_server()->nfds != lastClientCount)
        {
            lastClientCount = arduino_homekit_get_running_server()->nfds;
            JSON_ADD_INT("clients", lastClientCount);
        }
#endif
        JSON_END();
        JSON_REMOVE_NL(json);
        // v24: copy the formatted SSE event to a local stack buffer
        // BEFORE releasing the mutex (because writeBuffer is shared
        // between heartbeat callers and could be overwritten by another
        // SSE channel firing simultaneously). Then release the mutex
        // and write — clientWrite is now bounded by SO_SNDTIMEO and
        // skips full TCP buffers via availableForWrite.
        char localBuf[sizeof(writeBuffer)];
        snprintf_P(localBuf, sizeof(localBuf), PSTR("event: message\ndata: %s\n\n"), json);
        GIVE_MUTEX();
        // v27: capture clientWrite return so we can stamp lastActivity
        // only on a successful write. Failed writes already mark the
        // socket via clientWrite's internal stop()/skip path; bumping
        // lastActivity on a failure would mask the orphan sweep's
        // idle-detection (5c) for up to SSE_IDLE_TIMEOUT_MS.
        // v29: use clientWriteEx tri-state. Stamp on OK or BUFFER_FULL
        // (both = "broadcast loop reached this slot and tried"; peer is
        // alive enough to keep). Only FAILED skips the stamp — that's
        // the real wedge signal where lwIP rejected bytes for delivery.
        SseWriteResult r = clientWriteEx(s->client, localBuf);
        if (r != SseWriteResult::FAILED)
        {
            s->lastActivity = (uint32_t)_millis();
        }
        YIELD();
    }
    else
    {
        // v22: defer to main-loop context (see SSEheartbeat top comment).
        ESP_LOGD(TAG, "Client %s (%s) not listening (heartbeat), marking for deferred remove", s->clientIP.toString().c_str(), s->clientUUID.c_str());
        s->pendingRemove = true;
        YIELD();
    }
}

void SSEHandler(uint32_t channel)
{
    if (server.args() != 1)
    {
        ESP_LOGE(TAG, "Sending %s, for: %s", response400missing, server.uri().c_str());
        server.send_P(400, type_txt, response400missing);
        return;
    }

    SSESubscription &s = subscription[channel];
    s.client = server.client(); // capture SSE server client connection
    if (s.clientUUID != server.arg(0))
    {
        ESP_LOGE(TAG, "Client %s (%s) tries to listen for SSE but not subscribed", s.client.remoteIP().toString().c_str(), server.arg(0).c_str());
        return handle_notfound();
    }
    s.client.setNoDelay(true);
    s.client.setTimeout(CLIENT_WRITE_TIMEOUT);       // default is 5000ms which is way too long (Watchdog will fire)
    // v24: setTimeout() in Arduino-ESP32 only bounds READS, not WRITES
    // — a wedged subscriber would block client.write() indefinitely
    // even with the timeout above. Set SO_SNDTIMEO directly via lwIP
    // setsockopt so writes return after CLIENT_SLOW_WRITE_MS even on
    // a stuck socket. Combined with the availableForWrite fast-path
    // and pendingRemove cleanup, a single dead client can no longer
    // wedge the broadcast loop.
#ifndef ESP8266
    int fd = s.client.fd();
    if (fd >= 0)
    {
        struct timeval sndto;
        sndto.tv_sec = 0;
        sndto.tv_usec = CLIENT_SLOW_WRITE_MS * 1000;
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &sndto, sizeof(sndto));
    }
#endif
    server.setContentLength(CONTENT_LENGTH_UNKNOWN); // the payload can go on forever
    server.sendContent_P(PSTR("HTTP/1.1 200 OK\nContent-Type: text/event-stream;\nConnection: keep-alive\nCache-Control: no-cache\nAccess-Control-Allow-Origin: *\n\n"));
    s.SSEconnected = true;
    s.SSEfailCount = 0;
    // v27: stamp lastActivity on successful EventSource handshake. Without
    // this a slot that subscribes + connects but never receives a broadcast
    // would have lastActivity=0 forever, causing the 5c idle check to
    // skip it (the !=0 guard) but also masking real wedges. Every other
    // success-path keeps it fresh; this seeds it.
    s.lastActivity = (uint32_t)_millis();
    if (s.heartbeatInterval)
    {
        s.heartbeatTimer.attach_ms(s.heartbeatInterval * 1000, [&s]
                                   {
#ifdef ESP8266
                                       schedule_recurrent_function_us([&s]()
                                                                      {
                                                                          SSEheartbeat(&s);
                                                                          return false; // run the fn only once
                                                                      },
                                                                      0); // zero micro seconds (run asap)
#else
                                       SSEheartbeat(&s);
                                       return;
#endif
                                   });
    }
    ESP_LOGD(TAG, "Client %s (%s) listening for SSE events on channel %d", s.client.remoteIP().toString().c_str(), s.clientUUID.c_str(), channel);
}

// v27/v28: heartbeat-interval bounds.
//
//   MIN = 30s  — coerce-target when client EXPLICITLY sends heartbeat=0.
//                Pre-v27 the lower bound was 0 (with 0 == "no heartbeat"),
//                which combined with the slot-leak bug to leave heartbeat=0
//                slots without any Ticker-driven liveness. We now coerce
//                explicit 0 → MIN so every server-acknowledged "I'd rather
//                have no heartbeat" client still gets a Ticker as backup
//                to the orphan sweep.
//
//   MAX = 60s  — one ESP32 Ticker tick cap.
//
//   DEFAULT = 1s — applies ONLY when the client sends NO heartbeat= arg
//                  at all. 1s is more liveness than MIN, not less, so
//                  this is functionally correct — the constant is
//                  named DEFAULT, not MIN, and the F5 coerce
//                  intentionally only fires when the arg is explicitly
//                  provided. Any client that wants the cheaper 30s
//                  cadence should send heartbeat=30 (or heartbeat=0
//                  which gets coerced to 30).
constexpr uint32_t SSE_HEARTBEAT_MIN_SEC = 30;
constexpr uint32_t SSE_HEARTBEAT_MAX_SEC = 60;
constexpr uint32_t SSE_HEARTBEAT_DEFAULT_SEC = 1;
// v27: refuse new SSE subscriptions when free heap is below this
// threshold. WebServer-side allocations (chunked response buffers,
// SSL handshake buffers if HTTPS, lwIP TCP PCBs) total >8KB per
// active connection on ESP32; under heap pressure we'd just OOM
// later in clientWrite anyway. 16KB picked empirically — a healthy
// idle device sits at ~140K, the watchdog trips at <50K, so 16K is
// safely past any normal load.
constexpr uint32_t SSE_MIN_HEAP_FREE = 16384;

void handle_subscribe()
{
    uint32_t channel;
    IPAddress clientIP = server.client().remoteIP(); // get IP address of client
    std::string SSEurl = restEvents;

    if (subscriptionCount == SSE_MAX_CHANNELS)
    {
        ESP_LOGE(TAG, "Client %s SSE Subscription declined, subscription count: %d", clientIP.toString().c_str(), subscriptionCount);
        for (channel = 0; channel < SSE_MAX_CHANNELS; channel++)
        {
            ESP_LOGD(TAG, "Client %d: %s at %s", channel, subscription[channel].clientUUID.c_str(), subscription[channel].clientIP.toString().c_str());
        }
        return handle_notfound(); // We ran out of channels
    }

    // v27 ORDER NOTE: every validation now runs BEFORE we touch any slot
    // state or subscriptionCount. Pre-v27 layout interleaved validations
    // (heartbeat range, client.connected) with the allocate-and-increment
    // path — when a validation rejected, the counter was already bumped
    // and the slot half-committed, leaking until the next reboot. v27
    // pulls everything up so we can `return` from any rejection without
    // unwinding partial state.

    // 1. clientIP
    if (clientIP == IPAddress(INADDR_NONE))  // v25: explicit cast — sys/socket.h now defines INADDR_NONE as u32_t, ambiguous with the IPAddress overload
    {
        ESP_LOGE(TAG, "Sending %s, for: %s as clientIP missing", response400invalid, server.uri().c_str());
        server.send_P(400, type_txt, response400invalid);
        return;
    }

    // 2. at least one argument
    if (server.args() < 1)
    {
        ESP_LOGE(TAG, "Sending %s, for: %s", response400missing, server.uri().c_str());
        server.send_P(400, type_txt, response400missing);
        return;
    }

    // 3. parse argument indices (no slot state mutated)
    int id = 0;
    bool logViewer = false;
    int heartbeatIntervalArgIdx = -1;
    for (int i = 0; i < server.args(); i++)
    {
        if (server.argName(i).equals("id"))
            id = i;
        else if (server.argName(i).equals("log"))
            logViewer = true;
        else if (server.argName(i).equals("heartbeat"))
            heartbeatIntervalArgIdx = i;
    }

    // 4. heartbeat interval (F5 coerce)
    uint32_t heartbeatInterval = SSE_HEARTBEAT_DEFAULT_SEC;
    if (heartbeatIntervalArgIdx >= 0)
    {
        int hbi = server.arg(heartbeatIntervalArgIdx).toInt();
        if (hbi < 0 || hbi > (int)SSE_HEARTBEAT_MAX_SEC)
        {
            ESP_LOGE(TAG, "Invalid heartbeat interval (0 - %u) for SSE subscription", (unsigned)SSE_HEARTBEAT_MAX_SEC);
            server.send(400, type_txt, "Invalid heartbeat interval");
            return;
        }
        if (hbi == 0)
        {
            // v27: caller asked for "no heartbeat" but the orphan sweep needs a
            // Ticker running on every slot to detect class-5b drops in real-time.
            // Coerce to MIN. logs.html ships v27-and-later with heartbeat=10 so
            // this branch only fires for older logs.html (cache) or third-party clients.
            ESP_LOGI(TAG, "v27: heartbeat=0 coerced to %u for client %s — orphan sweep needs Ticker liveness driver",
                     (unsigned)SSE_HEARTBEAT_MIN_SEC, clientIP.toString().c_str());
            heartbeatInterval = SSE_HEARTBEAT_MIN_SEC;
        }
        else
        {
            heartbeatInterval = (uint32_t)hbi;
        }
    }

    // 5. low-heap rejection (F6) — return 503 without touching slot state.
    if (ESP.getFreeHeap() < SSE_MIN_HEAP_FREE)
    {
        ESP_LOGW(TAG, "SSE subscription rejected - low heap (%u < %u)",
                 (unsigned)ESP.getFreeHeap(), (unsigned)SSE_MIN_HEAP_FREE);
        server.send(503, type_txt, "Low memory, try again shortly\n");
        return;
    }

    // 6. client validity — checked BEFORE we capture into the slot.
    WiFiClient client = server.client();
    if (!client || !client.connected())
    {
        ESP_LOGE(TAG, "Invalid client for SSE subscription");
        server.send(400, type_txt, "Invalid client connection");
        return;
    }

    // 7. find existing UUID. removeSSEsubscription decrements
    //    subscriptionCount; we don't pre-increment here so re-subscribing
    //    is a clean wash.
    bool foundExisting = false;
    for (channel = 0; channel < SSE_MAX_CHANNELS; channel++)
    {
        if (subscription[channel].clientUUID == server.arg(id))
        {
            if (subscription[channel].SSEconnected)
            {
                // Already connected.  We need to close it down as client will be reconnecting
                ESP_LOGD(TAG, "Client %s (%s) already connected on channel %d, remove SSE subscription", clientIP.toString().c_str(), server.arg(id).c_str(), channel);
                removeSSEsubscription(&subscription[channel]);
                break; // without setting foundExisting... so we create new instance.
            }
            else
            {
                // Subscribed but not connected yet, so nothing to close down.
                ESP_LOGD(TAG, "Client %s (%s) already subscribed for SSE but not connected on channel %d", clientIP.toString().c_str(), server.arg(id).c_str(), channel);
            }
            foundExisting = true;
            break;
        }
    }

    // 8. allocate a new slot if needed
    if (!foundExisting)
    {
        // v28 BUG FIX: match the canonical "free" marker used by the orphan
        // sweep at line 1642 — `clientIP == IPAddress(INADDR_NONE)`.
        //
        // Pre-v28 this scan was `if (!subscription[channel].clientIP)`,
        // which calls IPAddress::operator bool(). That returns false ONLY
        // when the address is 0.0.0.0 (the default-constructed state) —
        // NOT when it's INADDR_NONE (0xFFFFFFFF, the marker that
        // setup_web + removeSSEsubscription assign to "free" the slot).
        //
        // Effect: the very first 8 subscribes worked (initial slots have
        // dword=0 from struct init, so !clientIP was true). Every slot
        // subsequently freed via removeSSEsubscription was set to
        // INADDR_NONE — at which point the scan never saw it as free
        // again. After all 8 slots had been used + freed once, the device
        // permanently rejected new subscribes with 503 "no free slots
        // available," even though the sweep correctly reported
        // sseSlotsAlloc=0. The sweep and the scan disagreed about what
        // "free" meant, and the scan had it wrong. v22-v26 hit this
        // identically; the v22 SSE deadlock crashed the device before
        // slot 9 was attempted, masking the bug. v27's deadlock fix
        // exposed it.
        for (channel = 0; channel < SSE_MAX_CHANNELS; channel++)
            if (subscription[channel].clientIP == IPAddress(INADDR_NONE))
                break;
    }

    // 9. no free slot — 503 (still no state mutated for this request)
    if (channel >= SSE_MAX_CHANNELS)
    {
        ESP_LOGE(TAG, "SSE subscription failed - no free slots available");
        server.send(503, type_txt, "No free subscription slots available");
        return;
    }

    // 10. commit slot fields (F7: explicit reset of v27 lifecycle fields).
    subscription[channel].clientIP = clientIP;
    subscription[channel].client = client;
    subscription[channel].heartbeatTimer = Ticker();
    subscription[channel].SSEconnected = false;
    subscription[channel].SSEfailCount = 0;
    subscription[channel].clientUUID = server.arg(id);
    subscription[channel].logViewer = logViewer;
    subscription[channel].heartbeatInterval = heartbeatInterval;
    subscription[channel].pendingRemove = false;
    subscription[channel].subscribedAt = (uint32_t)_millis();
    subscription[channel].lastActivity = (uint32_t)_millis();

    // 11. counter bumped only AFTER all rejection paths exhausted.
    if (!foundExisting)
    {
        subscriptionCount++;
    }

    SSEurl += std::to_string(channel);
    ESP_LOGD(TAG, "Client %s (%s) SSE subscription: %s, Total: %d, Heartbeat: %d, Log: %d", clientIP.toString().c_str(), server.arg(id).c_str(), SSEurl.c_str(), subscriptionCount, heartbeatInterval, (int)logViewer);
    server.sendHeader(F("Cache-Control"), F("no-cache, no-store"));
    server.send_P(200, type_txt, SSEurl.c_str());
}

// v27/v28: best-effort cleanup endpoint. Browser calls this via
// navigator.sendBeacon() on beforeunload to release the SSE slot
// without waiting for the orphan sweep timeout. Browsers don't
// guarantee beacon delivery (especially on mobile background tabs),
// so the orphan sweep is still the authoritative cleanup path —
// this just cuts leak rate during normal navigation.
//
// v28: Same-origin guard added. v27's comment claimed "sendBeacon
// cannot set custom headers" — true for X-* headers, but BROWSERS
// DO populate Origin / Referer / Host on sendBeacon POSTs, which is
// exactly what enforce_same_origin checks. Adding the guard blocks
// drive-by cross-origin closes (e.g. malicious LAN page knocking
// the user offline) without breaking the legitimate beacon flow.
// No AUTH because the UUID is the only authority required and it's
// already 128-bit random; an attacker without the UUID can't target
// a specific session.
void handle_unsubscribe()
{
    if (!enforce_same_origin("/rest/events/unsubscribe")) return;
    String uuid;
    for (int i = 0; i < server.args(); i++)
    {
        if (server.argName(i).equals("id"))
        {
            uuid = server.arg(i);
            break;
        }
    }
    if (uuid.isEmpty())
    {
        server.send_P(400, type_txt, response400missing);
        return;
    }
    int matched = 0;
    for (uint32_t i = 0; i < SSE_MAX_CHANNELS; i++)
    {
        // v28: same INADDR_NONE-vs-zero distinction as line 2003 — must
        // explicitly compare to IPAddress(INADDR_NONE) to recognize the
        // "freed" marker. The previous `&& subscription[i].clientIP`
        // gating was technically the inverse problem (an IPAddress that
        // is INADDR_NONE evaluates as truthy, so this branch fired even
        // for freed slots) — harmless because we additionally checked
        // UUID equality, but tightening for consistency with the rest
        // of the v28 fix.
        if (subscription[i].clientUUID == uuid &&
            subscription[i].clientIP != IPAddress(INADDR_NONE))
        {
            ESP_LOGD(TAG, "v27: unsubscribe beacon for UUID %s on channel %u — flag pendingRemove",
                     uuid.c_str(), (unsigned)i);
            subscription[i].pendingRemove = true;
            matched++;
        }
    }
    server.send(204, type_txt, "");
    (void)matched;
}

void handle_crashlog()
{
    server.client().print(response200);
    ratgdoLogger->printCrashLog(server.client());
}

void handle_showlog()
{
    server.client().print(response200);
    ratgdoLogger->printMessageLog(server.client());
}

void handle_showrebootlog()
{
    server.client().print(response200);
#ifdef ESP8266
    File file = LittleFS.open(REBOOT_LOG_MSG_FILE, "r");
    ratgdoLogger->printSavedLog(file, server.client());
    file.close();
#else
    ratgdoLogger->printSavedLog(server.client());
#endif
}

void handle_clearcrashlog()
{
    AUTHENTICATE();
    ESP_LOGI(TAG, "Clear saved crash log");
    ratgdoLogger->clearCrashLog();
    server.send_P(200, type_txt, PSTR("Crash log cleared\n"));
}

#ifdef CRASH_DEBUG
void handle_crash_oom()
{
    ESP_LOGI(TAG, "Attempting to use up all memory");
    server.send_P(200, type_txt, PSTR("Attempting to use up all memory\n"));
    delay(1000);
    for (int i = 0; i < 30; i++)
    {
        ESP_LOGI(TAG, "malloc(1024)");
        crashptr = malloc(1024);
    }
}

void handle_forcecrash()
{
    ESP_LOGI(TAG, "Attempting to null ptr deref");
    server.send_P(200, type_txt, PSTR("Attempting to null ptr deref\n"));
    delay(1000);
    ESP_LOGI(TAG, "Result: %s", test_str);
}
#endif // CRASH_DEBUG

void SSEBroadcastState(const char *data, BroadcastType type)
{
    if (!web_setup_done)
        return;

    // Flash LED to signal activity
    // led.flash(FLASH_MS);

    // if nothing subscribed, then return
    if (subscriptionCount == 0)
        return;

    for (uint32_t i = 0; i < SSE_MAX_CHANNELS; i++)
    {
        YIELD(); // yield between each SSE client
        if (subscription[i].SSEconnected)
        {
            if (subscription[i].client.connected())
            {
                if (type == LOG_MESSAGE)
                {
                    if (subscription[i].logViewer)
                    {
                        if (snprintf_P(writeBuffer, sizeof(writeBuffer), PSTR("event: logger\ndata: %s\n\n"), data) >= (int)sizeof(writeBuffer))
                        {
                            // Will not fit in our write buffer, let system printf handle
#ifdef ESP8266
                            subscription[i].client.flush(); // make sure previous data all sent.
#endif
                            // v28: capture printf return — only stamp on success.
                            // Pre-v28 we stamped unconditionally, which could mask
                            // a wedged subscriber whose oversized log writes were
                            // failing silently (delayed orphan-sweep detection by
                            // one cycle). Print returns size_t bytes written.
                            size_t pwrote = subscription[i].client.printf("event: logger\ndata: %s\n\n", data);
                            if (pwrote > 0)
                                subscription[i].lastActivity = (uint32_t)_millis();
                        }
                        else
                        {
                            // v27: only stamp lastActivity on a successful write so
                            // the orphan-sweep idle check (5c) actually sees idle slots.
                            // v29: tri-state — stamp on OK or BUFFER_FULL, only skip
                            // on FAILED (real wedge). Tailscale / congested-link
                            // subscribers no longer get reaped every 120s.
                            SseWriteResult r = clientWriteEx(subscription[i].client, writeBuffer);
                            if (r != SseWriteResult::FAILED)
                            {
                                subscription[i].lastActivity = (uint32_t)_millis();
                            }
                        }
                    }
                }
                else if (type == RATGDO_STATUS)
                {
                    ESP_LOGV(TAG, "Client %s (%s) send status SSE on channel %d, data: %s",
                             IPAddress(subscription[i].clientIP).toString().c_str(),
                             subscription[i].clientUUID.c_str(), i, data);
                    if (snprintf_P(writeBuffer, sizeof(writeBuffer), PSTR("event: message\ndata: %s\n\n"), data) >= (int)sizeof(writeBuffer))
                    {
                        // Will not fit in our write buffer, let system printf handle
#ifdef ESP8266
                        subscription[i].client.flush(); // make sure previous data all sent.
#endif
                        // v28: gate on printf return — see LOG_MESSAGE path above.
                        size_t pwrote = subscription[i].client.printf("event: message\ndata: %s\n\n", data);
                        if (pwrote > 0)
                            subscription[i].lastActivity = (uint32_t)_millis();
                    }
                    else
                    {
                        // v29: tri-state — stamp on OK or BUFFER_FULL, only skip
                        // on FAILED (real wedge). Same rationale as LOG_MESSAGE.
                        SseWriteResult r = clientWriteEx(subscription[i].client, writeBuffer);
                        if (r != SseWriteResult::FAILED)
                        {
                            subscription[i].lastActivity = (uint32_t)_millis(); // v27
                        }
                    }
                }
            }
            else
            {
                // Client connection has gone.  Remove from our subscribed client list
                ESP_LOGD(TAG, "Client %s (%s) not listening (broadcast), remove SSE subscription", subscription[i].clientIP.toString().c_str(), subscription[i].clientUUID.c_str());
                removeSSEsubscription(&subscription[i]);
            }
        }
    }
    YIELD();
}

// Implement our own firmware update so can enforce MD5 check.
// Based on HTTPUpdateServer
void _setUpdaterError()
{
    StreamString str;
    Update.printError(str);
    _updaterError = str.c_str();
    ESP_LOGE(TAG, "Update error: %s", str.c_str());
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
#ifdef ESP8266
        eboot_command_clear();
#else
        // TODO how to handle firmware upload failure on ESP32?
#endif
        firmwareUpdateSub = NULL;
        ESP_LOGE(TAG, "Firmware upload error. Aborting update, not rebooting");
        server.send(400, type_txt, _updaterError.c_str());
        return;
    }

    if (server.args() > 0)
    {
        firmwareUpdateSub = NULL;
        // Don't reboot, user/client must explicitly request reboot.
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
    static uint32_t nextPrintPercent;
    HTTPUpload &upload = server.upload();
    static bool verify = false;
    static size_t size = 0;
    static const char *md5 = NULL;

    if (upload.status == UPLOAD_FILE_START)
    {
        _updaterError.clear();

#ifdef ESP8266
        _authenticatedUpdate = !userConfig->getPasswordRequired() || server.authenticateDigest(userConfig->getwwwUsername(), userConfig->getwwwCredentials());
#else
        _authenticatedUpdate = !userConfig->getPasswordRequired() || server.authenticate(ratgdoAuthenticate);
#endif
        if (!_authenticatedUpdate)
        {
            ESP_LOGE(TAG, "Unauthenticated Update");
            return;
        }
        ESP_LOGI(TAG, "Update: %s", upload.filename.c_str());
        verify = !strcmp(server.arg("action").c_str(), "verify");
        size = atoi(server.arg("size").c_str());
        md5 = server.arg("md5").c_str();

        // We are updating.  If size and MD5 provided, save them
        firmwareSize = size;
        if (strlen(md5) > 0)
            strlcpy(firmwareMD5, md5, sizeof(firmwareMD5));

        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        ESP_LOGI(TAG, "Available space for upload: %lu", maxSketchSpace);
        ESP_LOGI(TAG, "Firmware size: %s", (firmwareSize > 0) ? std::to_string(firmwareSize).c_str() : "Unknown");
        ESP_LOGI(TAG, "Flash chip speed %d MHz", ESP.getFlashChipSpeed() / 1000000);
        // struct eboot_command ebootCmd;
        // eboot_command_read(&ebootCmd);
        // ESP_LOGI(TAG, "eboot_command: 0x%08X 0x%08X [0x%08X 0x%08X 0x%08X (%d)]", ebootCmd.magic, ebootCmd.action, ebootCmd.args[0], ebootCmd.args[1], ebootCmd.args[2], ebootCmd.args[2]);
        if (firmwareSize > maxSketchSpace)
        {
            ESP_LOGE(TAG, "Firmware size is larger than available OTA upload space");
            // If we detect this error then we will not shut down all our services, because upload will fail.
            // Failure is detected on first call to Update.write() where it will set UPDATE_ERROR_SPACE.
            // This is passed back to the client with a http 400 error and the string "Not Enough Space"
        }
        else if (!verify)
        {
            // Close services so we don't have to handle network traffic during update
            // Only if not verifying as either will have been shutdown on immediately prior upload, or we
            // just want to verify without disrupting operation of the HomeKit service.
            ESP_LOGI(TAG, "Shutdown HomeKit and GDO communications");

            // Service loop has things like reboot after X days, homekit notifications, etc. that we don't want during OTA
            suspend_service_loop = true;
#ifdef RATGDO32_DISCO
            // Ignore vehicle distance sensor
            vehicle_setup_done = false;
#endif
            shutdown_comms();
#ifdef ESP8266
            // Shutdown HomeKit
            homekit_setup_done = false;
            arduino_homekit_close();
#else
            // Shutdown HomeSpan server
            vTaskDelete(homeSpan.getAutoPollTask());
#endif
        }

        if (!verify && !Update.begin((firmwareSize > 0) ? firmwareSize : maxSketchSpace, U_FLASH))
        {
            _setUpdaterError();
        }
        else if (strlen(firmwareMD5) > 0)
        {
            // uncomment for testing...
            // char firmwareMD5[] = "675cbfa11d83a792293fdc3beb199cXX";
            ESP_LOGI(TAG, "Expected MD5: %s", firmwareMD5);
            Update.setMD5(firmwareMD5);
            if (firmwareSize > 0)
            {
                uploadProgress = 0;
                nextPrintPercent = 5;
                ESP_LOGI(TAG, "%s progress: 00", verify ? "Verify" : "Update");
            }
        }
    }
    else if (_authenticatedUpdate && upload.status == UPLOAD_FILE_WRITE && !_updaterError.length())
    {
        // Progress dot dot dot
        Serial.print(".");
        if (firmwareSize > 0)
        {
            uploadProgress += upload.currentSize;
            uint32_t uploadPercent = (uploadProgress * 100) / firmwareSize;
            if (uploadPercent >= nextPrintPercent)
            {
                Serial.print("\n"); // newline after the dot dot dots
                ESP_LOGI(TAG, "%s progress: %d", verify ? "Verify" : "Update", uploadPercent);
                nextPrintPercent += 5;
                // Report percentage to browser client if it is listening
                if (firmwareUpdateSub && firmwareUpdateSub->client.connected())
                {
                    static char *json = status_json;
                    TAKE_MUTEX();
                    JSON_START(json);
                    JSON_ADD_INT("uploadPercent", uploadPercent);
                    JSON_END();
                    JSON_REMOVE_NL(json);
                    snprintf_P(writeBuffer, sizeof(writeBuffer), PSTR("event: uploadStatus\ndata: %s\n\n"), json);
                    // v28: stamp lastActivity on success — matches the
                    // SSEBroadcastState pattern. Prevents the orphan
                    // sweep from reaping the slot mid-update if a slow
                    // upload spans >300s (large firmware on slow link).
                    // v29: tri-state — stamp on OK or BUFFER_FULL, only
                    // skip on FAILED (real wedge).
                    SseWriteResult r = clientWriteEx(firmwareUpdateSub->client, writeBuffer);
                    if (r != SseWriteResult::FAILED)
                        firmwareUpdateSub->lastActivity = (uint32_t)_millis();
                    GIVE_MUTEX();
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
        Serial.print("\n"); // newline after last of the dot dot dots
        if (!verify)
        {
            if (Update.end(true))
            {
                ESP_LOGI(TAG, "Upload size: %zu", upload.totalSize);
            }
            else
            {
                _setUpdaterError();
            }
            firmwareUpdateSub = NULL;
        }
    }
    else if (_authenticatedUpdate && upload.status == UPLOAD_FILE_ABORTED)
    {
        if (!verify)
            Update.end();
        ESP_LOGI(TAG, "%s was aborted", verify ? "Verify" : "Update");
        firmwareUpdateSub = NULL;
    }
}
