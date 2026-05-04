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
 *
 */
#pragma once

// C/C++ language includes
// none

// ESP system includes
#ifdef ESP32
#define PROGMEM // so it is no-op in webcontent.h
#include <WebServer.h>
extern WebServer server;
#else
#include <ESP8266WebServer.h>
extern ESP8266WebServer server;
#endif

// RATGDO project includes
// none

extern void setup_web();
extern void web_loop();
// v22: drain SSE subscriptions flagged for deferred remove (set from
// inside Ticker callbacks to avoid the in-callback Ticker.detach() →
// vTaskDelete → uxListRemove crash). Call from main-loop context.
extern void process_sse_pending_removes();
// v27: orphan-slot sweep — see web.cpp. Flags slots that have leaked
// past the heartbeat-Ticker safety net (heartbeat=0, pre-handshake
// abandons, idle wedged sockets). Called from service_timer_loop.
extern void sweep_sse_orphans();

extern void handle_notfound();
extern void handle_reboot();
extern void handle_reconnect_homekit();
extern void handle_refresh_mdns();
extern void handle_dump_homekit_state();
extern void factoryReset();

extern void load_page(const char *page);

extern void build_status_json(char *json);

extern const char response400invalid[];
extern const char type_txt[];
extern const char type_html[];

extern _millis_t lastDoorUpdateAt;
extern _millis_t lastDoorOpenAt;
extern _millis_t lastDoorCloseAt;

extern bool web_setup_done;

extern void notify_new_ipv4_address();
extern void notify_new_ipv6_address();

enum BroadcastType : uint8_t
{
    RATGDO_STATUS = 1,
    LOG_MESSAGE = 2,
};
void SSEBroadcastState(const char *data, BroadcastType type = RATGDO_STATUS);
