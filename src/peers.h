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

#pragma once

#include <Arduino.h>
#include "utilities.h"

// Initialize peer discovery service.
extern void peers_begin();

// Periodic loop to maintain peer cache and update service TXT records.
extern void peers_loop();

// Returns a JSON payload describing self + discovered peers.
extern char *peers_json();

// Indicates whether any peers are currently known.
extern bool peers_available();

// Sends a key/value command to the specified peer via its HTTP interface.
// Optionally accepts a base64-encoded Basic auth token and returns the HTTP status.
extern bool peers_send_command(const String &peerId, const String &key, const String &value, const String &authToken, int &statusCode, String &message);

// PeerInfo struct for peer representation
struct PeerInfo
{
	String id;
	String name;
	String host;
	IPAddress ip;
	uint16_t port = 80;
	String firmware;
	String doorState;
	String lockState;
	bool lightOn = false;
	bool hasLaser = false;
	bool paired = false;
	int32_t rssi = 0;
	_millis_t lastSeen = 0;
};
