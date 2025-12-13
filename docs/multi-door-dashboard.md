# Multi-Door Dashboard Plan

## Current Implementation (v3.4.x)

- ESP32 builds now advertise a dedicated `_ratgdo._tcp` mDNS service that includes live TXT metadata (door state, lock state, light, firmware, pairing, RSSI, uptime).
- The firmware scans for peers every ~15 seconds, maintains a short-lived cache, and exposes the aggregated data via `GET /peers.json` plus a POSTable `/peer-command` relay for door/light/remote actions.
- The web UI shows a contextual banner whenever at least one peer is discovered and provides a right-hand drawer that lists `self` plus every reachable RATGDO with basic connection/states and in-line controls (door toggle, light toggle, remote lock toggle).
- When a peer requires HTTP auth, the UI now prompts once for credentials, stores them in `localStorage`, and forwards them via the proxy so commands succeed without retyping.
- The drawer updates roughly every 20 seconds while the page is visible and automatically hides if no peers remain.

### Known Limitations

- Drawer quick actions currently cover door/light/remote toggles. Broadcasts and additional controls remain planned.
- Credentials are stored per-browser in plain Base64 today; long term we still plan to encrypt-at-rest and add a management UI.
- Peers discovered via mDNS only—manual entries and persistence in config storage are not yet implemented.
- ESP8266 targets skip this feature due to missing `ESPmDNS` parity; the UI simply never shows the banner on that hardware.
- TXT metadata is only as fresh as the peer’s heartbeat; stale peers age out after 60 seconds to avoid surfacing outdated devices.

The remainder of this document captures the full dashboard vision so we can continue iterating beyond the minimal discovery/browse experience that shipped first.

## Goals

- Show the status of every RATGDO on the local network (including the device hosting the UI) from any RATGDO.
- Provide quick actions (door, light, lock, reboot) for each peer directly from the dashboard.
- Allow broadcasting a command to multiple units at once.
- Work even if mDNS discovery fails by letting users add peers manually.
- Respect authentication requirements on remote devices.

## Architecture Overview

1. **Discovery Service**
   - Each RATGDO advertises itself via mDNS/Bonjour (`_ratgdo._tcp`) with TXT records: `name`, `ip`, `port`, `fw`, `secure`, plus live snippets such as `door`, `lock`, `light`, `motion`, `paired`, `ttc`, `rssi`, and `uptime` for instant-at-a-glance status.
   - A new backend helper periodically (every 15 s) scans for peers and merges results with manually added entries stored in config (`peers[]`).
   - Backend exposes `GET /peers.json` returning:
     ```json
     {
       "self": { ...status summary... },
       "discovered": [ { "id": "MAC", "name": "Garage East", "ip": "192.168.1.23", "secure": true }, ... ],
       "manual": [ ... ]
     }
     ```

2. **Peer Status Aggregation**
   - A new endpoint `GET /peer-status.json?id=<peer-id>` fetches `/status.json` from the peer over HTTP(S), passing stored credentials if configured.
   - Responses are cached for ~5 seconds to keep the UI responsive; errors bubble up with timestamps.
   - For auth-enabled peers, the UI prompts once and stores credentials encrypted in `localStorage` (keyed by device id) to send with subsequent calls.

3. **Command Relay**
   - Backend endpoint `POST /peer-command` accepts `{ targetIds: [], command: { key: "garageDoorState", value: "1" } }` and forwards to each peer’s `/setgdo` handler using current credentials.
   - Responses per target include success flag and optional error message so UI can show toast results.

4. **UI/UX**
   - Main page gains a "Multi-Door Dashboard" drawer containing cards for self + peers. Each card shows:
     - Door state, vehicle status, light, lock state, Wi-Fi RSSI, uptime snippet.
     - Action buttons: `Toggle Door`, `Light On/Off`, `Lock/Unlock Remotes`, `Reboot`, `Logs`.
     - Status badges for firmware mismatch or low signal.
   - A management modal lets users:
     - Refresh discovery, rename cards, remove stale units.
     - Add manual peers (IP, friendly name, optional credentials requirement).
   - Broadcast panel at top with target multiselect + action buttons (Door, Lock, Light, Reboot).

5. **Config & Persistence**
   - New config entries under `cfg_peers` storing an array of `{ id, name, host, port, requiresAuth }`.
   - Backend merges config-saved peers with discovery results; manual peers override discovery metadata when hostnames match.

6. **Security**
   - No credentials stored on device; only the browser stores them via Web Crypto (AES-GCM using a key derived from login session) and sends them to the backend per request.
   - Backend validates that remote commands are only accepted from authenticated local users (same auth used for settings page).
   - Rate limiting ensures a rogue browser can’t flood peers (max 5 broadcasts/minute per session).

## Implementation Phases

1. **Backend Foundation**
   - Add mDNS advertiser (ESP32 `ESPmDNS`) + service browser.
   - Implement `/peers.json`, `/peer-status.json`, `/peer-command` handlers and helper classes (peer registry, HTTP client with auth support).
   - Extend config storage for manual peers (array serialized as JSON string).

2. **Frontend Dashboard**
   - Build new dashboard drawer in `index.html` with cards + action buttons.
   - Add `functions.js` modules for fetching peers/status, credential prompts, command relays, and UI updates.
   - Provide broadcast action UI and error/success toasts.

3. **Polish & Testing**
   - Handle offline peers gracefully with retry/backoff.
   - Surface firmware mismatches or stale data warnings.
   - Document feature in README + add screenshots.
   - Manual + automated testing with two physical RATGDOs and simulated peers.

## Acceptance Criteria

- Visiting any RATGDO shows all reachable peers within ~5 seconds.
- Quick actions operate reliably with success/failure feedback.
- Manual peers persist across reboots and appear alongside discovered ones.
- Users can broadcast at least door/light/lock commands to multiple peers at once.
- Feature degrades gracefully when other units are offline or require credentials.
