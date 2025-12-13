# Multi-door sidebar and authenticated peer controls

## Summary
- add the `peers` module to advertise discovered controllers over mDNS, expose `/peers.json`, and forward `/setgdo` commands with upstream HTTP status data
- ship the multi-door banner + drawer UI so every controller can show peers, live health, and quick action buttons without leaving the local web UI
- implement `/peer-command` plus credential prompts so users can operate auth-protected peers with stored Basic tokens and clear error feedback
- document the workflow in `docs/multi-door-dashboard.md`, including the new auth expectations and UI affordances

## Testing
- `platformio run --environment ratgdo_esp32dev`
