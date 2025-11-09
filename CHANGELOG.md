# Change Log

**v3.x.x firmware is for ratgdo32 and ratgdo32-disco boards only**

All notable changes to `homekit-ratgdo32` will be documented in this file. This project tries to adhere to [Semantic Versioning](http://semver.org/).

## v3.4.1 (2025-11-??)

### What's Changed

* Bugfix: Vehicle departing motion sensor may not trigger correctly (ratgdo32-disco only) 
* Feature: Add a warning and countdown timer to web page when Sec+2.0 doors have automatic door close (TTC) active
* Other: Display "Off" instead of "0" when settings sliders are set to zero seconds/minutes
* Other: Improved web page design for iPhone and iPad devices
* Other: Attempt to recover from out-of-sync Sec+2.0 rolling code https://github.com/ratgdo/homekit-ratgdo/issues/315
* Other: HomeSpan library updated to version 2.1.6 (ratgdo32 only)
* Other: Various log message cleanup to make debugging easier and reduce log clutter at default Info level

### Known Issues

* Sec+ 1.0 doors with digital wall panel (e.g. 889LM) sometimes do not close after a time-to-close delay. Please watch your door to make sure it closes after TTC delay.
* Sec+ 1.0 doors with "0x37" digital wall panel (e.g. 398LM) not working.  We now detect but will not support them.  Recommend replacing with 889LM panel.
* When creating automations in Apple Home the garage door may show only lock/unlock and not open/close as triggers.  This is a bug in Apple Home. Workaround is to use the Eve App to create the automation, it will show both options.

## v3.4.0 (2025-11-01)

### What's Changed

* Bugfix: User selected syslog facility not restored on startup. https://github.com/ratgdo/homekit-ratgdo32/issues/116
* Bugfix: Crash when HomeKit tries to open or close a dry contact door. https://github.com/ratgdo/homekit-ratgdo32/issues/117
* Bugfix: Sec+2.0 only, not handling packet transmit errors during initialization
* Feature: Sec+2.0 only, support garage door automatic close after selected delay, [SEE README](https://github.com/ratgdo/homekit-ratgdo32/blob/main/README.md#automatic-close)
* Feature: Add time-to-close countdown timer to web page
* Other: Allow user to disable triggering motion from Sec+2.0 wall panel motion sensors
* Other: Ratgdo-disco only, update vehicle presence algorithm for no vehicle present to match ESPhome
* Other: Adjust some Info-level log messages to Debug- or Error-level... reduces log clutter at default Info level

## v3.3.9 (2025-10-24)

### What's Changed

* Bugfix: Update function that calculates median door open/close duration. https://github.com/ratgdo/homekit-ratgdo/issues/309
* Bugfix: Do not cancel time-to-close if second door close request received. https://github.com/ratgdo/homekit-ratgdo32/issues/112
* Other: Ratgdo-disco only, update vehicle presence algorithm to match ESPhome https://github.com/ratgdo/esphome-ratgdo/pull/496
* Other: Additional Serial CLI commands for development and debugging to e.g, provision WiFi SSID and password.

## v3.3.8 (2025-10-19)

### What's Changed

* Bugfix: dry contact doors not reporting status correctly on web page. https://github.com/ratgdo/homekit-ratgdo32/issues/109
* Bugfix: Sec+1.0 add timeout when waiting for GDO reply to poll commands https://github.com/ratgdo/homekit-ratgdo32/issues/111
* Other: Save door open/close durations so not reset to unknown on a reboot
* Other: Add timers to check that door starts to open/close and reaches fully opened/closed state in expected time
* Other: Sec+2.0 use MotorOn packet to error correct if we miss notification packet of door opening or closing
* Other: Add serial CLI commands to scan WiFi networks and reset door ID & rolling codes
* Other: Remove known issues list from prior versions in CHANGELOG.md... because they are now repeating

## v3.3.7 (2025-10-12)

### What's Changed

* Bugfix: Sec+2.0 doors not opening or closing. Issue https://github.com/ratgdo/homekit-ratgdo/issues/305

## v3.3.6 (2025-10-12)

### What's Changed

* Bugfix: HomeKit pairing failed with out-of-compliance error. https://github.com/ratgdo/homekit-ratgdo/issues/300
* Bugfix: Web wage status occasionally getting out-of-sync with actual light/lock/door state.
* Bugfix: Door open/close duration calculation not handling cases where door reverses before reaching open/close state.
* Bugfix: Setting syslog port not taking effect until after reboot. https://github.com/ratgdo/homekit-ratgdo/issues/304
* Bugfix: do not attempt to act on Sec+2.0 packet that failed to decode. Issue https://github.com/ratgdo/homekit-ratgdo32/issues/106
* Feature: Change WiFi and MDNS hostname when user changes GDO name. https://github.com/ratgdo/homekit-ratgdo32/issues/93
* Feature: Add clipboard copy icon to IP address and mDNS name.
* Other: Detect (but do not support) 0x37 wall panels like LiftMaster 398LM. https://github.com/ratgdo/homekit-ratgdo32/issues/95
* Other: ESP8266 (original ratgdo) only... suspend GDO communications during HomeKit pairing process.
* Other: ESP8266 (original ratgdo) only... move more constants into PROGMEM and optimize use of system stack.

## v3.3.5 (2025-09-28)

### What's Changed

* Bugfix: Add error handling for a blank SSID... force boot into Soft AP mode, https://github.com/ratgdo/homekit-ratgdo/issues/295
* Bugfix: Buffer overrun that caused Improv setup to fail, https://github.com/ratgdo/homekit-ratgdo/issues/298
* Bugfix: Aog messages that are truncated for exceeding buffer size not null terminated
* Other: Add simple serial console CLI (when HomeSpan CLI disabled) to allow setting debug level, displaying saved logs and request reboot.

## v3.3.4 (2025-09-27)

### What's Changed

* Bugfix: Activity LED blink was not obeying user preferences (e.g. always off). https://github.com/ratgdo/homekit-ratgdo/issues/292
* Bugfix: Ignore implausibly long door opening / closing times. https://github.com/ratgdo/homekit-ratgdo32/issues/98
* Bugfix: Room occupancy state not set on motion (ratgdo32 only). https://github.com/ratgdo/homekit-ratgdo32/issues/96
* Feature: Allow user selection of Syslog facility number (Local0 .. Local7). https://github.com/ratgdo/homekit-ratgdo32/issues/94
* Feature: Add MDNS service for _http._tcp to allow local network name discovery. https://github.com/ratgdo/homekit-ratgdo32/issues/93
* Other: Disable HomeKit and garage door communications during OTA firmware update.
* Other: Miscellaneous stability improvements.

## v3.3.3 (2025-09-20)

### What's Changed

* Bugfix... Date and time on web page now displayed in the time zone of the server (NTP server feature must be enabled).
* Bugfix... Log level was not getting set correctly (should take effect immediately but it required a reboot).
* Bugfix... Park assist laser was not activating on vehicle arrival (since v3.3.0).
* Bugfix... Displaying last reboot log was missing time stamps.
* New Feature... last door open and close date and time is displayed under opening/closing status (NTP server feature must be enabled).
* Other... Add a "home" button to system logs page because iOS and iPad OS 26 have removed the "done" button.

## v3.3.2 (2025-09-14)

### What's Changed

* Other... Add release notes to firmware update dialog.
* Other... Reduce unnecessary network traffic during firmware update

## v3.3.1 (2025-09-13)

### What's Changed

* Bugfix... Updating from GitHub was not finding the firmware download file.

## v3.3.0 (2025-09-13)

Version 3.3.0 is a significant upgrade for ESP32-based ratgdo boards. Almost all source files for the ESP8266 and ESP32 versions of ratgdo have been merged which results in minor changes to the underlying features and function for ESP32 versions. The main benefit is for the original ESP8266-based ratgdo boards.

While source files have been merged there remain significant differences between the two board types, most notably in the library used to communicate with HomeKit which are completely different.

* Before an Over-The-Air (OTA) upgrade it is good to first reboot your current version.

### What's Changed

* Other... Significant source code changes to support move towards single code base for ESP8266 and ESP32
* Other... Change garage door communications from using GDOLIB library to same code as used for ESP8266

## v3.2.1 (2025-07-??)

### What's Changed

* Bugfix: IPv6 support should now be working (updated HomeSpan to release 2.1.3).
* Bugfix: Boot safely when no GDO connected (updates GDOLIB to release 1.2.1).

## v3.2.0 (2025-07-04)

### What's Changed

* Bugfix: Doors without obstruction sensor may not respond to door action CLOSE commands, use TOGGLE instead (Issue #81).

## v3.1.12 (2025-06-15)

### What's Changed

* Bugfix: Sec+ 1.0 doors may not close at end of time-to-close delay (Issue #77).

## v3.1.11 (2025-06-13)

### What's Changed

* Feature: Make IPv6 optional, disabled by default.

## v3.1.10 (2025-06-08)

### What's Changed

* Feature: Enable IPv6 (PR #74) with thanks to https://github.com/apearson

## v3.1.9 (2025-05-11)

### What's Changed

* Bugfix: Hostname not set correctly to comply with RFC952 (Issue #72)
* Bugfix: Door close duration not reported correctly on web page
* Other: Add console CLI command to allow setting debug message log level
* Other: Add console CLI command to facilitate testing moving door to position between 0% and 100%
* Other: Update to use version 2.1.2 of upstream HomeSpan library
  
## v3.1.8 (2025-04-27)

### What's Changed

* Bugfix: show motion status and controls when any motion trigger type is set
* Bugfix: room occupancy and motion reset time not working as expected

### Known Issues

* Still testing... Future updates MAY include breaking changes requiring a flash erase and re-upload.

## v3.1.7 (2025-04-27)

### What's Changed

* Feature: Allow user setting to disable light flashing during time-to-close
* Feature: Add a HomeKit occupancy sensor triggered by motion that is active for user specified duration between 0 and 120 minutes, Requested in issue #45
* Bugfix: Do not run setup function(s) if already run.  Hope that this fixes issue #51
* Bugfix: Obstruction sensor not triggering HomeKit motion, fix issue #65
* Other: Additional error checking and changes to log verbosity for some messages.

### Known Issues

* Still testing... Future updates MAY include breaking changes requiring a flash erase and re-upload.

## v3.1.6 (2025-04-01)

Version 3.1.x include major change to garage door opener (GDO) communications.  This has major benefit in separating out the details of communicating with the garage door from HomeKit and the ratgdo user interface, greatly simplifying our code.

### What's Changed

* Feature: Allow user setting for dry contact debounce timer from 50ms to 1000ms
* Bugfix: HomeKit door status not always correct, reported in issue #57
* Bugfix: Fis time-to-close light flashing to work as intended (flash every 1/2 second)

## v3.1.5 (2025-03-31)

Version 3.1.x include major change to garage door opener (GDO) communications.  This has major benefit in separating out the details of communicating with the garage door from HomeKit and the ratgdo user interface, greatly simplifying our code.

### What's Changed

* Bugfix: Dry contact wall panel button not working, fixes issue #57

### Known Issues

* Still testing... Future updates MAY include breaking changes requiring a flash erase and re-upload.

## v3.1.4 (2025-03-30)

Version 3.1.x include major change to garage door opener (GDO) communications.  This has major benefit in separating out the details of communicating with the garage door from HomeKit and the ratgdo user interface, greatly simplifying our code.

### What's Changed

* Feature: Allow selection between software serial port emulation and hardware UART.  May help with issue #48
* Feature: Add debug terminal command to print out buffered message log
* Bugfix: Default to always use ratgdo's own timer for door time-to-close (TTC) and not the TTC built-in to garage door opener.  Partially fixes issue #50
* Bugfix: Remove user interface setting for built-in time-to-close, also addresses issue #50
* Bugfix: Garage door opening/closing status not reported correctly to HomeKit, fixes issue #53
* Buffix: Sec+ 1.0 garage door was unexpectedly opening on reboot when switching between hardware UART and s/w serial
* Other: Add additional debug and error checking to assist with fixing issue #51

### Known Issues

* Still testing... Future updates MAY include breaking changes requiring a flash erase and re-upload.

## v3.1.0 - v3.1.3 (2025-03-07)

Version 3.1.x include major change to garage door opener (GDO) communications.  This has major benefit in separating out the details of communicating with the garage door from HomeKit and the ratgdo user interface, greatly simplifying our code.

As this is a major change, thorough testing is required.

### What's Changed

* Feature: Updated [HomeSpan](https://github.com/HomeSpan/HomeSpan) to version 2.1.1
* Feature: Use new [library](https://github.com/dkerr64/gdolib) for garage door communications
* Feature: For Sec+2.0 use GDO's built-in Time-to-Close (TTC).  Option on user interface to disable and use ratgdo timer.
* Feature: Add user option to disable HomeKit motion and occupancy accessories for vehicle presence.
* Feature: Add door opening and closing duration to web page (calculated first time door operated after reboot).
* Feature: Added message log level to user interface, defaults to INFO.
* Bugfix: Dry contact should be working now, fixes issue #41
* Other: Replace all our RINFO()/RERROR() message log macros with standard ESP_LOGx() macros.

Note: The built-in TTC feature operates slightly differently than ratgdo's timer.  If you set the TTC to e.g. 20 seconds, then on requesting a door close it will wait, in silence, for 20 seconds then sound its internal beeper and flash the lights for 10 seconds before closing the door... thus 30 seconds before the door closes.  The TTC timer in ratgdo will immediately sound its buzzer and flash lights, and close the door after 20 seconds.

### Known Issues

* Still testing... Future updates MAY include breaking changes requiring a flash erase and re-upload.

## v3.0.7 (2025-02-07)

### What's Changed

* Feature: Report door open/close cycle count and emergency backup battery charging/full status to web page (Sec+2.0 only).
* Bugfix: Problem with time-to-close and Sec+1.0 on doors with digital wall panel, issue #35
* Other: Support logging of ESP_LOGx() messages through our logger facility.

### Known Issues

* Still testing... Future updates MAY include breaking changes requiring a flash erase and re-upload.

## v3.0.6 (2025-01-17)

### What's Changed

* Bugfix: Crash in esp_timer.c possibly related to multi-thread use of Ticker for LED timer (Issue #30).
* Bugfix: Motion sensor status should not show if there is no motion sensor.
* Bugfix: Crash reported at end of time-to-close delay (Issue #32).
* Bugfix: Could not set vehicle threshold above 200cm
* Feature: Set default time-to-close delay to 5 seconds and add warning if user selects lower value (Issue #33).
* Feature: Dynamically create HomeKit QR setup ID, and QR code graphic for pairing.

### Known Issues

* Still testing... Future updates MAY include breaking changes requiring a flash erase and re-upload.

## v3.0.5 (2025-01-05)

### What's Changed

* Feature: Add user setting to enable/disable parking assist laser, and set duration of assist laser.
* Feature: Added support to save and view message log on crash (Issue #2).
* Bugfix: Remove multiple copies of web page content from the firmware binary.
* Bugfix: Use 64-bit integer to handle milliseconds since last boot.
* Bugfix: Handle more possible return codes from vehicle distance sensor.
* Bugfix: Door status incorrectly reported to HomeKit if close requested for already closed door (Issue #28).
* Bugfix: Last door change date/time was not always been set correctly after reboot.
* Bugfix: Activity LED options not correctly shown in web page.
* Bugfix: Activity LED constantly on with Sec+ 1.0 protocol.
* Bugfix: Remove wait for incoming serial packet before starting to detect Sec+ 1.0 digital wall panel.
* Updated copyright statement(s) to include year 2025.

### Known Issues

* Still testing... Future updates MAY include breaking changes requiring a flash erase and re-upload.

## v3.0.4 (2024-12-21)

### What's Changed

* Feature: Add HomeKit light switch and web page button for parking assist laser
* Feature: Change web page separated buttons for on/off, open/close, etc. to single buttons

### Known Issues

* Still testing... Future updates MAY include breaking changes requiring a flash erase and re-upload.

## v3.0.3 (2024-12-19)

### What's Changed

* Bugfix: Soft AP list of available WiFi networks not properly terminated
* Bugfix: Setting static IP address did not set correct subnet mask
* Bugfix: Crash in soft AP because memory buffer was not allocated
* Feature: Add support Improv-based WiFi provisioning
* Feature: Add the 10 minute timeout on soft AP mode
* Feature: Add the check for WiFi connectivity 30 seconds after change to static IP

### Known Issues

* Still testing... Future updates MAY include breaking changes requiring a flash erase and re-upload.

## v3.0.2 (2024-12-16)

### What's Changed

* Feature: Dry contact support, with thanks to @tlhagan
* Feature: Add support for HomeKit Identify characteristic
* Feature: Support for ratgdo32 as well as ratgdo32-disco
* Bugfix: Vehicle distance sensor logic

### Known Issues

* Still testing... Future updates MAY include breaking changes requiring a flash erase and re-upload.

## v3.0.1 (2024-12-11)

### What's Changed

* Feature: Beeps during time-to-close delay
* Feature: Vehicle presence, arriving, departing sensing, and parking assist laser
* Bugfix: Web-based flash installer now working
* Bugfix: OTA update from GitHub now working
* Bugfix: Blue LED was not blinking
* Bugfix: Device name not initialized to default on startup

### Known Issues

* Still testing... Future updates MAY include breaking changes requiring a flash erase and re-upload.

## v3.0.0 (2024-11-30)

### What's Changed

* New release for Ratgdo32 - DISCO

### Known Issues

* THIS IS PRE-RELEASE FIRMWARE for testing purposes. Future updates MAY include breaking changes requiring a flash erase and re-upload.

