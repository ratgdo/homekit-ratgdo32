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
 * Thomas Hagan...     https://github.com/tlhagan
 * Brandon Matthews... https://github.com/thenewwazoo
 * Jonathan Stroud...  https://github.com/jgstroud
 *
 */

// C/C++ language includes
// none

// Arduino includes
#include <Ticker.h>

// RATGDO project includes
#include "SoftwareSerial.h"
#include "ratgdo.h"
#include "homekit.h"
#include "Reader.h"
#include "secplus2.h"
#include "utilities.h"
#include "comms.h"
#include "config.h"
#include "led.h"
#include "drycontact.h"

static const char *TAG = "ratgdo-comms";

static bool comms_setup_done = false;

/********************************** LOCAL STORAGE *****************************************/

struct PacketAction
{
    Packet pkt;
    bool inc_counter;
    uint32_t delay;
};

QueueHandle_t pkt_q;
SoftwareSerial sw_serial;

extern struct GarageDoor garage_door;
extern bool status_done;

uint32_t doorControlType = 0;

// For Time-to-close control
Ticker TTCtimer = Ticker();
uint8_t TTCcountdown = 0;
bool TTCwasLightOn = false;
void (*TTC_Action)(void) = NULL;

struct ForceRecover force_recover;
#define force_recover_delay 3

/******************************* OBSTRUCTION SENSOR *********************************/

struct obstruction_sensor_t
{
    unsigned int low_count = 0; // count obstruction low pulses
    uint64_t last_asleep = 0;   // count time between high pulses from the obst ISR
} obstruction_sensor;

void IRAM_ATTR isr_obstruction()
{
    obstruction_sensor.low_count++;
}

/******************************* SECURITY 2.0 *********************************/

SecPlus2Reader reader;
uint32_t id_code = 0;
uint32_t rolling_code = 0;
uint32_t last_saved_code = 0;
#define MAX_CODES_WITHOUT_FLASH_WRITE 10

/******************************* SECURITY 1.0 *********************************/

static const uint8_t RX_LENGTH = 2;
typedef uint8_t RxPacket[RX_LENGTH * 4];
uint64_t last_rx;
uint64_t last_tx;

#define MAX_COMMS_RETRY 10

bool wallplateBooting = false;
bool wallPanelDetected = false;
DoorState doorState = DoorState::Unknown;
uint8_t lightState;
uint8_t lockState;

// keep this here incase at somepoint its needed
// it is used for emulation of wall panel
// byte secplus1States[19] = {0x35,0x35,0x35,0x35,0x33,0x33,0x53,0x53,0x38,0x3A,0x3A,0x3A,0x39,0x38,0x3A, 0x38,0x3A,0x39,0x3A};
// this is what MY 889LM exhibited when powered up (release of all buttons, and then polls)
byte secplus1States[] = {0x35, 0x35, 0x33, 0x33, 0x38, 0x3A, 0x39};

// values for SECURITY+1.0 communication
enum secplus1Codes : uint8_t
{
    DoorButtonPress = 0x30,
    DoorButtonRelease = 0x31,
    LightButtonPress = 0x32,
    LightButtonRelease = 0x33,
    LockButtonPress = 0x34,
    LockButtonRelease = 0x35,

    Unkown_0x36 = 0x36,
    Unknown_0x37 = 0x37,

    DoorStatus = 0x38,
    ObstructionStatus = 0x39, // this is not proven
    LightLockStatus = 0x3A,
    Unknown = 0xFF
};

/*************************** FORWARD DECLARATIONS ******************************/

void sync();
bool process_PacketAction(PacketAction &pkt_ac);
void door_command(DoorAction action);
void send_get_status();
bool transmitSec1(byte toSend);
bool transmitSec2(PacketAction &pkt_ac);
void TTCdelayLoop();
void manual_recovery();
void obstruction_timer();

/****************************************************************************
 * Initialize communications with garage door.
 */
void setup_comms()
{
    // Create packet queue
    pkt_q = xQueueCreate(5, sizeof(PacketAction));

    if (doorControlType == 0)
        doorControlType = userConfig->getGDOSecurityType();

    if (doorControlType == 1)
    {
        RINFO(TAG, "=== Setting up comms for SECURITY+1.0 protocol");

        sw_serial.begin(1200, SWSERIAL_8E1, UART_RX_PIN, UART_TX_PIN, true);

        wallPanelDetected = false;
        wallplateBooting = false;
        doorState = DoorState::Unknown;
        lightState = 2;
        lockState = 2;
    }
    else if (doorControlType == 2)
    {
        RINFO(TAG, "=== Setting up comms for SECURITY+2.0 protocol");

        sw_serial.begin(9600, SWSERIAL_8N1, UART_RX_PIN, UART_TX_PIN, true);
        sw_serial.enableIntTx(false);
        sw_serial.enableAutoBaud(true); // found in ratgdo/espsoftwareserial branch autobaud

        // read from flash, default of 0 if file not exist
        id_code = nvRam->read(nvram_id_code);
        if (!id_code)
        {
            RINFO(TAG, "id code not found");
            id_code = (random(0x1, 0xFFF) << 12) | 0x539;
            nvRam->write(nvram_id_code, id_code);
        }
        RINFO(TAG, "id code %lu (0x%02lX)", id_code, id_code);

        // read from flash, default of 0 if file not exist
        rolling_code = nvRam->read(nvram_rolling, 0);
        // last saved rolling code may be behind what the GDO thinks, so bump it up so that it will
        // always be ahead of what the GDO thinks it should be, and save it.
        rolling_code = (rolling_code != 0) ? rolling_code + MAX_CODES_WITHOUT_FLASH_WRITE : 0;
        save_rolling_code();
        RINFO(TAG, "rolling code %lu (0x%02X)", rolling_code, rolling_code);
        sync();

        // Get the initial state of the door
        if (!digitalRead(UART_RX_PIN))
        {
            send_get_status();
        }
        force_recover.push_count = 0;
    }
    else
    {
        RINFO(TAG, "=== Setting up comms for dry contact protocol");
        pinMode(UART_TX_PIN, OUTPUT);
    }

    /* pin-based obstruction detection
    // FALLING from https://github.com/ratgdo/esphome-ratgdo/blob/e248c705c5342e99201de272cb3e6dc0607a0f84/components/ratgdo/ratgdo.cpp#L54C14-L54C14
     */
    RINFO(TAG, "Initialize for obstruction detection");
    pinMode(INPUT_OBST_PIN, INPUT);
    pinMode(STATUS_OBST_PIN, OUTPUT);
    attachInterrupt(INPUT_OBST_PIN, isr_obstruction, FALLING);

    comms_setup_done = true;
}

/****************************************************************************
 * Helper functions for GDO communications.
 */
void save_rolling_code()
{
    nvRam->write(nvram_rolling, rolling_code);
    last_saved_code = rolling_code;
}

void reset_door()
{
    rolling_code = 0; // because sync_and_reboot writes this.
    nvRam->erase(nvram_rolling);
    nvRam->erase(nvram_id_code);
    nvRam->erase(nvram_has_motion);
}

/****************************************************************************
 * Sec+ 1.0 loop functions.
 */
void wallPlate_Emulation()
{

    if (wallPanelDetected)
        return;

    uint64_t currentMillis = millis64();
    static uint64_t lastRequestMillis = 0;
    static bool emulateWallPanel = false;
    static uint64_t serialDetected = 0;
    static uint8_t stateIndex = 0;

    if (!serialDetected)
    {
        serialDetected = currentMillis;
        return;
    }

    // wait up to 15 seconds to look for an existing wallplate or it could be booting, so need to wait
    if (currentMillis - serialDetected < 15000 || wallplateBooting == true)
    {
        if (currentMillis - lastRequestMillis > 1000)
        {
            RINFO(TAG, "Looking for security+ 1.0 DIGITAL wall panel...");
            lastRequestMillis = currentMillis;
        }

        if (!wallPanelDetected && (doorState != DoorState::Unknown || lightState != 2))
        {
            wallPanelDetected = true;
            wallplateBooting = false;
            RINFO(TAG, "DIGITAL Wall panel detected.");
            return;
        }
    }
    else
    {
        if (!emulateWallPanel && !wallPanelDetected)
        {
            emulateWallPanel = true;
            RINFO(TAG, "No DIGITAL wall panel detected. Switching to emulation mode.");
        }

        // transmit every 250ms
        if (emulateWallPanel && (currentMillis - lastRequestMillis) > 250)
        {
            lastRequestMillis = currentMillis;

            byte secplus1ToSend = byte(secplus1States[stateIndex]);

            // send through queue
            PacketData data;
            data.type = PacketDataType::Status;
            data.value.cmd = secplus1ToSend;
            Packet pkt = Packet(PacketCommand::GetStatus, data, id_code);
            PacketAction pkt_ac = {pkt, true, 20}; // 20ms delay for SECURITY1.0 (which is minimum delay)
            if (xQueueSendToBack(pkt_q, &pkt_ac, 0) == errQUEUE_FULL)
            {
                RERROR(TAG, "packet queue full");
            }

            // send direct
            // transmitSec1(secplus1ToSend);

            stateIndex++;
            if (stateIndex == sizeof(secplus1States))
            {
                stateIndex = sizeof(secplus1States) - 3;
            }
        }
    }
}

void comms_loop_sec1()
{
    static bool reading_msg = false;
    static uint16_t byte_count = 0;
    static RxPacket rx_packet;
    bool gotMessage = false;

    if (sw_serial.available())
    {
        uint8_t ser_byte = sw_serial.read();
        last_rx = millis64();

        if (!reading_msg)
        {
            // valid?
            if (ser_byte >= 0x30 && ser_byte <= 0x3A)
            {
                byte_count = 0;
                rx_packet[byte_count++] = ser_byte;
                reading_msg = true;
            }
            // is it single byte command?
            // really all commands are single byte
            // is it a button push or release? (FROM WALL PANEL)
            if (ser_byte >= 0x30 && ser_byte <= 0x37)
            {
                rx_packet[1] = 0;
                reading_msg = false;
                byte_count = 0;

                gotMessage = true;
            }
        }
        else
        {
            // save next byte
            rx_packet[byte_count++] = ser_byte;

            if (byte_count == RX_LENGTH)
            {
                reading_msg = false;
                byte_count = 0;

                gotMessage = true;
            }

            if (gotMessage == false && (millis64() - last_rx) > 100)
            {
                RINFO(TAG, "RX message timeout");
                // if we have a partial packet and it's been over 100ms since last byte was read,
                // the rest is not coming (a full packet should be received in ~20ms),
                // discard it so we can read the following packet correctly
                reading_msg = false;
                byte_count = 0;
            }
        }
    }

    // got data?
    if (gotMessage)
    {
        gotMessage = false;

        // get kvp
        // button press/release have no val, just a single byte
        uint8_t key = rx_packet[0];
        uint8_t val = rx_packet[1];

        if (key == secplus1Codes::DoorButtonPress)
        {
            RINFO(TAG, "0x30 RX (door press)");
            manual_recovery();
            if (motionTriggers.bit.doorKey)
            {
                garage_door.motion_timer = millis64() + MOTION_TIMER_DURATION;
                garage_door.motion = true;
                notify_homekit_motion();
            }
        }
        // wall panel is sending out 0x31 (Door Button Release) when it starts up
        // but also on release of door button
        else if (key == secplus1Codes::DoorButtonRelease)
        {
            RINFO(TAG, "0x31 RX (door release)");

            // Possible power up of 889LM
            if ((DoorState)doorState == DoorState::Unknown)
            {
                wallplateBooting = true;
            }
        }
        else if (key == secplus1Codes::LightButtonPress)
        {
            RINFO(TAG, "0x32 RX (light press)");
            manual_recovery();
        }
        else if (key == secplus1Codes::LightButtonRelease)
        {
            RINFO(TAG, "0x33 RX (light release)");
        }

        // 2 byte status messages (0x38 - 0x3A)
        // its the byte sent out by the wallplate + the byte transmitted by the opener
        if (key == secplus1Codes::DoorStatus || key == secplus1Codes::ObstructionStatus || key == secplus1Codes::LightLockStatus)
        {

            // RINFO(TAG, "SEC1 STATUS MSG: %X%02X",key,val);

            switch (key)
            {
            // door status
            case secplus1Codes::DoorStatus:

                // RINFO(TAG, "0x38 MSG: %02X",val);

                // 0x5X = stopped
                // 0x0X = moving
                // best attempt to trap invalid values (due to collisions)
                if (((val & 0xF0) != 0x00) && ((val & 0xF0) != 0x50) && ((val & 0xF0) != 0xB0))
                {
                    RINFO(TAG, "0x38 val upper nible not 0x0 or 0x5 or 0xB: %02X", val);
                    break;
                }

                val = (val & 0x7);
                // 000 0x0 stopped
                // 001 0x1 opening
                // 010 0x2 open
                // 100 0x4 closing
                // 101 0x5 closed
                // 110 0x6 stopped

                // sec+1 doors sometimes report wrong door status
                // require two sequential matching door states
                // I have not seen this to be the case on my unit (MJS)
                static uint8_t prevDoor;
                if (prevDoor != val)
                {
                    prevDoor = val;
                    break;
                }

                switch (val)
                {
                case 0x00:
                    doorState = DoorState::Stopped;
                    break;
                case 0x01:
                    doorState = DoorState::Opening;
                    break;
                case 0x02:
                    doorState = DoorState::Open;
                    break;
                // no 0x03 known
                case 0x04:
                    doorState = DoorState::Closing;
                    break;
                case 0x05:
                    doorState = DoorState::Closed;
                    break;
                case 0x06:
                    doorState = DoorState::Stopped;
                    break;
                default:
                    doorState = DoorState::Unknown;
                    break;
                }

                // RINFO(TAG, "doorstate: %d", doorState);

                switch (doorState)
                {
                case DoorState::Open:
                    garage_door.current_state = CURR_OPEN;
                    garage_door.target_state = TGT_OPEN;
                    break;
                case DoorState::Closed:
                    garage_door.current_state = CURR_CLOSED;
                    garage_door.target_state = TGT_CLOSED;
                    break;
                case DoorState::Stopped:
                    garage_door.current_state = CURR_STOPPED;
                    garage_door.target_state = TGT_OPEN;
                    break;
                case DoorState::Opening:
                    garage_door.current_state = CURR_OPENING;
                    garage_door.target_state = TGT_OPEN;
                    break;
                case DoorState::Closing:
                    garage_door.current_state = CURR_CLOSING;
                    garage_door.target_state = TGT_CLOSED;
                    break;
                case DoorState::Unknown:
                    RERROR(TAG, "Got door state unknown");
                    break;
                }

                if ((garage_door.current_state == CURR_CLOSING) && (TTCcountdown > 0))
                {
                    // We are in a time-to-close delay timeout, cancel the timeout
                    RINFO(TAG, "Canceling time-to-close delay timer");
                    TTCtimer.detach();
                    TTCcountdown = 0;
                }

                if (!garage_door.active)
                {
                    RINFO(TAG, "activating door");
                    garage_door.active = true;
                    if (garage_door.current_state == CURR_OPENING || garage_door.current_state == CURR_OPEN)
                    {
                        garage_door.target_state = TGT_OPEN;
                    }
                    else
                    {
                        garage_door.target_state = TGT_CLOSED;
                    }
                }

                static GarageDoorCurrentState gd_currentstate;
                if (garage_door.current_state != gd_currentstate)
                {
                    gd_currentstate = garage_door.current_state;

                    const char *l = "unknown door state";
                    switch (gd_currentstate)
                    {
                    case GarageDoorCurrentState::CURR_STOPPED:
                        l = "Stopped";
                        break;
                    case GarageDoorCurrentState::CURR_OPEN:
                        l = "Open";
                        break;
                    case GarageDoorCurrentState::CURR_OPENING:
                        l = "Opening";
                        break;
                    case GarageDoorCurrentState::CURR_CLOSED:
                        l = "Closed";
                        break;
                    case GarageDoorCurrentState::CURR_CLOSING:
                        l = "Closing";
                        break;
                    }
                    RINFO(TAG, "status DOOR: %s", l);

                    notify_homekit_current_door_state_change();
                }

                static GarageDoorTargetState gd_TargetState;
                if (garage_door.target_state != gd_TargetState)
                {
                    gd_TargetState = garage_door.target_state;
                    notify_homekit_target_door_state_change();
                }

                break;

            // obstruction states (not confirmed)
            case secplus1Codes::ObstructionStatus:
                // currently not using
                break;

            // light & lock
            case secplus1Codes::LightLockStatus:

                // RINFO(TAG, "0x3A MSG: %X%02X",key,val);

                // upper nibble must be 5
                if ((val & 0xF0) != 0x50)
                {
                    RINFO(TAG, "0x3A val upper nible not 5: %02X", val);
                    break;
                }

                lightState = bitRead(val, 2);
                lockState = !bitRead(val, 3);

                // light status
                static uint8_t lastLightState = 0xff;
                // light state change?
                if (lightState != lastLightState)
                {
                    RINFO(TAG, "status LIGHT: %s", lightState ? "On" : "Off");
                    lastLightState = lightState;

                    garage_door.light = (bool)lightState;
                    notify_homekit_light();
                    if (motionTriggers.bit.lightKey)
                    {
                        garage_door.motion_timer = millis64() + MOTION_TIMER_DURATION;
                        garage_door.motion = true;
                        notify_homekit_motion();
                    }
                }

                // lock status
                static uint8_t lastLockState = 0xff;
                // lock state change?
                if (lockState != lastLockState)
                {
                    RINFO(TAG, "status LOCK: %s", lockState ? "Secured" : "Unsecured");
                    lastLockState = lockState;

                    if (lockState)
                    {
                        garage_door.current_lock = CURR_LOCKED;
                        garage_door.target_lock = TGT_LOCKED;
                    }
                    else
                    {
                        garage_door.current_lock = CURR_UNLOCKED;
                        garage_door.target_lock = TGT_UNLOCKED;
                    }
                    notify_homekit_target_lock();
                    notify_homekit_current_lock();
                    if (motionTriggers.bit.lockKey)
                    {
                        garage_door.motion_timer = millis64() + MOTION_TIMER_DURATION;
                        garage_door.motion = true;
                        notify_homekit_motion();
                    }
                }

                break;
            }
        }
    }

    //
    // PROCESS TRANSMIT QUEUE
    //
    PacketAction pkt_ac;
    static uint32_t cmdDelay = 0;
    uint64_t now;
    bool okToSend = false;
    static uint16_t retryCount = 0;

    if (uxQueueMessagesWaiting(pkt_q) > 0)
    {
        now = millis64();

        // if there is no wall panel, no need to check 200ms since last rx
        // (yes some duped code here, but its clearer)
        if (!wallPanelDetected)
        {
            // no wall panel
            okToSend = (now - last_rx > 20);        // after 20ms since last rx
            okToSend &= (now - last_tx > 20);       // after 20ms since last tx
            okToSend &= (now - last_tx > cmdDelay); // after any command delays
        }
        else
        {
            // digital wall panel
            okToSend = (now - last_rx > 20);        // after 20ms since last rx
            okToSend &= (now - last_rx < 200);      // before 200ms since last rx
            okToSend &= (now - last_tx > 20);       // after 20ms since last tx
            okToSend &= (now - last_tx > cmdDelay); // after any command delays
        }

        // OK to send based on above rules
        if (okToSend)
        {

            if (uxQueueMessagesWaiting(pkt_q) > 0)
            {
                ESP_LOGD(TAG, "packet ready for tx");
                xQueueReceive(pkt_q, &pkt_ac, 0); // ignore errors
                if (process_PacketAction(pkt_ac))
                {
                    // get next delay "between" transmits
                    cmdDelay = pkt_ac.delay;
                }
                else
                {
                    cmdDelay = 0;
                    if (retryCount++ < MAX_COMMS_RETRY)
                    {
                        RERROR(TAG, "transmit failed, will retry");
                        xQueueSendToFront(pkt_q, &pkt_ac, 0); // ignore errors
                    }
                    else
                    {
                        RERROR(TAG, "transmit failed, exceeded max retry, aborting");
                        retryCount = 0;
                    }
                }
            }
        }
    }

    // check for wall panel and provide emulator
    wallPlate_Emulation();
}

/****************************************************************************
 * Sec+ 2.0 loop functions.
 */
void comms_loop_sec2()
{
    static uint16_t retryCount = 0;

    // no incoming data, check if we have command queued
    if (!sw_serial.available())
    {
        PacketAction pkt_ac;

        if (uxQueueMessagesWaiting(pkt_q) > 0)
        {
            ESP_LOGD(TAG, "packet ready for tx");
            xQueueReceive(pkt_q, &pkt_ac, 0); // ignore errors
            if (!process_PacketAction(pkt_ac))
            {

                if (retryCount++ < MAX_COMMS_RETRY)
                {
                    RERROR(TAG, "transmit failed, will retry");
                    xQueueSendToFront(pkt_q, &pkt_ac, 0); // ignore errors
                }
                else
                {
                    RERROR(TAG, "transmit failed, exceeded max retry, aborting");
                    retryCount = 0;
                }
            }
        }
    }
    else
    {
        // spin on receiving data until the whole packet has arrived
        uint8_t ser_data = sw_serial.read();
        if (reader.push_byte(ser_data))
        {
            Packet pkt = Packet(reader.fetch_buf());
            pkt.print();

            switch (pkt.m_pkt_cmd)
            {
            case PacketCommand::Status:
            {
                GarageDoorCurrentState current_state = garage_door.current_state;
                GarageDoorTargetState target_state = garage_door.target_state;
                switch (pkt.m_data.value.status.door)
                {
                case DoorState::Open:
                    current_state = CURR_OPEN;
                    target_state = TGT_OPEN;
                    break;
                case DoorState::Closed:
                    current_state = CURR_CLOSED;
                    target_state = TGT_CLOSED;
                    break;
                case DoorState::Stopped:
                    current_state = CURR_STOPPED;
                    target_state = TGT_OPEN;
                    break;
                case DoorState::Opening:
                    current_state = CURR_OPENING;
                    target_state = TGT_OPEN;
                    break;
                case DoorState::Closing:
                    current_state = CURR_CLOSING;
                    target_state = TGT_CLOSED;
                    break;
                case DoorState::Unknown:
                    RERROR(TAG, "Got door state unknown");
                    break;
                }

                if ((current_state == CURR_CLOSING) && (TTCcountdown > 0))
                {
                    // We are in a time-to-close delay timeout, cancel the timeout
                    RINFO(TAG, "Canceling time-to-close delay timer");
                    TTCtimer.detach();
                    TTCcountdown = 0;
                }

                if (!garage_door.active)
                {
                    RINFO(TAG, "activating door");
                    garage_door.active = true;
                    if (current_state == CURR_OPENING || current_state == CURR_OPEN)
                    {
                        target_state = TGT_OPEN;
                    }
                    else
                    {
                        target_state = TGT_CLOSED;
                    }
                }

                RINFO(TAG, "tgt %d curr %d", target_state, current_state);

                if ((target_state != garage_door.target_state) ||
                    (current_state != garage_door.current_state))
                {
                    garage_door.target_state = target_state;
                    garage_door.current_state = current_state;

                    notify_homekit_current_door_state_change();
                    notify_homekit_target_door_state_change();
                }

                if (pkt.m_data.value.status.light != garage_door.light)
                {
                    RINFO(TAG, "Light Status %s", pkt.m_data.value.status.light ? "On" : "Off");
                    garage_door.light = pkt.m_data.value.status.light;
                    notify_homekit_light();
                }

                LockCurrentState current_lock;
                LockTargetState target_lock;
                if (pkt.m_data.value.status.lock)
                {
                    current_lock = CURR_LOCKED;
                    target_lock = TGT_LOCKED;
                }
                else
                {
                    current_lock = CURR_UNLOCKED;
                    target_lock = TGT_UNLOCKED;
                }
                if (current_lock != garage_door.current_lock)
                {
                    garage_door.target_lock = target_lock;
                    garage_door.current_lock = current_lock;
                    notify_homekit_target_lock();
                    notify_homekit_current_lock();
                }

                status_done = true;
                break;
            }

            case PacketCommand::Lock:
            {
                LockTargetState lock = garage_door.target_lock;
                switch (pkt.m_data.value.lock.lock)
                {
                case LockState::Off:
                    lock = TGT_UNLOCKED;
                    break;
                case LockState::On:
                    lock = TGT_LOCKED;
                    break;
                case LockState::Toggle:
                    if (lock == TGT_LOCKED)
                    {
                        lock = TGT_UNLOCKED;
                    }
                    else
                    {
                        lock = TGT_LOCKED;
                    }
                    break;
                }
                if (lock != garage_door.target_lock)
                {
                    RINFO(TAG, "Lock Cmd %d", lock);
                    garage_door.target_lock = lock;
                    notify_homekit_target_lock();
                    if (motionTriggers.bit.lockKey)
                    {
                        garage_door.motion_timer = millis64() + MOTION_TIMER_DURATION;
                        garage_door.motion = true;
                        notify_homekit_motion();
                    }
                }
                // Send a get status to make sure we are in sync
                send_get_status();
                break;
            }

            case PacketCommand::Light:
            {
                bool l = garage_door.light;
                manual_recovery();
                switch (pkt.m_data.value.light.light)
                {
                case LightState::Off:
                    l = false;
                    break;
                case LightState::On:
                    l = true;
                    break;
                case LightState::Toggle:
                case LightState::Toggle2:
                    l = !garage_door.light;
                    break;
                }
                if (l != garage_door.light)
                {
                    RINFO(TAG, "Light Cmd %s", l ? "On" : "Off");
                    garage_door.light = l;
                    notify_homekit_light();
                    if (motionTriggers.bit.lightKey)
                    {
                        garage_door.motion_timer = millis64() + MOTION_TIMER_DURATION;
                        garage_door.motion = true;
                        notify_homekit_motion();
                    }
                }
                // Send a get status to make sure we are in sync
                // Should really only need to do this on a toggle,
                // But safer to do it always
                send_get_status();
                break;
            }

            case PacketCommand::Motion:
            {
                RINFO(TAG, "Motion Detected");
                // We got a motion message, so we know we have a motion sensor
                // If it's not yet enabled, add the service
                if (!garage_door.has_motion_sensor)
                {
                    RINFO(TAG, "Detected new Motion Sensor. Enabling Service");
                    garage_door.has_motion_sensor = true;
                    motionTriggers.bit.motion = 1;
                    userConfig->set(cfg_motionTriggers, motionTriggers.asInt);
                    enable_service_homekit_motion();
                }

                /* When we get the motion detect message, notify HomeKit. Motion sensor
                    will continue to send motion messages every 5s until motion stops.
                    set a timer for 5 seconds to disable motion after the last message */
                garage_door.motion_timer = millis64() + MOTION_TIMER_DURATION;
                if (!garage_door.motion)
                {
                    garage_door.motion = true;
                    notify_homekit_motion();
                }
                // Update status because things like light may have changed states
                send_get_status();
                break;
            }

            case PacketCommand::DoorAction:
            {
                RINFO(TAG, "Door Action");
                if (pkt.m_data.value.door_action.pressed)
                {
                    manual_recovery();
                }
                if (pkt.m_data.value.door_action.pressed && motionTriggers.bit.doorKey)
                {
                    garage_door.motion_timer = millis64() + MOTION_TIMER_DURATION;
                    garage_door.motion = true;
                    notify_homekit_motion();
                }
                break;
            }

            default:
                RINFO(TAG, "Support for %s packet unimplemented. Ignoring.", PacketCommand::to_string(pkt.m_pkt_cmd));
                break;
            }
        }
    }

    // Save rolling code if we have exceeded max limit.
    if (rolling_code >= (last_saved_code + MAX_CODES_WITHOUT_FLASH_WRITE))
    {
        save_rolling_code();
    }
}

void comms_loop_drycontact()
{
    static DoorState previousDoorState = DoorState::Unknown;

    // Notify HomeKit when the door state changes
    if (doorState != previousDoorState)
    {
        switch (doorState)
        {
        case DoorState::Open:
            garage_door.current_state = GarageDoorCurrentState::CURR_OPEN;
            garage_door.target_state = GarageDoorTargetState::TGT_OPEN;
            break;
        case DoorState::Closed:
            garage_door.current_state = GarageDoorCurrentState::CURR_CLOSED;
            garage_door.target_state = GarageDoorTargetState::TGT_CLOSED;
            break;
        case DoorState::Opening:
            garage_door.current_state = GarageDoorCurrentState::CURR_OPENING;
            garage_door.target_state = GarageDoorTargetState::TGT_OPEN;
            break;
        case DoorState::Closing:
            garage_door.current_state = GarageDoorCurrentState::CURR_CLOSING;
            garage_door.target_state = GarageDoorTargetState::TGT_CLOSED;
            break;
        default:
            garage_door.current_state = GarageDoorCurrentState::CURR_STOPPED;
            break;
        }

        notify_homekit_current_door_state_change();
        notify_homekit_target_door_state_change();

        previousDoorState = doorState;

        // Log the state change for debugging
        RINFO(TAG, "Door state updated: Current: %d, Target: %d", garage_door.current_state, garage_door.target_state);
    }
}

void comms_loop()
{
    if (!comms_setup_done)
        return;

    if (doorControlType == 1)
        comms_loop_sec1();
    else if (doorControlType == 2)
        comms_loop_sec2();
    else
        comms_loop_drycontact();

    // Motion Clear Timer
    if (garage_door.motion && (millis64() > garage_door.motion_timer))
    {
        RINFO(TAG, "Motion Cleared");
        garage_door.motion = false;
        notify_homekit_motion();
    }

    // Service the Obstruction Timer
    obstruction_timer();
}

/**************************** CONTROLLER CODE *******************************
 * SECURITY+1.0
 */
bool transmitSec1(byte toSend)
{

    // safety
    if (digitalRead(UART_RX_PIN) || sw_serial.available())
    {
        return false;
    }

    // sending a poll?
    bool poll_cmd = (toSend == 0x38) || (toSend == 0x39) || (toSend == 0x3A);
    // if not a poll command (and polls only with wall panel emulation),
    // disable disable rx (allows for cleaner tx, and no echo)
    if (!poll_cmd)
    {
        // Use LED to signal activity
        led.flash(FLASH_MS);
        sw_serial.enableRx(false);
    }

    sw_serial.write(toSend);
    last_tx = millis64();

    // RINFO(TAG, "SEC1 SEND BYTE: %02X",toSend);

    // re-enable rx
    if (!poll_cmd)
    {
        sw_serial.enableRx(true);
    }

    return true;
}

/**************************** CONTROLLER CODE *******************************
 * SECURITY+2.0
 */
bool transmitSec2(PacketAction &pkt_ac)
{

    // inverted logic, so this pulls the bus low to assert it
    digitalWrite(UART_TX_PIN, HIGH);
    delayMicroseconds(1300);
    digitalWrite(UART_TX_PIN, LOW);
    delayMicroseconds(130);

    // check to see if anyone else is continuing to assert the bus after we have released it
    if (digitalRead(UART_RX_PIN))
    {
        RINFO(TAG, "Collision detected, waiting to send packet");
        return false;
    }
    else
    {
        uint8_t buf[SECPLUS2_CODE_LEN];
        if (pkt_ac.pkt.encode(rolling_code, buf) != 0)
        {
            RERROR(TAG, "Could not encode packet");
            pkt_ac.pkt.print();
        }
        else
        {
            // Use LED to signal activity
            led.flash(FLASH_MS);
            sw_serial.write(buf, SECPLUS2_CODE_LEN);
            delayMicroseconds(100);
        }

        if (pkt_ac.inc_counter)
        {
            rolling_code = (rolling_code + 1) & 0xfffffff;
        }
    }

    return true;
}

bool process_PacketAction(PacketAction &pkt_ac)
{

    bool success = false;

    if (doorControlType == 1)
    {
        // check which action
        switch (pkt_ac.pkt.m_data.type)
        {
        // using this type for emulation of wall panel
        case PacketDataType::Status:
        {
            // 0x38 || 0x39 || 0x3A
            if (pkt_ac.pkt.m_data.value.cmd)
            {
                success = transmitSec1(pkt_ac.pkt.m_data.value.cmd);
                if (success)
                {
                    last_tx = millis64();
                    // RINFO(TAG, "sending 0x%02X query", pkt_ac.pkt.m_data.value.cmd);
                }
            }
            break;
        }
        case PacketDataType::DoorAction:
        {
            if (pkt_ac.pkt.m_data.value.door_action.pressed == true)
            {
                success = transmitSec1(secplus1Codes::DoorButtonPress);
                if (success)
                {
                    last_tx = millis64();
                    RINFO(TAG, "sending DOOR button press");
                }
            }
            else
            {
                success = transmitSec1(secplus1Codes::DoorButtonRelease);
                if (success)
                {
                    last_tx = millis64();
                    RINFO(TAG, "sending DOOR button release");
                }
            }
            break;
        }

        case PacketDataType::Light:
        {
            if (pkt_ac.pkt.m_data.value.light.pressed == true)
            {
                success = transmitSec1(secplus1Codes::LightButtonPress);
                if (success)
                {
                    last_tx = millis64();
                    RINFO(TAG, "sending LIGHT button press");
                }
            }
            else
            {
                success = transmitSec1(secplus1Codes::LightButtonRelease);
                if (success)
                {
                    last_tx = millis64();
                    RINFO(TAG, "Sending LIGHT button release");
                }
            }
            break;
        }

        case PacketDataType::Lock:
        {
            if (pkt_ac.pkt.m_data.value.lock.pressed == true)
            {
                success = transmitSec1(secplus1Codes::LockButtonPress);
                if (success)
                {
                    last_tx = millis64();
                    RINFO(TAG, "sending LOCK button press");
                }
            }
            else
            {
                success = transmitSec1(secplus1Codes::LockButtonRelease);
                if (success)
                {
                    last_tx = millis64();
                    RINFO(TAG, "sending LOCK button release");
                }
            }
            break;
        }

        default:
        {
            RINFO(TAG, "UNHANDLED pkt_ac.pkt.m_data.type=%d", pkt_ac.pkt.m_data.type);
            break;
        }
        }
    }
    else
    {
        success = transmitSec2(pkt_ac);
    }

    return success;
}

void sync()
{
    // only for SECURITY2.0
    // for exposition about this process, see docs/syncing.md
    RINFO(TAG, "Syncing rolling code counter after reboot...");
    PacketData d;
    d.type = PacketDataType::NoData;
    d.value.no_data = NoData();
    Packet pkt = Packet(PacketCommand::GetOpenings, d, id_code);
    PacketAction pkt_ac = {pkt, true};
    process_PacketAction(pkt_ac);
    delay(100);
    pkt = Packet(PacketCommand::GetStatus, d, id_code);
    pkt_ac.pkt = pkt;
    process_PacketAction(pkt_ac);
}

void door_command(DoorAction action)
{
    if (doorControlType != 3)
    {
        // SECURITY1.0/2.0 commands
        PacketData data;
        data.type = PacketDataType::DoorAction;
        data.value.door_action.action = action;
        data.value.door_action.pressed = true;
        data.value.door_action.id = 1;

        Packet pkt = Packet(PacketCommand::DoorAction, data, id_code);
        PacketAction pkt_ac = {pkt, false, 250}; // 250ms delay for SECURITY1.0

        if (xQueueSendToBack(pkt_q, &pkt_ac, 0) == errQUEUE_FULL)
        {
            RERROR(TAG, "packet queue full, dropping door command pressed pkt");
        }

        // do button release
        pkt_ac.pkt.m_data.value.door_action.pressed = false;
        pkt_ac.inc_counter = true;
        pkt_ac.delay = 40; // 40ms delay for SECURITY1.0

        if (xQueueSendToBack(pkt_q, &pkt_ac, 0) == errQUEUE_FULL)
        {
            RERROR(TAG, "packet queue full, dropping door command release pkt");
        }
        // when observing wall panel 2 releases happen, so we do the same
        if (doorControlType == 1)
        {
            if (xQueueSendToBack(pkt_q, &pkt_ac, 0) == errQUEUE_FULL)
            {
                RERROR(TAG, "packet queue full, dropping door command release pkt");
            }
        }

        send_get_status();
    }
    else
    {
        // Dry contact commands (only toggle functionality, open/close/toggle/stop -> toggle)
        // Toggle signal
        digitalWrite(UART_TX_PIN, HIGH);
        delay(500);
        digitalWrite(UART_TX_PIN, LOW);
    }
}

void door_command_close()
{
    door_command(DoorAction::Close);
}

bool open_door()
{
    // return value: true = set state to OPENING, else leave state unchanged
    if (TTCcountdown > 0)
    {
        // We are in a time-to-close delay timeout.
        // Effect of open is to cancel the timeout (leaving door open)
        RINFO(TAG, "Canceling time-to-close delay timer");
        TTCtimer.detach();
        TTCcountdown = 0;
        // Reset light to state it was at before delay start.
        set_light(TTCwasLightOn);
        return false;
    }

    // safety
    if (garage_door.current_state == GarageDoorCurrentState::CURR_OPEN)
    {
        RINFO(TAG, "Door already open; ignored request");
        return false;
    }

    if (garage_door.current_state == GarageDoorCurrentState::CURR_CLOSING)
    {
        RINFO(TAG, "Door is closing; do stop");
        door_command(DoorAction::Stop);
        return false;
    }
    RINFO(TAG, "Opening door");
    door_command(DoorAction::Open);
    return true;
}

void TTCdelayLoop()
{
    if (--TTCcountdown > 0)
    {
        if (garage_door.light)
        {
            // play alert beep every other loop
            tone(BEEPER_PIN, 1300, 500);
        }
        // If light is on, turn it off.  If off, turn it on.
        set_light(!garage_door.light);
    }
    else
    {
        // End of delay period
        tone(BEEPER_PIN, 2000, 500);
        TTCtimer.detach();
        if (TTC_Action)
            (*TTC_Action)();
    }
    return;
}

bool close_door()
{
    // return value: true = set state to CLOSING, else leave state unchanged
    if (garage_door.current_state == GarageDoorCurrentState::CURR_CLOSED)
    {
        RINFO(TAG, "Door already closed; ignored request");
        return false;
    }

    if (garage_door.current_state == GarageDoorCurrentState::CURR_OPENING)
    {
        RINFO(TAG, "Door already opening; do stop");
        door_command(DoorAction::Stop);
        return false;
    }

    if (userConfig->getTTCseconds() == 0)
    {
        RINFO(TAG, "Closing door");
        door_command(DoorAction::Close);
    }
    else
    {
        if (TTCcountdown > 0)
        {
            // We are in a time-to-close delay timeout.
            // Effect of second click is to cancel the timeout and close immediately
            RINFO(TAG, "Canceling time-to-close delay timer");
            TTCtimer.detach();
            TTCcountdown = 0;
            RINFO(TAG, "Closing door");
            door_command(DoorAction::Close);
        }
        else
        {
            RINFO(TAG, "Delay door close by %d seconds", userConfig->getTTCseconds());
            // Call delay loop every 0.5 seconds to flash light.
            TTCcountdown = userConfig->getTTCseconds() * 2;
            // Remember whether light was on or off
            TTCwasLightOn = garage_door.light;
            TTC_Action = &door_command_close;
            TTCtimer.attach_ms(500, TTCdelayLoop);
        }
    }
    return true;
}

void send_get_status()
{
    // only used with SECURITY2.0
    if (doorControlType == 2)
    {
        PacketData d;
        d.type = PacketDataType::NoData;
        d.value.no_data = NoData();
        Packet pkt = Packet(PacketCommand::GetStatus, d, id_code);
        PacketAction pkt_ac = {pkt, true};
        if (xQueueSendToBack(pkt_q, &pkt_ac, 0) == errQUEUE_FULL)
        {
            RERROR(TAG, "packet queue full, dropping get status pkt");
        }
    }
}

bool set_lock(bool value)
{
    // return value: true = lock state changed, else state unchanged
    PacketData data;
    data.type = PacketDataType::Lock;
    if (value)
    {
        if (garage_door.current_lock == LockCurrentState::CURR_LOCKED)
        {
            RINFO(TAG, "Lock already locked; ignored request");
            return false;
        }
        RINFO(TAG, "Locking Garage Door Remotes");
        data.value.lock.lock = LockState::On;
        garage_door.target_lock = TGT_LOCKED;
    }
    else
    {
        if (garage_door.current_lock == LockCurrentState::CURR_UNLOCKED)
        {
            RINFO(TAG, "Lock already unlocked; ignored request");
            return false;
        }
        RINFO(TAG, "Unlocking Garage Door Remotes");
        data.value.lock.lock = LockState::Off;
        garage_door.target_lock = TGT_UNLOCKED;
    }

    // SECURITY1.0
    if (doorControlType == 1)
    {
        // this emulates the "look" button press+release
        // - PRESS (0x34)
        // - DELAY 3000ms
        // - RELEASE (0x35)
        // - DELAY 40ms
        // - RELEASE (0x35)
        // - DELAY 40ms

        data.value.lock.pressed = true;
        Packet pkt = Packet(PacketCommand::Lock, data, id_code);
        PacketAction pkt_ac = {pkt, true, 3000}; // 3000ms delay for SECURITY1.0

        if (xQueueSendToBack(pkt_q, &pkt_ac, 0) == errQUEUE_FULL)
        {
            RERROR(TAG, "packet queue full, dropping lock pkt");
        }
        // button release
        pkt_ac.pkt.m_data.value.lock.pressed = false;
        pkt_ac.delay = 40; // 40ms delay for SECURITY1.0
                           // observed the wall plate does 2 releases, so we will too
        if (xQueueSendToBack(pkt_q, &pkt_ac, 0) == errQUEUE_FULL)
        {
            RERROR(TAG, "packet queue full, dropping lock pkt");
        }
        if (xQueueSendToBack(pkt_q, &pkt_ac, 0) == errQUEUE_FULL)
        {
            RERROR(TAG, "packet queue full, dropping lock pkt");
        }
    }
    // SECURITY2.0
    else
    {
        Packet pkt = Packet(PacketCommand::Lock, data, id_code);
        PacketAction pkt_ac = {pkt, true};

        if (xQueueSendToBack(pkt_q, &pkt_ac, 0) == errQUEUE_FULL)
        {
            RERROR(TAG, "packet queue full, dropping lock pkt");
        }
        send_get_status();
    }
    return true;
}

bool set_light(bool value)
{
    // return value: true = light state changed, else state unchanged
    PacketData data;
    data.type = PacketDataType::Light;
    if (value)
    {
        if (garage_door.light == true)
        {
            RINFO(TAG, "Light already On; ignored request");
            return false;
        }
        RINFO(TAG, "Turn on Garage Door Light");
        data.value.light.light = LightState::On;
    }
    else
    {
        if (garage_door.light == false)
        {
            RINFO(TAG, "Light already Off; ignored request");
            return false;
        }
        RINFO(TAG, "Turn off Garage Door Light");
        data.value.light.light = LightState::Off;
    }

    // SECURITY+1.0
    if (doorControlType == 1)
    {
        // this emulates the "light" button press+release
        // - PRESS (0x32)
        // - DELAY 250ms
        // - RELEASE (0x33)
        // - DELAY 40ms
        // - RELEASE (0x33)
        // - DELAY 40ms
        data.value.light.pressed = true;

        Packet pkt = Packet(PacketCommand::Light, data, id_code);
        PacketAction pkt_ac = {pkt, true, 250}; // 250ms delay for SECURITY1.0

        if (xQueueSendToBack(pkt_q, &pkt_ac, 0) == errQUEUE_FULL)
        {
            RERROR(TAG, "packet queue full, dropping light pkt");
        }
        // button release
        pkt_ac.pkt.m_data.value.light.pressed = false;
        pkt_ac.delay = 40; // 40ms delay for SECURITY1.0
                           // observed the wall plate does 2 releases, so we will too
        if (xQueueSendToBack(pkt_q, &pkt_ac, 0) == errQUEUE_FULL)
        {
            RERROR(TAG, "packet queue full, dropping light pkt");
        }
        if (xQueueSendToBack(pkt_q, &pkt_ac, 0) == errQUEUE_FULL)
        {
            RERROR(TAG, "packet queue full, dropping light pkt");
        }
    }
    // SECURITY+2.0
    else
    {
        Packet pkt = Packet(PacketCommand::Light, data, id_code);
        PacketAction pkt_ac = {pkt, true};

        if (xQueueSendToBack(pkt_q, &pkt_ac, 0) == errQUEUE_FULL)
        {
            RERROR(TAG, "packet queue full, dropping light pkt");
        }
        send_get_status();
    }
    return true;
}

void manual_recovery()
{
    // Increment counter every time button is pushed.  If we hit 5 in 3 seconds,
    // go to WiFi recovery mode
    if (force_recover.push_count++ == 0)
    {
        RINFO(TAG, "Push count start");
        force_recover.timeout = millis64() + 3000;
    }
    else if (millis64() > force_recover.timeout)
    {
        RINFO(TAG, "Push count reset");
        force_recover.push_count = 0;
    }
    RINFO(TAG, "Push count %d", force_recover.push_count);

    if (force_recover.push_count >= 5)
    {
        RINFO(TAG, "Request to boot into soft access point mode in %ds", force_recover_delay);
        userConfig->set(cfg_softAPmode, true);
        // Call delay loop every 0.5 seconds to flash light.
        TTCcountdown = force_recover_delay * 2;
        // Remember whether light was on or off
        TTCwasLightOn = garage_door.light;
        TTC_Action = &sync_and_restart;
        TTCtimer.attach_ms(500, TTCdelayLoop);
    }
}

/*************************** OBSTRUCTION DETECTION **************************
 *
 */
void obstruction_timer()
{
    uint64_t current_millis = millis64();
    static uint64_t last_millis = 0;

    // the obstruction sensor has 3 states: clear (HIGH with LOW pulse every 7ms), obstructed (HIGH), asleep (LOW)
    // the transitions between awake and asleep are tricky because the voltage drops slowly when falling asleep
    // and is high without pulses when waking up

    // If at least 3 low pulses are counted within 50ms, the door is awake, not obstructed and we don't have to check anything else

    const long CHECK_PERIOD = 50;
    const long PULSES_LOWER_LIMIT = 3;
    if (current_millis - last_millis > CHECK_PERIOD)
    {
        // check to see if we got more then PULSES_LOWER_LIMIT pulses
        if (obstruction_sensor.low_count > PULSES_LOWER_LIMIT)
        {
            // Only update if we are changing state
            if (garage_door.obstructed)
            {
                RINFO(TAG, "Obstruction Clear");
                garage_door.obstructed = false;
                notify_homekit_obstruction();
                digitalWrite(STATUS_OBST_PIN, garage_door.obstructed);
                if (motionTriggers.bit.obstruction)
                {
                    garage_door.motion = false;
                    notify_homekit_motion();
                }
            }
        }
        else if (obstruction_sensor.low_count == 0)
        {
            // if there have been no pulses the line is steady high or low
            if (!digitalRead(INPUT_OBST_PIN))
            {
                // asleep
                obstruction_sensor.last_asleep = current_millis;
            }
            else
            {
                // if the line is high and was last asleep more than 700ms ago, then there is an obstruction present
                if (current_millis - obstruction_sensor.last_asleep > 700)
                {
                    // Only update if we are changing state
                    if (!garage_door.obstructed)
                    {
                        RINFO(TAG, "Obstruction Detected");
                        garage_door.obstructed = true;
                        notify_homekit_obstruction();
                        digitalWrite(STATUS_OBST_PIN, garage_door.obstructed);
                        if (motionTriggers.bit.obstruction)
                        {
                            garage_door.motion = true;
                            notify_homekit_motion();
                        }
                    }
                }
            }
        }

        last_millis = current_millis;
        obstruction_sensor.low_count = 0;
    }
}