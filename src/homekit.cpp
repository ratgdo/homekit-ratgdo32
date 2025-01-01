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

// ESP system includes

// RATGDO project includes
#include "ratgdo.h"
#include "config.h"
#include "comms.h"
#include "utilities.h"
#include "homekit.h"
#include "web.h"
#include "softAP.h"
#include "led.h"
#include "vehicle.h"
#include "drycontact.h"

// Logger tag
static const char *TAG = "ratgdo-homekit";

static DEV_GarageDoor *door;
static DEV_Light *light;
static DEV_Motion *motion;
static DEV_Motion *arriving;
static DEV_Motion *departing;
static DEV_Occupancy *vehicle;
static DEV_Light *assistLaser;

static bool isPaired = false;
static bool rebooting = false;

/****************************************************************************
 * Callback functions, notify us of significant events
 */
void wifiCallbackAll(int count)
{
    if (rebooting)
        return;

    RINFO(TAG, "WiFi established, count: %d, IP: %s, Mask: %s, Gateway: %s, DNS: %s", count, WiFi.localIP().toString().c_str(),
          WiFi.subnetMask().toString().c_str(), WiFi.gatewayIP().toString().c_str(), WiFi.dnsIP().toString().c_str());
    userConfig->set(cfg_localIP, WiFi.localIP().toString().c_str());
    userConfig->set(cfg_gatewayIP, WiFi.gatewayIP().toString().c_str());
    userConfig->set(cfg_subnetMask, WiFi.subnetMask().toString().c_str());
    userConfig->set(cfg_nameserverIP, WiFi.dnsIP().toString().c_str());
    // With FiFi connected, we can now initialize the rest of our app.
    if (!softAPmode)
    {
        if (userConfig->getTimeZone() == "")
        {
            get_auto_timezone();
        }
        setup_vehicle();
        setup_comms();
        setup_drycontact();
        setup_web();
    }
    // beep on completing startup.
    tone(BEEPER_PIN, 2000, 500);
}

void statusCallback(HS_STATUS status)
{
    switch (status)
    {
    case HS_WIFI_NEEDED:
        RINFO(TAG, "Status: No WiFi Credentials, need to provision");
        break;
    case HS_WIFI_CONNECTING:
        RINFO(TAG, "Status: WiFi connecting, set hostname: %s", device_name_rfc952);
        // HomeSpan has notcalled WiFi.begin() yet, so we can set options here.
        WiFi.setSleep(WIFI_PS_NONE); // Improves performance, at cost of power consumption
        WiFi.hostname((const char *)device_name_rfc952);
        if (userConfig->getStaticIP())
        {
            IPAddress ip;
            IPAddress gw;
            IPAddress nm;
            IPAddress dns;
            if (ip.fromString(userConfig->getLocalIP().c_str()) &&
                gw.fromString(userConfig->getGatewayIP().c_str()) &&
                nm.fromString(userConfig->getSubnetMask().c_str()) &&
                dns.fromString(userConfig->getNameserverIP().c_str()))
            {
                RINFO(TAG, "Set static IP: %s, Mask: %s, Gateway: %s, DNS: %s",
                      ip.toString().c_str(), nm.toString().c_str(), gw.toString().c_str(), dns.toString().c_str());
                WiFi.config(ip, gw, nm, dns);
            }
            else
            {
                RINFO(TAG, "Failed to set static IP address, error parsing addresses");
            }
        }
        break;
    case HS_PAIRING_NEEDED:
        RINFO(TAG, "Status: Need to pair");
        isPaired = false;
        break;
    case HS_PAIRED:
        RINFO(TAG, "Status: Paired");
        isPaired = true;
        break;
    case HS_REBOOTING:
        rebooting = true;
        RINFO(TAG, "Status: Rebooting");
        break;
    case HS_FACTORY_RESET:
        RINFO(TAG, "Status: Factory Reset");
        break;
    default:
        RINFO(TAG, "HomeSpan Status: %s", homeSpan.statusString(status));
        break;
    }
}

#ifdef CONFIG_FREERTOS_USE_TRACE_FACILITY
void printTaskInfo(const char *buf)
{
    int count = uxTaskGetNumberOfTasks();
    TaskStatus_t *tasks = (TaskStatus_t *)pvPortMalloc(sizeof(TaskStatus_t) * count);
    if (tasks != NULL)
    {
        uxTaskGetSystemState(tasks, count, NULL);
        Serial.printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
        for (size_t i = 0; i < count; i++)
        {
            Serial.printf("%s\t%s\t%d\t\t%d\n", (char *)tasks[i].pcTaskName,
                          strlen((char *)tasks[i].pcTaskName) > 7 ? "" : "\t",
                          (int)tasks[i].uxBasePriority, (int)tasks[i].usStackHighWaterMark);
        }
        Serial.printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n\n");
    }
    vPortFree(tasks);
};
#endif

/****************************************************************************
 * Initialize HomeKit (with HomeSpan)
 */

void createMotionAccessories()
{

    // Exit if already setup
    if (motion)
        return;

    // Define the Motion Sensor accessory...
    new SpanAccessory(HOMEKIT_AID_MOTION);
    new DEV_Info("Motion");
    motion = new DEV_Motion("Motion");
}

void createVehicleAccessories()
{
    // Exit if already setup
    if (arriving)
        return;

    // Define Motion Sensor accessory for vehicle arriving
    new SpanAccessory(HOMEKIT_AID_ARRIVING);
    new DEV_Info("Arriving");
    arriving = new DEV_Motion("Arriving");

    // Define Motion Sensor accessory for vehicle departing
    new SpanAccessory(HOMEKIT_AID_DEPARTING);
    new DEV_Info("Departing");
    departing = new DEV_Motion("Departing");

    // Define Motion Sensor accessory for vehicle occupancy (parked or away)
    new SpanAccessory(HOMEKIT_AID_VEHICLE);
    new DEV_Info("Vehicle");
    vehicle = new DEV_Occupancy();
}

void enable_service_homekit_vehicle()
{
    // only create if not already created
    if (!garage_door.has_distance_sensor)
    {
        nvRam->write(nvram_has_distance, 1);
        garage_door.has_distance_sensor = true;
        createVehicleAccessories();
        if (!enable_service_homekit_laser(true))
        {
            // if enabling laser did not update database then we need to do it now.
            homeSpan.updateDatabase();
        }
    }
}

bool enable_service_homekit_laser(bool enable)
{
    if (enable)
    {
        if (!assistLaser && userConfig->getLaserEnabled() && userConfig->getLaserHomeKit())
        {
            // Define Light accessory for parking assist laser
            // Create only if not already created, and user config requires it.
            new SpanAccessory(HOMEKIT_AID_LASER);
            new DEV_Info("Laser");
            assistLaser = new DEV_Light(Light_t::ASSIST_LASER);
            homeSpan.updateDatabase();
            return true;
        }
    }
    else if (assistLaser)
    {
        // Delete the accessory, if it exists
        if (homeSpan.deleteAccessory(HOMEKIT_AID_LASER))
        {
            assistLaser = nullptr;
            homeSpan.updateDatabase();
            return true;
        }
    }
    return false;
}

void setup_homekit()
{
    RINFO(TAG, "=== Setup HomeKit accessories and services ===");

    homeSpan.setLogLevel(0);
    homeSpan.setSketchVersion(AUTO_VERSION);
    homeSpan.setHostNameSuffix("");
    homeSpan.setPortNum(5556);
    // We will manage LED flashing ourselves
    // homeSpan.setStatusPin(LED_BUILTIN);

    homeSpan.enableAutoStartAP();
    homeSpan.setApFunction(start_soft_ap);

    homeSpan.setQRID("RTGO");
    homeSpan.setPairingCode("25102023"); // On Oct 25, 2023, Chamberlain announced they were disabling API
                                         // access for "unauthorized" third parties.

    homeSpan.setWifiCallbackAll(wifiCallbackAll);
    homeSpan.setStatusCallback(statusCallback);

    homeSpan.begin(Category::Bridges, device_name, device_name_rfc952, "ratgdo-ESP32");

#ifdef CONFIG_FREERTOS_USE_TRACE_FACILITY
    new SpanUserCommand('t', "print FreeRTOS task info", printTaskInfo);
#endif
    // Define a bridge (as more than 3 accessories)
    new SpanAccessory(HOMEKIT_AID_BRIDGE);
    new DEV_Info(default_device_name);

    // Define the Garage Door accessory...
    new SpanAccessory(HOMEKIT_AID_GARAGE_DOOR);
    new DEV_Info(device_name);
    new Characteristic::Manufacturer("Ratcloud llc");
    new Characteristic::SerialNumber(Network.macAddress().c_str());
    new Characteristic::Model("ratgdo-ESP32");
    new Characteristic::FirmwareRevision(AUTO_VERSION);
    door = new DEV_GarageDoor();

    // Dry contact (security type 3) cannot control lights
    if (userConfig->getGDOSecurityType() != 3)
    {
        // Define the Light accessory...
        new SpanAccessory(HOMEKIT_AID_LIGHT_BULB);
        new DEV_Info("Light");
        light = new DEV_Light();
    }
    else
    {
        RINFO(TAG, "Dry contact mode. Disabling light switch service");
    }

    // only create motion if we know we have motion sensor(s)
    garage_door.has_motion_sensor = (bool)nvRam->read(nvram_has_motion);
    if (garage_door.has_motion_sensor || userConfig->getMotionTriggers() != 0)
    {
        createMotionAccessories();
    }
    else
    {
        RINFO(TAG, "No motion sensor. Skipping motion service");
    }

    // only create sensors if we know we have time-of-flight distance sensor
    garage_door.has_distance_sensor = (bool)nvRam->read(nvram_has_distance);
    if (garage_door.has_distance_sensor)
    {
        createVehicleAccessories();
        enable_service_homekit_laser(true);
    }
    else
    {
        RINFO(TAG, "No vehicle presence sensor. Skipping motion and occupancy services");
    }

    // Auto poll starts up a new FreeRTOS task to do the HomeKit comms
    // so no need to handle in our Arduino loop.
    homeSpan.autoPoll((1024 * 16), 1, 0);
}

void queueSendHelper(QueueHandle_t q, GDOEvent e, const char *txt)
{
    if (!q || xQueueSend(q, &e, 0) == errQUEUE_FULL)
    {
        RERROR(TAG, "Could not queue homekit notify of %s state: %d", txt, e.value.u);
    }
}

void homekit_unpair()
{
    if (!isPaired)
        return;

    homeSpan.processSerialCommand("U");
}

bool homekit_is_paired()
{
    return isPaired;
}

/****************************************************************************
 * Accessory Information Handler
 */
DEV_Info::DEV_Info(const char *name) : Service::AccessoryInformation()
{
    new Characteristic::Identify();
    new Characteristic::Name(name);
}

boolean DEV_Info::update()
{
    RINFO(TAG, "Request to identify accessory, flash LED, etc.");
    // LED, Laser and Tone calls are all asynchronous.  We will illuminate LED and Laser
    // for 2 seconds, during which we will play tone.  Function will return after 1.5 seconds.
    led.flash(2000);
    laser.flash(2000);
    tone(BEEPER_PIN, 1300);
    delay(500);
    tone(BEEPER_PIN, 2000);
    delay(500);
    tone(BEEPER_PIN, 1300);
    delay(500);
    tone(BEEPER_PIN, 2000, 500);
    return true;
}

/****************************************************************************
 * Garage Door Service Handler
 */
void notify_homekit_target_door_state_change()
{
    if (!isPaired)
        return;

    GDOEvent e;
    e.c = door->target;
    e.value.u = garage_door.target_state;
    queueSendHelper(door->event_q, e, "target door");
}

void notify_homekit_current_door_state_change()
{
    // Notify the vehicle presence code that door state is changing
    if (garage_door.current_state == GarageDoorCurrentState::CURR_OPENING)
        doorOpening();
    if (garage_door.current_state == GarageDoorCurrentState::CURR_CLOSING)
        doorClosing();

    if (!isPaired)
        return;

    GDOEvent e;
    e.c = door->current;
    e.value.u = garage_door.current_state;
    queueSendHelper(door->event_q, e, "current door");
}

void notify_homekit_target_lock()
{
    if (!isPaired)
        return;

    GDOEvent e;
    e.c = door->lockTarget;
    e.value.u = garage_door.target_lock;
    queueSendHelper(door->event_q, e, "target lock");
}

void notify_homekit_current_lock()
{
    if (!isPaired)
        return;

    GDOEvent e;
    e.c = door->lockCurrent;
    e.value.u = garage_door.current_lock;
    queueSendHelper(door->event_q, e, "current lock");
}

void notify_homekit_obstruction()
{
    if (!isPaired)
        return;

    GDOEvent e;
    e.c = door->obstruction;
    e.value.b = garage_door.obstructed;
    queueSendHelper(door->event_q, e, "obstruction");
}

DEV_GarageDoor::DEV_GarageDoor() : Service::GarageDoorOpener()
{
    RINFO(TAG, "Configuring HomeKit Garage Door Service");
    event_q = xQueueCreate(5, sizeof(GDOEvent));
    current = new Characteristic::CurrentDoorState(current->CLOSED);
    target = new Characteristic::TargetDoorState(target->CLOSED);
    obstruction = new Characteristic::ObstructionDetected(obstruction->NOT_DETECTED);
    if (userConfig->getGDOSecurityType() != 3)
    {
        // Dry contact cannot control lock ?
        lockCurrent = new Characteristic::LockCurrentState(lockCurrent->UNKNOWN);
        lockTarget = new Characteristic::LockTargetState(lockTarget->UNLOCK);
    }
    else
    {
        lockCurrent = nullptr;
        lockTarget = nullptr;
    }
    // We can set current lock state to unknown as HomeKit has value for that.
    // But we can't do the same for door state as HomeKit has no value for that.
    garage_door.current_lock = CURR_UNKNOWN;
}

boolean DEV_GarageDoor::update()
{
    RINFO(TAG, "Garage Door Characteristics Update");
    if (target->getNewVal() == target->OPEN)
    {
        if (open_door())
        {
            obstruction->setVal(false);
            current->setVal(current->OPENING);
        }
    }
    else
    {
        if (close_door())
        {
            obstruction->setVal(false);
            current->setVal(current->CLOSING);
        }
    }

    if (userConfig->getGDOSecurityType() != 3)
    {
        // Dry contact cannot control lock
        set_lock(lockTarget->getNewVal() == lockTarget->LOCK);
    }
    return true;
}

void DEV_GarageDoor::loop()
{
    if (uxQueueMessagesWaiting(event_q) > 0)
    {
        GDOEvent e;
        xQueueReceive(event_q, &e, 0);
        if (e.c == current)
            RINFO(TAG, "Garage door set CurrentDoorState: %d", e.value.u);
        else if (e.c == target)
            RINFO(TAG, "Garage door set TargetDoorState: %d", e.value.u);
        else if (e.c == obstruction)
            RINFO(TAG, "Garage door set ObstructionDetected: %d", e.value.u);
        else if (e.c == lockCurrent)
            RINFO(TAG, "Garage door set LockCurrentState: %d", e.value.u);
        else if (e.c == lockTarget)
            RINFO(TAG, "Garage door set LockTargetState: %d", e.value.u);
        else
            RINFO(TAG, "Garage door set Unknown: %d", e.value.u);
        e.c->setVal(e.value.u);
    }
}

/****************************************************************************
 * Light Service Handler
 */
void notify_homekit_light()
{
    if (!isPaired || !light)
        return;

    GDOEvent e;
    e.value.b = garage_door.light;
    queueSendHelper(light->event_q, e, "light");
}

void notify_homekit_laser(bool on)
{
    if (!isPaired || !assistLaser)
        return;

    GDOEvent e;
    e.value.b = on;
    queueSendHelper(assistLaser->event_q, e, "laser");
}

DEV_Light::DEV_Light(Light_t type) : Service::LightBulb()
{
    DEV_Light::type = type;
    if (type == Light_t::GDO_LIGHT)
        RINFO(TAG, "Configuring HomeKit Light Service for GDO Light");
    else if (type == Light_t::ASSIST_LASER)
        RINFO(TAG, "Configuring HomeKit Light Service for Laser");
    event_q = xQueueCreate(5, sizeof(GDOEvent));
    DEV_Light::on = new Characteristic::On(DEV_Light::on->OFF);
}

boolean DEV_Light::update()
{
    if (this->type == Light_t::GDO_LIGHT)
    {
        set_light(DEV_Light::on->getNewVal<bool>());
    }
    else if (this->type == Light_t::ASSIST_LASER)
    {
        if (on->getNewVal<bool>())
        {
            RINFO(TAG, "Turn parking assist laser on");
            laser.on();
        }
        else
        {
            RINFO(TAG, "Turn parking assist laser off");
            laser.off();
        }
    }
    return true;
}

void DEV_Light::loop()
{
    if (uxQueueMessagesWaiting(event_q) > 0)
    {
        GDOEvent e;
        xQueueReceive(event_q, &e, 0);
        if (this->type == Light_t::GDO_LIGHT)
            RINFO(TAG, "Light has turned %s", e.value.b ? "on" : "off");
        else if (this->type == Light_t::ASSIST_LASER)
            RINFO(TAG, "Parking assist laster has turned %s", e.value.b ? "on" : "off");
        DEV_Light::on->setVal(e.value.b);
    }
}

/****************************************************************************
 * Motion Service Handler
 */
void enable_service_homekit_motion()
{
    // only create if not already created
    if (!garage_door.has_motion_sensor)
    {
        nvRam->write(nvram_has_motion, 1);
        garage_door.has_motion_sensor = true;
        createMotionAccessories();
    }
}

void notify_homekit_motion()
{
    if (!isPaired || !motion)
        return;

    GDOEvent e;
    e.value.b = garage_door.motion;
    queueSendHelper(motion->event_q, e, "motion");
}

void notify_homekit_vehicle_arriving(bool vehicleArriving)
{
    if (!isPaired || !arriving)
        return;

    GDOEvent e;
    e.value.b = vehicleArriving;
    queueSendHelper(arriving->event_q, e, "arriving");
}

void notify_homekit_vehicle_departing(bool vehicleDeparting)
{
    if (!isPaired || !departing)
        return;

    GDOEvent e;
    e.value.b = vehicleDeparting;
    queueSendHelper(departing->event_q, e, "departing");
}

DEV_Motion::DEV_Motion(const char *name) : Service::MotionSensor()
{
    RINFO(TAG, "Configuring HomeKit Motion Service for %s", name);
    event_q = xQueueCreate(5, sizeof(GDOEvent));
    strlcpy(this->name, name, sizeof(this->name));
    DEV_Motion::motion = new Characteristic::MotionDetected(motion->NOT_DETECTED);
}

void DEV_Motion::loop()
{
    if (uxQueueMessagesWaiting(event_q) > 0)
    {
        GDOEvent e;
        xQueueReceive(event_q, &e, 0);
        RINFO(TAG, "%s %s", name, e.value.b ? "detected" : "reset");
        DEV_Motion::motion->setVal(e.value.b);
    }
}

/****************************************************************************
 * Occupancy Service Handler
 */
void notify_homekit_vehicle_occupancy(bool vehicleDetected)
{
    if (!isPaired || !vehicle)
        return;

    GDOEvent e;
    e.value.b = vehicleDetected;
    queueSendHelper(vehicle->event_q, e, "vehicle");
}

DEV_Occupancy::DEV_Occupancy() : Service::OccupancySensor()
{
    RINFO(TAG, "Configuring HomeKit Occupancy Service");
    event_q = xQueueCreate(5, sizeof(GDOEvent));
    DEV_Occupancy::occupied = new Characteristic::OccupancyDetected(occupied->NOT_DETECTED);
}

void DEV_Occupancy::loop()
{
    if (uxQueueMessagesWaiting(event_q) > 0)
    {
        GDOEvent e;
        xQueueReceive(event_q, &e, 0);
        RINFO(TAG, "Vehicle occupancy %s", e.value.b ? "detected" : "reset");
        DEV_Occupancy::occupied->setVal(e.value.b);
    }
}
