/****************************************************************************
 * RATGDO HomeKit for ESP32
 * https://ratcloud.llc
 * https://github.com/PaulWieland/ratgdo
 *
 * Copyright (c) 2023-24 David A Kerr... https://github.com/dkerr64/
 * All Rights Reserved.
 * Licensed under terms of the GPL-3.0 License.
 *
 * Contributions acknowledged from
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

static bool isPaired = false;
static bool rebooting = false;

/****************************************************************************
 * Callback functions, notify us of significant events
 */
void wifiCallbackAll(int count)
{
    if (rebooting)
        return;

    RINFO(TAG, "WiFi established, IP: %s, count: %d", WiFi.localIP().toString().c_str(), count);
    userConfig->set(cfg_localIP, WiFi.localIP().toString().c_str());
    userConfig->set(cfg_gatewayIP, WiFi.gatewayIP().toString().c_str());
    userConfig->set(cfg_subnetMask, WiFi.gatewayIP().toString().c_str());
    userConfig->set(cfg_nameserverIP, WiFi.dnsIP().toString().c_str());
    RINFO(TAG, "WiFi Got IP: %s, Mask: %s, Gateway: %s, DNS: %s", userConfig->getLocalIP().c_str(),
          userConfig->getSubnetMask().c_str(), userConfig->getGatewayIP().c_str(), userConfig->getNameserverIP().c_str());
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
    laser.off();
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
                RINFO(TAG, "Set static IP address");
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
    new SpanAccessory();
    new Service::AccessoryInformation();
    new Characteristic::Identify();
    new Characteristic::Name("Motion");
    motion = new DEV_Motion("Motion");
}

void createVehicleAccessories()
{
    // Exit if already setup
    if (arriving)
        return;

    // Define Motion Sensor accessory for vehicle arriving
    new SpanAccessory();
    new Service::AccessoryInformation();
    new Characteristic::Identify();
    new Characteristic::Name("Arriving");
    arriving = new DEV_Motion("Arriving");

    // Define Motion Sensor accessory for vehicle departing
    new SpanAccessory();
    new Service::AccessoryInformation();
    new Characteristic::Identify();
    new Characteristic::Name("Departing");
    departing = new DEV_Motion("Departing");

    // Define Motion Sensor accessory for vehicle occupancy (parked or away)
    new SpanAccessory();
    new Service::AccessoryInformation();
    new Characteristic::Identify();
    new Characteristic::Name("Vehicle");
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
    }
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
    new SpanAccessory();
    new Service::AccessoryInformation();
    new Characteristic::Identify();

    // Define the Garage Door accessory...
    new SpanAccessory();
    new Service::AccessoryInformation();
    new Characteristic::Identify();
    new Characteristic::Name(device_name);
    new Characteristic::Manufacturer("Ratcloud llc");
    new Characteristic::SerialNumber(Network.macAddress().c_str());
    new Characteristic::Model("ratgdo-ESP32");
    new Characteristic::FirmwareRevision(AUTO_VERSION);
    door = new DEV_GarageDoor();

    // Dry contact (security type 3) cannot control lights
    if (userConfig->getGDOSecurityType() != 3)
    {
        // Define the Light accessory...
        new SpanAccessory();
        new Service::AccessoryInformation();
        new Characteristic::Identify();
        new Characteristic::Name("Light");
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
    if (target->getNewVal() == target->OPEN)
    {
        RINFO(TAG, "Opening Garage Door");
        current->setVal(current->OPENING);
        obstruction->setVal(false);
        open_door();
    }
    else
    {
        RINFO(TAG, "Closing Garage Door");
        current->setVal(current->CLOSING);
        obstruction->setVal(false);
        close_door();
    }

    if (userConfig->getGDOSecurityType() != 3)
    {
        // Dry contact cannot control lock ?
        if (lockTarget->getNewVal() == lockTarget->LOCK)
        {
            RINFO(TAG, "Locking Garage Door Remotes");
            set_lock(lockTarget->LOCK);
        }
        else
        {
            RINFO(TAG, "Unlocking Garage Door Remotes");
            set_lock(lockTarget->UNLOCK);
        }
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

DEV_Light::DEV_Light() : Service::LightBulb()
{
    RINFO(TAG, "Configuring HomeKit Light Service");
    event_q = xQueueCreate(5, sizeof(GDOEvent));
    on = new Characteristic::On(on->OFF);
}

boolean DEV_Light::update()
{
    RINFO(TAG, "Turn light %s", on->getNewVal<bool>() ? "on" : "off");
    set_light(on->getNewVal<bool>());
    return true;
}

void DEV_Light::loop()
{
    if (uxQueueMessagesWaiting(event_q) > 0)
    {
        GDOEvent e;
        xQueueReceive(event_q, &e, 0);
        RINFO(TAG, "Light has turned %s", e.value.b ? "on" : "off");
        on->setVal(e.value.b);
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
    motion = new Characteristic::MotionDetected(motion->NOT_DETECTED);
}

void DEV_Motion::loop()
{
    if (uxQueueMessagesWaiting(event_q) > 0)
    {
        GDOEvent e;
        xQueueReceive(event_q, &e, 0);
        RINFO(TAG, "%s %s", name, e.value.b ? "detected" : "reset");
        motion->setVal(e.value.b);
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
    occupied = new Characteristic::OccupancyDetected(occupied->NOT_DETECTED);
}

void DEV_Occupancy::loop()
{
    if (uxQueueMessagesWaiting(event_q) > 0)
    {
        GDOEvent e;
        xQueueReceive(event_q, &e, 0);
        RINFO(TAG, "Vehicle occupancy %s", e.value.b ? "detected" : "reset");
        occupied->setVal(e.value.b);
    }
}
