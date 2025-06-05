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

// 3rd party includes
#include <magic_enum.hpp>

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
#ifndef USE_GDOLIB
#include "drycontact.h"
#else
#include "gdo.h"
#endif

// Logger tag
static const char *TAG = "ratgdo-homekit";

static DEV_GarageDoor *door;
static DEV_Light *light;
static DEV_Motion *motion;
static DEV_Motion *arriving;
static DEV_Motion *departing;
static DEV_Occupancy *vehicle;
static DEV_Light *assistLaser;
static DEV_Occupancy *roomOccupancy;

static bool isPaired = false;
static bool rebooting = false;

char qrPayload[21];

// Buffer to hold all IPv6 addresses as a single string (max 8 addresses, ~40 chars each)
char ipv6_addresses[320] = {0};

/****************************************************************************
 * Callback functions, notify us of significant events
 */
void wifiBegin(const char *ssid, const char *pw)
{
    ESP_LOGI(TAG, "Wifi begin for SSID: %s", ssid);
    WiFi.setSleep(WIFI_PS_NONE); // Improves performance, at cost of power consumption
    WiFi.hostname((const char *)device_name_rfc952);
    WiFi.enableIPv6(); // Enable IPv6 support
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
            ESP_LOGI(TAG, "Set static IP: %s, Mask: %s, Gateway: %s, DNS: %s",
                     ip.toString().c_str(), nm.toString().c_str(), gw.toString().c_str(), dns.toString().c_str());
            WiFi.config(ip, gw, nm, dns);
        }
        else
        {
            ESP_LOGI(TAG, "Failed to set static IP address, error parsing addresses");
        }
    }
    WiFi.begin(ssid, pw);
}

void connectionCallback(int count)
{
    if (rebooting)
        return;

    ESP_LOGI(TAG, "WiFi established, count: %d, IP: %s, Mask: %s, Gateway: %s, DNS: %s Link_IPv6: %s",
        count,
        WiFi.localIP().toString().c_str(),
        WiFi.subnetMask().toString().c_str(),
        WiFi.gatewayIP().toString().c_str(),
        WiFi.dnsIP().toString().c_str(),
        WiFi.linkLocalIPv6().toString().c_str());

    // IPv4 Config
    userConfig->set(cfg_localIP, WiFi.localIP().toString().c_str());
    userConfig->set(cfg_gatewayIP, WiFi.gatewayIP().toString().c_str());
    userConfig->set(cfg_subnetMask, WiFi.subnetMask().toString().c_str());

    // Only update cfg_nameserverIP if it is an IPv4 address. .dnsIP() can return an IPv6 address if we have one from SLAAC
    if (WiFi.dnsIP().type() == IPv4)
        userConfig->set(cfg_nameserverIP, WiFi.dnsIP().toString().c_str());

    // IPv6 SLAAC
    esp_ip6_addr_t if_ip6[LWIP_IPV6_NUM_ADDRESSES];
    int nIPv6 = esp_netif_get_all_preferred_ip6(WiFi.STA.netif(), if_ip6);
    ESP_LOGI(TAG, "Found %d IPv6 addresses:", nIPv6);

    ipv6_addresses[0] = '\0'; // Clear the buffer
    for (int i = 0; i < nIPv6; i++)
    {
        String addrStr = IPAddress(IPv6, (const uint8_t *)if_ip6[i].addr, if_ip6[i].zone).toString();

        ESP_LOGI(TAG,"  %s", addrStr.c_str());

        // Append to buffer, separated by comma if not first
        if (i > 0) {
            strlcat(ipv6_addresses, ",", sizeof(ipv6_addresses));
        }

        strlcat(ipv6_addresses, addrStr.c_str(), sizeof(ipv6_addresses));
    }

    // With WiFi connected, we can now initialize the rest of our app.
    if (!softAPmode)
    {
        if (userConfig->getTimeZone() == "")
        {
            get_auto_timezone();
        }
        setup_vehicle();
        setup_comms();
#ifndef USE_GDOLIB
        setup_drycontact();
#endif
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
        ESP_LOGI(TAG, "Status: No WiFi Credentials, need to provision");
        break;
    case HS_WIFI_CONNECTING:
        ESP_LOGI(TAG, "Status: WiFi connecting");
        break;
    case HS_PAIRING_NEEDED:
        ESP_LOGI(TAG, "Status: Need to pair");
        isPaired = false;
        break;
    case HS_PAIRED:
        ESP_LOGI(TAG, "Status: Paired");
        isPaired = true;
        break;
    case HS_REBOOTING:
        rebooting = true;
        ESP_LOGI(TAG, "Status: Rebooting");
        break;
    case HS_FACTORY_RESET:
        ESP_LOGI(TAG, "Status: Factory Reset");
        break;
    case HS_WIFI_SCANNING:
        ESP_LOGI(TAG, "Status: WiFi Scanning");
        break;
    default:
        ESP_LOGI(TAG, "HomeSpan Status: %s", homeSpan.statusString(status));
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
        Serial.printf("Name                    Core\tPri\tStack\tState\n");
        for (size_t i = 0; i < count; i++)
        {
            Serial.printf("%s\t%s\t%4d\t%3d\t%5d\t%s\n", (char *)tasks[i].pcTaskName,
                          strlen((char *)tasks[i].pcTaskName) > 7 ? "" : "\t",
                          (int)(tasks[i].xCoreID < 16) ? tasks[i].xCoreID : -1,
                          (int)tasks[i].uxBasePriority,
                          (int)tasks[i].usStackHighWaterMark,
                          (magic_enum::enum_name(tasks[i].eCurrentState)).data());
        }
        Serial.printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n\n");
    }
    vPortFree(tasks);
};
#endif

void printLogInfo(const char *buf)
{
    ratgdoLogger->printMessageLog(Serial);
}

void setLogLevel(const char *buf)
{
    long value = 0;
    char *p = (char *)buf;
    while (*p)
    {
        if (isdigit(*p))
        {
            value = strtol(p, &p, 10);
        }
        else
        {
            p++;
        }
    }
    if (value >= 0 && value <= 5)
    {
        Serial.printf("Set log level to %d\n", value);
        userConfig->set(cfg_logLevel, (int)value);
        esp_log_level_set("*", (esp_log_level_t)userConfig->getLogLevel());
    }
    else
    {
        Serial.printf("Invalid log level, value must be between 0(none) and 5(verbose)\n");
    }
}

#ifdef USE_GDOLIB
void testMoveDoor(const char *buf)
{
    long value = 0;
    char *p = (char *)buf;
    while (*p)
    {
        if (isdigit(*p))
        {
            value = strtol(p, &p, 10);
        }
        else
        {
            p++;
        }
    }
    if (value >= 0 && value <= 100)
    {
        Serial.printf("Move door to: %d%\n", value);
        gdo_door_move_to_target(value * 100);
    }
    else
    {
        Serial.printf("Invalid door postion, value must be between 0(open) and 100(closed)\n");
    }
}
#endif

#ifdef CRASH_DEBUG
extern void delayFnCall(uint32_t ms, void (*callback)());
void testDelayFn(const char *buf)
{
    delayFnCall(5000, (void (*)())NULL);
}
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

void enable_service_homekit_vehicle(bool enable)
{
    if (enable)
    {
        if (!vehicle) // using "vehicle" as proxy for all three motion sensors
        {
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

            homeSpan.updateDatabase();
        }
    }
    else if (vehicle) // using "vehicle" as proxy for all three motion sensors
    {
        // Delete the accessories, if they exists
        ESP_LOGI(TAG, "Deleting HomeKit Motion and Occupancy Accessories for vehicle presense");
        homeSpan.deleteAccessory(HOMEKIT_AID_VEHICLE);
        vehicle = nullptr;
        homeSpan.deleteAccessory(HOMEKIT_AID_ARRIVING);
        arriving = nullptr;
        homeSpan.deleteAccessory(HOMEKIT_AID_DEPARTING);
        departing = nullptr;

        homeSpan.updateDatabase();
    }

    enable_service_homekit_laser(userConfig->getLaserEnabled() && userConfig->getLaserHomeKit());
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
        ESP_LOGI(TAG, "Deleting HomeKit Light Switch for Laser");
        if (homeSpan.deleteAccessory(HOMEKIT_AID_LASER))
        {
            assistLaser = nullptr;
            homeSpan.updateDatabase();
            return true;
        }
    }
    return false;
}

bool enable_service_homekit_room_occupancy(bool enable)
{
    // Only enable room occupancy if we have a motion sensor as well
    if (enable && motion)
    {
        if (!roomOccupancy)
        {
            // Define the Room Occupancy Sensor accessory...
            new SpanAccessory(HOMEKIT_AID_ROOM_OCCUPANCY);
            new DEV_Info("Room Occupancy");
            roomOccupancy = new DEV_Occupancy();
            return true;
        }
    }
    else if (roomOccupancy)
    {
        // Delete the accessory, if it exists
        ESP_LOGI(TAG, "Deleting HomeKit Occupancy Sensor accessory");
        if (homeSpan.deleteAccessory(HOMEKIT_AID_ROOM_OCCUPANCY))
        {
            roomOccupancy = nullptr;
            homeSpan.updateDatabase();
            garage_door.room_occupied = false;
            return true;
        }
    }
    return false;
}

char *toBase62(char *base62, size_t len, uint32_t base10)
{
    static const char base62Chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    int i = 0;
    // Will pad with zeros until base62 buffer filled (to len)
    while ((base10 > 0) || (i < len - 1))
    {
        base62[i++] = base62Chars[base10 % 62];
        base10 /= 62;
    }
    // null terminate
    base62[i] = 0;
    // Now reverse order of the string;
    char *str = base62;
    char *end = str + strlen(str) - 1;
    while (str < end)
    {
        *str ^= *end;
        *end ^= *str;
        *str ^= *end;
        str++;
        end--;
    }
    return base62;
}

void setup_homekit()
{
    ESP_LOGI(TAG, "=== Setup HomeKit accessories and services ===");

    homeSpan.setLogLevel(0);
    homeSpan.setSketchVersion(AUTO_VERSION);
    homeSpan.setHostNameSuffix("");
    homeSpan.setPortNum(5556);
    // We will manage LED flashing ourselves
    // homeSpan.setStatusPin(LED_BUILTIN);
    homeSpan.enableAutoStartAP();
    homeSpan.setApFunction(start_soft_ap);

    // Generate a QR Code ID from our MAC address, which should create unique pairing QR codes
    // for each of multiple devices on a network... although we do have to clip to 4 characters,
    // so we loose ~2 most significant bits.
    uint8_t mac[6];
    Network.macAddress(mac);
    uint32_t uid = (mac[3] << 16) + (mac[4] << 8) + mac[5];
    char qrID[6];
    toBase62(qrID, sizeof(qrID), uid); // always includes leading zeros
    ESP_LOGI(TAG, "HomeKit pairing QR Code ID: %s", &qrID[1]);
    HapQR qrCode;
    strlcpy(qrPayload, qrCode.get((uint32_t)25102023, &qrID[1], (uint8_t)Category::GarageDoorOpeners), sizeof(qrPayload));
    ESP_LOGI(TAG, "HomeKit QR setup payload: %s", qrPayload);
    homeSpan.setQRID(&qrID[1]);
    homeSpan.setPairingCode("25102023"); // On Oct 25, 2023, Chamberlain announced they were disabling API
                                         // access for "unauthorized" third parties.
    homeSpan.setWifiBegin(wifiBegin);
    homeSpan.setConnectionCallback(connectionCallback);
    homeSpan.setStatusCallback(statusCallback);

    homeSpan.begin(Category::Bridges, device_name, device_name_rfc952, "ratgdo-ESP32");

#ifdef CONFIG_FREERTOS_USE_TRACE_FACILITY
    new SpanUserCommand('t', "- print FreeRTOS task info", printTaskInfo);
#endif
    new SpanUserCommand('l', "- print RATGDO buffered message log", printLogInfo);
    new SpanUserCommand('d', "<level> - set ESP log level 0(none), 1(error), 2(warn), 3(info), 4(debug), 5(verbose)", setLogLevel);
#ifdef USE_GDOLIB
    new SpanUserCommand('m', "<percent> - move door to position between 0(open) and 100 (closed)", testMoveDoor);
#endif
#ifdef CRASH_DEBUG
    new SpanUserCommand('z', "- test function", testDelayFn);
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
        ESP_LOGI(TAG, "Dry contact mode. Disabling light switch service");
    }

    // only create motion if we know we have motion sensor(s)
    garage_door.has_motion_sensor = (bool)nvRam->read(nvram_has_motion);
    if (garage_door.has_motion_sensor || userConfig->getMotionTriggers() != 0)
    {
        createMotionAccessories();
    }
    else
    {
        ESP_LOGI(TAG, "No motion sensor. Skipping motion service");
    }

    // only create sensors if we know we have time-of-flight distance sensor
    garage_door.has_distance_sensor = (bool)nvRam->read(nvram_has_distance);
    if (garage_door.has_distance_sensor)
    {
        enable_service_homekit_vehicle(userConfig->getVehicleHomeKit());
    }
    else
    {
        ESP_LOGI(TAG, "No vehicle presence sensor. Skipping motion and occupancy services");
    }

    // Create a room occupancy sensor if timer for it is greater than 0
    enable_service_homekit_room_occupancy(userConfig->getOccupancyDuration() > 0);

    // Auto poll starts up a new FreeRTOS task to do the HomeKit comms
    // so no need to handle in our Arduino loop.
    homeSpan.autoPoll((1024 * 16), 1, 0);
}

void queueSendHelper(QueueHandle_t q, GDOEvent e, const char *txt)
{
    if (!q || xQueueSend(q, &e, 0) == errQUEUE_FULL)
    {
        ESP_LOGE(TAG, "Could not queue homekit notify of %s state: %d", txt, e.value.u);
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
    ESP_LOGI(TAG, "Request to identify accessory, flash LED, etc.");
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
void notify_homekit_target_door_state_change(GarageDoorTargetState state)
{
    if (!isPaired)
        return;

    GDOEvent e;
    e.c = door->target;
    e.value.u = (uint8_t)(garage_door.target_state = state);
    queueSendHelper(door->event_q, e, "target door");
}

void notify_homekit_current_door_state_change(GarageDoorCurrentState state)
{
    if (!isPaired)
        return;

    GDOEvent e;
    e.c = door->current;
    e.value.u = (uint8_t)(garage_door.current_state = state);
    queueSendHelper(door->event_q, e, "current door");

    // Set target door state to match.
    switch (state)
    {
    case CURR_OPENING:
        doorOpening(); // Fall through...
    case CURR_OPEN:
        notify_homekit_target_door_state_change(TGT_OPEN);
        break;

    case CURR_CLOSING:
        doorClosing(); // Fall through...
    case CURR_CLOSED:
        notify_homekit_target_door_state_change(TGT_CLOSED);
        break;

    default:
        // Ignore other states.
        break;
    }
}

void notify_homekit_target_lock(LockTargetState state)
{
    if (!isPaired)
        return;

    GDOEvent e;
    e.c = door->lockTarget;
    e.value.u = (uint8_t)(garage_door.target_lock = state);
    queueSendHelper(door->event_q, e, "target lock");
}

void notify_homekit_current_lock(LockCurrentState state)
{
    if (!isPaired)
        return;

    GDOEvent e;
    e.c = door->lockCurrent;
    e.value.u = (uint8_t)(garage_door.current_lock = state);
    queueSendHelper(door->event_q, e, "current lock");
}

void notify_homekit_obstruction(bool state)
{
    if (!isPaired)
        return;

    GDOEvent e;
    e.c = door->obstruction;
    e.value.b = garage_door.obstructed = state;
    queueSendHelper(door->event_q, e, "obstruction");
}

DEV_GarageDoor::DEV_GarageDoor() : Service::GarageDoorOpener()
{
    ESP_LOGI(TAG, "Configuring HomeKit Garage Door Service");
    event_q = xQueueCreate(10, sizeof(GDOEvent));
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
    ESP_LOGI(TAG, "Garage Door Characteristics Update");
    GarageDoorCurrentState state = (target->getNewVal() == target->OPEN) ? open_door() : close_door();
    obstruction->setVal(false);
    current->setVal(state);

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
            ESP_LOGI(TAG, "Garage door set CurrentDoorState: %d", e.value.u);
        else if (e.c == target)
            ESP_LOGI(TAG, "Garage door set TargetDoorState: %d", e.value.u);
        else if (e.c == obstruction)
            ESP_LOGI(TAG, "Garage door set ObstructionDetected: %d", e.value.u);
        else if (e.c == lockCurrent)
            ESP_LOGI(TAG, "Garage door set LockCurrentState: %d", e.value.u);
        else if (e.c == lockTarget)
            ESP_LOGI(TAG, "Garage door set LockTargetState: %d", e.value.u);
        else
            ESP_LOGI(TAG, "Garage door set Unknown: %d", e.value.u);
        e.c->setVal(e.value.u);
    }
}

/****************************************************************************
 * Light Service Handler
 */
void notify_homekit_light(bool state)
{
    if (!isPaired || !light)
        return;

    GDOEvent e;
    e.value.b = garage_door.light = state;
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
        ESP_LOGI(TAG, "Configuring HomeKit Light Service for GDO Light");
    else if (type == Light_t::ASSIST_LASER)
        ESP_LOGI(TAG, "Configuring HomeKit Light Service for Laser");
    event_q = xQueueCreate(10, sizeof(GDOEvent));
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
            ESP_LOGI(TAG, "Turn parking assist laser on");
            laser.on();
        }
        else
        {
            ESP_LOGI(TAG, "Turn parking assist laser off");
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
            ESP_LOGI(TAG, "Light has turned %s", e.value.b ? "on" : "off");
        else if (this->type == Light_t::ASSIST_LASER)
            ESP_LOGI(TAG, "Parking assist laster has turned %s", e.value.b ? "on" : "off");
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

void notify_homekit_motion(bool state)
{
    if (!isPaired || !motion)
        return;

    GDOEvent e;
    e.value.b = garage_door.motion = state;
    garage_door.motion_timer = (!state) ? 0 : millis64() + MOTION_TIMER_DURATION;
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
    ESP_LOGI(TAG, "Configuring HomeKit Motion Service for %s", name);
    event_q = xQueueCreate(10, sizeof(GDOEvent));
    strlcpy(this->name, name, sizeof(this->name));
    DEV_Motion::motion = new Characteristic::MotionDetected(motion->NOT_DETECTED);
}

void DEV_Motion::loop()
{
    if (uxQueueMessagesWaiting(event_q) > 0)
    {
        GDOEvent e;
        xQueueReceive(event_q, &e, 0);
        ESP_LOGI(TAG, "%s %s", name, e.value.b ? "detected" : "reset");
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

void notify_homekit_room_occupancy(bool occupied)
{
    if (!isPaired || !roomOccupancy)
        return;

    GDOEvent e;
    e.value.b = garage_door.room_occupied = occupied;
    garage_door.room_occupancy_timeout = (!occupied) ? 0 : millis64() + userConfig->getOccupancyDuration() * 1000; // convert seconds to milliseconds
    queueSendHelper(roomOccupancy->event_q, e, "room occupancy");
}

DEV_Occupancy::DEV_Occupancy() : Service::OccupancySensor()
{
    ESP_LOGI(TAG, "Configuring HomeKit Occupancy Service");
    event_q = xQueueCreate(10, sizeof(GDOEvent));
    DEV_Occupancy::occupied = new Characteristic::OccupancyDetected(occupied->NOT_DETECTED);
}

void DEV_Occupancy::loop()
{
    if (uxQueueMessagesWaiting(event_q) > 0)
    {
        GDOEvent e;
        xQueueReceive(event_q, &e, 0);
        ESP_LOGI(TAG, "%s occupancy %s", (this == vehicle) ? "Vehicle" : "Room", e.value.b ? "detected" : "reset");
        DEV_Occupancy::occupied->setVal(e.value.b);
    }
}
