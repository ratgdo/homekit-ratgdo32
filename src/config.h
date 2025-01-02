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
 * Brandon Matthews... https://github.com/thenewwazoo
 * Jonathan Stroud...  https://github.com/jgstroud
 *
 */
#pragma once

// C/C++ language includes
#include <variant>
#include <string>
#include <map>

// Arduino includes
#include <Print.h>

// ESP system includes
#include <nvs.h>

// RATGDO project includes
// none

// Globals, for easy access...
#define DEVICE_NAME_SIZE 32
// TODO support WiFi TX Power... adjust max value if necessary (on ESP32 const are defined)
#define WIFI_POWER_MAX 20

extern char default_device_name[DEVICE_NAME_SIZE];
extern char device_name[DEVICE_NAME_SIZE];
extern char device_name_rfc952[DEVICE_NAME_SIZE];

// Define all the user setting keys as consts so we don't repeat strings
// throughout the code... and compiler will pick up any typos for us.
// const char cfg_deviceName[] = "deviceName";
// NOTE... truncated to 15 chars when saving to NVRAM !!!
constexpr char cfg_deviceName[] = "deviceName";
constexpr char cfg_wifiChanged[] = "wifiChanged";
constexpr char cfg_wifiPower[] = "wifiPower";
constexpr char cfg_wifiPhyMode[] = "wifiPhyMode";
constexpr char cfg_staticIP[] = "staticIP";
constexpr char cfg_localIP[] = "localIP";
constexpr char cfg_subnetMask[] = "subnetMask";
constexpr char cfg_gatewayIP[] = "gatewayIP";
constexpr char cfg_nameserverIP[] = "nameserverIP";
constexpr char cfg_passwordRequired[] = "passwordRequired";
constexpr char cfg_wwwUsername[] = "wwwUsername";
constexpr char cfg_wwwCredentials[] = "wwwCredentials";
constexpr char cfg_GDOSecurityType[] = "GDOSecurityType";
constexpr char cfg_TTCseconds[] = "TTCseconds";
constexpr char cfg_rebootSeconds[] = "rebootSeconds";
constexpr char cfg_LEDidle[] = "LEDidle";
constexpr char cfg_motionTriggers[] = "motionTriggers";
constexpr char cfg_enableNTP[] = "enableNTP";
constexpr char cfg_doorUpdateAt[] = "doorUpdateAt";
constexpr char cfg_timeZone[] = "timeZone";
constexpr char cfg_softAPmode[] = "softAPmode";
constexpr char cfg_syslogEn[] = "syslogEn";
constexpr char cfg_syslogIP[] = "syslogIP";
constexpr char cfg_syslogPort[] = "syslogPort";
constexpr char cfg_vehicleThreshold[] = "vehicleThreshold";
constexpr char cfg_laserEnabled[] = "laserEnabled";
constexpr char cfg_laserHomeKit[] = "laserHomeKit";
constexpr char cfg_assistDuration[] = "assistDuration";

constexpr char nvram_messageLog[] = "messageLog";
constexpr char nvram_id_code[] = "id_code";
constexpr char nvram_rolling[] = "rolling";
constexpr char nvram_has_motion[] = "has_motion";
constexpr char nvram_ratgdo_pw[] = "ratgdo_pw";
constexpr char nvram_has_distance[] = "has_distance";

struct configSetting
{
    bool reboot;
    bool wifiChanged;
    std::variant<bool, int, std::string> value;
    bool (*fn)(const std::string &key, const std::string &value, configSetting *actions);
};

class userSettings
{
private:
    std::map<std::string, configSetting> settings;
    std::string configFile;
    static userSettings *instancePtr;
    userSettings();
    void toFile(Print &file);
    SemaphoreHandle_t mutex;

public:
    userSettings(const userSettings &obj) = delete;
    static userSettings *getInstance() { return instancePtr; }

    bool contains(const std::string &key);
    bool set(const std::string &key, const bool value);
    bool set(const std::string &key, const int value);
    bool set(const std::string &key, const std::string &value);
    bool set(const std::string &key, const char *value);
    std::variant<bool, int, std::string> get(const std::string &key);
    configSetting getDetail(const std::string &key);
    void toStdOut();
    void save();
    void load();

    std::string getDeviceName() { return std::get<std::string>(get(cfg_deviceName)); };
    bool getWifiChanged() { return std::get<bool>(get(cfg_wifiChanged)); };
    int getWifiPower() { return std::get<int>(get(cfg_wifiPower)); };
    int getWifiPhyMode() { return std::get<int>(get(cfg_wifiPhyMode)); };
    bool getStaticIP() { return std::get<bool>(get(cfg_staticIP)); };
    std::string getLocalIP() { return std::get<std::string>(get(cfg_localIP)); };
    std::string getSubnetMask() { return std::get<std::string>(get(cfg_subnetMask)); };
    std::string getGatewayIP() { return std::get<std::string>(get(cfg_gatewayIP)); };
    std::string getNameserverIP() { return std::get<std::string>(get(cfg_nameserverIP)); };
    bool getPasswordRequired() { return std::get<bool>(get(cfg_passwordRequired)); };
    std::string getwwwUsername() { return std::get<std::string>(get(cfg_wwwUsername)); };
    std::string getwwwCredentials() { return std::get<std::string>(get(cfg_wwwCredentials)); };
    int getGDOSecurityType() { return std::get<int>(get(cfg_GDOSecurityType)); };
    int getTTCseconds() { return std::get<int>(get(cfg_TTCseconds)); };
    int getRebootSeconds() { return std::get<int>(get(cfg_rebootSeconds)); };
    int getLEDidle() { return std::get<int>(get(cfg_LEDidle)); };
    int getMotionTriggers() { return std::get<int>(get(cfg_motionTriggers)); };
    bool getEnableNTP() { return std::get<bool>(get(cfg_enableNTP)); };
    int getDoorUpdateAt() { return std::get<int>(get(cfg_doorUpdateAt)); };
    std::string getTimeZone() { return std::get<std::string>(get(cfg_timeZone)); };
    bool getSoftAPmode() { return std::get<bool>(get(cfg_softAPmode)); };
    bool getSyslogEn() { return std::get<bool>(get(cfg_syslogEn)); };
    std::string getSyslogIP() { return std::get<std::string>(get(cfg_syslogIP)); };
    int getSyslogPort() { return std::get<int>(get(cfg_syslogPort)); };
    int getVehicleThreshold() { return std::get<int>(get(cfg_vehicleThreshold)); };
    bool getLaserEnabled() { return std::get<bool>(get(cfg_laserEnabled)); };
    bool getLaserHomeKit() { return std::get<bool>(get(cfg_laserHomeKit)); };
    int getAssistDuration() { return std::get<int>(get(cfg_assistDuration)); };
};
extern userSettings *userConfig;

class nvRamClass
{
private:
    nvs_handle_t nvHandle;
    static nvRamClass *instancePtr;
    nvRamClass();

public:
    nvRamClass(const nvRamClass &obj) = delete;
    static nvRamClass *getInstance() { return instancePtr; };

    void checkStats();
    int32_t read(const std::string &constKey, const int32_t dflt);
    int32_t read(const std::string &constKey) { return read(constKey, 0); };
    std::string read(const std::string &constKey, const std::string &dflt);
    bool write(const std::string &constKey, const int32_t value, bool commit);
    bool write(const std::string &constKey, const int32_t value) { return write(constKey, value, true); };
    bool write(const std::string &constKey, const std::string &value, bool commit);
    bool write(const std::string &constKey, const std::string &value) { return write(constKey, value, true); };
    bool writeBlob(const std::string &constKey, const char *value, size_t size, bool commit);
    bool writeBlob(const std::string &constKey, const char *value, size_t size) { return writeBlob(constKey, value, size, true); };
    bool readBlob(const std::string &constKey, char *value, size_t size);
    bool erase(const std::string &constKey);
    void erase();
};

extern nvRamClass *nvRam;