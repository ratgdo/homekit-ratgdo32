#include "utilities.h"
/****************************************************************************
 * RATGDO HomeKit for ESP32
 * https://ratcloud.llc
 * https://github.com/PaulWieland/ratgdo
 *
 * Copyright (c) 2024-25 David A Kerr... https://github.com/dkerr64/
 * All Rights Reserved.
 * Licensed under terms of the GPL-3.0 License.
 *
 */
#ifdef RATGDO32_DISCO
// C/C++ language includes
#include <bitset>

// Arduino includes
#include <Wire.h>
#include <vl53l4cx_class.h>
#include "Ticker.h"

// RATGDO project includes
#include "ratgdo.h"
#include "led.h"
#include "vehicle.h"
#include "homekit.h"
#include "config.h"

// Logger tag
static const char *TAG = "ratgdo-vehicle";
bool vehicle_setup_done = false;
bool vehicle_setup_error = false;

VL53L4CX distanceSensor(&Wire, SENSOR_SHUTDOWN_PIN);

static const int32_t MIN_DISTANCE = 25; // ignore bugs crawling on the distance sensor

int16_t vehicleDistance = 0;
int16_t vehicleThresholdDistance = 1000; // set by user
char vehicleStatus[16] = "Away";         // or Arriving or Departing or Parked
bool vehicleStatusChange = false;

static bool vehicleDetected = false;
static bool vehicleArriving = false;
static bool vehicleDeparting = false;
_millis_t lastVehicleChangeAt = 0;
static _millis_t presence_timer = 0; // to be set by door open action
static _millis_t vehicle_motion_timer = 0;

static constexpr uint32_t PRESENCE_DETECT_DURATION = (5 * 60 * 1000); // how long to calculate presence after door state change
// increasing these values increases reliability but also increases detection time
static constexpr uint32_t PRESENCE_DETECTION_ON_THRESHOLD = 5; // Minimum percentage of valid samples required to detect vehicle
static constexpr uint32_t PRESENCE_DETECTION_OFF_DEBOUNCE = 2; // The number of consecutive iterations that must be 0 before clearing vehicle detected state
static constexpr int32_t VEHICLE_AVERAGE_OVER = 50;            // vehicle distance measure is averaged over last X samples
static std::bitset<256> distanceInRange;                       // the length of this bitset determines how many out of range readings are required for presence detection to change states
void calculatePresence(int32_t distance);

void setup_vehicle()
{
    VL53L4CX_Error rc = VL53L4CX_ERROR_NONE;

    if (vehicle_setup_done || vehicle_setup_error)
        return;

    ESP_LOGI(TAG, "=== Setup VL53L4CX time-of-flight sensor ===");

    if (!Wire.begin(SENSOR_SDA_PIN, SENSOR_SCL_PIN))
    {
        ESP_LOGE(TAG, "VL53L4CX pin setup failed");
        vehicle_setup_error = true;
        return;
    }
    distanceSensor.begin();
    distanceSensor.VL53L4CX_Off();
    rc = distanceSensor.InitSensor(0x59);
    if (rc != VL53L4CX_ERROR_NONE)
    {
        ESP_LOGE(TAG, "VL53L4CX failed to initialize error: %d", rc);
        vehicle_setup_error = true;
        return;
    }
    rc = distanceSensor.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_LONG);
    if (rc != VL53L4CX_ERROR_NONE)
    {
        ESP_LOGE(TAG, "VL53L4CX_SetDistanceMode error: %d", rc);
        vehicle_setup_error = true;
        return;
    }
    rc = distanceSensor.VL53L4CX_StartMeasurement();
    if (rc != VL53L4CX_ERROR_NONE)
    {
        ESP_LOGE(TAG, "VL53L4CX_StartMeasurement error: %d", rc);
        vehicle_setup_error = true;
        return;
    }

    garage_door.has_distance_sensor = true;
    nvRam->write(nvram_has_distance, 1);
    vehicleThresholdDistance = userConfig->getVehicleThreshold() * 10; // convert centimeters to millimeters
    enable_service_homekit_vehicle(userConfig->getVehicleHomeKit());
    vehicle_setup_done = true;
}

void vehicle_loop()
{
    if (!vehicle_setup_done)
        return;

    uint8_t dataReady = 0;
    VL53L4CX_Error err = VL53L4CX_ERROR_NONE;
    if (((err = distanceSensor.VL53L4CX_GetMeasurementDataReady(&dataReady)) == VL53L4CX_ERROR_NONE) && (dataReady > 0))
    {
        VL53L4CX_MultiRangingData_t distanceData;
        if ((err = distanceSensor.VL53L4CX_GetMultiRangingData(&distanceData)) == VL53L4CX_ERROR_NONE)
        {
            if (distanceData.NumberOfObjectsFound > 0)
            {
                int32_t distance = -1;
                // Multiple objects could be found. During testing if I wave my hand in front of the
                // sensor I get two distances... that of my hand, and that of the background.
                // We will only record the furthest away.
                for (int i = 0; i < distanceData.NumberOfObjectsFound; i++)
                {
                    // In testing I am seeing range status of 0, 1, 4, 7 and 12.  These represent
                    // 0:  VL53L4CX_RANGESTATUS_RANGE_VALID
                    // 1:  VL53L4CX_RANGESTATUS_SIGMA_FAIL
                    // 4:  VL53L4CX_RANGESTATUS_OUTOFBOUNDS_FAIL
                    // 7:  VL53L4CX_RANGESTATUS_WRAP_TARGET_FAIL
                    // 12: VL53L4CX_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL
                    // Documentation also suggests that valid data can be returned with:
                    // 3:  VL53L4CX_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED
                    // 6:  VL53L4CX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL
                    int32_t reportedRange = static_cast<int32_t>(distanceData.RangeData[i].RangeMilliMeter);
                    switch (distanceData.RangeData[i].RangeStatus)
                    {
                    case VL53L4CX_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL:
                        // Unusual, but docs say that range data is valid.
                        ESP_LOGV(TAG, "Unusual VL53L4CX Range Status: %d, Range: %dmm", distanceData.RangeData[i].RangeStatus, reportedRange);
                        // fall through...
                    case VL53L4CX_RANGESTATUS_RANGE_VALID:
                        // The Range is valid.
                    case VL53L4CX_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED:
                        // Target is below minimum detection threshold.
                    case VL53L4CX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL:
                        // The Range is valid but the wraparound check has not been done.
                        distance = std::max(distance, reportedRange);
                        break;
                    case VL53L4CX_RANGESTATUS_OUTOFBOUNDS_FAIL:
                        // Target below threshold... we will ignore.
                        ESP_LOGV(TAG, "Vehicle distance out of bounds: %dmm", reportedRange);
                        break;
                    case VL53L4CX_RANGESTATUS_RANGE_INVALID:
                        // Typically a negative value... we will ignore.
                        ESP_LOGV(TAG, "Vehicle distance range invalid: %dmm", reportedRange);
                        break;
                    case VL53L4CX_RANGESTATUS_SIGMA_FAIL:
                        // Sigma fail indicates sensor has low confidence in the range value returned.  Sensor may be pointed at glass.
                        ESP_LOGW(TAG, "Vehicle distance sensor sigma fail. Sensor may be pointing at glass, try repositioning: %dmm", reportedRange);
                        distance = std::max(distance, reportedRange);
                        break;
                    case VL53L4CX_RANGESTATUS_WRAP_TARGET_FAIL:
                        // Wrapped target - no matching phase in other VCSEL period timing
                        ESP_LOGV(TAG, "Vehicle distance wrap target fail: %dmm", reportedRange);
                        break;
                    default:
                        ESP_LOGE(TAG, "Unhandled VL53L4CX Range Status: %d, Range: %dmm", distanceData.RangeData[i].RangeStatus, reportedRange);
                        break;
                    }
                }
                // If no distance is set in above switch statement, we calculate presence based on the most recent set.
                calculatePresence(distance);
            }
            // And start the sensor measuring again...
            distanceSensor.VL53L4CX_ClearInterruptAndStartMeasurement();
        }
        else
        {
            ESP_LOGE(TAG, "VL53L4CX_GetMultiRangingData reports error: %d", err);
        }
    }
    else
    {
        if (err)
            ESP_LOGE(TAG, "VL53L4CX_GetMeasurementDataReady reports error: %d", err);
    }

    _millis_t current_millis = _millis();
    // Vehicle Arriving Clear Timer
    if (vehicleArriving && (current_millis - vehicle_motion_timer) > MOTION_TIMER_DURATION)
    {
        vehicleArriving = false;
        strlcpy(vehicleStatus, vehicleDetected ? "Parked" : "Away", sizeof(vehicleStatus));
        vehicleStatusChange = true;
        ESP_LOGI(TAG, "Vehicle %s at %s", vehicleStatus, timeString());
        notify_homekit_vehicle_arriving(false);
    }
    // Vehicle Departing Clear Timer
    if (vehicleDeparting && (current_millis - vehicle_motion_timer) > MOTION_TIMER_DURATION)
    {
        vehicleDeparting = false;
        strlcpy(vehicleStatus, vehicleDetected ? "Parked" : "Away", sizeof(vehicleStatus));
        vehicleStatusChange = true;
        ESP_LOGI(TAG, "Vehicle %s at %s", vehicleStatus, timeString());
        notify_homekit_vehicle_departing(false);
    }
}

void setArriveDepart(bool vehiclePresent)
{
    static bool lastVehiclePresent = false;
    static bool doneOnce = false;

    if (doneOnce && vehiclePresent == lastVehiclePresent)
        return;

    // Only continue if the vehicle presence state has changed, or we don't know previous state.
    lastVehiclePresent = vehiclePresent;
    doneOnce = true;
    if (vehiclePresent)
    {
        if (!vehicleArriving)
        {
            vehicleArriving = true;
            vehicleDeparting = false;
            vehicle_motion_timer = lastVehicleChangeAt;
            strlcpy(vehicleStatus, "Arriving", sizeof(vehicleStatus));
            if (userConfig->getAssistDuration() > 0)
                laser.flash(userConfig->getAssistDuration() * 1000);
            vehicleStatusChange = true;
            ESP_LOGI(TAG, "Vehicle %s at %s", vehicleStatus, timeString());
            notify_homekit_vehicle_arriving(true);
        }
    }
    else
    {
        if (!vehicleDeparting)
        {
            vehicleArriving = false;
            vehicleDeparting = true;
            vehicle_motion_timer = lastVehicleChangeAt;
            strlcpy(vehicleStatus, "Departing", sizeof(vehicleStatus));
            vehicleStatusChange = true;
            ESP_LOGI(TAG, "Vehicle %s at %s", vehicleStatus, timeString());
            notify_homekit_vehicle_departing(true);
        }
    }
}

void calculatePresence(int32_t distance)
{
    if (distance < MIN_DISTANCE)
        return;

    // Test for change in vehicle presence
    bool priorVehicleDetected = vehicleDetected;

    distanceInRange <<= 1;
    distanceInRange.set(0, distance <= vehicleThresholdDistance);
    uint32_t percent = distanceInRange.count() * 100 / distanceInRange.size();
    static uint32_t last_percent = UINT32_MAX;
    static uint32_t off_counter = 0;

    if (percent >= PRESENCE_DETECTION_ON_THRESHOLD)
    {
        vehicleDetected = true;
    }
    else if (percent == 0 && vehicleDetected)
    {
        off_counter++;
        ESP_LOGV(TAG, "Vehicle distance off_counter: %d", off_counter);
        if (off_counter / distanceInRange.size() >= PRESENCE_DETECTION_OFF_DEBOUNCE)
        {
            off_counter = 0;
            vehicleDetected = false;
        }
    }

    if (percent != last_percent)
    {
        last_percent = percent;
        off_counter = 0;
        ESP_LOGV(TAG, "Vehicle distance in-range: %d%%", percent);
    }

    // calculate average over sample size to smooth out changes
    static double average = 0;
    static int32_t count = 0;
    if (count < VEHICLE_AVERAGE_OVER)
        count++;
    // must use double float math, integer math does not work
    average = average + (static_cast<double>(distance) - average) / count;
    // convert from millimeters to centimeters
    vehicleDistance = static_cast<int16_t>(std::round(average / 10));
    static int16_t lastDistance = 0;
    static uint32_t lastChange = 0;
    lastChange++;
    if (vehicleDistance != lastDistance)
    {

        ESP_LOGD(TAG, "Vehicle distance: %dcm (average over %d samples, last changed %d samples ago), current measured: %dcm", vehicleDistance, count, lastChange, distance / 10);
        lastDistance = vehicleDistance;
        lastChange = 0;
    }

    if (vehicleDetected != priorVehicleDetected)
    {
        // if change occurs with arrival/departure window then record motion,
        // presence timer is set when door opens or closes.
        lastVehicleChangeAt = _millis();
        if (presence_timer && ((lastVehicleChangeAt - presence_timer) < PRESENCE_DETECT_DURATION))
        {
            presence_timer = 0;
            setArriveDepart(vehicleDetected);
        }
        else
        {
            strlcpy(vehicleStatus, vehicleDetected ? "Parked" : "Away", sizeof(vehicleStatus));
            ESP_LOGI(TAG, "Vehicle %s at %s", vehicleStatus, timeString());
        }
        vehicleStatusChange = true;
        notify_homekit_vehicle_occupancy(vehicleDetected);
    }
}

// if notified of door opening, set timeout during which we check for arriving/departing vehicle (looking forward)
void doorOpening()
{
    if (!vehicle_setup_done)
        return;

    presence_timer = _millis();
}

// if notified of door closing, check for arrived/departed vehicle within time window (looking back)
void doorClosing()
{
    if (!vehicle_setup_done)
        return;

    presence_timer = _millis();
    // On door closing, the vehicle status change could already have happened...
    if (lastVehicleChangeAt && ((presence_timer - lastVehicleChangeAt) < PRESENCE_DETECT_DURATION))
    {
        setArriveDepart(vehicleDetected);
    }
}
#endif // RATGDO32_DISCO
