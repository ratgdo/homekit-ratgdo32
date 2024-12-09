/****************************************************************************
 * RATGDO HomeKit for ESP32
 * https://ratcloud.llc
 * https://github.com/PaulWieland/ratgdo
 *
 * Copyright (c) 2023-24 David A Kerr... https://github.com/dkerr64/
 * All Rights Reserved.
 * Licensed under terms of the GPL-3.0 License.
 *
 */

// C/C++ language includes
// none

// Arduino includes
#include <Wire.h>
#include <vl53l4cx_class.h>

// RATGDO project includes
#include "ratgdo.h"
#include "led.h"
#include "distance.h"

// Logger tag
static const char *TAG = "ratgdo-distance";
bool distance_setup_done = false;

VL53L4CX distanceSensor(&Wire, SHUTDOWN_PIN);
int16_t vehicleDistance = 0;

void setup_distance()
{
    RINFO(TAG, "=== Starting VL5314CX time-of-flight sensor ===");

    Wire.begin(19, 18);
    distanceSensor.begin();
    distanceSensor.InitSensor(0x59);
    distanceSensor.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_LONG);
    distanceSensor.VL53L4CX_StartMeasurement();

    distance_setup_done = true;
}

void distance_loop()
{
    if (!distance_setup_done)
        return;

    uint8_t dataReady = 0;
    int status = distanceSensor.VL53L4CX_GetMeasurementDataReady(&dataReady);
    if (status)
    {
        RERROR(TAG, "Error return from VL53L4CX_GetMeasurementDataReady: %d", status);
        return;
    }

    if (dataReady == 0)
        return;

    // Got this far then something to process
    VL53L4CX_MultiRangingData_t distanceData;
    VL53L4CX_MultiRangingData_t *pDistanceData = &distanceData;
    int objCount = 0;
    int16_t distance = 0;
    int16_t rangeHigh = vehicleDistance * 11 / 10; // 110% of existing distance
    int16_t rangeLow = vehicleDistance * 8 / 10;   // 90% of existing distance

    status = distanceSensor.VL53L4CX_GetMultiRangingData(pDistanceData);
    objCount = pDistanceData->NumberOfObjectsFound;
    vehicleDistance = (objCount == 0) ? -1 : pDistanceData->RangeData[objCount - 1].RangeMilliMeter;
    if ((vehicleDistance > 0) && (vehicleDistance < rangeLow || vehicleDistance > rangeHigh))
    {
        int i = objCount - 1;
        led.flash();
        RINFO(TAG, "status: %d, D: %dmm, Signal: %f Mcps, Ambient: %f Mcps",
              pDistanceData->RangeData[i].RangeStatus,
              pDistanceData->RangeData[i].RangeMilliMeter,
              (float)pDistanceData->RangeData[i].SignalRateRtnMegaCps / 65536.0,
              (float)pDistanceData->RangeData[i].AmbientRateRtnMegaCps / 65536.0);
    }

    if (status == 0)
    {
        status = distanceSensor.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
}
