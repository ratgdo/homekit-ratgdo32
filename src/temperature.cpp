/****************************************************************************
 * RATGDO HomeKit for ESP32
 * https://ratcloud.llc
 * https://github.com/PaulWieland/ratgdo
 *
 * Copyright (c) 2023-26 David A Kerr... https://github.com/dkerr64/
 * All Rights Reserved.
 * Licensed under terms of the GPL-3.0 License.
 *
 * Contributions acknowledged from
 * GelidusResearch... https://github.com/GelidusResearch
 *
 */
#ifdef USE_DHT22
// Arduino includes
#include <DHT.h>

// RATGDO project includes
#include "ratgdo.h"
#include "config.h"
#include "homekit.h"
#include "temperature.h"

#define TEMPERATURE_READ_INTERVAL (60 * 1000) // 60 seconds

// Latest reading in Celsius, NAN until the sensor has been read
float temperature = NAN;
float humidity = NAN;

void temperature_loop()
{
    if (dht22Pin < 0)
        return;

    static _millis_t lastRead = 0;
    _millis_t currentMillis = _millis();
    // Read sensor initially (lastRead == 0) or after 60 seconds
    if (lastRead == 0 || (currentMillis - lastRead >= TEMPERATURE_READ_INTERVAL))
    {
        lastRead = currentMillis;
        // Constructed on first use, only when a pin is set
        static DHT dht(dht22Pin, DHT22);
        dht.begin();
        temperature = dht.readTemperature();
        humidity = dht.readHumidity();

        if (!isnan(temperature) && !isnan(humidity))
            notify_homekit_temperature_humidity(temperature, humidity);
    }
}
#endif // USE_DHT22
