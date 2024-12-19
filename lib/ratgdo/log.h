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
#pragma once

// C/C++ language includes

// Arduino includes
#include <Arduino.h>

// RATGDO project includes
#include "HomeSpan.h"

void print_packet(uint8_t *pkt);

// #define LOG_MSG_BUFFER

#ifdef LOG_MSG_BUFFER

#define CRASH_LOG_MSG_FILE "/crash_log"
#define REBOOT_LOG_MSG_FILE "/reboot_log"

#if defined(MMU_IRAM_HEAP) || !defined(ESP8266)
// This can be large, but not too large.  IRAM heap is approx 18KB, we also need
// space for other data in here, so during development monitor logs and adjust
// this smaller if necessary.  IRAM malloc's are all done during startup.
#define LOG_BUFFER_SIZE 2048
#else
#define LOG_BUFFER_SIZE 1024
#endif
#define LINE_BUFFER_SIZE 256

#ifdef ENABLE_CRASH_LOG
void crashCallback();
#endif

extern bool syslogEn;
extern uint16_t syslogPort;
extern char syslogIP[16];
extern bool suppressSerialLog;

typedef struct logBuffer
{
    uint16_t wrapped;                 // two bytes
    uint16_t head;                    // two bytes
    char buffer[LOG_BUFFER_SIZE - 4]; // sized so whole struct is LOG_BUFFER_SIZE bytes
} logBuffer;

class LOG
{
private:
    char *lineBuffer = NULL; // Buffer for single message line
    SemaphoreHandle_t logMutex = NULL;

    static LOG *instancePtr;
    LOG();

public:
    logBuffer *msgBuffer = NULL; // Buffer to save log messages as they occur

    LOG(const LOG &obj) = delete;
    static LOG *getInstance() { return instancePtr; }

    void logToBuffer(const char *fmt, ...);
    void printSavedLog(Print &outDevice = Serial);
    void printMessageLog(Print &outDevice = Serial);
    void saveMessageLog();
};

extern LOG *ratgdoLogger;

#define RATGDO_PRINTF(message, ...) ratgdoLogger->logToBuffer(PSTR(message), ##__VA_ARGS__)

#define RINFO(tag, message, ...) RATGDO_PRINTF(">>> [%7lu] %s: " message "\n", millis(), tag, ##__VA_ARGS__)
#define RERROR(tag, message, ...) RATGDO_PRINTF("!!! [%7lu] %s: " message "\n", millis(), tag, ##__VA_ARGS__)
#else // LOG_MSG_BUFFER

#ifndef UNIT_TEST

#define RINFO(tag, message, ...) LOG0(">>> [%7lu] %s: " message "\n", millis(), tag, ##__VA_ARGS__)
#define RERROR(tag, message, ...) LOG0("!!! [%7lu] %s: " message "\n", millis(), tag, ##__VA_ARGS__)

#else // UNIT_TEST

#include <stdio.h>
#define RINFO(tag, message, ...) printf(">>> %s: " message "\n", tag, ##__VA_ARGS__)
#define RERROR(tag, message, ...) printf("!!! %s: " message "\n", tag, ##__VA_ARGS__)

#endif // UNIT_TEST

#endif // LOG_MSG_BUFFER
