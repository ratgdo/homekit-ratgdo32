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
#include <stdint.h>

// Arduino includes
#include <WiFiUdp.h>

// RATGDO project includes
#include "ratgdo.h"
#include "log.h"
#include "config.h"
#include "utilities.h"
#include "secplus2.h"
// #include "comms.h"
#include "web.h"

// Logger tag
static const char *TAG = "ratgdo-logger";

#ifndef UNIT_TEST
void print_packet(uint8_t *pkt)
{
    RINFO(TAG, "decoded packet: [%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X]",
          pkt[0], pkt[1], pkt[2], pkt[3], pkt[4], pkt[5], pkt[6], pkt[7], pkt[8], pkt[9],
          pkt[10], pkt[11], pkt[12], pkt[13], pkt[14], pkt[15], pkt[16], pkt[17], pkt[18]);
}
#else  // UNIT_TEST
void print_packet(uint8_t pkt[SECPLUS2_CODE_LEN]) {}
#endif // UNIT_TEST

#ifdef LOG_MSG_BUFFER
// Construct the singleton object for logger access
LOG *LOG::instancePtr = new LOG();
LOG *ratgdoLogger = LOG::getInstance();

void logToSyslog(char *message);
bool syslogEn = false;
uint16_t syslogPort = 514;
char syslogIP[16] = "";
WiFiUDP syslog;
bool suppressSerialLog = false;

// Constructor for LOG class
LOG::LOG()
{
    logMutex = xSemaphoreCreateRecursiveMutex();
    msgBuffer = (logBuffer *)malloc(sizeof(logBuffer));
    // Fill the buffer with space chars... because if we crash and dump buffer before it fills
    // up, we want blank space not garbage!
    memset(msgBuffer->buffer, 0x20, sizeof(msgBuffer->buffer));
    msgBuffer->wrapped = 0;
    msgBuffer->head = 0;
    lineBuffer = (char *)malloc(LINE_BUFFER_SIZE);
}

void LOG::logToBuffer(const char *fmt, ...)
{
    if (!lineBuffer)
    {
        static char buf[LINE_BUFFER_SIZE];
        // parse the format string into lineBuffer
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, LINE_BUFFER_SIZE, fmt, args);
        va_end(args);
        // print line to the serial port
        if (!suppressSerialLog)
            Serial.print(buf);
        return;
    }

    xSemaphoreTakeRecursive(logMutex, portMAX_DELAY);
    // parse the format string into lineBuffer
    va_list args;
    va_start(args, fmt);
    vsnprintf(lineBuffer, LINE_BUFFER_SIZE, fmt, args);
    va_end(args);
    // print line to the serial port
    if (!suppressSerialLog)
        Serial.print(lineBuffer);

    // copy the line into the message save buffer
    size_t len = strlen(lineBuffer);
    size_t available = sizeof(msgBuffer->buffer) - msgBuffer->head;
    memcpy(&msgBuffer->buffer[msgBuffer->head], lineBuffer, min(available, len));
    if (available < len)
    {
        // we wrapped on the available buffer space
        msgBuffer->wrapped = 1;
        msgBuffer->head = len - available;
        memcpy(msgBuffer->buffer, &lineBuffer[available], msgBuffer->head);
    }
    else
    {
        msgBuffer->head += len;
    }
    msgBuffer->buffer[msgBuffer->head] = 0; // null terminate
    // send it to subscribed browsers
    SSEBroadcastState(lineBuffer, LOG_MESSAGE);
    logToSyslog(lineBuffer);
    xSemaphoreGiveRecursive(logMutex);
    return;
}

void LOG::saveMessageLog()
{
    RINFO(TAG, "Save message log buffer to NVRAM");
    xSemaphoreTakeRecursive(logMutex, portMAX_DELAY);
    // We start by rotating the circular buffer so it is all in order.
    uint16_t first = 0;
    uint16_t head = (msgBuffer->head + 1) % sizeof(msgBuffer->buffer); // adjust for null terminator.
    // uint16_t head = msgBuffer->head;
    uint16_t next = head;
    while (first != next)
    {
        std::swap(msgBuffer->buffer[first++], msgBuffer->buffer[next++]);
        if (next == sizeof(msgBuffer->buffer))
            next = head;
        else if (first == head)
            head = next;
    }
    //  reset the index
    msgBuffer->head = 0;
    msgBuffer->wrapped = 0;
    nvRam->writeBlob(nvram_messageLog, msgBuffer->buffer, sizeof(msgBuffer->buffer));
    xSemaphoreGiveRecursive(logMutex);
}

void LOG::printSavedLog(Print &outputDev)
{
    RINFO(TAG, "Print saved log from NVRAM");
    char *buf = (char *)malloc(sizeof(msgBuffer->buffer));
    if (buf)
    {
        nvRam->readBlob(nvram_messageLog, buf, sizeof(msgBuffer->buffer));
        outputDev.print(buf);
        free(buf);
    }
}

#ifdef ESP8266
// These are defined in the linker script, and filled in by the elf2bin.py util
extern "C" uint32_t __crc_len;
extern "C" uint32_t __crc_val;
#endif

void LOG::printMessageLog(Print &outputDev)
{
    xSemaphoreTakeRecursive(logMutex, portMAX_DELAY);
#ifdef NTP_CLIENT
    if (enableNTP && clockSet)
    {
        outputDev.printf("Server time: %s\n", timeString());
    }
#endif
    outputDev.printf("Server uptime (ms): %lu\n", millis());
    outputDev.printf("Firmware version: %s\n", AUTO_VERSION);
#ifdef ESP8266
    outputDev.write("Flash CRC: 0x");
    outputDev.println(__crc_val, 16);
    outputDev.write("Flash length: ");
    outputDev.println(__crc_len);
#endif
    outputDev.printf("Free heap: %lu\n", free_heap);
    outputDev.printf("Minimum heap: %lu\n\n", min_heap);
    if (msgBuffer)
    {
        // head points to a zero (null terminator of previous log line) which we need to skip.
        size_t start = (msgBuffer->head + 1) % sizeof(msgBuffer->buffer);
        if (msgBuffer->wrapped != 0)
        {
            outputDev.write(&msgBuffer->buffer[start], sizeof(msgBuffer->buffer) - start);
        }
        outputDev.print(msgBuffer->buffer); // assumes null terminated
        // outputDev.write(msgBuffer->buffer, start);
    }
    xSemaphoreGiveRecursive(logMutex);
}

/****************************************************************************
 * Syslog
 */
#define SYSLOG_LOCAL0 16
#define SYSLOG_EMERGENCY 0
#define SYSLOG_ALERT 1
#define SYSLOG_CRIT 2
#define SYSLOG_ERROR 3
#define SYSLOG_WARN 4
#define SYSLOG_NOTICE 5
#define SYSLOG_INFO 6
#define SYSLOG_DEBUG 7
#define SYSLOG_NIL "-"
#define SYSLOG_BOM "\xEF\xBB\xBF"

void logToSyslog(char *message)
{
    if (!syslogEn || !WiFi.isConnected())
        return;

    uint8_t PRI = SYSLOG_LOCAL0 * 8;
    if (*message == '>')
        PRI += SYSLOG_INFO;
    else if (*message == '!')
        PRI += SYSLOG_ERROR;

    char *app_name;
    char *msg;

    app_name = strtok(message, "]");
    while (*app_name == ' ')
        app_name++;
    app_name = strtok(NULL, ":");
    while (*app_name == ' ')
        app_name++;
    msg = strtok(NULL, "\r\n");
    while (*msg == ' ')
        msg++;

    syslog.beginPacket(syslogIP, syslogPort);
    // Use RFC5424 Format
    syslog.printf("<%u>1 ", PRI); // PRI code
#if defined(NTP_CLIENT) && defined(USE_NTP_TIMESTAMP)
    syslog.print((enableNTP && clockSet) ? timeString(0, true) : SYSLOG_NIL);
#else
    syslog.print(SYSLOG_NIL); // Time - let the syslog server insert time
#endif
    syslog.print(" ");
    syslog.print(device_name_rfc952); // hostname
    syslog.print(" ");
    syslog.print(app_name);     // application name
    syslog.printf(" 0");        // process ID
    syslog.print(" " SYSLOG_NIL // message ID
                 " " SYSLOG_NIL // structured data
#ifdef USE_UTF8_BOM
                 " " SYSLOG_BOM); // BOM - indicates UTF-8 encoding
#else
                 " "); // No BOM
#endif
    syslog.print(msg); // message
    syslog.endPacket();
}

#ifdef ENABLE_CRASH_LOG
// TODO handle crashdump log
void crashCallback()
{
    if (ratgdoLogger->msgBuffer && ratgdoLogger->logMessageFile)
    {
        // ratgdoLogger->logMessageFile.truncate(0);
        ratgdoLogger->logMessageFile.seek(0, fs::SeekSet);
        ratgdoLogger->logMessageFile.println();
        ratgdoLogger->printMessageLog(ratgdoLogger->logMessageFile);
        ratgdoLogger->logMessageFile.close();
    }
    // We may not have enough memory to open the file and save the code
    // save_rolling_code();
}
#endif

#endif // LOG_MSG_BUFFER
