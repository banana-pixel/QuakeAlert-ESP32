/**
 * QuakeAlert ESP32 - Shared State
 * Global variables and structures used across modules.
 */

#ifndef STATE_H
#define STATE_H

#include <Arduino.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include "config.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ========================================
// CREDENTIALS (from secrets.h)
// ========================================
extern const char* mqtt_server;
extern const int mqtt_port;
extern const char* mqtt_user;
extern const char* mqtt_password;

// ========================================
// NTP
// ========================================
extern const char* ntpServer;
extern const long gmtOffset_sec;
extern const int daylightOffset_sec;

// ========================================
// GLOBAL OBJECTS
// ========================================
extern MPU6050 mpu;
extern TaskHandle_t SensorTask;
extern WiFiClient espClient;
extern PubSubClient mqttClient;
extern SemaphoreHandle_t i2cMutex;

// ========================================
// STRUCTURES
// ========================================
struct EventReport {
    bool ready;
    float maxPga;
    float duration;
    unsigned long timestamp;
    bool processed;
};

// ========================================
// MUTEXES
// ========================================
extern portMUX_TYPE reportMux;
extern portMUX_TYPE eventTriggerMux;

// ========================================
// VOLATILE / SYNC STATE
// ========================================
extern volatile EventReport pendingReport;
extern volatile bool MPUInterrupt;
extern volatile bool eventTriggered;
extern volatile bool rebootRequestReceived;
extern volatile unsigned long lastLoopHeartbeat;

// ========================================
// SENSOR STATE
// ========================================
extern bool DMPReady;
extern uint8_t devStatus;
extern uint16_t packetSize;
extern uint8_t FIFOBuffer[64];
extern Quaternion q;
extern VectorInt16 aa;
extern VectorInt16 aaReal;
extern VectorFloat gravity;

// ========================================
// APP STATE
// ========================================
extern String StationID;
extern String lokasiAlat;
extern bool potentialEvent;
extern bool eventInProgress;
extern bool alertSent;
extern float pga;
extern bool ledState;
extern bool isNtpSynced;
extern bool startupMessageSent;

// ========================================
// TIMERS
// ========================================
extern unsigned long potentialEventTime;
extern unsigned long eventStartTime;
extern unsigned long lastReportTime;
extern unsigned long lastBlinkTime;
extern unsigned long lastNtpSync;
extern unsigned long lastNtpAttempt;
extern unsigned long lastWifiCheck;
extern unsigned long lastMqttAttempt;
extern unsigned long lastHeartbeat;

// ========================================
// COUNTERS & METRICS
// ========================================
extern uint32_t mpuOverflowCount;
extern uint32_t totalEventsDetected;
extern int mpuErrorCounter;
extern uint32_t minHeapSeen;
extern unsigned long lastHeapCheck;
extern int bootCount;
extern Preferences preferences;

// ========================================
// LAST EVENT INFO
// ========================================
extern char lastPgaStr[16];
extern String lastIntensity;
extern String lastEventTime;

// ========================================
// MOVING AVERAGE STATE
// ========================================
extern float readings[MOVING_AVERAGE_WINDOW_SIZE];
extern int readIndex;
extern float total;
extern float averageMagnitude;
extern bool isFirstReading;

// ========================================
// GEO
// ========================================
extern float stationLat;
extern float stationLon;

#endif  // STATE_H
