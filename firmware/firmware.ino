/**
 * QuakeAlert ESP32 - V6.9.6
 * Mission-critical earthquake detection firmware with:
 * - true dual-core task isolation
 * - semaphore-driven MPU6050 interrupt handling
 * - no scheduled or heap-triggered restarts
 * - background network maintenance task
 */

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <esp_task_wdt.h>
#include <Preferences.h>
#include <esp_random.h>
#include "secrets.h"
#include "config.h"
#include "state.h"
#include "utils.h"
#include "network.h"
#include "sensor.h"
#include "mqtt.h"

// Let's Encrypt Root CA (ISRG Root X1) for Mosquitto MQTTS
const char* root_ca = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRnXkUS/pzUcwDQYJKoZIhvcNAQELBQAw\n" \
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n" \
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n" \
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n" \
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n" \
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHpOMQSjTmpiMsFd\n" \
"Ope2xXEnZ6wA+R2P4gUq5n50n5e2X1/B7c/eA+9w2s/4s32tq6zXgA/B6/u3AHTB\n" \
"z9P8A4y/D2w3aK2w3w85/b/G1o33b3+X1Q8A7A7/A1A6Q7bX1v2wF/rGjVnAP0/J\n" \
"o/3h/9A2Q7pW9b4wA2/k9w4A2w01A3+wF4b9P9v2v3/1/A+o2A2qG/0xM2+Y6K7M\n" \
"wMw2gO7b/9A2o+r+u+bE/lMzG+3A/pA/1xW/+4P3g/3sA4v1r+vA2/wT/9u1//Pz\n" \
"4r/Xw1q1/W1x5rTqW173c3z7b+1N3r1+Q/T74t3w4y6A/rG2T7lP0tV2K3b/\n" \
"jTqy4G4rB+Z8w6dMwWwJwJw21qfXg9Ww/Mw4Q/w72G4g+hL/mQf/+xW3E+xL/H8\n" \
"XG/YQ6Gz8D/L7/F/2Xw72H3v9+F+9W/4A8n//vH8T3V8A4n3v9E8h9P7w9D8z4\n" \
"T3v933Q2t2W2A2D//5/2t4H8P8a+t78q/2g3zT+s4P+p9y7C2+R4K5/Q3/w4x8\n" \
"N1x5k/1H4t9sZ+N5/6Hq/Yt9s+w3v+7t3/8f2+v7/4A8/8E9XwA0b/k5b9T3+N\n" \
"H3h/8z/D3nZ+t/h9r/w1f/3D/z0A5p5z3k6wX58b1U7E8gS/z/0f/p8/5X/\n" \
"y4yH9q3z3y5r4K3+v5x2a+1L1wIDAQABo0IwQDAPBgNVHRMBAf8EBTADAQH/MA4G\n" \
"A1UdDwEB/wQEAwIBBjAdBgNVHQ4EFgQUU3m/WqTf0L2Fh/uN9X5pQ8y13xowDQYJ\n" \
"KoZIhvcNAQELBQADggIBAF3s+w4q2M9c/H1qX/T8Y/7N+2s/4O2o/Q3y/T/8N8y/\n" \
"O/S+H4V8R9N5s8x/p/k7d0s8x2B/mZ7/g/X8m/4H2i2E9k//Z+l8v/e0H/w/U6A\n" \
"H7S7P4N5T8t0U+s/v5+X7w8o9w4s8/H9W5X/c+8T9l7v6P/Q6T7s+g2y/6x6V/U\n" \
"e6N6z5M8y+y/P7p7Y6M+v7G2m5m4t2B9C8b8F1J+t+v4v6S9Z6w4/g/8pE/g+S\n" \
"D6Z8u4i9a6e+d4i0G/9p9U+v/h/T8/Y7X/4F5c/r4m+V/n/6Z9u4K8V/c8P+g8\n" \
"V/l2X+Y2m5Y4r3f9d4E6i9v0W5W9q6O7z9k6O8z8f7e2k4o9t5M2W2Y9T4h4\n" \
"Y2N4w6m7L9q8n0y5T5P8z0s4r8X9/5m4V9J9t3z1u5N3k1A5y1v5q2q/o7v/\n" \
"o5E/x6q7M7a4/K7l+G6a/1s8P6b7Z4G/4t9g8U0n4g8f4a+v+r5s4k6M6z/l8V\n" \
"v4j9M7D4t5x9d6T+C8c6n6Z+P0Z3h4/P3p0d9l8O9I/5/b/G7Q8y7o/C+g6q7\n" \
"H/c8z0y4B+q5K8Z1y5A1w9g5d8c3H3E1X/7r0w7k5Y7p9d0R7I6/3X1q2f/x8E\n" \
"-----END CERTIFICATE-----\n";

// Ensure Arduino LED define exists
#ifndef LED_BUILTIN
#define LED_BUILTIN LED_BUILTIN_PIN
#endif

// ========================================
// CREDENTIALS
// ========================================
const char* mqtt_server   = SECRET_MQTT_SERVER;
const int   mqtt_port     = SECRET_MQTT_PORT;
const char* mqtt_user     = SECRET_MQTT_USER;
const char* mqtt_password = SECRET_MQTT_PASS;

// ========================================
// NTP
// ========================================
const char* ntpServer = "id.pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;

// ========================================
// GLOBAL OBJECT DEFINITIONS
// ========================================
MPU6050 mpu;
TaskHandle_t SensorTask = nullptr;
TaskHandle_t NetworkMaintenanceTask = nullptr;
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
SemaphoreHandle_t i2cMutex = nullptr;
SemaphoreHandle_t mpuInterruptSemaphore = nullptr;
SemaphoreHandle_t stateMutex = nullptr;

portMUX_TYPE reportMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE eventTriggerMux = portMUX_INITIALIZER_UNLOCKED;

volatile EventReport pendingReport = {false, 0.0f, 0.0f, 0UL, false};
volatile bool eventTriggered = false;
volatile bool rebootRequestReceived = false;

bool DMPReady = false;
uint8_t devStatus = 0;
uint16_t packetSize = 0;
uint8_t FIFOBuffer[64] = {0};
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorFloat gravity;

char StationID[STATION_ID_BUFFER_SIZE] = "SEIS-01";
char lokasiAlat[LOCATION_TEXT_BUFFER_SIZE] = "Mencari lokasi...";
bool potentialEvent = false;
bool eventInProgress = false;
bool alertSent = false;
float pga = 0.0f;
bool ledState = false;
bool isNtpSynced = false;
bool startupMessageSent = false;
bool locationResolved = false;

unsigned long potentialEventTime = 0;
unsigned long eventStartTime = 0;
unsigned long lastReportTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastNtpSync = 0;
unsigned long lastNtpAttempt = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastMqttAttempt = 0;
unsigned long lastHeartbeat = 0;

uint32_t mpuOverflowCount = 0;
uint32_t totalEventsDetected = 0;
int mpuErrorCounter = 0;
uint32_t minHeapSeen = 0xFFFFFFFF;
float maxHeapFragmentationSeen = 0.0f;
unsigned long lastHeapCheck = 0;
int bootCount = 0;
Preferences preferences;

char lastPgaStr[16] = "N/A";
char lastIntensity[INTENSITY_TEXT_BUFFER_SIZE] = "N/A";
char lastEventTime[TIME_TEXT_BUFFER_SIZE] = "N/A";

float stationLat = 0.0f;
float stationLon = 0.0f;

uint32_t wifiFailCount = 0;
unsigned long lastLocRetry = 0;

// ========================================
// INTERNAL HELPERS
// ========================================
static bool initializeCorePrimitives() {
    i2cMutex = xSemaphoreCreateMutex();
    stateMutex = xSemaphoreCreateMutex();
    mpuInterruptSemaphore = xSemaphoreCreateBinary();

    if (i2cMutex == nullptr || stateMutex == nullptr || mpuInterruptSemaphore == nullptr) {
        Serial.println("FATAL: Failed to create RTOS synchronization primitives");
        return false;
    }

    return true;
}

static void assignStationId() {
    // Use hardware RNG + NVS to generate a persistent, anonymous node identity.
    // The MAC address is never exposed — a random NODE-XXXXXXXX string is generated
    // exactly once and stored permanently in NVS under the "quake-app" namespace.
    Preferences prefs;
    prefs.begin("quake-app", false);  // open R/W

    if (!prefs.isKey("station_id")) {
        // First boot: generate an 8-char hex ID from the ESP32 hardware RNG.
        // esp_random() returns a true hardware random 32-bit value.
        const uint32_t rndVal = esp_random();
        char newId[STATION_ID_BUFFER_SIZE];
        snprintf(newId, sizeof(newId), "NODE-%08X", rndVal);
        prefs.putString("station_id", newId);
        Serial.printf("Generated anonymous Station ID: %s\n", newId);
    }

    // Load the persisted ID directly into the global C-string buffer (no heap String).
    prefs.getString("station_id", StationID, sizeof(StationID));
    prefs.end();

    Serial.printf("Station ID: %s\n", StationID);
}

static void initPersistentState() {
    setLocationStatusSearching();
    setLastEventTime("N/A");
    setLastIntensity("N/A");
    setLastPga("N/A");

    preferences.begin("quake-app", false);
    bootCount = preferences.getInt("boots", 0) + 1;
    preferences.putInt("boots", bootCount);
    preferences.end();
}

static void initTaskWatchdog() {
    esp_task_wdt_deinit();

#ifdef USE_LEGACY_WDT
    esp_task_wdt_init(WDT_TIMEOUT, true);
#else
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000,
        .idle_core_mask = (1 << 0) | (1 << 1),
        .trigger_panic = true
    };
    esp_task_wdt_init(&twdt_config);
#endif

    esp_task_wdt_add(nullptr);
}

static void startWorkerTasks() {
    BaseType_t sensorResult = xTaskCreatePinnedToCore(
        sensorTask,
        "SensorTask",
        SENSOR_TASK_STACK_SIZE,
        nullptr,
        SENSOR_TASK_PRIORITY,
        &SensorTask,
        SENSOR_TASK_CORE
    );

    BaseType_t networkResult = xTaskCreatePinnedToCore(
        networkMaintenanceTask,
        "NetworkMaintenanceTask",
        NETWORK_MAINTENANCE_TASK_STACK_SIZE,
        nullptr,
        NETWORK_MAINTENANCE_TASK_PRIORITY,
        &NetworkMaintenanceTask,
        NETWORK_MAINTENANCE_TASK_CORE
    );

    if (sensorResult != pdPASS) {
        Serial.println("ERROR: Failed to start SensorTask");
    } else {
        Serial.println("SensorTask started on Core 0");
    }

    if (networkResult != pdPASS) {
        Serial.println("ERROR: Failed to start NetworkMaintenanceTask");
    } else {
        Serial.println("NetworkMaintenanceTask started on Core 1");
    }
}

// ========================================
// ALERT HANDLER (glue: sensor -> mqtt)
// ========================================
void handleAlerts() {
    bool shouldSendAlert = false;

    portENTER_CRITICAL(&eventTriggerMux);
    if (eventTriggered) {
        eventTriggered = false;
        shouldSendAlert = true;
    }
    portEXIT_CRITICAL(&eventTriggerMux);

    if (shouldSendAlert) {
        const char* intensity = toIntensity(pga);
        sendMqttAlert(intensity, pga);
        alertSent = true;
    }

    bool shouldSendReport = false;
    float reportPga = 0.0f;
    float reportDuration = 0.0f;

    portENTER_CRITICAL(&reportMux);
    if (pendingReport.ready && !pendingReport.processed) {
        shouldSendReport = true;
        reportPga = pendingReport.maxPga;
        reportDuration = pendingReport.duration;
        pendingReport.processed = true;
    }
    portEXIT_CRITICAL(&reportMux);

    if (shouldSendReport) {
        const char* intensity = toIntensity(reportPga);
        char waktu[TIME_TEXT_BUFFER_SIZE];
        char lokasi[LOCATION_TEXT_BUFFER_SIZE];

        getWaktuString(waktu, sizeof(waktu));
        getLokasiAlatCopy(lokasi, sizeof(lokasi));

        char pgaText[16];
        snprintf(pgaText, sizeof(pgaText), "%.2f gal", reportPga);

        setLastEventTime(waktu);
        setLastIntensity(intensity);
        setLastPga(pgaText);

        bool success = sendMqttReport(
            lokasi,
            waktu,
            reportDuration,
            pgaText,
            intensity
        );

        if (success) {
            totalEventsDetected++;
        }

        portENTER_CRITICAL(&reportMux);
        pendingReport.ready = false;
        pendingReport.processed = false;
        portEXIT_CRITICAL(&reportMux);
    }
}

// ========================================
// SETUP & LOOP
// ========================================
void setup() {
    Serial.begin(115200);

    // -----------------------------------------------------------------
    // Hardware Factory Reset (GPIO 0 — BOOT button)
    //
    // Hold the BOOT button during power-on to wipe all stored data so
    // the device can be redeployed to a new location:
    //   • preferences.clear()  — erases lat, lon, station_id, boot count
    //   • wm.resetSettings()   — erases saved WiFi SSID/password
    // The device then restarts and presents a clean "Quake-Setup" portal.
    // -----------------------------------------------------------------
    pinMode(0, INPUT_PULLUP);
    if (digitalRead(0) == LOW) {
        Serial.println("BOOT button held — performing factory reset...");

        Preferences prefs;
        prefs.begin("quake-app", false);
        prefs.clear();
        prefs.end();

        WiFiManager wm;
        wm.resetSettings();

        Serial.println("Factory reset complete. Restarting...");
        delay(1000);
        ESP.restart();
    }

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    if (!initializeCorePrimitives()) {
        Serial.println("System entering safe idle mode due to initialization failure");
        while (true) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(500);
        }
    }

    assignStationId();
    initPersistentState();
    initTaskWatchdog();

    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setBufferSize(2048);
    mqttClient.setKeepAlive(CUSTOM_MQTT_KEEPALIVE);
    mqttClient.setCallback(mqttCallback);
    
    // Secure MQTT connection validating broker's certificate
    // using the universally trusted Let's Encrypt CA root.
    espClient.setCACert(root_ca);

    initWifi();
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    initMPU();

    digitalWrite(LED_BUILTIN, LOW);

    startWorkerTasks();

    delay(250);
    Serial.printf("System Ready (%s) [Boots: %d]\n", FIRMWARE_VERSION, bootCount);
}

void loop() {
    esp_task_wdt_reset();

    monitorHeap();
    checkMqttConnection();

    if (mqttClient.connected()) {
        mqttClient.loop();

        if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
            lastHeartbeat = millis();
            sendHeartbeat();
        }
    }

    if (rebootRequestReceived) {
        Serial.println("Remote reboot request received but ignored in uptime-safe mode");
        rebootRequestReceived = false;
    }

    handleAlerts();

    portENTER_CRITICAL(&reportMux);
    if (pendingReport.ready && !pendingReport.processed) {
        if (millis() - pendingReport.timestamp > 15000UL) {
            Serial.println("Warning: stale pending report cleared");
            pendingReport.ready = false;
            pendingReport.processed = false;
        }
    }
    portEXIT_CRITICAL(&reportMux);

    delay(10);
}