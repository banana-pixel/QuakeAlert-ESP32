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

        // Custom WiFi credentials have been erased by prefs.clear()

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
    
    // Server is using a self-signed certificate (ca.crt), so we cannot use
    // Let's Encrypt Root CA. We use setInsecure() to encrypt the TLS payload
    // without dropping connection due to invalid chain validation.
    espClient.setInsecure();

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

    handleProvisioningLoop();

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
        Serial.println("Remote reboot request received! Rebooting system in 1s...");
        rebootRequestReceived = false;
        delay(1000);
        ESP.restart();
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