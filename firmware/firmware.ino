/**
 * QuakeAlert ESP32 - V6.9.5
 * Earthquake detection using MPU6050 with MQTT reporting,
 * NTP time sync, and IP-based geolocation.
 */

#include <Wire.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <Preferences.h>
#include "esp_mac.h"
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
TaskHandle_t SensorTask;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
SemaphoreHandle_t i2cMutex = NULL;

portMUX_TYPE reportMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE eventTriggerMux = portMUX_INITIALIZER_UNLOCKED;

volatile EventReport pendingReport = {false, 0, 0, 0, false};
volatile bool MPUInterrupt = false;
volatile bool eventTriggered = false;
volatile bool rebootRequestReceived = false;
volatile unsigned long lastLoopHeartbeat = 0;

bool DMPReady = false;
uint8_t devStatus = 0;
uint16_t packetSize = 0;
uint8_t FIFOBuffer[64];
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorFloat gravity;

String StationID = "SEIS-01";
String lokasiAlat = "Mencari lokasi...";
bool potentialEvent = false;
bool eventInProgress = false;
bool alertSent = false;
float pga = 0;
bool ledState = false;
bool isNtpSynced = false;
bool startupMessageSent = false;

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
unsigned long lastHeapCheck = 0;
int bootCount = 0;
Preferences preferences;

char lastPgaStr[16] = "N/A";
String lastIntensity = "N/A";
String lastEventTime = "N/A";

float readings[MOVING_AVERAGE_WINDOW_SIZE];
int readIndex = 0;
float total = 0;
float averageMagnitude = 0;
bool isFirstReading = true;

float stationLat = 0.0;
float stationLon = 0.0;

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
        String intensity = toIntensity(pga);
        sendMqttAlert(intensity, pga);
        alertSent = true;
    }

    bool shouldSendReport = false;
    float reportPga = 0;
    float reportDuration = 0;

    portENTER_CRITICAL(&reportMux);
    if (pendingReport.ready && !pendingReport.processed) {
        shouldSendReport = true;
        reportPga = pendingReport.maxPga;
        reportDuration = pendingReport.duration;
        pendingReport.processed = true;
    }
    portEXIT_CRITICAL(&reportMux);

    if (shouldSendReport) {
        String intensity = toIntensity(reportPga);
        lastEventTime = getWaktuString();
        snprintf(lastPgaStr, sizeof(lastPgaStr), "%.2f gal", reportPga);
        lastIntensity = intensity;

        bool success = sendMqttReport(lokasiAlat, lastEventTime, reportDuration,
                                     String(lastPgaStr), lastIntensity);
        if (success) totalEventsDetected++;

        portENTER_CRITICAL(&reportMux);
        pendingReport.ready = false;
        pendingReport.processed = false;
        portEXIT_CRITICAL(&reportMux);
    }
}

// ========================================
// SENSOR TASK (runs on Core 1)
// ========================================
void sensorTask(void* pvParameters) {
    Serial.println("Sensor Task running on Core 1");
    esp_task_wdt_add(NULL);

    for (;;) {
        esp_task_wdt_reset();

        if (millis() - lastLoopHeartbeat > SOFT_WATCHDOG_LIMIT_MS && lastLoopHeartbeat != 0) {
            Serial.println("SOFT WDT: Main Loop Stuck! Restarting...");
            delay(100);
            ESP.restart();
        }

        if (MPUInterrupt) {
            processSensorData();
        }

        if (mpuErrorCounter > 5) {
            if (millis() - lastBlinkTime > 150) {
                lastBlinkTime = millis();
                ledState = !ledState;
                digitalWrite(LED_BUILTIN, ledState);
            }
        } else if (eventInProgress) {
            if (millis() - lastBlinkTime > 250) {
                lastBlinkTime = millis();
                ledState = !ledState;
                digitalWrite(LED_BUILTIN, ledState);
            }
        } else {
            digitalWrite(LED_BUILTIN, (WiFi.status() == WL_CONNECTED) ? LOW : HIGH);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ========================================
// SETUP & LOOP
// ========================================
void setup() {
    Serial.begin(115200);

    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        char idFormat[16];
        snprintf(idFormat, sizeof(idFormat), "SEIS-%02X%02X%02X", mac[3], mac[4], mac[5]);
        StationID = String(idFormat);
    } else {
        StationID = "SEIS-UNKNOWN";
    }
    Serial.println("Unique Station ID Assigned: " + StationID);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    preferences.begin("quake-app", false);
    bootCount = preferences.getInt("boots", 0) + 1;
    preferences.putInt("boots", bootCount);
    preferences.end();

    lastLoopHeartbeat = millis();
    esp_task_wdt_deinit();
#ifdef USE_LEGACY_WDT
    /* PlatformIO bundled framework: 2-arg API */
    esp_task_wdt_init(WDT_TIMEOUT, true);
#else
    /* Arduino IDE 3.3.6: config struct API */
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000,
        .idle_core_mask = (1 << 0) | (1 << 1),
        .trigger_panic = true
    };
    esp_task_wdt_init(&twdt_config);
#endif
    esp_task_wdt_add(NULL);

    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) {
        Serial.println("FATAL: Failed to create I2C mutex");
        ESP.restart();
    }

    for (int i = 0; i < MOVING_AVERAGE_WINDOW_SIZE; i++) readings[i] = 0.0f;

    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setBufferSize(2048);
    mqttClient.setKeepAlive(CUSTOM_MQTT_KEEPALIVE);
    mqttClient.setCallback(mqttCallback);

    initWifi();

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    Serial.println("Stabilizing Network...");
    delay(2000);

    getLokasi();

    initMPU();

    digitalWrite(LED_BUILTIN, LOW);
    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 10000, NULL, 1, &SensorTask, 1);

    delay(500);
    Serial.printf("System Ready (%s) [Boots: %d]\n", FIRMWARE_VERSION, bootCount);
}

void loop() {
    esp_task_wdt_reset();
    lastLoopHeartbeat = millis();

    monitorHeap();
    if (millis() > UPTIME_RESTART_THRESHOLD_MS) {
        Serial.println("Scheduled 7-day Maintenance Restart...");
        delay(1000);
        ESP.restart();
    }

    checkMqttConnection();

    if (mqttClient.connected()) {
        mqttClient.loop();

        if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL_MS) {
            lastHeartbeat = millis();
            sendHeartbeat();
        }
    }

    if (rebootRequestReceived) {
        Serial.println("Remote reboot requested...");
        delay(1000);
        ESP.restart();
    }

    handleNetworkTasks();
    handleAlerts();

    portENTER_CRITICAL(&reportMux);
    if (pendingReport.ready && !pendingReport.processed) {
        if (millis() - pendingReport.timestamp > 15000) {
            Serial.println("Warning: Stuck report cleared");
            pendingReport.ready = false;
        }
    }
    portEXIT_CRITICAL(&reportMux);

    delay(50);
}
