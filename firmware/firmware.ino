/**
 * QuakeAlert ESP32 - V6.8
 * Description: Earthquake detection system using MPU6050 with MQTT reporting,
 * NTP time sync, and IP-based geolocation.
 */

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
#include <string.h>
#include <ESPmDNS.h>
#include <WiFiManager.h>      // https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>     // https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>      // https://arduinojson.org/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "esp_system.h"
#include "esp_tls.h"
#include <esp_task_wdt.h>
#include <Preferences.h>
#include "secrets.h"

// ========================================
// 1. SYSTEM CONFIGURATION
// ========================================
#define WDT_TIMEOUT 30
const unsigned long UPTIME_RESTART_THRESHOLD = 604800000; // 7 Days
const char* firmwareVersion = "6.8"; // Fixed version match
const char *StationID = "SEIS-01"; // e.g., SEIS-01, SEIS-02

const unsigned long SOFT_WATCHDOG_LIMIT = 60000; // 60s Loop Timeout
const unsigned long BOOT_SETTLING_TIME = 15000;  // 15s Cold Start Ignore
const float MAX_FRAGMENTATION_PERCENT = 50.0;    // Restart if frag > 50%

uint32_t minHeapSeen = 0xFFFFFFFF;
unsigned long lastHeapCheck = 0;
volatile unsigned long lastLoopHeartbeat = 0;    // Soft Watchdog Heartbeat

int bootCount = 0;
Preferences preferences;

// ========================================
// 2. MQTT CONFIGURATION
// ========================================
// [SECURITY] CHANGE THESE BEFORE DEPLOYING
const char* mqtt_server   = SECRET_MQTT_SERVER;
const int   mqtt_port     = SECRET_MQTT_PORT;
const char* mqtt_user     = SECRET_MQTT_USER;
const char* mqtt_password = SECRET_MQTT_PASS;

const int   CUSTOM_MQTT_KEEPALIVE = 15;
const char* mqtt_topic_alert   = "seismo/alert";
const char* mqtt_topic_report  = "seismo/report";
const char* mqtt_topic_command = "seismo/command";
const char* mqtt_topic_status  = "seismo/status";

// ========================================
// 3. DETECTION CONFIGURATION
// ========================================
const float TRIGGER_THRESHOLD = 8.0;
const float SUSTAIN_THRESHOLD = 4.0;

const unsigned long CONFIRMATION_DURATION = 1000;
const unsigned long MAX_EVENT_DURATION = 60000;
const unsigned long EVENT_COOLDOWN_PERIOD = 60000;
const int MOVING_AVERAGE_WINDOW_SIZE = 5;

// Calibration: 1g (980 gal) = 8192 LSB (FS_4) -> Ratio ~0.119
const float dataRatio = 980.0 / 8192.0;

// ========================================
// 4. TIMING & STABILITY
// ========================================
const int HTTP_TIMEOUT = 3000;
const int MPU_ERROR_THRESHOLD_NOTIF = 25;
const unsigned long WIFI_RECONNECT_INTERVAL = 30000;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;
const unsigned long NTP_SYNC_INTERVAL = 1800000; // 30 Minutes
const unsigned long NTP_RETRY_INTERVAL = 60000;  // 1 Minute retry on fail

const char* ntpServer = "id.pool.ntp.org";
const long  gmtOffset_sec = 7 * 3600;
const int   daylightOffset_sec = 0;

// ========================================
// 5. GLOBAL OBJECTS
// ========================================
MPU6050 mpu;
TaskHandle_t SensorTask;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
SemaphoreHandle_t i2cMutex;

#define LED_BUILTIN 2
const int INTERRUPT_PIN = 15;
const int SDA_PIN = 21; // Standard ESP32 SDA
const int SCL_PIN = 22; // Standard ESP32 SCL

struct EventReport {
    bool ready;
    float maxPga;
    float duration;
    unsigned long timestamp;
    bool processed;
};

// Flags & Sync
portMUX_TYPE reportMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE eventTriggerMux = portMUX_INITIALIZER_UNLOCKED;

volatile EventReport pendingReport = {false, 0, 0, 0, false};
volatile bool MPUInterrupt = false;
volatile bool eventTriggered = false;
volatile bool rebootRequestReceived = false;

// Sensor State
bool DMPReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorFloat gravity;

// App State
String lokasiAlat = "Mencari lokasi...";
bool potentialEvent = false;
bool eventInProgress = false;
bool alertSent = false;
float pga = 0;
bool ledState = false;
bool isNtpSynced = false;
bool startupMessageSent = false;

// Timers
unsigned long potentialEventTime = 0;
unsigned long eventStartTime = 0;
unsigned long lastReportTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastNtpSync = 0;
unsigned long lastNtpAttempt = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastMqttAttempt = 0;

uint32_t mpuOverflowCount = 0;
uint32_t totalEventsDetected = 0;
int mpuErrorCounter = 0;

char lastPgaStr[16] = "N/A";
String lastIntensity = "N/A";
String lastEventTime = "N/A";

float readings[MOVING_AVERAGE_WINDOW_SIZE];
int readIndex = 0;
float total = 0;
float averageMagnitude = 0;
bool isFirstReading = true;

// ========================================
// 6. CORE UTILITIES
// ========================================

void monitorHeap() {
    if (millis() - lastHeapCheck > 10000) {
        lastHeapCheck = millis();
        uint32_t freeHeap = ESP.getFreeHeap();
        uint32_t maxAllocHeap = ESP.getMaxAllocHeap();

        if (freeHeap < minHeapSeen) minHeapSeen = freeHeap;

        // Critical Low Memory Check
        if (freeHeap < 15000) {
            Serial.println("CRITICAL: Heap exhausted (<15KB), restarting...");
            delay(100);
            ESP.restart();
        }

        // Heap Fragmentation Check
        float fragmentation = 100.0 * (1.0 - ((float)maxAllocHeap / (float)freeHeap));
        if (fragmentation > MAX_FRAGMENTATION_PERCENT) {
            Serial.printf("CRITICAL: Heap Fragmentation High (%.2f%%). Restarting...\n", fragmentation);
            delay(100);
            ESP.restart();
        }
    }
}

String getWaktuString() {
    if (!isNtpSynced) return "Sinkronisasi Waktu Gagal";
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){ return "Gagal mengambil waktu lokal"; }
    char timeStringBuff[50];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%d-%m-%Y %H:%M:%S WIB", &timeinfo);
    return String(timeStringBuff);
}

void checkNtpSync() {
    if (WiFi.status() != WL_CONNECTED) return;
    unsigned long now = millis();

    if (isNtpSynced) {
        if (now - lastNtpSync > NTP_SYNC_INTERVAL) {
            isNtpSynced = false;
        }
        return;
    }

    if (now - lastNtpAttempt < NTP_RETRY_INTERVAL && lastNtpAttempt != 0) return;

    Serial.println("Performing NTP Sync...");
    lastNtpAttempt = now;
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        isNtpSynced = true;
        lastNtpSync = now;
        Serial.println("NTP Synced Successfully");
    } else {
        Serial.println("NTP Sync Failed - Retrying in 60s");
    }
}

void getLokasi() {
    if (WiFi.status() != WL_CONNECTED) {
        lokasiAlat = "WiFi terputus";
        return;
    }

    WiFiClient client;
    HTTPClient http;
    http.setConnectTimeout(HTTP_TIMEOUT);
    http.setTimeout(HTTP_TIMEOUT);

    StaticJsonDocument<512> doc;
    String city, region, country;
    bool success = false;

    // Primary Provider (ip-api.com)
    http.begin(client, "http://ip-api.com/json/?fields=country,regionName,city");
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        DeserializationError error = deserializeJson(doc, http.getString());
        if (!error && !doc["city"].isNull()) {
            city = doc["city"].as<String>();
            region = doc["regionName"].as<String>();
            country = doc["country"].as<String>();
            lokasiAlat = city + ", " + region + ", " + country;
            success = true;
            Serial.println("Lokasi Updated: " + lokasiAlat);
        }
    }
    http.end();

    // Fallback Provider (ipinfo.io)
    if (!success) {
        http.begin(client, "http://ipinfo.io/json");
        httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
            DeserializationError error = deserializeJson(doc, http.getString());
            if (!error && !doc["city"].isNull()) {
                city = doc["city"].as<String>();
                region = doc["region"].as<String>();
                country = doc["country"].as<String>();
                lokasiAlat = city + ", " + region + ", " + country;
                success = true;
                Serial.println("Lokasi Updated (Fallback): " + lokasiAlat);
            }
        }
        http.end();
    }

    if (!success) {
        lokasiAlat = "Lokasi Tidak Diketahui (API Error)";
        Serial.println("Lokasi Lookup Failed");
    }
}

// ========================================
// 7. INTENSITY CALCULATIONS (USGS Standard)
// ========================================
String toIntensity(float pga_val) {
    if (pga_val < 0.5)  return "I (Tidak Terasa)";
    if (pga_val < 2.8)  return "II-III (Lemah)";
    if (pga_val < 6.2)  return "IV (Ringan)";
    if (pga_val < 12.0) return "V (Sedang)";
    if (pga_val < 22.0) return "VI (Kuat)";
    if (pga_val < 40.0) return "VII (Sangat Kuat)";
    if (pga_val < 75.0) return "VIII (Merusak)";
    if (pga_val < 139.0) return "IX (Hebat)";
    return "X+ (Ekstrem)";
}

String getMmiDescription(String intensity) {
    if (intensity.startsWith("I ")) return "Hanya tercatat instrumen.";
    if (intensity.startsWith("II-III")) return "Dirasakan orang diam di atas.";
    if (intensity.startsWith("IV ")) return "Jendela/pintu berderik.";
    if (intensity.startsWith("V ")) return "Barang ringan terpelanting.";
    if (intensity.startsWith("VI ")) return "Plester dinding retak.";
    if (intensity.startsWith("VII ")) return "Sulit berdiri, kerusakan ringan.";
    if (intensity.startsWith("VIII ")) return "Dinding lepas, cerobong roboh.";
    if (intensity.startsWith("IX ")) return "Kerusakan hebat, pondasi geser.";
    if (intensity.startsWith("X+ ")) return "Hancur total.";
    return "Tidak ada data.";
}

void IRAM_ATTR DMPDataReady() {
    MPUInterrupt = true;
}

// ========================================
// 8. INITIALIZATION
// ========================================
void initWifi() {
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);
    // Suggestion: If you want to force reset wifi settings for testing, uncomment below:
    // wm.resetSettings();
    if (!wm.autoConnect("Quake-Setup")) {
        ESP.restart();
    }
}

void initMPU() {
    // FIX: Explicitly define pins for stability
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Wire.setTimeOut(3000);

    if (xSemaphoreTake(i2cMutex, 2000 / portTICK_PERIOD_MS)) {
        Serial.println("Initializing MPU...");
        mpu.initialize();

        mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
        mpu.setDLPFMode(MPU6050_DLPF_BW_5);
        mpu.setRate(99);

        pinMode(INTERRUPT_PIN, INPUT);
        devStatus = mpu.dmpInitialize();
        if (devStatus == 0) {
            mpu.CalibrateAccel(15);
            mpu.CalibrateGyro(15);
            mpu.setDMPEnabled(true);
            mpu.resetFIFO();
            attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
            DMPReady = true;
            packetSize = mpu.dmpGetFIFOPacketSize();
            Serial.println("MPU6050 Initialized & Configured (FS_4/100Hz)");
        } else {
            DMPReady = false;
            Serial.printf("MPU6050 init failed: %d\n", devStatus);
        }
        xSemaphoreGive(i2cMutex);
    } else {
        Serial.println("Failed to take mutex for MPU Init");
    }
}

// ========================================
// 9. MQTT LOGIC
// ========================================
void sendMqttAlert(String intensity, float pga_value) {
    if (!mqttClient.connected()) return;
    StaticJsonDocument<512> doc;
    doc["stationId"] = StationID;
    doc["lokasi"] = lokasiAlat;
    doc["waktu"] = getWaktuString();
    doc["intensitas"] = intensity;
    doc["pga"] = String(pga_value, 2);
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    if (mqttClient.publish(mqtt_topic_alert, jsonBuffer)) {
        Serial.println("Alert Published!");
    } else {
        Serial.println("Alert Publish Failed!");
    }
}

bool sendMqttReport(String lokasi, String waktu, float durasi, String pga_str,
                    String intensitas, String deskripsi) {
    if (!mqttClient.connected()) return false;
    // Increased buffer size slightly for safety
    DynamicJsonDocument doc(2048);
    doc["stationId"] = StationID;
    doc["lokasi"] = lokasi;
    doc["waktu"] = waktu;
    doc["durasi"] = durasi;
    doc["pga"] = pga_str;
    doc["intensitas"] = intensitas;
    doc["deskripsi"] = deskripsi;

    size_t len = measureJson(doc);
    // Explicit buffer check
    if (len >= 2048) {
        Serial.println("Payload too large for buffer!");
        return false;
    }

    char jsonBuffer[2048];
    serializeJson(doc, jsonBuffer);
    for (int i = 0; i < 3; i++) {
        if (mqttClient.publish(mqtt_topic_report, jsonBuffer, len)) {
            Serial.println("Report Published!");
            return true;
        }
        delay(100);
    }
    Serial.println("Report Failed after 3 retries");
    return false;
                    }

                    void sendMqttStartupMessage() {
                        if (!mqttClient.connected()) return;
                        StaticJsonDocument<512> doc;
                        doc["event"] = "startup";
                        doc["stationId"] = StationID;
                        doc["lokasi"] = lokasiAlat;
                        doc["version"] = firmwareVersion;
                        doc["restarts"] = bootCount;
                        char jsonBuffer[512];
                        serializeJson(doc, jsonBuffer);
                        mqttClient.publish(mqtt_topic_status, jsonBuffer);
                    }

                    void mqttCallback(char* topic, byte* payload, unsigned int length) {
                        String message;
                        message.reserve(length);
                        for (unsigned int i = 0; i < length; i++) message += (char)payload[i];

                        if (String(topic) == mqtt_topic_command) {
                            if (message == "ping") {
                                unsigned long uptimeMillis = millis();
                                int days = uptimeMillis / 86400000;
                                int hours = (uptimeMillis % 86400000) / 3600000;
                                int minutes = (uptimeMillis % 3600000) / 60000;

                                long rssi = WiFi.RSSI();
                                String rssi_ket = (rssi > -67) ? " (Bagus)" : (rssi > -80) ? " (Cukup)" : " (Lemah)";

                                float tempC = 0;
                                bool sensorConnected = false;
                                if (xSemaphoreTake(i2cMutex, 100 / portTICK_PERIOD_MS)) {
                                    tempC = (mpu.getTemperature() / 340.0) + 36.53;
                                    sensorConnected = mpu.testConnection();
                                    xSemaphoreGive(i2cMutex);
                                }

                                DynamicJsonDocument doc(1024);
                                doc["stationId"] = StationID;
                                doc["lokasi"] = lokasiAlat;
                                doc["uptime"] = String(days) + "d " + String(hours) + "h " + String(minutes) + "m";
                                doc["heap"] = ESP.getFreeHeap();
                                doc["currentTime"] = getWaktuString();
                                doc["ntpStatus"] = isNtpSynced ? "Tersinkronisasi" : "Gagal";
                                doc["wifiRssi"] = rssi;
                                doc["wifiStrength"] = rssi_ket;
                                doc["sensorStatus"] = sensorConnected ? "Terhubung" : "Gagal";
                                doc["dmpStatus"] = DMPReady ? "Siap" : "Gagal";
                                doc["chipTemp"] = String(tempC, 1);
                                doc["lastEventTime"] = lastEventTime;
                                doc["lastIntensity"] = lastIntensity;
                                doc["lastPga"] = lastPgaStr;
                                doc["mpuOverflows"] = mpuOverflowCount;
                                doc["restarts"] = bootCount;

                                char output[1024];
                                serializeJson(doc, output);
                                mqttClient.publish(mqtt_topic_status, output);

                            } else if (message == "reboot") {
                                rebootRequestReceived = true;
                            } else if (message == "stats") {
                                DynamicJsonDocument doc(1024);
                                doc["stationId"] = StationID;
                                doc["firmware"] = firmwareVersion;
                                doc["mpuErrors"] = mpuErrorCounter;
                                doc["mpuOverflows"] = mpuOverflowCount;
                                doc["totalEvents"] = totalEventsDetected;
                                doc["minHeapEver"] = minHeapSeen;
                                doc["currentHeap"] = ESP.getFreeHeap();
                                doc["ntpSynced"] = isNtpSynced;
                                doc["restarts"] = bootCount;

                                char output[1024];
                                serializeJson(doc, output);
                                mqttClient.publish(mqtt_topic_status, output);
                            }
                        }
                    }

                    void checkMqttConnection() {
                        if (WiFi.status() != WL_CONNECTED) return;
                        if (!mqttClient.connected()) {
                            unsigned long now = millis();
                            if (now - lastMqttAttempt > MQTT_RECONNECT_INTERVAL) {
                                lastMqttAttempt = now;
                                // Generate Random Client ID to prevent collision
                                String clientId = "ESP32-Seismo-" + String(random(0xffff), HEX);
                                if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
                                    mqttClient.subscribe(mqtt_topic_command);
                                    Serial.println("MQTT Connected");
                                    if (!startupMessageSent) {
                                        sendMqttStartupMessage();
                                        startupMessageSent = true;
                                    }
                                } else {
                                    Serial.printf("MQTT failed, rc=%d\n", mqttClient.state());
                                }
                            }
                        }
                    }

                    // ========================================
                    // 10. PROCESSING LOGIC
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
                                                          String(lastPgaStr), lastIntensity, getMmiDescription(intensity));
                            if (success) totalEventsDetected++;

                            portENTER_CRITICAL(&reportMux);
                            pendingReport.ready = false;
                            pendingReport.processed = false;
                            portEXIT_CRITICAL(&reportMux);
                        }
                    }

                    void processSensorData() {
                        // GATE: Check MPU Interrupt Flag
                        if (!MPUInterrupt) return;

                        // MUTEX: 50ms timeout to prevent locking
                        if (!xSemaphoreTake(i2cMutex, 50 / portTICK_PERIOD_MS)) return;

                        MPUInterrupt = false;

                        // Check DMP state
                        if (!DMPReady) {
                            Serial.println("DMP not ready - skipping sensor read");
                            xSemaphoreGive(i2cMutex);
                            return;
                        }

                        uint8_t intStatus = mpu.getIntStatus();
                        // Check for Overflow (Critical Fail)
                        if (intStatus & 0x10) {
                            Serial.println("MPU FIFO overflow, resetting...");
                            mpu.resetFIFO();
                            mpuErrorCounter = 0;
                            mpuOverflowCount++;
                            xSemaphoreGive(i2cMutex);
                            return;
                        }

                        // Read Packet
                        if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer) == 0) {
                            mpuErrorCounter++;
                            if (mpuErrorCounter > MPU_ERROR_THRESHOLD_NOTIF) {
                                Serial.println("MPU Read Errors exceeded threshold. Resetting FIFO...");
                                mpu.resetFIFO();
                                mpuErrorCounter = 0;
                            }
                            xSemaphoreGive(i2cMutex);
                            return;
                        }

                        mpuErrorCounter = 0; // Success, reset counter
                        mpu.dmpGetQuaternion(&q, FIFOBuffer);
                        mpu.dmpGetGravity(&gravity, &q);
                        mpu.dmpGetAccel(&aa, FIFOBuffer);
                        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                        xSemaphoreGive(i2cMutex);

                        float vectorMagnitude = sqrt(pow(aaReal.x, 2) + pow(aaReal.y, 2) + pow(aaReal.z, 2)) * dataRatio;

                        // Cold Start Buffer Logic
                        // We update the buffer, but we REFUSE to trigger events if we are in settling time
                        bool systemSettling = (millis() < BOOT_SETTLING_TIME);

                        // MOVING AVERAGE LOGIC (Drift Fix)
                        if (isFirstReading) {
                            for (int i=0; i < MOVING_AVERAGE_WINDOW_SIZE; i++) {
                                readings[i] = vectorMagnitude;
                            }
                            total = vectorMagnitude * MOVING_AVERAGE_WINDOW_SIZE;
                            readIndex = 0;
                            isFirstReading = false;
                            averageMagnitude = vectorMagnitude;
                        } else {
                            readings[readIndex] = vectorMagnitude;
                            readIndex = (readIndex + 1) % MOVING_AVERAGE_WINDOW_SIZE;

                            // Recalculate from scratch to prevent drift
                            total = 0;
                            for (int i = 0; i < MOVING_AVERAGE_WINDOW_SIZE; i++) {
                                total += readings[i];
                            }
                            averageMagnitude = total / MOVING_AVERAGE_WINDOW_SIZE;
                        }

                        // Temperature Compensation / Auto-Calibration
                        // CRITICAL: Done BEFORE cooldown gate to ensure baseline updates
                        static float baselineOffset = 0;
                        static unsigned long lastCalibration = 0;

                        // Auto-calibrate every 60s during quiet periods
                        if (!eventInProgress && !potentialEvent && (millis() - lastCalibration > 60000)) {
                            if (averageMagnitude < 3.0) { // Only when truly quiet
                                baselineOffset = averageMagnitude;
                                lastCalibration = millis();
                                Serial.printf("Baseline recalibrated: %.2f gal\n", baselineOffset);
                            }
                        }

                        // Apply correction before trigger check
                        float correctedMagnitude = averageMagnitude - baselineOffset;
                        if (correctedMagnitude < 0) correctedMagnitude = 0; // Safety clamp

                        // If system is settling, stop here. Do not check triggers.
                        if (systemSettling) return;

                        // Cooldown Logic moved AFTER temperature compensation
                        // This ensures baseline updates even during cooldown
                        if (!eventInProgress && (millis() - lastReportTime < EVENT_COOLDOWN_PERIOD)) return;

                        // Use correctedMagnitude for trigger
                        if (!eventInProgress && !potentialEvent && correctedMagnitude >= TRIGGER_THRESHOLD) {
                            potentialEvent = true;
                            potentialEventTime = millis();
                        }

                        if (potentialEvent && !eventInProgress) {
                            // Check SUSTAIN using corrected values
                            if (correctedMagnitude >= SUSTAIN_THRESHOLD) {
                                if (millis() - potentialEventTime >= CONFIRMATION_DURATION) {
                                    eventInProgress = true;
                                    eventStartTime = potentialEventTime;
                                    pga = correctedMagnitude; // Record corrected PGA

                                    portENTER_CRITICAL(&eventTriggerMux);
                                    eventTriggered = true;
                                    portEXIT_CRITICAL(&eventTriggerMux);
                                }
                            } else {
                                potentialEvent = false;
                            }
                        }

                        if (eventInProgress) {
                            pga = max(pga, correctedMagnitude);
                            // TIMEOUT FAILSAFE: 60s Max Duration
                            bool timeOutReached = (millis() - eventStartTime > MAX_EVENT_DURATION);
                            if (correctedMagnitude < SUSTAIN_THRESHOLD || timeOutReached) {
                                if (timeOutReached) Serial.println("Event Forced End (Timeout)");
                                portENTER_CRITICAL(&reportMux);
                                pendingReport.maxPga = pga;
                                pendingReport.duration = (millis() - eventStartTime) / 1000.0;
                                pendingReport.timestamp = millis();
                                pendingReport.ready = true;
                                pendingReport.processed = false;
                                portEXIT_CRITICAL(&reportMux);

                                lastReportTime = millis();
                                eventInProgress = false;
                                potentialEvent = false;
                                alertSent = false;
                                pga = 0;
                            }
                        }
                    }

                    // ========================================
                    // 11. TASKS (CORE 1 & NETWORK)
                    // ========================================
                    void sensorTask(void *pvParameters) {
                        Serial.println("Sensor Task running on Core 1");
                        esp_task_wdt_add(NULL);

                        for (;;) {
                            esp_task_wdt_reset();

                            // Soft Watchdog Check
                            // If Loop (Core 0) hasn't updated heartbeat in 60s, restart.
                            if (millis() - lastLoopHeartbeat > SOFT_WATCHDOG_LIMIT && lastLoopHeartbeat != 0) {
                                Serial.println("SOFT WDT: Main Loop Stuck! Restarting...");
                                delay(100);
                                ESP.restart();
                            }

                            if (MPUInterrupt) {
                                processSensorData();
                            }

                            // LED Logic
                            if (mpuErrorCounter > 5) {
                                if (millis() - lastBlinkTime > 150) {
                                    lastBlinkTime = millis();
                                    ledState = !ledState; digitalWrite(LED_BUILTIN, ledState);
                                }
                            } else if (eventInProgress) {
                                if (millis() - lastBlinkTime > 250) {
                                    lastBlinkTime = millis();
                                    ledState = !ledState; digitalWrite(LED_BUILTIN, ledState);
                                }
                            } else {
                                digitalWrite(LED_BUILTIN, (WiFi.status() == WL_CONNECTED) ? LOW : HIGH);
                            }

                            vTaskDelay(10 / portTICK_PERIOD_MS);
                        }
                    }

                    void handleNetworkTasks() {
                        static uint32_t wifiFailCount = 0;
                        static unsigned long lastLocRetry = 0;

                        if (WiFi.status() != WL_CONNECTED) {
                            if (millis() - lastWifiCheck > WIFI_RECONNECT_INTERVAL) {
                                lastWifiCheck = millis();
                                wifiFailCount++;
                                Serial.printf("WiFi Lost. Attempt %d/6\n", wifiFailCount);

                                if (wifiFailCount <= 3) {
                                    WiFi.reconnect();
                                } else if (wifiFailCount <= 6) {
                                    Serial.println("WiFi Hard Reset...");
                                    WiFi.disconnect();
                                    delay(1000);
                                    WiFi.reconnect();
                                } else {
                                    Serial.println("WiFi Persistent Failure. System Restart.");
                                    delay(1000);
                                    ESP.restart();
                                }
                            }
                        } else {
                            if (wifiFailCount > 0) {
                                Serial.println("WiFi Recovered!");
                                wifiFailCount = 0;
                                // Immediate NTP retry after recovery
                                lastNtpAttempt = 0;
                                checkNtpSync();
                            }

                            bool locInvalid = (lokasiAlat.indexOf("Mencari") >= 0 || lokasiAlat.indexOf("Gagal") >= 0 || lokasiAlat.indexOf("WiFi") >= 0);
                            if (locInvalid && (millis() - lastLocRetry > 60000)) {
                                lastLocRetry = millis();
                                Serial.println("Retrying location lookup...");
                                getLokasi();
                            } else if (!startupMessageSent && locInvalid) {
                                getLokasi();
                            }

                            checkNtpSync();
                        }
                    }

                    // ========================================
                    // 12. SETUP & LOOP
                    // ========================================
                    void setup() {
                        Serial.begin(115200);
                        pinMode(LED_BUILTIN, OUTPUT);
                        digitalWrite(LED_BUILTIN, HIGH);

                        preferences.begin("quake-app", false);
                        bootCount = preferences.getInt("boots", 0) + 1;
                        preferences.putInt("boots", bootCount);
                        preferences.end();

                        // Init Heartbeat
                        lastLoopHeartbeat = millis();
                        esp_task_wdt_config_t twdt_config = {
                            .timeout_ms = WDT_TIMEOUT * 1000,
                            .idle_core_mask = (1 << 0) | (1 << 1),
                            .trigger_panic = true
                        };
                        esp_task_wdt_deinit();
                        esp_task_wdt_init(&twdt_config);
                        esp_task_wdt_add(NULL);

                        i2cMutex = xSemaphoreCreateMutex();
                        if (i2cMutex == NULL) {
                            Serial.println("FATAL: Failed to create I2C mutex");
                            ESP.restart();
                        }

                        for (int i = 0; i < MOVING_AVERAGE_WINDOW_SIZE; i++) readings[i] = 0.0;

                        mqttClient.setServer(mqtt_server, mqtt_port);
                        // Increased buffer size to match JsonDocument
                        mqttClient.setBufferSize(2048);
                        mqttClient.setKeepAlive(CUSTOM_MQTT_KEEPALIVE);
                        mqttClient.setCallback(mqttCallback);

                        initWifi();
                        initMPU();

                        digitalWrite(LED_BUILTIN, LOW);
                        xTaskCreatePinnedToCore(sensorTask, "SensorTask", 10000, NULL, 1, &SensorTask, 1);

                        delay(500);
                        Serial.printf("System Ready (%s) [Boots: %d]\n", firmwareVersion, bootCount);
                    }

                    void loop() {
                        esp_task_wdt_reset();
                        // Update Heartbeat for Soft Watchdog
                        lastLoopHeartbeat = millis();

                        monitorHeap();
                        if (millis() > UPTIME_RESTART_THRESHOLD) {
                            Serial.println("Scheduled 7-day Maintenance Restart...");
                            delay(1000);
                            ESP.restart();
                        }

                        checkMqttConnection();

                        if (mqttClient.connected()) {
                            mqttClient.loop();
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
