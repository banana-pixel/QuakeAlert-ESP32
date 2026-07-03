/**
 * QuakeAlert ESP32 - MQTT Logic Implementation
 */

#include "mqtt.h"
#include "config.h"
#include "state.h"
#include "utils.h"
#include "sensor.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

namespace {
// Truncate a coordinate to 2 decimal places, creating a ~1.1 km anonymity box.
// Pure arithmetic: zero heap allocation, equivalent to String(coord, 2).toFloat().
static float maskCoord(float coord) {
    return roundf(coord * 100.0f) / 100.0f;
}

bool serializeDocToBuffer(JsonDocument& doc, char* buffer, size_t bufferSize, size_t& outLength) {
    outLength = serializeJson(doc, buffer, bufferSize);
    if (outLength == 0 || outLength >= bufferSize || doc.overflowed()) {
        Serial.println("MQTT JSON serialization failed or payload too large");
        return false;
    }
    return true;
}

bool publishBuffer(const char* topic, const char* payload, size_t payloadLength) {
    if (topic == nullptr || payload == nullptr || payloadLength == 0) {
        return false;
    }

    if (!mqttClient.connected()) {
        return false;
    }

    return mqttClient.publish(topic, reinterpret_cast<const uint8_t*>(payload), payloadLength, false);
}
}  // namespace

bool mqttPublishJson(const char* topic, const char* payload, size_t payloadLength) {
    return publishBuffer(topic, payload, payloadLength);
}

bool mqttIsCommandTopic(const char* topic) {
    return topic != nullptr && strcmp(topic, MQTT_TOPIC_COMMAND) == 0;
}

bool mqttPayloadToCString(const byte* payload, unsigned int length, char* output, size_t outputSize) {
    if (output == nullptr || outputSize == 0) {
        return false;
    }

    if (payload == nullptr) {
        output[0] = '\0';
        return false;
    }

    const size_t copyLength = (length < (outputSize - 1)) ? length : (outputSize - 1);
    memcpy(output, payload, copyLength);
    output[copyLength] = '\0';

    return copyLength == length;
}

void sendMqttAlert(const char* intensity, float pgaValue) {
    if (!mqttClient.connected()) {
        return;
    }

    char stationId[STATION_ID_BUFFER_SIZE];
    char lokasi[LOCATION_TEXT_BUFFER_SIZE];
    char waktu[TIME_TEXT_BUFFER_SIZE];
    char pgaText[16];

    getStationIdCopy(stationId, sizeof(stationId));
    getLokasiAlatCopy(lokasi, sizeof(lokasi));
    getWaktuString(waktu, sizeof(waktu));
    snprintf(pgaText, sizeof(pgaText), "%.2f", pgaValue);

    char latStr[16];
    char lonStr[16];
    snprintf(latStr, sizeof(latStr), "%.2f", maskCoord(stationLat));
    snprintf(lonStr, sizeof(lonStr), "%.2f", maskCoord(stationLon));

    StaticJsonDocument<MQTT_ALERT_JSON_CAPACITY> doc;
    doc["stationId"] = stationId;
    doc["lokasi"] = lokasi;
    doc["lat"] = latStr;
    doc["lon"] = lonStr;
    doc["waktu"] = waktu;
    doc["intensitas"] = intensity != nullptr ? intensity : "N/A";
    doc["pga"] = pgaText;

    char jsonBuffer[MQTT_ALERT_BUFFER_SIZE];
    size_t jsonLength = 0;
    if (!serializeDocToBuffer(doc, jsonBuffer, sizeof(jsonBuffer), jsonLength)) {
        return;
    }

    if (mqttPublishJson(MQTT_TOPIC_ALERT, jsonBuffer, jsonLength)) {
        Serial.println("Alert Published!");
    } else {
        Serial.println("Alert Publish Failed!");
    }
}

bool sendMqttReport(const char* lokasi,
                    const char* waktu,
                    float durasi,
                    const char* pgaText,
                    const char* intensitas) {
    if (!mqttClient.connected()) {
        return false;
    }

    char stationId[STATION_ID_BUFFER_SIZE];
    getStationIdCopy(stationId, sizeof(stationId));

    char latStr[16];
    char lonStr[16];
    snprintf(latStr, sizeof(latStr), "%.2f", maskCoord(stationLat));
    snprintf(lonStr, sizeof(lonStr), "%.2f", maskCoord(stationLon));

    StaticJsonDocument<MQTT_REPORT_JSON_CAPACITY> doc;
    doc["stationId"] = stationId;
    doc["lokasi"] = lokasi != nullptr ? lokasi : "";
    doc["lat"] = latStr;
    doc["lon"] = lonStr;
    doc["waktu"] = waktu != nullptr ? waktu : "N/A";
    doc["durasi"] = durasi;
    doc["pga"] = pgaText != nullptr ? pgaText : "N/A";
    doc["intensitas"] = intensitas != nullptr ? intensitas : "N/A";

    char jsonBuffer[MQTT_REPORT_BUFFER_SIZE];
    size_t jsonLength = 0;
    if (!serializeDocToBuffer(doc, jsonBuffer, sizeof(jsonBuffer), jsonLength)) {
        return false;
    }

    if (mqttPublishJson(MQTT_TOPIC_REPORT, jsonBuffer, jsonLength)) {
        Serial.println("Report Published!");
        return true;
    }

    Serial.println("Report Publish Failed!");
    return false;
}

// ---------------------------------------------------------------------------
// sendHeartbeat — Architecture Spec §2.1 / seismo/heartbeat (QoS 0)
// Payload: { id, version, lat, lon, lokasi, pga, rssi, uptime }
// All fields strictly match the architecture specification so the server's
// clustering engine and station-health endpoint can parse them without
// any field-name translation.
// ---------------------------------------------------------------------------
void sendHeartbeat() {
    if (!mqttClient.connected()) {
        return;
    }

    char stationId[STATION_ID_BUFFER_SIZE];
    char lokasi[LOCATION_TEXT_BUFFER_SIZE];
    getStationIdCopy(stationId, sizeof(stationId));
    getLokasiAlatCopy(lokasi, sizeof(lokasi));

    // Use real (unmasked) coordinates for the heartbeat so the server can
    // geo-locate the station accurately. The spec §2.2 shows full precision.
    char latStr[16];
    char lonStr[16];
    snprintf(latStr, sizeof(latStr), "%.4f", stationLat);
    snprintf(lonStr, sizeof(lonStr), "%.4f", stationLon);

    // Current noise-floor PGA from the last sensor sample (gal)
    char pgaStr[16];
    snprintf(pgaStr, sizeof(pgaStr), "%.4f", sta); // STA ≈ current baseline activity

    const unsigned long uptimeSec = millis() / 1000UL;
    const long rssi = WiFi.RSSI();

    StaticJsonDocument<MQTT_HEARTBEAT_JSON_CAPACITY> doc;
    doc["id"]      = stationId;          // matches spec field name
    doc["version"] = FIRMWARE_VERSION;
    doc["lat"]     = latStr;
    doc["lon"]     = lonStr;
    doc["lokasi"]  = lokasi;
    doc["pga"]     = pgaStr;
    doc["rssi"]    = (int)rssi;
    doc["uptime"]  = (unsigned long)uptimeSec;

    char jsonBuffer[MQTT_HEARTBEAT_BUFFER_SIZE];
    size_t jsonLength = 0;
    if (!serializeDocToBuffer(doc, jsonBuffer, sizeof(jsonBuffer), jsonLength)) {
        return;
    }

    // QoS 0 — fire-and-forget, consistent with the spec table
    mqttClient.publish(MQTT_TOPIC_HEARTBEAT, reinterpret_cast<const uint8_t*>(jsonBuffer), jsonLength, false);
}

void sendMqttStartupMessage() {
    if (!mqttClient.connected()) {
        return;
    }

    char stationId[STATION_ID_BUFFER_SIZE];
    char lokasi[LOCATION_TEXT_BUFFER_SIZE];

    getStationIdCopy(stationId, sizeof(stationId));
    getLokasiAlatCopy(lokasi, sizeof(lokasi));

    StaticJsonDocument<MQTT_ALERT_JSON_CAPACITY> doc;
    doc["event"] = "startup";
    doc["stationId"] = stationId;
    doc["lokasi"] = lokasi;
    doc["version"] = FIRMWARE_VERSION;
    doc["restarts"] = bootCount;

    char jsonBuffer[MQTT_ALERT_BUFFER_SIZE];
    size_t jsonLength = 0;
    if (!serializeDocToBuffer(doc, jsonBuffer, sizeof(jsonBuffer), jsonLength)) {
        return;
    }

    mqttPublishJson(MQTT_TOPIC_STATUS, jsonBuffer, jsonLength);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    if (!mqttIsCommandTopic(topic)) {
        return;
    }

    char message[64];
    mqttPayloadToCString(payload, length, message, sizeof(message));

    if (strcmp(message, "ping") == 0) {
        char stationId[STATION_ID_BUFFER_SIZE];
        char lokasi[LOCATION_TEXT_BUFFER_SIZE];
        char uptime[32];
        char currentTime[TIME_TEXT_BUFFER_SIZE];
        char eventTime[TIME_TEXT_BUFFER_SIZE];
        char intensity[INTENSITY_TEXT_BUFFER_SIZE];
        char wifiStrength[24];
        char chipTemp[16];

        getStationIdCopy(stationId, sizeof(stationId));
        getLokasiAlatCopy(lokasi, sizeof(lokasi));
        getUptimeString(uptime, sizeof(uptime));
        getWaktuString(currentTime, sizeof(currentTime));
        getLastEventTimeCopy(eventTime, sizeof(eventTime));
        getLastIntensityCopy(intensity, sizeof(intensity));

        const long rssi = WiFi.RSSI();
        snprintf(
            wifiStrength,
            sizeof(wifiStrength),
            "%s",
            (rssi > -67) ? "Bagus" : (rssi > -80) ? "Cukup" : "Lemah"
        );

        float tempC = 0.0f;
        bool sensorConnected = false;
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            tempC = (mpu.getTemperature() / 340.0f) + 36.53f;
            sensorConnected = mpu.testConnection();
            xSemaphoreGive(i2cMutex);
        }
        snprintf(chipTemp, sizeof(chipTemp), "%.1f", tempC);

        StaticJsonDocument<MQTT_STATUS_JSON_CAPACITY> doc;
        doc["stationId"] = stationId;
        doc["lokasi"] = lokasi;
        doc["uptime"] = uptime;
        doc["heap"] = ESP.getFreeHeap();
        doc["currentTime"] = currentTime;
        doc["ntpStatus"] = isNtpSynced ? "Tersinkronisasi" : "Gagal";
        doc["wifiRssi"] = rssi;
        doc["wifiStrength"] = wifiStrength;
        doc["sensorStatus"] = sensorConnected ? "Terhubung" : "Gagal";
        doc["dmpStatus"] = DMPReady ? "Siap" : "Gagal";
        doc["chipTemp"] = chipTemp;
        doc["lastEventTime"] = eventTime;
        doc["lastIntensity"] = intensity;
        doc["lastPga"] = lastPgaStr;
        doc["mpuOverflows"] = mpuOverflowCount;
        doc["restarts"] = bootCount;

        char output[MQTT_STATUS_BUFFER_SIZE];
        size_t outLen = 0;
        if (serializeDocToBuffer(doc, output, sizeof(output), outLen)) {
            mqttPublishJson(MQTT_TOPIC_STATUS, output, outLen);
        }

    } else if (strcmp(message, "reboot") == 0) {
        rebootRequestReceived = true;

    } else if (strcmp(message, "stats") == 0) {
        char stationId[STATION_ID_BUFFER_SIZE];
        getStationIdCopy(stationId, sizeof(stationId));

        StaticJsonDocument<MQTT_STATUS_JSON_CAPACITY> doc;
        doc["stationId"] = stationId;
        doc["firmware"] = FIRMWARE_VERSION;
        doc["mpuErrors"] = mpuErrorCounter;
        doc["mpuOverflows"] = mpuOverflowCount;
        doc["totalEvents"] = totalEventsDetected;
        doc["minHeapEver"] = minHeapSeen;
        doc["maxFragmentation"] = maxHeapFragmentationSeen;
        doc["currentHeap"] = ESP.getFreeHeap();
        doc["ntpSynced"] = isNtpSynced;
        doc["restarts"] = bootCount;

        char output[MQTT_STATUS_BUFFER_SIZE];
        size_t outLen = 0;
        if (serializeDocToBuffer(doc, output, sizeof(output), outLen)) {
            mqttPublishJson(MQTT_TOPIC_STATUS, output, outLen);
        }
    }
}

void checkMqttConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        return;
    }

    if (mqttClient.connected()) {
        return;
    }

    const unsigned long now = millis();
    if (now - lastMqttAttempt <= MQTT_RECONNECT_INTERVAL_MS) {
        return;
    }

    lastMqttAttempt = now;

    char clientId[40];
    snprintf(clientId, sizeof(clientId), "%s%04X", MQTT_CLIENT_ID_PREFIX, static_cast<unsigned int>(random(0x10000)));

    // --- Last Will & Testament (seismo/status / QoS 1) ---
    // Spec §2.1: LWT is published automatically by the broker if the sensor
    // disconnects unexpectedly, so the server marks the station offline.
    char stationId[STATION_ID_BUFFER_SIZE];
    getStationIdCopy(stationId, sizeof(stationId));

    StaticJsonDocument<256> lwtDoc;
    lwtDoc["id"]     = stationId;
    lwtDoc["status"] = "offline";
    char lwtBuffer[256];
    size_t lwtLen = serializeJson(lwtDoc, lwtBuffer, sizeof(lwtBuffer));

    mqttClient.setWill(
        MQTT_TOPIC_STATUS,
        reinterpret_cast<const uint8_t*>(lwtBuffer),
        lwtLen,
        /*retained=*/false,
        /*qos=*/1
    );

    if (mqttClient.connect(clientId, mqtt_user, mqtt_password)) {
        mqttClient.subscribe(MQTT_TOPIC_COMMAND);
        Serial.println("MQTT Connected");
        if (!startupMessageSent) {
            sendMqttStartupMessage();
            startupMessageSent = true;
        }
    } else {
        Serial.printf("MQTT failed, rc=%d\n", mqttClient.state());
    }
}