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

void sendMqttAlert(String intensity, float pga_value) {
    if (!mqttClient.connected()) return;
    StaticJsonDocument<512> doc;
    doc["stationId"] = StationID;
    doc["lokasi"] = lokasiAlat;
    doc["lat"] = stationLat;
    doc["lon"] = stationLon;
    doc["waktu"] = getWaktuString();
    doc["intensitas"] = intensity;
    doc["pga"] = String(pga_value, 2);
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    if (mqttClient.publish(MQTT_TOPIC_ALERT, jsonBuffer)) {
        Serial.println("Alert Published!");
    } else {
        Serial.println("Alert Publish Failed!");
    }
}

bool sendMqttReport(String lokasi, String waktu, float durasi, String pga_str, String intensitas) {
    if (!mqttClient.connected()) return false;
    DynamicJsonDocument doc(1536);
    doc["stationId"] = StationID;
    doc["lokasi"] = lokasi;
    doc["lat"] = stationLat;
    doc["lon"] = stationLon;
    doc["waktu"] = waktu;
    doc["durasi"] = durasi;
    doc["pga"] = pga_str;
    doc["intensitas"] = intensitas;

    static char jsonBuffer[1536];
    size_t len = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
    if (len == 0) {
        Serial.println("Payload too large for buffer!");
        return false;
    }
    for (int i = 0; i < 3; i++) {
        if (mqttClient.publish(MQTT_TOPIC_REPORT, jsonBuffer, len)) {
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
    doc["version"] = FIRMWARE_VERSION;
    doc["restarts"] = bootCount;
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    mqttClient.publish(MQTT_TOPIC_STATUS, jsonBuffer);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    message.reserve(length);
    for (unsigned int i = 0; i < length; i++) message += (char)payload[i];

    if (String(topic) == MQTT_TOPIC_COMMAND) {
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
                tempC = (mpu.getTemperature() / 340.0f) + 36.53f;
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
            size_t outLen = serializeJson(doc, output, sizeof(output));
            if (outLen > 0)
                mqttClient.publish(MQTT_TOPIC_STATUS, output, outLen);

        } else if (message == "reboot") {
            rebootRequestReceived = true;
        } else if (message == "stats") {
            DynamicJsonDocument doc(1024);
            doc["stationId"] = StationID;
            doc["firmware"] = FIRMWARE_VERSION;
            doc["mpuErrors"] = mpuErrorCounter;
            doc["mpuOverflows"] = mpuOverflowCount;
            doc["totalEvents"] = totalEventsDetected;
            doc["minHeapEver"] = minHeapSeen;
            doc["currentHeap"] = ESP.getFreeHeap();
            doc["ntpSynced"] = isNtpSynced;
            doc["restarts"] = bootCount;

            char output[1024];
            size_t outLen = serializeJson(doc, output, sizeof(output));
            if (outLen > 0)
                mqttClient.publish(MQTT_TOPIC_STATUS, output, outLen);
        }
    }
}

void checkMqttConnection() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (!mqttClient.connected()) {
        unsigned long now = millis();
        if (now - lastMqttAttempt > MQTT_RECONNECT_INTERVAL_MS) {
            lastMqttAttempt = now;
            String clientId = "ESP32-Seismo-" + String(random(0xffff), HEX);
            if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
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
    }
}
