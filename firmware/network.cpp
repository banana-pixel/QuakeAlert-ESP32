/**
 * QuakeAlert ESP32 - Network & Location Implementation
 */

#include "network.h"
#include "config.h"
#include "state.h"
#include "utils.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

void initWifi() {
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);
    if (!wm.autoConnect("Quake-Setup")) {
        ESP.restart();
    }
}

void checkNtpSync() {
    if (WiFi.status() != WL_CONNECTED) return;
    unsigned long now = millis();

    if (isNtpSynced) {
        if (now - lastNtpSync > NTP_SYNC_INTERVAL_MS) {
            isNtpSynced = false;
        }
        return;
    }

    if (now - lastNtpAttempt < NTP_RETRY_INTERVAL_MS && lastNtpAttempt != 0) return;

    Serial.println("Performing NTP Sync...");
    lastNtpAttempt = now;
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 2000)) {
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
    http.setConnectTimeout(HTTP_TIMEOUT_MS);
    http.setTimeout(HTTP_TIMEOUT_MS);

    StaticJsonDocument<512> doc;
    String city, region, country;
    bool success = false;

    http.begin(client, "http://ip-api.com/json/?fields=country,regionName,city,lat,lon");
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        DeserializationError error = deserializeJson(doc, http.getString());
        if (!error && !doc["city"].isNull()) {
            city = doc["city"].as<String>();
            region = doc["regionName"].as<String>();
            country = doc["country"].as<String>();
            stationLat = doc["lat"];
            stationLon = doc["lon"];
            lokasiAlat = city + ", " + region + ", " + country;
            success = true;
            Serial.println("Lokasi Updated: " + lokasiAlat);
        }
    }
    http.end();

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
                if (!doc["loc"].isNull()) {
                    String loc = doc["loc"].as<String>();
                    int comma = loc.indexOf(',');
                    if (comma > 0) {
                        stationLat = loc.substring(0, comma).toFloat();
                        stationLon = loc.substring(comma + 1).toFloat();
                    }
                }
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

void handleNetworkTasks() {
    static uint32_t wifiFailCount = 0;
    static unsigned long lastLocRetry = 0;

    if (WiFi.status() != WL_CONNECTED) {
        if (millis() - lastWifiCheck > WIFI_RECONNECT_INTERVAL_MS) {
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

void sendHeartbeat() {
    if (!mqttClient.connected()) return;

    static unsigned long lastLatency = 0;
    unsigned long startTimer = millis();

    StaticJsonDocument<320> doc;
    doc["stationId"] = StationID;
    doc["rssi"] = String(WiFi.RSSI()) + " dBm";
    doc["status"] = "online";
    doc["lokasi"] = lokasiAlat;
    doc["latency"] = String(lastLatency) + " ms";
    if (stationLat != 0.0f || stationLon != 0.0f) {
        doc["lat"] = stationLat;
        doc["lon"] = stationLon;
    }

    char jsonBuffer[320];
    serializeJson(doc, jsonBuffer);

    if (mqttClient.publish("seismo/heartbeat", jsonBuffer))
        lastLatency = millis() - startTimer;
}
