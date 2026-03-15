/**
 * QuakeAlert ESP32 - Network & Location Implementation
 *
 * Privacy-centric architecture:
 *   - No IP-geolocation HTTP calls (HTTPClient removed entirely)
 *   - HTML5 navigator.geolocation injected into the WiFiManager captive
 *     portal so the user's phone GPS auto-fills the lat/lon fields once
 *   - Coordinates persisted to NVS (Preferences) and loaded on every boot
 *   - Raw coordinates kept full-precision in memory; only the MQTT payloads
 *     receive 2-decimal-place masking (~1.1 km anonymity box)
 */

#include "network.h"
#include "config.h"
#include "state.h"
#include "utils.h"
#include "mqtt.h"

#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <HTTPClient.h>
#include "secrets.h"

// ============================================================
// Internal helpers (anonymous namespace — translation-unit scope)
// ============================================================
namespace {

// ------------------------------------------------------------------
// Coordinate masking
// Truncates a coordinate to 2 decimal places using pure arithmetic
// (no heap allocation).  Equivalent to String(coord, 2).toFloat()
// but avoids the Arduino String allocator entirely.
//   e.g.  -6.83312f  →  -6.83f
//          106.78901f →  106.79f
// ------------------------------------------------------------------
float maskCoord(float coord) {
    return roundf(coord * 100.0f) / 100.0f;
}

// ------------------------------------------------------------------
// Thread-safe location state writers
// ------------------------------------------------------------------
void setLocationCoordinates(float latitude, float longitude) {
    if (stateMutex != nullptr) {
        xSemaphoreTake(stateMutex, portMAX_DELAY);
    }

    stationLat = latitude;
    stationLon = longitude;

    if (stateMutex != nullptr) {
        xSemaphoreGive(stateMutex);
    }
}

void setLocationTextAndCoordinates(const char* lokasi,
                                   float latitude,
                                   float longitude) {
    setLokasiAlat(lokasi);
    setLocationCoordinates(latitude, longitude);

    if (stateMutex != nullptr) {
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        locationResolved = true;
        xSemaphoreGive(stateMutex);
    } else {
        locationResolved = true;
    }
}

void clearLocationCoordinates() {
    setLocationCoordinates(0.0f, 0.0f);
}

}  // namespace

// ============================================================
// initWifi — WiFiManager portal with simple credentials
// ============================================================
void initWifi() {
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);

    // Static HTML info panel explaining the automated process
    WiFiManagerParameter portal_info(
        "<div style='background:#f0f4ff;border-left:4px solid #1fa3ec;"
                    "padding:12px 14px;margin:10px 0;border-radius:3px;"
                    "font-size:13px;line-height:1.6;color:#333;'>"
          "<b>&#128205; Automatic Geolocation Setup</b><br>"
          "Just connect to your home WiFi. This device will scan "
          "nearby router signals to determine its location automatically."
        "</div>"
    );

    wm.addParameter(&portal_info);

    // If no WiFi credentials exist (or reset button was pushed), start AP
    bool portalSuccess = wm.autoConnect("Quake-Setup");

    if (!portalSuccess) {
        Serial.println("WiFiManager connect/portal failed. Continuing in degraded mode.");
        setLocationStatusWifiDisconnected();
        clearLocationCoordinates();
        return;
    }

    Serial.println("WiFi connected.");
}

// ============================================================
// maintainWifiConnection
// ============================================================
bool maintainWifiConnection() {
    if (WiFi.status() == WL_CONNECTED) {
        if (wifiFailCount > 0) {
            Serial.println("WiFi Recovered!");
            wifiFailCount    = 0;
            lastNtpAttempt   = 0;
        }
        return true;
    }

    if (millis() - lastWifiCheck < WIFI_RECONNECT_INTERVAL_MS) {
        return false;
    }

    lastWifiCheck = millis();
    wifiFailCount++;

    Serial.printf("WiFi Lost. Attempt %lu\n",
                  static_cast<unsigned long>(wifiFailCount));

    if (wifiFailCount <= 3) {
        WiFi.reconnect();
    } else {
        Serial.println("WiFi Hard Reset...");
        WiFi.disconnect();
        vTaskDelay(pdMS_TO_TICKS(NETWORK_RECOVERY_DELAY_MS));
        WiFi.reconnect();
    }

    setLocationStatusWifiDisconnected();
    clearLocationCoordinates();
    return false;
}

// ============================================================
// checkNtpSync
// ============================================================
void checkNtpSync() {
    if (WiFi.status() != WL_CONNECTED) {
        setNtpSyncStatus(false);
        return;
    }

    const unsigned long now = millis();

    if (isNtpSynced) {
        if (now - lastNtpSync > NTP_SYNC_INTERVAL_MS) {
            setNtpSyncStatus(false);
        } else {
            return;
        }
    }

    if (lastNtpAttempt != 0 && (now - lastNtpAttempt < NTP_RETRY_INTERVAL_MS)) {
        return;
    }

    lastNtpAttempt = now;
    Serial.println("Performing NTP Sync...");

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 2000)) {
        setNtpSyncStatus(true);
        lastNtpSync = now;
        Serial.println("NTP Synced Successfully");
    } else {
        setNtpSyncStatus(false);
        Serial.println("NTP Sync Failed - Retrying later");
    }
}

// ============================================================
// refreshLocation — Auto-Geolocation via Mozilla Location Service
//
// Scans nearby WiFi networks and securely POSTs the BSSIDs
// to Mozilla's Free Geolocation API to resolve latitude/longitude.
// Coordinates are cached in NVS to preserve API quota if the
// device reboots without moving.
// ============================================================
bool refreshLocation() {
    Preferences prefs;
    prefs.begin("quake-app", false); 

    float lat = prefs.getFloat("lat", 0.0f);
    float lon = prefs.getFloat("lon", 0.0f);

    // If we already have coordinates from a previous run, use them to save Quota
    if (lat != 0.0f && lon != 0.0f) {
        setLocationTextAndCoordinates("Community Node", lat, lon);
        Serial.printf("Location loaded from NVS cache: %.6f, %.6f\n", lat, lon);
        prefs.end();
        return true;
    }

    Serial.println("No location in NVS. Scanning WiFi to auto-locate...");
    
    // 1. Scan nearby networks
    int numNetworks = WiFi.scanNetworks(false, true); // Async=false, showHidden=true
    if (numNetworks < 2) {
        Serial.println("Not enough WiFi networks nearby to triangulate location.");
        setLocationStatusUnknown();
        prefs.end();
        return false;
    }

    // 2. Build Mozilla API JSON Payload
    DynamicJsonDocument doc(4096);
    doc["considerIp"] = "false";
    JsonArray wifiAccessPoints = doc.createNestedArray("wifiAccessPoints");
    
    // Only send the top 15 strongest networks to save memory and bandwidth
    int maxNets = (numNetworks > 15) ? 15 : numNetworks;
    for (int i = 0; i < maxNets; i++) {
        JsonObject ap = wifiAccessPoints.createNestedObject();
        ap["macAddress"] = WiFi.BSSIDstr(i);
        ap["signalStrength"] = WiFi.RSSI(i);
        // Do not include SSID to maximize privacy
    }
    WiFi.scanDelete();

    String jsonPayload;
    serializeJson(doc, jsonPayload);

    // 3. Make the API Call to Free BeaconDB Location Service
    HTTPClient http;
    // BeaconDB is a public-domain drop-in replacement for Mozilla Location Service
    // It requires no API keys and is permanently free for community use.
    String url = "https://beacondb.net/v1/geolocate";
    
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    
    Serial.println("Requesting coordinates from Mozilla Geolocation API...");
    int httpResponseCode = http.POST(jsonPayload);

    if (httpResponseCode == 200) {
        String response = http.getString();
        
        DynamicJsonDocument jsonResponse(1024);
        DeserializationError error = deserializeJson(jsonResponse, response);
        
        if (!error) {
            lat = jsonResponse["location"]["lat"];
            lon = jsonResponse["location"]["lng"];
            
            // Save to NVS so we don't spam the API on every boot
            prefs.putFloat("lat", lat);
            prefs.putFloat("lon", lon);
            
            setLocationTextAndCoordinates("Community Node", lat, lon);
            Serial.printf("Auto-Geolocation Successful: %.6f, %.6f\n", lat, lon);
            
            prefs.end();
            http.end();
            return true;
        }
    } else {
        Serial.printf("Geolocation API error: %d\n", httpResponseCode);
        Serial.println(http.getString());
    }
    
    setLocationStatusUnknown();
    prefs.end();
    http.end();
    return false;
}

// ============================================================
// networkMaintenanceTask — pinned to Core 1
// ============================================================
void networkMaintenanceTask(void* pvParameters) {
    (void)pvParameters;

    Serial.println("Network Maintenance Task running on Core 1");
    // Ensure the Task Watchdog monitors this thread so it catches any deadlocks
    esp_task_wdt_add(nullptr);

    for (;;) {
        esp_task_wdt_reset();

        const bool wifiConnected = maintainWifiConnection();

        if (wifiConnected) {
            checkNtpSync();

            // refreshLocation() is a cheap NVS read (< 1 ms), not an HTTP
            // call.  We only retry while locationResolved is false — once the
            // NVS key is present it will succeed and the flag stays true until
            // the next WiFi drop (which calls clearLocationCoordinates() and
            // sets locationResolved = false via setLocationStatusWifiDisconnected).
            if (!locationResolved) {
                lastLocRetry = millis();
                refreshLocation();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(NETWORK_MAINTENANCE_INTERVAL_MS));
    }
}

// ============================================================
// sendHeartbeat — coordinates masked to 2 dp before publish
// ============================================================
void sendHeartbeat() {
    if (!mqttClient.connected()) {
        return;
    }

    static unsigned long lastLatency = 0;
    const unsigned long startTimer   = millis();

    char stationId[STATION_ID_BUFFER_SIZE];
    char lokasi[LOCATION_TEXT_BUFFER_SIZE];
    char rssiText[20];
    char latencyText[20];

    getStationIdCopy(stationId, sizeof(stationId));
    getLokasiAlatCopy(lokasi, sizeof(lokasi));

    snprintf(rssiText,    sizeof(rssiText),    "%ld dBm", WiFi.RSSI());
    snprintf(latencyText, sizeof(latencyText), "%lu ms",  lastLatency);

    StaticJsonDocument<MQTT_HEARTBEAT_JSON_CAPACITY> doc;
    doc["stationId"] = stationId;
    doc["rssi"]      = rssiText;
    doc["status"]    = "online";
    doc["lokasi"]    = lokasi;
    doc["latency"]   = latencyText;

    // Mask to 2 decimal places, then format as a strict string to avoid
    // IEEE-754 binary approximation artifacts when ArduinoJson serializes
    // a raw float (e.g. -6.15 → "-6.150000095").
    if (stationLat != 0.0f || stationLon != 0.0f) {
        char latStr[16];
        char lonStr[16];
        snprintf(latStr, sizeof(latStr), "%.2f", maskCoord(stationLat));
        snprintf(lonStr, sizeof(lonStr), "%.2f", maskCoord(stationLon));
        doc["lat"] = latStr;
        doc["lon"] = lonStr;
    }

    char jsonBuffer[MQTT_HEARTBEAT_BUFFER_SIZE];
    const size_t jsonLength = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
    if (jsonLength == 0 || jsonLength >= sizeof(jsonBuffer) || doc.overflowed()) {
        Serial.println("Heartbeat serialization failed");
        return;
    }

    if (mqttPublishJson("seismo/heartbeat", jsonBuffer, jsonLength)) {
        lastLatency = millis() - startTimer;
    }
}