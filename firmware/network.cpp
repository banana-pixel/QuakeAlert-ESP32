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
#include <WebServer.h>
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
// initWifi — ESPAsyncWebServer custom portal
// ============================================================
bool isProvisioningMode = false;
WebServer* configServer = nullptr;

void initWifi() {
    Preferences prefs;
    prefs.begin("quake-app", false);
    String ssid = prefs.getString("ssid", "");
    String pass = prefs.getString("password", "");
    prefs.end();

    if (ssid.length() > 0) {
        Serial.println("Saved WiFi credentials found. Starting in STATION mode.");
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid.c_str(), pass.c_str());
        return;
    }

    Serial.println("No WiFi credentials found. Starting in AP mode for provisioning.");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("QuakeSetup");
    isProvisioningMode = true;

    configServer = new WebServer(80);
    
    configServer->on("/config", HTTP_POST, []() {
        if (configServer->hasArg("plain")) {
            String body = configServer->arg("plain");
            DynamicJsonDocument doc(1024);
            DeserializationError err = deserializeJson(doc, body);
            if (!err) {
                Preferences p;
                p.begin("quake-app", false);
                p.putString("ssid", doc["ssid"] | "");
                p.putString("password", doc["password"] | "");
                
                float lat = doc["lat"] | 0.0f;
                float lon = doc["lon"] | 0.0f;
                if (lat != 0.0f || lon != 0.0f) {
                    p.putFloat("lat", lat);
                    p.putFloat("lon", lon);
                }
                p.end();
                
                configServer->send(200, "application/json", "{\"status\":\"success\"}");
                Serial.println("Successfully saved new credentials via /config API.");
                extern volatile bool rebootRequestReceived;
                rebootRequestReceived = true;
            } else {
                configServer->send(400, "application/json", "{\"status\":\"error\"}");
                Serial.printf("Failed to parse config JSON: %s\n", err.c_str());
            }
        } else {
            configServer->send(400, "application/json", "{\"status\":\"error\"}");
        }
    });

    configServer->on("/scan", HTTP_GET, []() {
        Serial.println("Starting WiFi scan...");
        WiFi.mode(WIFI_AP_STA);
        int n = WiFi.scanNetworks();
        DynamicJsonDocument doc(2048);
        JsonArray array = doc.to<JsonArray>();
        
        if (n > 0) {
            for (int i = 0; i < n; ++i) {
                String currentSSID = WiFi.SSID(i);
                if (currentSSID.length() > 0) {
                    bool duplicate = false;
                    for (JsonVariant v : array) {
                        if (v.as<String>() == currentSSID) {
                            duplicate = true;
                            break;
                        }
                    }
                    if (!duplicate && currentSSID != "QuakeSetup") {
                        array.add(currentSSID);
                    }
                }
            }
        }
        
        String jsonString;
        serializeJson(doc, jsonString);
        configServer->send(200, "application/json", jsonString);
        WiFi.scanDelete();
        WiFi.mode(WIFI_AP);
        Serial.println("WiFi scan complete & results sent.");
    });

    configServer->begin();
    Serial.println("Synchronous WebServer started successfully (tcp_alloc core-lock safe)");
}

void handleProvisioningLoop() {
    if (isProvisioningMode && configServer != nullptr) {
        configServer->handleClient();
    }
}

// ============================================================
// maintainWifiConnection
// ============================================================
bool maintainWifiConnection() {
    extern bool isProvisioningMode;
    if (isProvisioningMode) {
        // We are waiting for the API injection; skip connection maintenance logs
        return false;
    }

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
        
        // Use custom preferences connection string to safely hard reset without
        // relying strictly on the active WiFi struct cache.
        Preferences p;
        p.begin("quake-app", true);
        String s = p.getString("ssid", "");
        String pw = p.getString("password", "");
        p.end();
        if (s.length() > 0) {
            WiFi.begin(s.c_str(), pw.c_str());
        } else {
            WiFi.reconnect();
        }
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
// resolveLocationFromIP — IP-based fallback (city-level, ~1-5 km)
//
// Used when WiFi BSSID triangulation returns no results (e.g. the local
// area has not yet been crowdsourced into BeaconDB).  For a fixed seismic
// sensor, city-level accuracy is perfectly acceptable.
// Returns true on success and saves coordinates to NVS.
// ============================================================
static bool resolveLocationFromIP(Preferences& prefs, float& lat, float& lon) {
    HTTPClient http;
    // https://ipinfo.io/json — completely free, no key required, rate limit 50k/month
    http.begin("https://ipinfo.io/json");
    http.addHeader("Accept", "application/json");

    Serial.println("BSSID triangulation failed. Falling back to IP geolocation...");
    int code = http.GET();

    if (code == 200) {
        String body = http.getString();
        DynamicJsonDocument resp(1024);
        if (!deserializeJson(resp, body)) {
            // ipinfo returns "loc" as "lat,lng" string, e.g. "-6.2146,106.8451"
            const char* locStr = resp["loc"];
            if (locStr != nullptr) {
                char buf[32];
                strncpy(buf, locStr, sizeof(buf) - 1);
                buf[sizeof(buf) - 1] = '\0';

                char* comma = strchr(buf, ',');
                if (comma != nullptr) {
                    *comma = '\0';
                    lat = strtof(buf,     nullptr);
                    lon = strtof(comma+1, nullptr);

                    prefs.putFloat("lat", lat);
                    prefs.putFloat("lon", lon);

                    Serial.printf("IP Geolocation Successful: %.6f, %.6f\n", lat, lon);
                    http.end();
                    return true;
                }
            }
        }
    }

    Serial.printf("IP Geolocation failed: HTTP %d\n", code);
    http.end();
    return false;
}

// ============================================================
// refreshLocation — Two-stage Auto-Geolocation
//
// Stage 1: BeaconDB BSSID triangulation (precise, no key needed).
//          Falls through to Stage 2 if the region has no coverage yet.
// Stage 2: ipinfo.io IP geolocation (city-level, always works).
// Coordinates are cached to NVS so neither API is hit again until
// the device is factory-reset.
// ============================================================
bool refreshLocation() {
    Preferences prefs;
    prefs.begin("quake-app", false);

    float lat = prefs.getFloat("lat", 0.0f);
    float lon = prefs.getFloat("lon", 0.0f);

    // Use cached NVS coordinates to avoid spamming remote APIs
    if (lat != 0.0f && lon != 0.0f) {
        setLocationTextAndCoordinates("Community Node", lat, lon);
        Serial.printf("Location loaded from NVS cache: %.6f, %.6f\n", lat, lon);
        prefs.end();
        return true;
    }

    // ---- Stage 1: BSSID Triangulation via BeaconDB ----
    Serial.println("No location in NVS. Scanning WiFi for BSSID triangulation...");
    int numNetworks = WiFi.scanNetworks(false, true);

    if (numNetworks >= 2) {
        DynamicJsonDocument doc(4096);
        doc["considerIp"] = false;
        JsonArray aps = doc.createNestedArray("wifiAccessPoints");

        int maxNets = (numNetworks > 15) ? 15 : numNetworks;
        for (int i = 0; i < maxNets; i++) {
            JsonObject ap = aps.createNestedObject();
            ap["macAddress"]    = WiFi.BSSIDstr(i);
            ap["signalStrength"] = WiFi.RSSI(i);
        }
        WiFi.scanDelete();

        String payload;
        serializeJson(doc, payload);

        HTTPClient http;
        http.begin("https://beacondb.net/v1/geolocate");
        http.addHeader("Content-Type", "application/json");

        Serial.println("Requesting coordinates from BeaconDB...");
        int code = http.POST(payload);

        if (code == 200) {
            String body = http.getString();
            DynamicJsonDocument resp(1024);
            if (!deserializeJson(resp, body)) {
                lat = resp["location"]["lat"];
                lon = resp["location"]["lng"];

                if (lat != 0.0f && lon != 0.0f) {
                    prefs.putFloat("lat", lat);
                    prefs.putFloat("lon", lon);
                    setLocationTextAndCoordinates("Community Node", lat, lon);
                    Serial.printf("BeaconDB Geolocation OK: %.6f, %.6f\n", lat, lon);
                    prefs.end();
                    http.end();
                    return true;
                }
            }
        } else {
            Serial.printf("BeaconDB API returned %d (no local coverage, trying IP fallback)\n", code);
        }
        http.end();
    } else {
        WiFi.scanDelete();
        Serial.println("Not enough nearby networks for BSSID triangulation.");
    }

    // ---- Stage 2: IP-based fallback via ipinfo.io ----
    if (resolveLocationFromIP(prefs, lat, lon)) {
        setLocationTextAndCoordinates("Community Node", lat, lon);
        prefs.end();
        return true;
    }

    setLocationStatusUnknown();
    prefs.end();
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