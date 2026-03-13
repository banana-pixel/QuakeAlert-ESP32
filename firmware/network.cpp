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
// initWifi — WiFiManager portal with HTML5 GPS capture
// ============================================================
void initWifi() {
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);

    // -----------------------------------------------------------------
    // Custom form fields.
    // The parameter id ("lat" / "lon") becomes the HTML <input id="…">
    // which the injected JavaScript targets with getElementById().
    // -----------------------------------------------------------------
    WiFiManagerParameter customLat("lat", "Latitude  (e.g. -6.200000)",  "", 20);
    WiFiManagerParameter customLon("lon", "Longitude (e.g. 106.816666)", "", 20);

    // Raw-HTML parameter: renders a styled button above the text fields.
    // WiFiManager accepts a single-argument constructor that injects the
    // string verbatim into the config page body — no id/label/value needed.
    WiFiManagerParameter gpsButton(
        "<p style='margin:10px 0;'>"
          "<button type='button' "
            "style='width:100%;padding:12px;background:#1fa3ec;"
                   "color:#fff;border:0;border-radius:3px;"
                   "font-size:14px;cursor:pointer;'"
            " onclick='getGPS()'>"
            "&#128205; Use This Device&apos;s GPS"
          "</button>"
        "</p>"
    );

    // JavaScript injected into the page <head>.
    // getCurrentPosition() populates both inputs; the user can then hit
    // "Save" to submit the form and connect to WiFi in one step.
    //
    // Note: navigator.geolocation works over HTTP on Android captive-portal
    // browsers (Chrome treats captive portals as a special case).
    // iOS Safari requires HTTPS, so iPhone users may need to type manually.
    const char* gpsHeadElement =
        "<script>"
        "function getGPS(){"
          "if(!navigator.geolocation){"
            "alert('Geolocation not supported. Please type coordinates manually.');"
            "return;"
          "}"
          "navigator.geolocation.getCurrentPosition("
            "function(p){"
              "document.getElementById('lat').value"
                "=p.coords.latitude.toFixed(6);"
              "document.getElementById('lon').value"
                "=p.coords.longitude.toFixed(6);"
            "},"
            "function(e){"
              "alert('GPS error: '+e.message+'. Please type coordinates manually.');"
            "},"
            "{enableHighAccuracy:true,timeout:15000}"
          ");"
        "}"
        "</script>";

    wm.setCustomHeadElement(gpsHeadElement);
    wm.addParameter(&gpsButton);
    wm.addParameter(&customLat);
    wm.addParameter(&customLon);

    // -----------------------------------------------------------------
    // Force Portal on Missing Data
    //
    // Before attempting any WiFi connection, peek at NVS to check whether
    // GPS coordinates have ever been saved.  Two cases:
    //
    //   lat == 0.0f  → first boot or after factory reset; no location data.
    //                  We MUST force the AP portal open even if the device
    //                  already has a saved WiFi password, so the user can
    //                  enter their coordinates.  startConfigPortal() does this.
    //
    //   lat != 0.0f  → location already stored; a silent background
    //                  reconnect is sufficient.  autoConnect() skips the
    //                  portal if the saved credentials still work.
    // -----------------------------------------------------------------
    {
        Preferences peekPrefs;
        peekPrefs.begin("quake-app", true);   // read-only
        const float savedLat = peekPrefs.getFloat("lat", 0.0f);
        peekPrefs.end();

        bool portalSuccess = false;
        if (savedLat == 0.0f) {
            Serial.println("No GPS location in NVS — forcing config portal...");
            portalSuccess = wm.startConfigPortal("Quake-Setup");
        } else {
            portalSuccess = wm.autoConnect("Quake-Setup");
        }

        if (!portalSuccess) {
            Serial.println("WiFiManager connect/portal failed. Continuing in degraded mode.");
            setLocationStatusWifiDisconnected();
            clearLocationCoordinates();
            return;
        }
    }

    Serial.println("WiFi connected.");

    // -----------------------------------------------------------------
    // Persist lat/lon to NVS only when the user actively filled them in
    // (i.e. the portal was shown and the fields are non-empty).
    // On subsequent boots where autoConnect() is used and no portal is
    // shown, getValue() returns "" — we skip the write safely.
    // -----------------------------------------------------------------
    const char* latStr = customLat.getValue();
    const char* lonStr = customLon.getValue();

    if (latStr != nullptr && lonStr != nullptr &&
        latStr[0] != '\0' && lonStr[0] != '\0') {

        const float lat = strtof(latStr, nullptr);
        const float lon = strtof(lonStr, nullptr);

        // Guard against accidental null-island (0, 0) submissions
        if (lat != 0.0f || lon != 0.0f) {
            Preferences prefs;
            prefs.begin("quake-app", false);  // read-write
            prefs.putFloat("lat", lat);
            prefs.putFloat("lon", lon);
            prefs.end();

            Serial.printf(
                "Location saved from portal: %.6f, %.6f\n",
                static_cast<double>(lat),
                static_cast<double>(lon)
            );
        }
    }
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
// refreshLocation — NVS-backed, zero HTTP traffic
//
// Reads the lat/lon saved by initWifi() (or a previous session) from
// the "quake-app" NVS namespace and populates the global state.
// If no location has ever been saved, logs a message and returns false
// so the caller can surface a "please configure" status.
// ============================================================
bool refreshLocation() {
    Preferences prefs;
    prefs.begin("quake-app", true);  // read-only — never wear-levels unnecessarily

    const float lat = prefs.getFloat("lat", 0.0f);
    const float lon = prefs.getFloat("lon", 0.0f);

    prefs.end();

    if (lat == 0.0f && lon == 0.0f) {
        setLocationStatusUnknown();
        clearLocationCoordinates();
        Serial.println(
            "No location stored in NVS. "
            "Connect to the 'Quake-Setup' portal and tap 'Use This Device's GPS'."
        );
        return false;
    }

    // The label "Community Node" is intentionally generic: it avoids
    // leaking suburb/street data to anyone inspecting the MQTT broker.
    setLocationTextAndCoordinates("Community Node", lat, lon);

    Serial.printf(
        "Location loaded from NVS: %.6f, %.6f\n",
        static_cast<double>(lat),
        static_cast<double>(lon)
    );
    return true;
}

// ============================================================
// networkMaintenanceTask — pinned to Core 1
// ============================================================
void networkMaintenanceTask(void* pvParameters) {
    (void)pvParameters;

    Serial.println("Network Maintenance Task running on Core 1");
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

    // Mask to 2 decimal places before publishing.
    // Full-precision values remain untouched in stationLat / stationLon.
    if (stationLat != 0.0f || stationLon != 0.0f) {
        doc["lat"] = maskCoord(stationLat);
        doc["lon"] = maskCoord(stationLon);
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