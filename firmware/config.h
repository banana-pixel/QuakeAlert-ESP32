/**
 * QuakeAlert ESP32 - Configuration
 * Constants, pin definitions, and compile-time settings.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ========================================
// 1. SYSTEM CONFIGURATION
// ========================================
#define WDT_TIMEOUT 30
#define FIRMWARE_VERSION "6.9.6"

#define BOOT_SETTLING_TIME_MS 15000
#define MAX_FRAGMENTATION_PERCENT 50.0f

// ========================================
// 2. RTOS / TASK CONFIGURATION
// ========================================
#define SENSOR_TASK_STACK_SIZE 10000
#define SENSOR_TASK_PRIORITY 1
#define SENSOR_TASK_CORE 0

#define NETWORK_MAINTENANCE_TASK_STACK_SIZE 8192
#define NETWORK_MAINTENANCE_TASK_PRIORITY 0
#define NETWORK_MAINTENANCE_TASK_CORE 1

#define NETWORK_MAINTENANCE_INTERVAL_MS 250
#define NETWORK_RECOVERY_DELAY_MS 1000

// ========================================
// 3. MQTT CONFIGURATION
// ========================================
#define CUSTOM_MQTT_KEEPALIVE 15
#define MQTT_TOPIC_ALERT   "seismo/alert"
#define MQTT_TOPIC_REPORT  "seismo/report"
#define MQTT_TOPIC_COMMAND "seismo/command"
#define MQTT_TOPIC_STATUS  "seismo/status"
#define MQTT_CLIENT_ID_PREFIX "ESP32-Seismo-"

// ========================================
// 4. DETECTION CONFIGURATION (STA/LTA)
// ========================================
// STA/LTA: Short-Term/Long-Term Average ratio algorithm.
//   STA tracks instantaneous shaking energy (fast window).
//   LTA tracks the ambient noise floor    (slow window).
//   A spike in STA/LTA reliably identifies a seismic onset,
//   adapting automatically to local background noise level.
//
// EMA Baseline: A very slow EMA tracks the combined gravity + DC offset so
//   that the corrected magnitude used by STA/LTA is gravity-free.
//   BASELINE_ALPHA must be very small — too large and it tracks real shaking.
//
// STA alpha  controls the speed of the short-term detector:
//   larger alpha = faster response, noisier; smaller = smoother, slower.
// LTA alpha  controls the speed of the noise-floor estimate:
//   smaller alpha = more stable floor; too small = slow to adapt after re-siting.
#define BASELINE_ALPHA           0.005f            // EMA baseline: ~200 sample time-constant
#define STA_ALPHA                (1.0f / 50.0f)    // STA window: ~0.5 s at 100 Hz
#define LTA_ALPHA                (1.0f / 2000.0f)  // LTA window: ~20 s at 100 Hz
#define MIN_LTA_CLAMP            0.5f              // Minimum LTA (gal) to prevent divide-by-zero
#define STA_LTA_TRIGGER_RATIO    4.0f              // STA/LTA ratio to open event
#define STA_LTA_DETRIGGER_RATIO  1.5f              // STA/LTA ratio to close event
#define MIN_STA_THRESHOLD_GAL    1.5f              // Minimum STA (gal) — suppresses noise triggers
#define CONFIRMATION_DURATION_MS 300               // ms STA/LTA must stay triggered before confirming
#define MAX_EVENT_DURATION_MS    60000             // Maximum event duration cap (ms)
#define EVENT_COOLDOWN_PERIOD_MS 60000             // Minimum gap between reported events (ms)
#define LTA_WARMUP_TIME_MS       45000             // Boot time before detection activates (ms)
#define DATA_RATIO               (980.665f / 8192.0f) // 1g = 980.665 gal; ±4g -> 8192 LSB/g

// ========================================
// 5. TIMING & STABILITY
// ========================================
#define HTTP_TIMEOUT_MS 3000
#define MPU_ERROR_THRESHOLD_NOTIF 25
#define WIFI_RECONNECT_INTERVAL_MS 30000
#define MQTT_RECONNECT_INTERVAL_MS 5000
#define NTP_SYNC_INTERVAL_MS 3600000
#define NTP_RETRY_INTERVAL_MS 60000
#define LOCATION_RETRY_INTERVAL_MS 60000
#define HEARTBEAT_INTERVAL_MS 60000

// ========================================
// 6. JSON / BUFFER SIZING
// ========================================
#define MQTT_ALERT_JSON_CAPACITY    512
#define MQTT_REPORT_JSON_CAPACITY   768
#define MQTT_STATUS_JSON_CAPACITY   1024
#define MQTT_HEARTBEAT_JSON_CAPACITY 320
#define MQTT_REPORT_BUFFER_SIZE     1024
#define MQTT_STATUS_BUFFER_SIZE     1024
#define MQTT_ALERT_BUFFER_SIZE      512
#define MQTT_HEARTBEAT_BUFFER_SIZE  320
#define LOCATION_TEXT_BUFFER_SIZE   96
#define STATION_ID_BUFFER_SIZE      16
#define INTENSITY_TEXT_BUFFER_SIZE  24
#define TIME_TEXT_BUFFER_SIZE       32

// ========================================
// 7. HARDWARE PINS
// ========================================
#define LED_BUILTIN_PIN 2
#define INTERRUPT_PIN 15
#define SDA_PIN 21
#define SCL_PIN 22

#endif  // CONFIG_H