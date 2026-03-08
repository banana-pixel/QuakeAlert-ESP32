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
#define UPTIME_RESTART_THRESHOLD_MS 604800000UL  // 7 Days
#define FIRMWARE_VERSION "6.9.5"

#define SOFT_WATCHDOG_LIMIT_MS 60000
#define BOOT_SETTLING_TIME_MS 15000
#define MAX_FRAGMENTATION_PERCENT 50.0f

// ========================================
// 2. MQTT CONFIGURATION
// ========================================
#define CUSTOM_MQTT_KEEPALIVE 15
#define MQTT_TOPIC_ALERT   "seismo/alert"
#define MQTT_TOPIC_REPORT  "seismo/report"
#define MQTT_TOPIC_COMMAND "seismo/command"
#define MQTT_TOPIC_STATUS  "seismo/status"

// ========================================
// 3. DETECTION CONFIGURATION (STA/LTA)
// ========================================
// STA/LTA: Short-Term/Long-Term Average ratio algorithm.
//   STA tracks instantaneous shaking energy (fast window).
//   LTA tracks the ambient noise floor    (slow window).
//   A spike in STA/LTA reliably identifies a seismic onset,
//   adapting automatically to local background noise level.
#define STA_ALPHA               (1.0f / 50.0f)    // STA window: ~0.5 s at 100 Hz
#define LTA_ALPHA               (1.0f / 2000.0f)  // LTA window: ~20 s at 100 Hz
#define STA_LTA_TRIGGER_RATIO   3.5f              // STA/LTA ratio to open event
#define STA_LTA_DETRIGGER_RATIO 1.5f              // STA/LTA ratio to close event
#define MIN_STA_THRESHOLD_GAL   1.5f              // Minimum STA (gal) — suppresses noise triggers
#define CONFIRMATION_DURATION_MS 300              // ms STA/LTA must stay triggered before confirming
#define MAX_EVENT_DURATION_MS   60000             // Maximum event duration cap (ms)
#define EVENT_COOLDOWN_PERIOD_MS 60000            // Minimum gap between reported events (ms)
#define LTA_WARMUP_TIME_MS      45000             // Boot time before detection activates (ms)
#define DATA_RATIO (980.665f / 8192.0f)           // Standard gravity: 1g = 980.665 gal; ±4g -> 8192 LSB/g

// ========================================
// 4. TIMING & STABILITY
// ========================================
#define HTTP_TIMEOUT_MS 3000
#define MPU_ERROR_THRESHOLD_NOTIF 25
#define WIFI_RECONNECT_INTERVAL_MS 30000
#define MQTT_RECONNECT_INTERVAL_MS 5000
#define NTP_SYNC_INTERVAL_MS 3600000
#define NTP_RETRY_INTERVAL_MS 60000
#define HEARTBEAT_INTERVAL_MS 60000

// ========================================
// 5. HARDWARE PINS
// ========================================
#define LED_BUILTIN_PIN 2
#define INTERRUPT_PIN 15
#define SDA_PIN 21
#define SCL_PIN 22

#endif  // CONFIG_H
