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
// 3. DETECTION CONFIGURATION
// ========================================
#define TRIGGER_THRESHOLD 8.0f
#define SUSTAIN_THRESHOLD 4.0f
#define CONFIRMATION_DURATION_MS 1000
#define MAX_EVENT_DURATION_MS 60000
#define EVENT_COOLDOWN_PERIOD_MS 60000
#define MOVING_AVERAGE_WINDOW_SIZE 5
#define DATA_RATIO (980.0f / 8192.0f)

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
