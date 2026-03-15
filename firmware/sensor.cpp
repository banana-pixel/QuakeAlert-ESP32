/**
 * QuakeAlert ESP32 - Sensor & Detection Implementation
 */

#include "sensor.h"
#include "config.h"
#include "state.h"
#include "utils.h"

#include <Wire.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <math.h>

// Ensure Arduino LED define exists in this translation unit too
#ifndef LED_BUILTIN
#define LED_BUILTIN LED_BUILTIN_PIN
#endif

void IRAM_ATTR DMPDataReady() {
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if (mpuInterruptSemaphore != nullptr) {
        xSemaphoreGiveFromISR(mpuInterruptSemaphore, &higherPriorityTaskWoken);
    }

    if (higherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

const char* toIntensity(float pga_val) {
    return intensityToText(pga_val);
}

bool initializeSensorInterrupts() {
    if (mpuInterruptSemaphore == nullptr) {
        return false;
    }

    xSemaphoreTake(mpuInterruptSemaphore, 0);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    return true;
}

void initMPU() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Wire.setTimeOut(3000);

    if (!xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(2000))) {
        Serial.println("Failed to take mutex for MPU init");
        return;
    }

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
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

        if (initializeSensorInterrupts()) {
            Serial.println("MPU6050 Initialized & Configured (FS_4/100Hz, ISR semaphore)");
        } else {
            Serial.println("MPU6050 init warning: interrupt semaphore not ready");
        }
    } else {
        DMPReady = false;
        Serial.printf("MPU6050 init failed: %u\n", devStatus);
    }

    xSemaphoreGive(i2cMutex);
}

void processSensorData() {
    if (!DMPReady) {
        return;
    }

    if (!xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100))) {
        return;
    }

    const uint8_t intStatus = mpu.getIntStatus();
    if (intStatus & 0x10) {
        Serial.println("MPU FIFO overflow, resetting...");
        mpu.resetFIFO();
        mpuErrorCounter = 0;
        mpuOverflowCount++;
        xSemaphoreGive(i2cMutex);
        return;
    }

    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer) == 0) {
        mpuErrorCounter++;
        if (mpuErrorCounter > MPU_ERROR_THRESHOLD_NOTIF) {
            Serial.println("MPU read errors exceeded threshold. Resetting FIFO...");
            mpu.resetFIFO();
            mpuErrorCounter = 0;
        }
        xSemaphoreGive(i2cMutex);
        return;
    }

    mpuErrorCounter = 0;
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    xSemaphoreGive(i2cMutex);

    // --- 1. Vector Magnitude (direct multiply — avoids slow powf()) ---
    const float vectorMagnitude = sqrtf(
        (aaReal.x * aaReal.x) +
        (aaReal.y * aaReal.y) +
        (aaReal.z * aaReal.z)
    ) * DATA_RATIO;

    // --- 2. EMA Baseline (gravity + DC-offset removal) ---
    // BASELINE_ALPHA is intentionally very small (0.005) so this EMA tracks
    // only the slow-moving gravity component and long-term sensor drift,
    // NOT the rapid accelerations of a seismic event.
    static float baselineEMA = 0.0f;
    baselineEMA = (BASELINE_ALPHA * vectorMagnitude) + ((1.0f - BASELINE_ALPHA) * baselineEMA);

    // correctedMagnitude is gravity-free; all downstream detection uses this.
    const float correctedMagnitude = fabsf(vectorMagnitude - baselineEMA);

    // --- 3. STA / LTA EMA filters ---
    // STA_ALPHA (1/50 ≈ 0.02):  ~0.5 s time-constant at 100 Hz.
    //   Increase to make the detector respond faster (but noisier).
    //   Decrease to smooth out transient spikes (but slower onset detection).
    //
    // LTA_ALPHA (1/2000 = 0.0005): ~20 s time-constant at 100 Hz.
    //   Increase to let the noise floor adapt faster after sensor re-siting.
    //   Decrease for a more stable, robust floor in a fixed installation.
    //
    // LTA is frozen while eventInProgress is true so that the earthquake
    // energy itself does not inflate the noise floor estimate, which would
    // prematurely lower the STA/LTA ratio and detrigger the event early.
    static float sta = 0.0f;
    static float lta = MIN_LTA_CLAMP;  // pre-loaded to clamp value for safe first ratio

    sta = (STA_ALPHA * correctedMagnitude) + ((1.0f - STA_ALPHA) * sta);
    if (!eventInProgress) {
        lta = (LTA_ALPHA * correctedMagnitude) + ((1.0f - LTA_ALPHA) * lta);
    }

    // Ignore all detection logic during the LTA warm-up period so the
    // noise floor has time to settle before any triggers are evaluated.
    if (millis() < LTA_WARMUP_TIME_MS) {
        return;
    }

    // Clamp LTA to a minimum to prevent divide-by-zero in near-silence.
    const float ltaClamped = max(lta, MIN_LTA_CLAMP);
    const float ratio = sta / ltaClamped;

    // --- 4. Cooldown gate (skip detection if we are between events) ---
    if (!eventInProgress && (millis() - lastReportTime < EVENT_COOLDOWN_PERIOD_MS)) {
        return;
    }

    // --- 5. Trigger: potential-event detection ---
    if (!eventInProgress && !potentialEvent) {
        if (ratio >= STA_LTA_TRIGGER_RATIO && sta >= MIN_STA_THRESHOLD_GAL) {
            potentialEvent = true;
            potentialEventTime = millis();
            Serial.printf(
                "[STA/LTA] Trigger! ratio=%.2f  STA=%.2f gal  LTA=%.2f gal\n",
                ratio,
                sta,
                ltaClamped
            );
        }
    }

    // --- 6. Confirmation: hold above threshold for CONFIRMATION_DURATION_MS ---
    if (potentialEvent && !eventInProgress) {
        const bool stillAbove =
            (ratio >= STA_LTA_TRIGGER_RATIO && sta >= MIN_STA_THRESHOLD_GAL);

        if (stillAbove) {
            if (millis() - potentialEventTime >= CONFIRMATION_DURATION_MS) {
                eventInProgress = true;
                eventStartTime = potentialEventTime;
                // PGA tracking uses correctedMagnitude (gal) — the real physical force,
                // not the dimensionless STA/LTA ratio, so MQTT reports are meaningful.
                pga = correctedMagnitude;
                Serial.println("[STA/LTA] Event confirmed - PGA tracking started.");

                portENTER_CRITICAL(&eventTriggerMux);
                eventTriggered = true;
                portEXIT_CRITICAL(&eventTriggerMux);
            }
        } else {
            potentialEvent = false;
            Serial.printf("[STA/LTA] Trigger cancelled (transient). ratio=%.2f\n", ratio);
        }
    }

    // --- 7. Active event: PGA tracking & de-trigger ---
    if (eventInProgress) {
        // Track peak correctedMagnitude (gal) throughout the event.
        pga = max(pga, correctedMagnitude);

        const bool timeOutReached = (millis() - eventStartTime > MAX_EVENT_DURATION_MS);
        const bool ratioDropped = (ratio < STA_LTA_DETRIGGER_RATIO);

        if (ratioDropped || timeOutReached) {
            if (timeOutReached) {
                Serial.println("[STA/LTA] Event forced end (timeout)");
            }

            portENTER_CRITICAL(&reportMux);
            pendingReport.maxPga = pga;
            pendingReport.duration = (millis() - eventStartTime) / 1000.0f;
            pendingReport.timestamp = millis();
            pendingReport.ready = true;
            pendingReport.processed = false;
            portEXIT_CRITICAL(&reportMux);

            lastReportTime = millis();
            eventInProgress = false;
            potentialEvent = false;
            alertSent = false;
            pga = 0.0f;
        }
    }
}

void updateSensorStatusLed() {
    if (mpuErrorCounter > 5) {
        if (millis() - lastBlinkTime > 150) {
            lastBlinkTime = millis();
            ledState = !ledState;
            digitalWrite(LED_BUILTIN, ledState);
        }
    } else if (eventInProgress) {
        if (millis() - lastBlinkTime > 250) {
            lastBlinkTime = millis();
            ledState = !ledState;
            digitalWrite(LED_BUILTIN, ledState);
        }
    } else {
        digitalWrite(LED_BUILTIN, (WiFi.status() == WL_CONNECTED) ? LOW : HIGH);
    }
}

void sensorTask(void* pvParameters) {
    (void)pvParameters;

    Serial.println("Sensor Task running on Core 0");
    esp_task_wdt_add(nullptr);

    for (;;) {
        if (mpuInterruptSemaphore == nullptr) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (xSemaphoreTake(mpuInterruptSemaphore, portMAX_DELAY) == pdTRUE) {
            esp_task_wdt_reset();
            processSensorData();
            updateSensorStatusLed();
        }
    }
}