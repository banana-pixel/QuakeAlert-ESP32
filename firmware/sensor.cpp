/**
 * QuakeAlert ESP32 - Sensor & Detection Implementation
 */

#include "sensor.h"
#include "config.h"
#include "state.h"
#include <Wire.h>
#include <math.h>

void IRAM_ATTR DMPDataReady() {
    MPUInterrupt = true;
}

String toIntensity(float pga_val) {
    if (pga_val < 0.5f)  return "I (Tidak Terasa)";
    if (pga_val < 2.8f)  return "II-III (Lemah)";
    if (pga_val < 6.2f)  return "IV (Ringan)";
    if (pga_val < 12.0f) return "V (Sedang)";
    if (pga_val < 22.0f) return "VI (Kuat)";
    if (pga_val < 40.0f) return "VII (Sangat Kuat)";
    if (pga_val < 75.0f) return "VIII (Merusak)";
    if (pga_val < 139.0f) return "IX (Hebat)";
    return "X+ (Ekstrem)";
}

void initMPU() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Wire.setTimeOut(3000);

    if (xSemaphoreTake(i2cMutex, 2000 / portTICK_PERIOD_MS)) {
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
            attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
            DMPReady = true;
            packetSize = mpu.dmpGetFIFOPacketSize();
            Serial.println("MPU6050 Initialized & Configured (FS_4/100Hz)");
        } else {
            DMPReady = false;
            Serial.printf("MPU6050 init failed: %d\n", devStatus);
        }
        xSemaphoreGive(i2cMutex);
    } else {
        Serial.println("Failed to take mutex for MPU Init");
    }
}

void processSensorData() {
    if (!MPUInterrupt) return;

    if (!xSemaphoreTake(i2cMutex, 100 / portTICK_PERIOD_MS)) return;

    MPUInterrupt = false;

    if (!DMPReady) {
        Serial.println("DMP not ready - skipping sensor read");
        xSemaphoreGive(i2cMutex);
        return;
    }

    uint8_t intStatus = mpu.getIntStatus();
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
            Serial.println("MPU Read Errors exceeded threshold. Resetting FIFO...");
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

    // Compute linear acceleration vector magnitude in gal (cm/s²)
    float magnitude = sqrtf(
        powf(aaReal.x, 2) + powf(aaReal.y, 2) + powf(aaReal.z, 2)
    ) * DATA_RATIO;

    // ── STA/LTA Algorithm ────────────────────────────────────────────────────
    // STA (Short-Term Average): fast EMA — tracks instantaneous shaking energy.
    // LTA (Long-Term Average):  slow EMA — represents the ambient noise floor.
    // A genuine seismic onset causes STA to spike while LTA stays stable,
    // producing a high STA/LTA ratio. This self-calibrates to the local noise
    // level: no manual threshold tuning needed for different placement sites.
    static float sta = 0.0f;
    static float lta = 0.5f;  // Non-zero init prevents divide-by-zero on first sample

    sta = (1.0f - STA_ALPHA) * sta + STA_ALPHA * magnitude;
    lta = (1.0f - LTA_ALPHA) * lta + LTA_ALPHA * magnitude;

    // Wait for LTA to warm up. LTA window ~20 s; 45 s total ensures it reflects
    // the true environmental noise floor before the first trigger is allowed.
    if (millis() < LTA_WARMUP_TIME_MS) return;

    // Guard against divide-by-zero in extremely silent environments
    float ratio = (lta > 0.05f) ? (sta / lta) : 0.0f;

    // Cooldown: suppress new event detection shortly after the previous report
    if (!eventInProgress && (millis() - lastReportTime < EVENT_COOLDOWN_PERIOD_MS)) return;

    // ── Phase 1: Potential Event ──────────────────────────────────────────────
    if (!eventInProgress && !potentialEvent) {
        if (ratio >= STA_LTA_TRIGGER_RATIO && sta >= MIN_STA_THRESHOLD_GAL) {
            potentialEvent     = true;
            potentialEventTime = millis();
            Serial.printf("[STA/LTA] Trigger! ratio=%.2f  STA=%.2f gal  LTA=%.2f gal\n",
                          ratio, sta, lta);
        }
    }

    // ── Phase 2: Confirmation (anti-spike filter) ─────────────────────────────
    // Ratio must stay elevated for CONFIRMATION_DURATION_MS before a full event
    // is declared. Single-spike mechanical jolts (footsteps, door slams) are
    // rejected because they cannot sustain a high ratio for 300 ms.
    if (potentialEvent && !eventInProgress) {
        bool stillAbove = (ratio >= STA_LTA_TRIGGER_RATIO && sta >= MIN_STA_THRESHOLD_GAL);
        if (stillAbove) {
            if (millis() - potentialEventTime >= CONFIRMATION_DURATION_MS) {
                eventInProgress = true;
                eventStartTime  = potentialEventTime;
                pga             = magnitude;
                Serial.println("[STA/LTA] Event Confirmed — PGA tracking started.");

                portENTER_CRITICAL(&eventTriggerMux);
                eventTriggered = true;
                portEXIT_CRITICAL(&eventTriggerMux);
            }
        } else {
            // Ratio fell before confirmation window — transient noise, discard
            potentialEvent = false;
            Serial.printf("[STA/LTA] Trigger cancelled (transient). ratio=%.2f\n", ratio);
        }
    }

    // ── Phase 3: Event Tracking & Termination ────────────────────────────────
    if (eventInProgress) {
        pga = max(pga, magnitude);  // Track peak ground acceleration
        bool timeOutReached = (millis() - eventStartTime > MAX_EVENT_DURATION_MS);
        bool ratioDropped   = (ratio < STA_LTA_DETRIGGER_RATIO);

        if (ratioDropped || timeOutReached) {
            if (timeOutReached) Serial.println("[STA/LTA] Event Forced End (Timeout)");

            portENTER_CRITICAL(&reportMux);
            pendingReport.maxPga    = pga;
            pendingReport.duration  = (millis() - eventStartTime) / 1000.0f;
            pendingReport.timestamp = millis();
            pendingReport.ready     = true;
            pendingReport.processed = false;
            portEXIT_CRITICAL(&reportMux);

            lastReportTime  = millis();
            eventInProgress = false;
            potentialEvent  = false;
            alertSent       = false;
            pga             = 0;
        }
    }
}
