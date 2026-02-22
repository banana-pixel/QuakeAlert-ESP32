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

    float vectorMagnitude = sqrtf(powf(aaReal.x, 2) + powf(aaReal.y, 2) + powf(aaReal.z, 2)) * DATA_RATIO;

    bool systemSettling = (millis() < BOOT_SETTLING_TIME_MS);

    if (isFirstReading) {
        for (int i = 0; i < MOVING_AVERAGE_WINDOW_SIZE; i++) {
            readings[i] = vectorMagnitude;
        }
        total = vectorMagnitude * MOVING_AVERAGE_WINDOW_SIZE;
        readIndex = 0;
        isFirstReading = false;
        averageMagnitude = vectorMagnitude;
    } else {
        readings[readIndex] = vectorMagnitude;
        readIndex = (readIndex + 1) % MOVING_AVERAGE_WINDOW_SIZE;

        total = 0;
        for (int i = 0; i < MOVING_AVERAGE_WINDOW_SIZE; i++) {
            total += readings[i];
        }
        averageMagnitude = total / MOVING_AVERAGE_WINDOW_SIZE;
    }

    static float baselineOffset = 0;
    static unsigned long lastCalibration = 0;

    if (!eventInProgress && !potentialEvent && (millis() - lastCalibration > 60000)) {
        if (averageMagnitude < 3.0f) {
            baselineOffset = averageMagnitude;
            lastCalibration = millis();
            Serial.printf("Baseline recalibrated: %.2f gal\n", baselineOffset);
        }
    }

    float correctedMagnitude = averageMagnitude - baselineOffset;
    if (correctedMagnitude < 0) correctedMagnitude = 0;

    if (systemSettling) return;

    if (!eventInProgress && (millis() - lastReportTime < EVENT_COOLDOWN_PERIOD_MS)) return;

    if (!eventInProgress && !potentialEvent && correctedMagnitude >= TRIGGER_THRESHOLD) {
        potentialEvent = true;
        potentialEventTime = millis();
    }

    if (potentialEvent && !eventInProgress) {
        if (correctedMagnitude >= SUSTAIN_THRESHOLD) {
            if (millis() - potentialEventTime >= CONFIRMATION_DURATION_MS) {
                eventInProgress = true;
                eventStartTime = potentialEventTime;
                pga = correctedMagnitude;

                portENTER_CRITICAL(&eventTriggerMux);
                eventTriggered = true;
                portEXIT_CRITICAL(&eventTriggerMux);
            }
        } else {
            potentialEvent = false;
        }
    }

    if (eventInProgress) {
        pga = max(pga, correctedMagnitude);
        bool timeOutReached = (millis() - eventStartTime > MAX_EVENT_DURATION_MS);
        if (correctedMagnitude < SUSTAIN_THRESHOLD || timeOutReached) {
            if (timeOutReached) Serial.println("Event Forced End (Timeout)");
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
            pga = 0;
        }
    }
}
