/**
 * QuakeAlert ESP32 - Sensor & Detection
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

void IRAM_ATTR DMPDataReady();
const char* toIntensity(float pga_val);
void initMPU();
void processSensorData();
void sensorTask(void* pvParameters);
bool initializeSensorInterrupts();
void updateSensorStatusLed();

#endif  // SENSOR_H