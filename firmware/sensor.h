/**
 * QuakeAlert ESP32 - Sensor & Detection
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

String toIntensity(float pga_val);
void initMPU();
void processSensorData();

#endif  // SENSOR_H
