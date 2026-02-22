/**
 * QuakeAlert ESP32 - MQTT Logic
 */

#ifndef MQTT_H
#define MQTT_H

#include <Arduino.h>

void sendMqttAlert(String intensity, float pga_value);
bool sendMqttReport(String lokasi, String waktu, float durasi, String pga_str, String intensitas);
void sendMqttStartupMessage();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void checkMqttConnection();

#endif  // MQTT_H
