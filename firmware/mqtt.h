/**
 * QuakeAlert ESP32 - MQTT Logic
 */

#ifndef MQTT_H
#define MQTT_H

#include <Arduino.h>
#include <stddef.h>

void sendMqttAlert(const char* intensity, float pgaValue);
bool sendMqttReport(const char* lokasi,
                    const char* waktu,
                    float durasi,
                    const char* pgaText,
                    const char* intensitas);
void sendMqttStartupMessage();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void checkMqttConnection();

bool mqttPublishJson(const char* topic, const char* payload, size_t payloadLength);
bool mqttIsCommandTopic(const char* topic);
bool mqttPayloadToCString(const byte* payload, unsigned int length, char* output, size_t outputSize);

#endif  // MQTT_H