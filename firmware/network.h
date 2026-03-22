/**
 * QuakeAlert ESP32 - Network & Location
 */

#ifndef NETWORK_H
#define NETWORK_H

#include <Arduino.h>

void initWifi();
bool maintainWifiConnection();
void checkNtpSync();
bool refreshLocation();
void networkMaintenanceTask(void* pvParameters);
void sendHeartbeat();
void handleProvisioningLoop();

#endif  // NETWORK_H