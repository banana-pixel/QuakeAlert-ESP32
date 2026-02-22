/**
 * QuakeAlert ESP32 - Network & Location
 */

#ifndef NETWORK_H
#define NETWORK_H

void initWifi();
void checkNtpSync();
void getLokasi();
void handleNetworkTasks();
void sendHeartbeat();

#endif  // NETWORK_H
