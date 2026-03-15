/**
 * QuakeAlert ESP32 - Core Utilities
 */

#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <stddef.h>

struct tm;

void monitorHeap();

bool copyStringSafe(char* destination, size_t destinationSize, const char* source);
bool formatStringSafe(char* destination, size_t destinationSize, const char* format, ...);
void setLokasiAlat(const char* lokasi);
void setLocationStatusUnknown();
void setLocationStatusSearching();
void setLocationStatusWifiDisconnected();
void setLastEventTime(const char* waktu);
void setLastIntensity(const char* intensity);
void setLastPga(const char* pgaText);
void setNtpSyncStatus(bool synced);

bool getLokasiAlatCopy(char* destination, size_t destinationSize);
bool getStationIdCopy(char* destination, size_t destinationSize);
bool getLastEventTimeCopy(char* destination, size_t destinationSize);
bool getLastIntensityCopy(char* destination, size_t destinationSize);

bool getWaktuString(char* destination, size_t destinationSize);
bool getUptimeString(char* destination, size_t destinationSize);
const char* intensityToText(float pgaValue);
float calculateHeapFragmentationPercent(uint32_t freeHeap, uint32_t maxAllocHeap);

#endif  // UTILS_H