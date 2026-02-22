/**
 * QuakeAlert ESP32 - Core Utilities Implementation
 */

#include "utils.h"
#include "state.h"

void monitorHeap() {
    if (millis() - lastHeapCheck > 10000) {
        lastHeapCheck = millis();
        uint32_t freeHeap = ESP.getFreeHeap();
        uint32_t maxAllocHeap = ESP.getMaxAllocHeap();

        if (freeHeap < minHeapSeen) minHeapSeen = freeHeap;

        if (freeHeap < 15000) {
            Serial.println("CRITICAL: Heap exhausted (<15KB), restarting...");
            delay(100);
            ESP.restart();
        }

        float fragmentation = 100.0f * (1.0f - ((float)maxAllocHeap / (float)freeHeap));
        if (fragmentation > MAX_FRAGMENTATION_PERCENT) {
            Serial.printf("CRITICAL: Heap Fragmentation High (%.2f%%). Restarting...\n", fragmentation);
            delay(100);
            ESP.restart();
        }
    }
}

String getWaktuString() {
    if (!isNtpSynced) return "N/A";
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return "N/A";
    char timeStringBuff[30];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(timeStringBuff);
}
