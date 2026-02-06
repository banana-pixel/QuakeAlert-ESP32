# ESP32 MQTT Earthquake Sensor

This project is a custom firmware for the ESP32 that detects seismic activity using an MPU6050 and reports data via MQTT.

## Attribution
This project is a heavily modified fork of **[QuakeCord](https://github.com/KnowScratcher/QuakeCord)**.
* **Original License:** GNU AGPL v3
* **Modifications:**
    * Replaced Discord Webhooks with MQTT for IoT integration.
    * Implemented FreeRTOS dual-core multitasking.
    * Added NTP time sync and drift compensation.

## License
Licensed under the GNU Affero General Public License v3.0.
