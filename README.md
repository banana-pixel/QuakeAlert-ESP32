# ESP32 MQTT Earthquake Sensor

This project is a custom firmware for the ESP32 that detects seismic activity using an MPU6050 and reports data via MQTT.

## Attribution
This project is a heavily modified fork of **[QuakeCord](https://github.com/KnowScratcher/QuakeCord)**.
* **Original License:** GNU AGPL v3
* **Modifications:**
    * Replaced Discord Webhooks with MQTT for direct IoT integration.
    * Implemented FreeRTOS dual-core multitasking (Sensor on Core 1, Network on Core 0) to ensure data integrity.
    * Added NTP time synchronization and automatic clock drift compensation.
    * Implemented automatic baseline calibration for temperature drift.

## License
Licensed under the GNU Affero General Public License v3.0 (AGPL v3). See the LICENSE file for full text.

## Hardware Requirements
* **Microcontroller:** ESP32 Development Board (e.g., ESP-WROOM-32).
* **Sensor:** MPU6050 Accelerometer/Gyroscope module (GY-521 recommended).
* **Connections:**
    * VCC -> 3.3V
    * GND -> GND
    * SDA -> GPIO 21
    * SCL -> GPIO 22
    * INT -> GPIO 15

## Configuration and Installation

### 1. Library Dependencies
Ensure the following libraries are installed in your Arduino IDE or PlatformIO environment:
* `WiFiManager` by tzapu
* `PubSubClient` by Nick O'Leary
* `ArduinoJson` by Benoit Blanchon
* `MPU6050` (ElectronicCats/mpu6050)

### 2. MQTT Credentials Setup
Before flashing the firmware, you must configure your MQTT broker credentials.

1. Copy the example secrets file and edit it with your real values (do **not** commit `secrets.h`):
   ```bash
   cp firmware/secrets.h.example firmware/secrets.h
   ```
2. Edit `firmware/secrets.h` and set:
   - `SECRET_MQTT_SERVER` — your broker hostname (e.g. `quakealert.example.com`)
   - `SECRET_MQTT_PORT` — usually `1883`
   - `SECRET_MQTT_USER` and `SECRET_MQTT_PASS` — must match the server's MQTT user/password (see QuakeAlert-Server `.env` and `config/pwfile`)

`secrets.h` is listed in `.gitignore`; never commit it or push it to the repo.

### 3. First Boot and WiFi Setup

This firmware uses WiFiManager to handle network credentials, avoiding the need to hardcode WiFi passwords.

1. Flash the firmware to your ESP32.
2. Upon first boot, the device will create a WiFi Access Point named "Quake-Setup".
3. Connect to this network using your phone or computer.
4. A captive portal should automatically open (or navigate to 192.168.4.1 in your browser).
5. Select your home WiFi network, enter the password, and save.
6. The device will reboot and connect to your WiFi and MQTT broker automatically.

## MQTT Interface

The device communicates using the following MQTT topics.

### Published Topics

#### seismo/alert

Published immediately when vibration exceeds the trigger threshold (PGA > 8.0 gal).

* **Payload Example:**
```json
{
  "lokasi": "Jakarta, Indonesia",
  "waktu": "06-02-2026 18:30:00 WIB",
  "intensitas": "IV (Ringan)",
  "pga": "15.40"
}

```



#### seismo/report

Published after a seismic event concludes. Contains a summary of the event.

* **Payload Example:**
```json
{
  "stationId": "SEIS-01",
  "lokasi": "Jakarta, Indonesia",
  "waktu": "06-02-2026 18:30:00 WIB",
  "durasi": 45.5,
  "pga": "15.40 gal",
  "intensitas": "IV (Ringan)",
  "deskripsi": "Jendela/pintu berderik."
}

```



#### seismo/status

Publishes device health statistics (uptime, heap memory, signal strength). Sent on startup and upon request.

### Subscribed Topics

#### seismo/command

Send text payloads to this topic to control the device remotely.

* **Commands:**
* `ping`: Forces the device to publish a status update to `seismo/status`.
* `reboot`: Remotely restarts the ESP32.
* `stats`: Publishes debug statistics, including total events detected and sensor error counts.



## Localization and Data Fields

The device uses Indonesian terms for JSON data fields and status messages. Below is a translation guide for the data keys.

### JSON Field Translations

* `lokasi`: Location (City, Region, Country)
* `waktu`: Time (Format: DD-MM-YYYY HH:MM:SS WIB)
* `intensitas`: Intensity (Modified Mercalli Scale)
* `durasi`: Duration (in seconds)
* `deskripsi`: Description of the intensity level

### Time Zone

The time is formatted in WIB (Waktu Indonesia Barat), which is UTC+7.

### Intensity Scale (MMI)

The device reports intensity using the following Indonesian descriptions:

* **Tidak Terasa:** Not Felt (I)
* **Lemah:** Weak (II-III)
* **Ringan:** Light (IV)
* **Sedang:** Moderate (V)
* **Kuat:** Strong (VI)
* **Sangat Kuat:** Very Strong (VII)
* **Merusak:** Destructive (VIII)
* **Hebat:** Violent (IX)
* **Ekstrem:** Extreme (X+)
