# 2-Axis Robotic Globe Controller — Firmware Plan

## 1. Purpose

Firmware for an **ESP32 DevKitV1** that:
- Hosts a temporary Wi-Fi access point for first-time setup
- Connects to user Wi-Fi after provisioning
- Runs an HTTP server that accepts latitude / longitude
- Converts lat/lng into two servo angles
- Drives two SG90 servos through a **PCA9685**
- Physically points a globe to the requested location

---

## 2. Operating Modes

### 2.1 Provisioning Mode (AP)
Triggered when:
- No Wi-Fi credentials are stored
- Wi-Fi connection fails at boot
- User requests Wi-Fi reset

Behavior:
- ESP32 creates AP: `ROBOGLOBE-<chip_id>`
- Serves a configuration webpage
- User enters Wi-Fi SSID + password
- Credentials stored in NVS
- Device reboots or switches to Station Mode

---

### 2.2 Station Mode (Normal Operation)

Behavior:
- Connects to configured Wi-Fi network
- Starts HTTP API server
- Accepts lat/lng commands
- Updates servo positions via PCA9685

---

## 3. Hardware Control

### 3.1 Components
- ESP32 DevKitV1
- PCA9685 (I2C PWM driver)
- 2 × SG90 servos
- External 5V servo power supply

### 3.2 I2C
- SDA: GPIO 21  
- SCL: GPIO 22  
- PCA9685 address: `0x40`

### 3.3 Servo Channels
| Function | PCA9685 Channel |
|--------|-----------------|
| Latitude | 0 |
| Longitude | 1 |

---

## 4. Coordinate Mapping

### 4.1 Input
- Latitude: `-90 → +90`
- Longitude: `-180 → +180` (or `0 → 360`)

### 4.2 Latitude Servo
- Controls globe tilt
- Linear mapping:
    lat_norm = (lat + 90) / 180
    servo_angle = lat_norm * 180
- Apply calibration, inversion, offset
- Clamp to valid range

### 4.3 Longitude Servo (2:1 Gear Ratio)
- Globe rotation: 0 → 360°
- Servo rotation: 0 → 180°
- Mapping:
    lng360 = (lng + 360) % 360
    servo_angle = lng360 / 2
- Apply calibration, inversion, offset
- Clamp to valid range

---

## 5. HTTP API

### 5.1 `GET /health`
Returns device status
```json
{
"status": "ok",
"mode": "sta",
"ip": "192.168.1.42",
"uptime_ms": 123456
}
5.2 POST /point

Primary control endpoint

Request:

{
  "lat": 41.8781,
  "lng": -87.6298
}


Response:

{
  "lat": 41.8781,
  "lng": -87.6298,
  "lat_servo_deg": 131.9,
  "lng_servo_deg": 136.2
}


Behavior:

Validate inputs

Compute servo angles

Send PWM updates to PCA9685

5.3 GET /point?lat=&lng=

Query-string variant of /point

5.4 GET /state

Returns last commanded position and calibration

5.5 POST /calibrate

Updates servo calibration parameters

5.6 POST /reset-wifi

Clears stored Wi-Fi credentials and reboots into AP mode

6. Wi-Fi Provisioning Flow

ESP32 starts AP

DNS resolves all domains to device

User opens captive portal

Enters Wi-Fi credentials

Credentials stored in NVS

ESP32 attempts connection

On success, restart into Station Mode

7. Servo Control
7.1 PWM

Frequency: 50 Hz

PCA9685 generates all servo pulses

7.2 Calibration (Per Servo)

pulse_min_us

pulse_max_us

invert

offset_deg

7.3 Motion Behavior

Default: immediate move

Optional: slew-rate limited movement

Configurable degrees/second

Updated in main loop or timer task

8. Software Structure
/src
 ├─ main.cpp
 ├─ wifi_manager.cpp
 ├─ http_api.cpp
 ├─ config_store.cpp
 ├─ globe_mapping.cpp
 ├─ servo_driver.cpp

Responsibilities

main.cpp: boot logic, mode selection

wifi_manager: AP + STA handling

http_api: request parsing, responses

config_store: NVS persistence

globe_mapping: lat/lng → angles

servo_driver: PCA9685 + servo output

9. Error Handling

400: invalid latitude / longitude

500: internal hardware or I2C error

Clamp or reject out-of-range values

10. Logging

Serial output:

Boot mode

Wi-Fi status

API requests

Computed servo angles (debug mode)

11. Test Plan
Mapping Tests

Lat −90 → min tilt

Lat 0 → midpoint

Lat +90 → max tilt

Lng 0 → 0° servo

Lng 180 → 90° servo

Lng 360 → 180° servo

Integration Tests

Fresh boot → AP visible

Wi-Fi provision → reconnect after reboot

HTTP commands → physical globe movement

12. Defaults

PCA9685 address: 0x40

PWM frequency: 50 Hz

HTTP port: 80

Provisioning SSID: ROBOGLOBE-XXXX

Servo range: 0–180°

13. Out of Scope

HTTPS or authentication

Map UI or frontend visualization

Encoder feedback

High-precision positioning


If you want, next I can:
- Convert this into a **PlatformIO task checklist**
- Stub out **header files + structs**
- Or reduce this into a **single README.md** suitable for the repo root