![Version](https://img.shields.io/github/v/release/A1Tman/scd30_zigbee?style=flat-square&color=blue)
![License](https://img.shields.io/github/license/A1Tman/scd30_zigbee?style=flat-square&color=green)
![ESP32](https://img.shields.io/badge/Platform-ESP32--C6-red?style=flat-square&logo=espressif)
![Zigbee](https://img.shields.io/badge/Protocol-Zigbee%203.0-blue?style=flat-square)
![Home Assistant](https://img.shields.io/badge/Integration-Home%20Assistant-41BDF5?style=flat-square&logo=home-assistant)
![Status](https://img.shields.io/badge/Status-Production%20Ready-brightgreen?style=flat-square)


# ESP32 Zigbee CO₂ Sensor

An ESP32-based CO₂, temperature, and humidity sensor that integrates with Zigbee networks using the SCD30 sensor module.

## Table of Contents

- [Hardware Requirements](#hardware-requirements)
  - [Wiring Diagram](#wiring-diagram)
  - [I2C Configuration](#i2c-configuration)
- [Features](#features)
- [Software Dependencies](#software-dependencies)
- [Building and Flashing](#building-and-flashing)
- [Zigbee Configuration](#zigbee-configuration)
- [Sensor Configuration](#sensor-configuration)
- [Project Structure](#project-structure)
- [Usage](#usage)
  - [Initial Startup](#initial-startup)
  - [Normal Operation](#normal-operation)
  - [Expected Readings](#expected-readings)
  - [Home Assistant Setup](#home-assistant-setup)
  - [Zigbee Cluster Attributes](#zigbee-cluster-attributes)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Hardware Requirements

- ESP32-C6 development board (with Zigbee support)
- Sensirion SCD30 CO₂ sensor

### Wiring Diagram

```
ESP32-C6                    SCD30 Sensor
------------------------------------
GPIO 21 (SDA) ------------- SDA
GPIO 22 (SCL) ------------- SCL  
3.3V          ------------- VIN
GND           ------------- GND
```

### I2C Configuration
- **SDA**: GPIO 21
- **SCL**: GPIO 22  
- **Power**: 3.3V supplied by ESP32
- **I2C Address**: 0x61 (SCD30 default)

For complete ESP32-C6 GPIO information, see the [official documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/api-reference/peripherals/gpio.html).

## Features

- **SCD30 Sensor Integration**
  - Uses Sensirion SCD30 sensor for CO₂, temperature, and humidity measurements
  - Integrates with Zigbee protocol for network communication
  - Configurable temperature offset and altitude compensation
  - Automatic measurement intervals

- **User Controls**
  - Boot button interface for debugging and troubleshooting
  - Factory reset and network rejoin capabilities
  - Diagnostic information display
  - Debug mode toggle

- **Reliability Features**
  - Automatic device reset on persistent errors
  - I2C bus recovery mechanisms
  - Measurement validation and retry logic
  - Comprehensive error logging

## Software Dependencies

**Required Framework:**
- **ESP-IDF v5.1 or later** (required for ESP32-C6 Zigbee support)
- **ESP Zigbee SDK** (included with ESP-IDF v5.1+)

**Installation:**
- Follow the [ESP-IDF Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/get-started/)
- Ensure Zigbee components are enabled in ESP-IDF installation

**Included Components:**
- FreeRTOS (built into ESP-IDF)
- Required ESP-IDF components: `driver`, `esp_timer`, `nvs_flash`, `esp_pm`

**Supported Platforms:**
- Windows, Linux, macOS

## Building and Flashing

1. Install ESP-IDF and set up the development environment

2. Configure the project:
   ```bash
   idf.py menuconfig
   ```
   - Enable Zigbee End Device role
   - Configure any necessary GPIO pins
   - Set up any additional project configurations

3. Build and flash the project:
   ```bash
   # One-step build, erase, and flash
   idf.py build erase-flash flash --port COM[X]
   
   # Or separately:
   idf.py build
   idf.py -p COM[X] flash
   ```

Replace `COM[X]` with your actual port (e.g., `COM3` on Windows, `/dev/ttyUSB0` on Linux).

## Zigbee Configuration

The device operates as a Zigbee End Device (ZED) with the following clusters:
- Basic Cluster
- Identify Cluster  
- Carbon Dioxide Measurement Cluster
- Temperature Measurement Cluster
- Humidity Measurement Cluster
- CO₂ Control Cluster (0xFC00) with `AUTO_CALIBRATE` attribute

Device specifications:
- Profile ID: Home Automation (0x0104)
- Device ID: Custom CO₂ Sensor (`ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID`)
- Endpoint: 12 (`HA_CUSTOM_CO₂_ENDPOINT`)
- Manufacturer: ESPRESSIF
- Model: ESP32-C6

Network configuration:
- Role: Zigbee End Device (ZED)
- Primary channels: All Zigbee channels (11-26)
- Keep alive: 3000ms
- End device aging timeout: 64 minutes

## Sensor Configuration

The SCD30 sensor is configured with:

**Measurement Settings:**
- Measurement interval: Configurable (`SCD30_MEASUREMENT_INTERVAL` seconds)
- Reading frequency: Every 5 seconds when data is available
- Warm-up delay: 5 seconds (`SCD30_WARMUP_TIME_MS`) after starting continuous measurement

**Compensation:**
- Temperature offset: Configurable (`SCD30_SW_TEMP_OFFSET`) with both hardware and software compensation
- Environmental compensation: Choose between:
  - Pressure compensation (`DEFAULT_PRESSURE_MBAR`) when `SCD30_USE_PRESSURE_COMP` is enabled
  - Altitude compensation (`SCD30_DEFAULT_ALTITUDE` meters) when pressure compensation is disabled

**Error Handling & Validation:**
- Automatic sensor initialization with up to 3 retry attempts
- Measurement validation against defined ranges:
  - CO₂: `SCD30_CO₂_MIN` to `SCD30_CO₂_MAX` ppm
  - Temperature: `SCD30_TEMP_MIN` to `SCD30_TEMP_MAX` °C  
  - Humidity: `SCD30_HUM_MIN` to `SCD30_HUM_MAX` %
- Automatic sensor reset after consecutive errors (`SCD30_MAX_CONSECUTIVE_ERRORS`)
- CRC validation for all I2C communication
- Recovery delay (`SCD30_RECOVERY_DELAY_MS`) after sensor reset

## Project Structure

```
├── main/
│   └── components/
│       ├── common/              # Shared definitions and utilities
│       ├── i2c_handler/         # I2C communication component
│       ├── scd30_driver/        # SCD30 sensor driver component
│       ├── troubleshooting/     # Diagnostic and debugging utilities
│       └── zigbee_handler/      # Zigbee communication component
├── CMakeLists.txt              # CMake build configuration
├── dependencies.lock           # ESP-IDF component dependencies
├── partitions.csv              # Flash partition table
├── sdkconfig                   # ESP-IDF project configuration
├── LICENSE                     # MIT License
├── NOTICE                      # Third-party notices
└── Sensirion_CO₂_Sensors_SCD30_Interface_Descr... # Sensor documentation
```

## Usage

### Initial Startup
1. **Power up the device** - The ESP32 will boot and initialize
2. **Sensor initialization** - SCD30 will be configured with temperature offset and environmental compensation
3. **Network joining** - Device automatically attempts to join available Zigbee networks across channels 11-26
4. **Warm-up period** - After Zigbee connection, wait 5 seconds for sensor stabilization

### Normal Operation
Once connected and warmed up, the device will:
- **Take measurements every 5 seconds** when data is available
- **Log readings to console**:
```bash
CO₂: 450.0 ppm, Temperature: 23.45°C, Humidity: 45.2%
```
- **Update Zigbee attributes** automatically for coordinator polling
- **Validate readings** against configured ranges and retry on errors
- **Auto-recover** from sensor communication issues
- **Control auto calibration** via the `AUTO_CALIBRATE` attribute in the CO₂ Control cluster

Home Assistant users can toggle this attribute from the device page to start or stop
the SCD30's automatic self calibration routine.

### Expected Readings
- **CO₂**: 400-10,000 ppm (typical indoor: 400-1000 ppm)
- **Temperature**: Ambient temperature minus configured offset
- **Humidity**: 0-100% relative humidity

### Network Behavior
- **Connection indicators**: Watch console logs for "Successfully joined network" messages
- **Coordinator polling**: Values available to Zigbee coordinator on-demand (no automatic reporting)
- **Auto-reconnection**: Device attempts to rejoin if connection is lost

### Button Controls

The boot button (GPIO 9) provides diagnostic and control functions:

#### Quick Press Patterns
Complete within 3 seconds:

* **Two quick presses** - Display diagnostic information
* **Three quick presses** - Toggle debug mode for detailed logging  
* **Four quick presses** - Factory reset (**WARNING**: Erases all Zigbee settings)

#### Long Press
* **Hold for 3 seconds** - Force network rejoin
  * Use when device can't connect or needs to join different network

> **Note:** All button presses include 100ms debounce protection.

## Home Assistant Setup

To integrate the ESP32 Zigbee CO₂ sensor with Home Assistant using ZHA and your custom quirks, add the following to your configuration.

---

### ZHA Configuration

```yaml
# configuration.yaml
zha:
  # … your existing ZHA setup …
  custom_quirks_path: '/config/custom_zha_quirks'
```

---

### Folder Structure

```
/config
└── custom_zha_quirks/
    └── custom_co2_sensor.py
```

---

### Scripts

```yaml
# scripts.yaml

# 1) Calibrate to outdoor baseline (400 ppm)
scd30_calibrate_outdoor:
  alias: "SCD30 – Calibrate to Outdoor Air (400 ppm)"
  sequence:
    - action: zha.set_zigbee_cluster_attribute
      data:
        ieee: "<YOUR_DEVICE_IEEE>"
        endpoint_id: 12
        cluster_id: 0xFC00            # CO2ControlCluster
        cluster_type: in
        attribute: 4                  # force_recalibration_ppm
        value: 400
    - service: persistent_notification.create
      data:
        title: "SCD30 Calibration"
        message: "Calibrated to 400 ppm outdoor baseline"

# 2) Enable auto-calibration
scd30_enable_auto_calibration:
  alias: "SCD30 – Enable Auto Calibration"
  sequence:
    - action: zha.set_zigbee_cluster_attribute
      data:
        ieee: "<YOUR_DEVICE_IEEE>"
        endpoint_id: 12
        cluster_id: 0xFC00            # CO2ControlCluster
        cluster_type: in
        attribute: 0                  # auto_calibrate
        value: true
    - service: persistent_notification.create
      data:
        title: "SCD30 Auto Calibration"
        message: "Auto-calibration enabled"
```

---

### Number Helpers

```yaml
# templates/numbers.yaml

- name: "SCD30 Temperature Offset"
  unique_id: scd30_temp_offset
  state: >-
    {{ (states('sensor.co2_control_cluster_temp_offset_x100')|int(0)) / 100 }}
  min: -10.0
  max: 10.0
  step: 0.1
  unit_of_measurement: "°C"
  set_value:
    - action: zha.set_zigbee_cluster_attribute
      data:
        ieee: "<YOUR_DEVICE_IEEE>"
        endpoint_id: 12
        cluster_id: 0xFC00
        cluster_type: in
        attribute: 1                  # temp_offset_x100
        value: "{{ (value * 100)|int }}"

- name: "SCD30 Pressure Compensation"
  unique_id: scd30_pressure_comp
  state: >-
    {{ states('sensor.co2_control_cluster_pressure_comp_mbar')|int(1013) }}
  min: 700
  max: 1400
  step: 1
  unit_of_measurement: "mbar"
  set_value:
    - action: zha.set_zigbee_cluster_attribute
      data:
        ieee: "<YOUR_DEVICE_IEEE>"
        endpoint_id: 12
        cluster_id: 0xFC00
        cluster_type: in
        attribute: 2                  # pressure_comp_mbar
        value: "{{ value|int }}"
```

---

### Switch Helpers

```yaml
# templates/switches.yaml

- name: "SCD30 Auto Calibration"
  unique_id: scd30_auto_calibration
  icon: mdi:auto-fix
  state: >
    {{ is_state('input_boolean.co2_auto_calibration','on') }}
  turn_on:
    - service: zha.set_zigbee_cluster_attribute
      data:
        ieee: "<YOUR_DEVICE_IEEE>"
        endpoint_id: 12
        cluster_id: 0xFC00
        cluster_type: in
        attribute: 0                  # auto_calibrate
        value: true
  turn_off:
    - service: zha.set_zigbee_cluster_attribute
      data:
        ieee: "<YOUR_DEVICE_IEEE>"
        endpoint_id: 12
        cluster_id: 0xFC00
        cluster_type: in
        attribute: 0                  # auto_calibrate
        value: false
```

> **Note:**
>
> * Replace `<YOUR_DEVICE_IEEE>` with your sensor’s IEEE address.
> * Adjust any min/max values or aliases to suit your setup.
> * Add additional scripts or helpers as needed.

---

## Zigbee Cluster Attributes

| Attribute ID | Name                      | Type    | Description                       |
| -----------: | ------------------------- | ------- | --------------------------------- |
|     `0x0000` | `auto_calibrate`          | Boolean | Enable/disable auto-calibration   |
|     `0x0001` | `temp_offset_x100`        | Int16   | Temperature offset (×100)         |
|     `0x0002` | `pressure_comp_mbar`      | Uint16  | Pressure compensation (mbar)      |
|     `0x0003` | `altitude_comp_m`         | Uint16  | Altitude compensation (meters)    |
|     `0x0004` | `force_recalibration_ppm` | Uint16  | Force a one-off recalibration     |
|     `0x0005` | `restart_measurement`     | Boolean | Restart sensor measurements       |
|     `0x0006` | `debug_command`           | Uint8   | Send a debug command code (0–255) |

---

## Troubleshooting

### A. Serial-Console Debugging  
When connected via USB and viewing logs:

> **Stuck in Bootloader**  
> - **Signs:** Repeated “Waiting for download” or no “chip Revision” messages.  
> - **Fix:** Hold BOOT while resetting; verify USB-to-UART wiring and drivers.

> **I²C Errors on Startup**  
> - **Signs:** “NACK” or “Sensor init failed” in logs.  
> - **Fix:** Double-check SDA/SCL wiring and pin assignments; confirm menuconfig pins match.

> **Zigbee Join Failures**  
> - **Signs:** “Network start error” or endless “Attempting to join” loops.  
> - **Fix:** Ensure coordinator channels (11–26) are permitted, correct PAN ID/join permissions, or increase the join timeout.

### B. Production / Headless Deployment  

> **Device fails to rejoin after power‑cycle**  
> - **Symptom:** Sensor disappears from Home Assistant when moved and won’t auto-rejoin.  
> - **Workaround:** Press the “Restart Measurements” button every few seconds until it reappears; if not, remove and re-pair in HA.  
> - **If HA won’t refresh:** Restart Home Assistant to clear cache, then re-pair.

> **No Readings Over Zigbee**  
> - **Symptom:** HA never receives updates.  
> - **Fix:** Look for join LED blink, temporarily connect via USB for logs, or force rejoin via button.

> **Stale or Drifting CO₂ Values**  
> - **Symptom:** Readings remain constant or slowly drift.  
> - **Fix:** Perform manual calibration under known CO₂; allow up to 14 days for auto-calibration to stabilize.

> **Unexpected Resets / Brown‑outs**  
> - **Symptom:** Device reboots intermittently.  
> - **Fix:** Verify a stable 3.3 V supply with sufficient current and solid wiring; add a small decoupling capacitor near VIN/GND if needed.


## Contributing

Please submit issues and pull requests with any improvements or bug fixes.

## License
This project is licensed under the MIT License. See [LICENSE](LICENSE).
