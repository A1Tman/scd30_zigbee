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
- [Changelog](CHANGELOG.md)
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
- **ESP-IDF v5.5.3** (tested)
- **ESP Zigbee SDK** (bundled with ESP-IDF)

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
   - Set flash size to **16 MB** for ESP32-C6-WROOM-1-N16 boards
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
- Startup stabilization: 10 seconds (`SCD30_STARTUP_STABILIZATION_MS`) after continuous measurement starts
  - Measurements are taken immediately
  - Zigbee publishes are deferred until readings stabilize

**Compensation:**
- Temperature offset: Hardware offset is set to 2.5 C by default
- Environmental compensation: choose one of these approaches, not both
  - Pressure compensation: default 1013 mbar
  - Altitude compensation: disabled by default until explicitly set

**Error Handling & Validation:**
- Automatic sensor initialization with up to 3 retry attempts
- Measurement validation against defined ranges:
  - CO₂: `SCD30_CO₂_MIN` to `SCD30_CO₂_MAX` ppm
  - Temperature: `SCD30_TEMP_MIN` to `SCD30_TEMP_MAX` °C  
  - Humidity: `SCD30_HUM_MIN` to `SCD30_HUM_MAX` %
- Automatic sensor reset after consecutive errors (`SCD30_MAX_CONSECUTIVE_ERRORS`)
- CRC validation for all I2C communication
- Recovery delay (`SCD30_RECOVERY_DELAY_MS`) after sensor reset
- Forced recalibration support
  - Use only when the sensor has been at a known reference concentration for at least 2 minutes

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
4. **Measurement task starts** - The application starts the SCD30 task as soon as the sensor is configured
5. **Startup stabilization** - The device waits for valid readings before publishing to Zigbee, without blocking boot

### Normal Operation
Once connected, the device will:
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

The device exposes standard measurement clusters plus a manufacturer-specific control cluster at `0xFC00`. To make those extra controls readable in ZHA, add a custom quirk and then build your helpers and scripts on top of the cluster attributes.

### 1. Enable custom quirks

```yaml
# configuration.yaml
zha:
  custom_quirks_path: '/config/custom_zha_quirks'
```

### 2. Add the custom quirk

```
/config
└── custom_zha_quirks/
    └── custom_co2_sensor.py
```

```python
from zigpy.profiles import zha
from zigpy.quirks import CustomCluster, CustomDevice
from zigpy.zcl.clusters.general import Basic, Identify
from zigpy.zcl.clusters.measurement import (
    CarbonDioxideConcentration,
    RelativeHumidity,
    TemperatureMeasurement,
)
from zigpy.zcl.clusters.homeautomation import Diagnostic
import zigpy.types as t
from zhaquirks.const import (
    DEVICE_TYPE,
    ENDPOINTS,
    INPUT_CLUSTERS,
    MODELS_INFO,
    OUTPUT_CLUSTERS,
    PROFILE_ID,
)


class CO2ControlCluster(CustomCluster, Diagnostic):
    """SCD30 control cluster exposed by the ESP32 firmware."""

    cluster_id = 0xFC00
    name = "CO2 Control"
    ep_attribute = "co2_control"

    attributes = {
        0x0000: ("auto_calibrate", t.Bool, True),
        0x0001: ("temp_offset_x100", t.int16s, True),
        0x0002: ("pressure_comp_mbar", t.uint16_t, True),
        0x0003: ("altitude_comp_m", t.uint16_t, True),
        0x0004: ("force_recalibration_ppm", t.uint16_t, True),
    }

    server_commands = {}
    client_commands = {}


class ESP32CO2Device(CustomDevice):
    signature = {
        MODELS_INFO: [("ESPRESSIF", "esp32c6")],
        ENDPOINTS: {
            12: {
                PROFILE_ID: zha.PROFILE_ID,
                DEVICE_TYPE: 0xFFF2,
                INPUT_CLUSTERS: [
                    Basic.cluster_id,
                    Identify.cluster_id,
                    CarbonDioxideConcentration.cluster_id,
                    TemperatureMeasurement.cluster_id,
                    RelativeHumidity.cluster_id,
                    0xFC00,
                ],
                OUTPUT_CLUSTERS: [],
            },
        },
    }

    replacement = {
        ENDPOINTS: {
            12: {
                PROFILE_ID: zha.PROFILE_ID,
                DEVICE_TYPE: 0xFFF2,
                INPUT_CLUSTERS: [
                    Basic,
                    Identify,
                    CarbonDioxideConcentration,
                    TemperatureMeasurement,
                    RelativeHumidity,
                    CO2ControlCluster,
                ],
                OUTPUT_CLUSTERS: [],
            },
        },
    }
```

Restart Home Assistant after adding the quirk. If the device was already paired before the quirk existed, re-interview it or re-pair it so ZHA picks up the custom cluster mapping.

If you intentionally enable the maintenance-only firmware controls, extend the quirk with:
- `0x0005: ("restart_measurement", t.Bool, True)`
- `0x0006: ("debug_command", t.uint8_t, True)`

### 3. Optional: use ZHA Toolkit for inspection and troubleshooting

ZHA Toolkit is useful for reading manufacturer-specific attributes, testing writes, and checking cluster behavior while building dashboards and scripts.

Suggested workflow:
- Install ZHA Toolkit in Home Assistant
- Restart Home Assistant
- Use it to inspect endpoint `12`, cluster `0xFC00`, and attributes `0x0000` through `0x0004`
- Use `zha_toolkit.attr_write` for writes to the manufacturer-specific control cluster

For this device, `zha_toolkit.attr_write` has proven more reliable than the built-in `zha.set_zigbee_cluster_attribute` action for custom `0xFC00` attributes. In Home Assistant, switch the action editor to YAML mode and use the device IEEE address directly.

Useful ZCL attribute types for this cluster:
- `0x0000 auto_calibrate` -> `attr_type: 16` (`0x10`, bool)
- `0x0001 temp_offset_x100` -> `attr_type: 41` (`0x29`, int16)
- `0x0002 pressure_comp_mbar` -> `attr_type: 33` (`0x21`, uint16)
- `0x0003 altitude_comp_m` -> `attr_type: 33` (`0x21`, uint16)
- `0x0004 force_recalibration_ppm` -> `attr_type: 33` (`0x21`, uint16)

### 4. Create helper entities

If you prefer dashboards based on helpers instead of writing raw cluster values directly, create `input_boolean` and `input_number` helpers and then sync them with ZHA scripts.

```yaml
# configuration.yaml or helpers package
input_boolean:
  scd30_auto_calibration:
    name: SCD30 Auto Calibration
    icon: mdi:auto-fix

input_number:
  scd30_temperature_offset:
    name: SCD30 Temperature Offset
    min: -10
    max: 10
    step: 0.1
    unit_of_measurement: "°C"
  scd30_pressure_compensation:
    name: SCD30 Pressure Compensation
    min: 700
    max: 1400
    step: 1
    unit_of_measurement: "mbar"
  scd30_altitude_compensation:
    name: SCD30 Altitude Compensation
    min: 0
    max: 3000
    step: 1
    unit_of_measurement: "m"
  scd30_force_recalibration:
    name: SCD30 Force Recalibration
    min: 400
    max: 2000
    step: 1
    unit_of_measurement: "ppm"
```

### 5. Write scripts that push helper values to the device

Replace `<YOUR_DEVICE_IEEE>` with the sensor's IEEE address from ZHA.

```yaml
# scripts.yaml
scd30_toggle_asc:
  alias: "SCD30 - Toggle auto calibration"
  sequence:
    - action: zha_toolkit.attr_write
      data:
        ieee: "<YOUR_DEVICE_IEEE>"
        endpoint: 12
        cluster: 0xFC00
        attribute: 0
        attr_type: 16
        attr_val: "{{ 1 if is_state('input_boolean.scd30_auto_calibration', 'on') else 0 }}"
        read_before_write: false
        read_after_write: false
        tries: 3

scd30_apply_temperature_offset:
  alias: "SCD30 - Apply temperature offset"
  sequence:
    - action: zha_toolkit.attr_write
      data:
        ieee: "<YOUR_DEVICE_IEEE>"
        endpoint: 12
        cluster: 0xFC00
        attribute: 1
        attr_type: 41
        attr_val: "{{ (states('input_number.scd30_temperature_offset') | float(0) * 100) | round(0) | int }}"
        read_before_write: false
        read_after_write: false
        tries: 3

scd30_apply_pressure_compensation:
  alias: "SCD30 - Apply pressure compensation"
  sequence:
    - action: zha_toolkit.attr_write
      data:
        ieee: "<YOUR_DEVICE_IEEE>"
        endpoint: 12
        cluster: 0xFC00
        attribute: 2
        attr_type: 33
        attr_val: "{{ states('input_number.scd30_pressure_compensation') | int(1013) }}"
        read_before_write: false
        read_after_write: false
        tries: 3

scd30_apply_altitude_compensation:
  alias: "SCD30 - Apply altitude compensation"
  sequence:
    - action: zha_toolkit.attr_write
      data:
        ieee: "<YOUR_DEVICE_IEEE>"
        endpoint: 12
        cluster: 0xFC00
        attribute: 3
        attr_type: 33
        attr_val: "{{ states('input_number.scd30_altitude_compensation') | int(0) }}"
        read_before_write: false
        read_after_write: false
        tries: 3

scd30_calibrate_outdoor:
  alias: "SCD30 - Calibrate to outdoor air"
  sequence:
    - action: zha_toolkit.attr_write
      data:
        ieee: "<YOUR_DEVICE_IEEE>"
        endpoint: 12
        cluster: 0xFC00
        attribute: 4
        attr_type: 33
        attr_val: "{{ states('input_number.scd30_force_recalibration') | int(400) }}"
        read_before_write: false
        read_after_write: false
        tries: 3
```

Recommended usage:
- Use either `pressure_comp_mbar` or `altitude_comp_m`
- Keep `force_recalibration_ppm` for deliberate maintenance actions, not routine automations
- After writing a new temperature offset or compensation value, wait for the next stabilization window before judging the reading

### 6. Optional dashboard layout

The custom quirk exposes readable entities for the `0xFC00` cluster, and the helper-based workflow gives you stable UI controls for toggles, numbers, and scripts. A practical dashboard split is:
- Current readings
- Quick actions such as outdoor calibration or environment presets
- Environmental compensation
- Advanced calibration controls

---

## Zigbee Cluster Attributes

| Attribute ID | Name                      | Type    | Description                       |
| -----------: | ------------------------- | ------- | --------------------------------- |
|     `0x0000` | `auto_calibrate`          | Boolean | Enable/disable auto-calibration   |
|     `0x0001` | `temp_offset_x100`        | Int16   | Temperature offset (×100)         |
|     `0x0002` | `pressure_comp_mbar`      | Uint16  | Pressure compensation (mbar)      |
|     `0x0003` | `altitude_comp_m`         | Uint16  | Altitude compensation (meters)    |
|     `0x0004` | `force_recalibration_ppm` | Uint16  | Force a one-off recalibration     |
|     `0x0005` | `restart_measurement`     | Boolean | Optional maintenance-only control |
|     `0x0006` | `debug_command`           | Uint8   | Optional maintenance-only control |

Notes:
- The default firmware exposes only attributes `0x0000` through `0x0004`
- Attributes `0x0005` and `0x0006` are available only if `ENABLE_MAINTENANCE_ZIGBEE_CONTROLS` is set to `1` in `main/components/zigbee_handler/zigbee_handler.h`
- Do not drive `pressure_comp_mbar` and `altitude_comp_m` at the same time

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
> - **Workaround:** Use the long-press rejoin action or re-pair the device in ZHA if network state is stale.  
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
