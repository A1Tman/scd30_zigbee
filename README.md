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
  - Simplified boot button maintenance controls
  - Hold-to-rejoin Zigbee recovery
  - Super-long-hold outdoor recalibration shortcut

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
  - The boot button shortcut uses `400 ppm` for outdoor air recalibration

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

The boot button (GPIO 9) is intentionally limited to two maintenance actions:

* **Hold for 3 seconds** - Force network rejoin
  * Use when the device cannot reconnect or needs to rejoin a coordinator after being moved
* **Hold for 12 seconds** - Trigger forced recalibration to outdoor air (`400 ppm`)
  * Use only after the sensor has been sitting in clean outdoor/reference air for at least 2 minutes
  * Keep the sensor away from your breath, doors, vents, vehicles, or other local CO₂ sources while calibrating

Short presses are ignored. All button actions include 100ms debounce protection.

## Home Assistant Setup

The device exposes standard measurement clusters plus a custom control cluster at `0xFC00`. To make those extra controls readable in ZHA, add a custom quirk and then build your helpers and scripts on top of the cluster attributes.

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
        0x0000: ("auto_calibrate", t.Bool),
        0x0001: ("temp_offset_x100", t.int16s),
        0x0002: ("pressure_comp_mbar", t.uint16_t),
        0x0003: ("altitude_comp_m", t.uint16_t),
        0x0004: ("force_recalibration_ppm", t.uint16_t),
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

Define these `0xFC00` attributes as ordinary typed attributes in the quirk. Marking them as manufacturer-specific can prevent reads from resolving correctly in ZHA.

If you intentionally enable the maintenance-only firmware controls, extend the quirk with:
- `0x0005: ("restart_measurement", t.Bool)`
- `0x0006: ("debug_command", t.uint8_t)`

### 3. Optional: use ZHA Toolkit for inspection and troubleshooting

ZHA Toolkit is useful for reading custom attributes, testing writes, and checking cluster behavior while building dashboards and scripts.

Suggested workflow:
- Install ZHA Toolkit in Home Assistant
- Restart Home Assistant
- Use it to inspect endpoint `12`, cluster `0xFC00`, and attributes `0x0000` through `0x0004`
- Use `zha_toolkit.attr_write` for writes to the custom control cluster

For this device, `zha_toolkit.attr_write` has proven more reliable than the built-in `zha.set_zigbee_cluster_attribute` action for custom `0xFC00` attributes. In Home Assistant, switch the action editor to YAML mode and use the device IEEE address directly.

Useful ZCL attribute types for this cluster:
- `0x0000 auto_calibrate` -> `attr_type: 16` (`0x10`, bool)
- `0x0001 temp_offset_x100` -> `attr_type: 41` (`0x29`, int16)
- `0x0002 pressure_comp_mbar` -> `attr_type: 33` (`0x21`, uint16)
- `0x0003 altitude_comp_m` -> `attr_type: 33` (`0x21`, uint16)
- `0x0004 force_recalibration_ppm` -> `attr_type: 33` (`0x21`, uint16)

### 4. Create helper entities

If you prefer dashboards based on helpers instead of writing raw cluster values directly, use separate helpers for the desired auto-calibration setting and the last confirmed hardware state.

```yaml
# configuration.yaml or helpers package
input_boolean:
  scd30_auto_calibration:
    name: SCD30 Desired Auto Calibration
    icon: mdi:auto-fix
  co2_auto_calibration:
    name: CO2 Hardware Auto Calibration
    icon: mdi:toggle-switch

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

Do not force `input_boolean.co2_auto_calibration` to an `initial:` value. Let Home Assistant restore the helper and then refresh it from the device after startup.

### 5. Write scripts that push helper values to the device and read them back

Replace `<YOUR_DEVICE_IEEE>` with the sensor's IEEE address from ZHA.

```yaml
# scripts.yaml
scd30_enable_auto_calibration:
  alias: "SCD30 - Enable Auto Calibration"
  sequence:
    - action: zha_toolkit.attr_write
      data:
        ieee: "<YOUR_DEVICE_IEEE>"
        endpoint: 12
        cluster: 0xFC00
        attribute: 0
        attr_type: 16
        attr_val: 1
        read_before_write: false
        read_after_write: false
        tries: 3
        fail_exception: true
    - action: input_boolean.turn_on
      target:
        entity_id: input_boolean.scd30_auto_calibration
    - delay:
        seconds: 2
    - action: script.scd30_refresh_auto_calibration_actual

scd30_disable_auto_calibration:
  alias: "SCD30 - Disable Auto Calibration"
  sequence:
    - action: zha_toolkit.attr_write
      data:
        ieee: "<YOUR_DEVICE_IEEE>"
        endpoint: 12
        cluster: 0xFC00
        attribute: 0
        attr_type: 16
        attr_val: 0
        read_before_write: false
        read_after_write: false
        tries: 3
        fail_exception: true
    - action: input_boolean.turn_off
      target:
        entity_id: input_boolean.scd30_auto_calibration
    - delay:
        seconds: 2
    - action: script.scd30_refresh_auto_calibration_actual

scd30_read_auto_calibrate:
  alias: "SCD30 - Read Auto Calibration Status"
  sequence:
    - action: script.scd30_refresh_auto_calibration_actual

scd30_sync_auto_calibration_after_startup:
  alias: "SCD30 - Sync Auto Calibration After Startup"
  mode: restart
  sequence:
    - wait_template: >-
        {{ states('sensor.espressif_esp32c6_carbon_dioxide') not in ['unknown', 'unavailable', 'none', ''] }}
      timeout: "00:01:30"
      continue_on_timeout: true
    - delay: "00:00:15"
    - action: script.scd30_refresh_auto_calibration_actual
      continue_on_error: true

scd30_refresh_auto_calibration_actual:
  alias: "SCD30 - Refresh Auto-Calibration Value"
  mode: restart
  sequence:
    - variables:
        auto_cal_actual_response: {}
    - repeat:
        count: 3
        sequence:
          - action: zha_toolkit.attr_read
            continue_on_error: true
            data:
              ieee: "<YOUR_DEVICE_IEEE>"
              endpoint: 12
              cluster: 0xFC00
              attribute: 0
              use_cache: false
              tries: 3
            response_variable: auto_cal_actual_response
          - choose:
              - conditions: >
                  {% set result_read = auto_cal_actual_response.result_read | default([], true) %}
                  {% set values = result_read[0] if result_read | count > 0 else {} %}
                  {% set errors = result_read[1] if result_read | count > 1 else {} %}
                  {{ auto_cal_actual_response.success | default(false) and errors | count == 0 and values | count > 0 }}
                sequence:
                  - variables:
                      auto_cal_value: >-
                        {% set values = auto_cal_actual_response.result_read[0] | default({}, true) %}
                        {{ values.get(0, values.get('0')) | string | trim | lower }}
                  - action: input_boolean.turn_{{ 'on' if auto_cal_value in ['true', '1', 'on', 'bool.true'] else 'off' }}
                    target:
                      entity_id: input_boolean.co2_auto_calibration
                  - stop: "SCD30 auto-calibration state refreshed"
          - delay: "00:00:08"
    - action: persistent_notification.create
      data:
        title: "SCD30 Auto-Calibration Refresh Failed"
        message: >-
          Unable to read the current hardware auto-calibration state from the SCD30 sensor.
          {{ auto_cal_actual_response.errors | default([], true) | join('; ') }}
          {{ auto_cal_actual_response.warnings | default([], true) | join('; ') }}

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

scd30_apply_force_recalibration:
  alias: "SCD30 - Apply forced recalibration"
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
        attr_val: 400
        read_before_write: false
        read_after_write: false
        tries: 3
```

Recommended usage:
- Treat `input_boolean.scd30_auto_calibration` as the desired setting.
- Treat `input_boolean.co2_auto_calibration` as the last confirmed hardware readback.
- Normalize readbacks such as `1`, `true`, and `Bool.true` to `on` in the refresh script.
- Use either `pressure_comp_mbar` or `altitude_comp_m`
- Keep `force_recalibration_ppm` for deliberate maintenance actions, not routine automations
- After writing a new temperature offset or compensation value, wait for the next stabilization window before judging the reading

### 6. Keep the hardware state synced after Home Assistant restarts

The live setup uses a delayed startup refresh instead of a single immediate read on boot. This avoids false `off` states when ZHA or the sleepy end device is not ready yet.

```yaml
# automations.yaml
- alias: Poll CO2 Auto Calibration Status
  trigger:
    - platform: time_pattern
      minutes: /5
      id: periodic
    - platform: homeassistant
      event: start
      id: startup
  action:
    - choose:
        - conditions: "{{ trigger.id == 'startup' }}"
          sequence:
            - service: script.scd30_sync_auto_calibration_after_startup
      default:
        - service: script.scd30_read_auto_calibrate
```

### 7. Optional Lovelace dashboard

This example keeps the live measurement entities, separates desired and actual auto-calibration state, and exposes refresh/apply scripts explicitly.

```yaml
views:
  - type: sections
    max_columns: 4
    title: Zigbee controls
    path: zigbee-controls
    sections:
      - type: grid
        cards:
          - type: vertical-stack
            cards:
              - type: markdown
                content: |
                  ## SCD30 CO2 Sensor Control
                  Sensor management and calibration
              - type: entities
                title: Current Readings
                icon: mdi:chart-line
                entities:
                  - entity: sensor.espressif_esp32c6_carbon_dioxide
                    name: CO2 Concentration
                    icon: mdi:molecule-co2
                  - entity: sensor.espressif_esp32c6_temperature
                    name: Temperature
                    icon: mdi:thermometer
                  - entity: sensor.espressif_esp32c6_humidity
                    name: Humidity
                    icon: mdi:water-percent
                  - entity: input_boolean.scd30_auto_calibration
                    name: Desired Auto Calibration
                    icon: mdi:auto-fix
                  - entity: input_boolean.co2_auto_calibration
                    name: Hardware Auto-Calibration Status
                    icon: mdi:toggle-switch
              - type: entities
                title: Quick Actions
                icon: mdi:lightning-bolt
                entities:
                  - entity: script.scd30_refresh_auto_calibration_actual
                    name: Refresh Hardware Auto-Calibration Status
                    icon: mdi:refresh
                    action_name: REFRESH
                  - entity: script.scd30_calibrate_outdoor
                    name: Calibrate to Outdoor Air (400 ppm)
                    icon: mdi:air-filter
                    action_name: CALIBRATE
                  - entity: script.scd30_enable_auto_calibration
                    name: Enable Auto Calibration
                    icon: mdi:auto-fix
                    action_name: ENABLE
                  - entity: script.scd30_disable_auto_calibration
                    name: Disable Auto Calibration
                    icon: mdi:auto-fix-off
                    action_name: DISABLE
              - type: entities
                title: Environmental Compensation
                icon: mdi:tune
                entities:
                  - entity: input_number.scd30_temperature_offset
                    name: Temperature Offset
                    icon: mdi:thermometer-lines
                  - entity: script.scd30_apply_temperature_offset
                    name: Apply Temperature Offset
                    icon: mdi:content-save-outline
                    action_name: APPLY
                  - entity: input_number.scd30_pressure_compensation
                    name: Pressure Compensation
                    icon: mdi:gauge
                  - entity: script.scd30_apply_pressure_compensation
                    name: Apply Pressure Compensation
                    icon: mdi:content-save-outline
                    action_name: APPLY
                  - entity: input_number.scd30_altitude_compensation
                    name: Altitude Compensation
                    icon: mdi:mountain
                  - entity: script.scd30_apply_altitude_compensation
                    name: Apply Altitude Compensation
                    icon: mdi:content-save-outline
                    action_name: APPLY
              - type: entities
                title: Advanced Controls
                icon: mdi:cog
                entities:
                  - entity: input_number.scd30_force_recalibration
                    name: Force Recalibration (ppm)
                    icon: mdi:bullseye-arrow
                  - entity: script.scd30_apply_force_recalibration
                    name: Apply Forced Recalibration
                    icon: mdi:content-save-outline
                    action_name: APPLY
```

Suggested cleanup from older dashboards:
- Remove `button.scd30_restart_measurements`
- Remove `select.scd30_debug_commands`
- Replace location-specific presets such as `script.scd30_set_oslo_environment` with generic pressure or altitude scripts
- Prefer helper entities plus explicit write/refresh scripts over direct writes from dashboard entity rows

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
> - **Fix:** Perform manual calibration under known CO₂, or use the 12-second outdoor recalibration shortcut after at least 2 minutes in clean outdoor/reference air; allow up to 14 days for auto-calibration to stabilize.

> **Unexpected Resets / Brown‑outs**  
> - **Symptom:** Device reboots intermittently.  
> - **Fix:** Verify a stable 3.3 V supply with sufficient current and solid wiring; add a small decoupling capacitor near VIN/GND if needed.


## Contributing

Please submit issues and pull requests with any improvements or bug fixes.

## License
This project is licensed under the MIT License. See [LICENSE](LICENSE).
