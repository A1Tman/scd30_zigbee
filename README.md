![Release](https://img.shields.io/github/v/release/A1Tman/scd30_zigbee)
![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![Hardware Tested](https://img.shields.io/badge/hardware-tested-brightgreen)
![ESP32](https://img.shields.io/badge/platform-ESP32-blue)

# ESP32 Zigbee CO2 Sensor

An ESP32-based CO2, temperature, and humidity sensor that integrates with Zigbee networks using the SCD30 sensor module.

## Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Features](#features)
- [Software Dependencies](#software-dependencies)
- [Building and Flashing](#building-and-flashing)
- [Zigbee Configuration](#zigbee-configuration)
- [Sensor Configuration](#sensor-configuration)
- [Project Structure](#project-structure)
- [Usage](#usage)
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
  - Uses Sensirion SCD30 sensor for CO2, temperature, and humidity measurements
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

- ESP-IDF (Espressif IoT Development Framework)
- FreeRTOS
- ESP Zigbee SDK
- Required ESP-IDF components:
  - `driver`
  - `esp_timer`
  - `nvs_flash`
  - `esp_pm`

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

Device specifications:
- Profile ID: Home Automation (0x0104)
- Device ID: Custom CO2 Sensor (`ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID`)
- Endpoint: 12 (`HA_CUSTOM_CO2_ENDPOINT`)
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
  - CO2: `SCD30_CO2_MIN` to `SCD30_CO2_MAX` ppm
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
└── Sensirion_CO2_Sensors_SCD30_Interface_Descr... # Sensor documentation
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

## Contributing

Please submit issues and pull requests with any improvements or bug fixes.

## License
This project is licensed under the MIT License. See [LICENSE](LICENSE).
