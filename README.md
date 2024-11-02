# ESP32 Zigbee CO2 Sensor

An ESP32-based CO2, temperature, and humidity sensor that integrates with Zigbee networks using the SCD30 sensor module.

## Hardware Requirements

- ESP32-C6 development board (with Zigbee support)
- Sensirion SCD30 CO2 sensor
- I2C connections:
  - SDA and SCL pins. In this case it's configured via an I2C handler to pins 21 and 22. See official documentation here https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/api-reference/peripherals/gpio.html for GPIO summary. 
  - 3.3V power supplied by the ESP32
  - GND 

## Features

- Measures:
  - CO2 concentration (400-10000 ppm)
  - Temperature (-40°C to 70°C)
  - Relative Humidity (0-100%)
- Zigbee 3.0 compatibility
- Automatic network joining
- Configurable measurement intervals
- Temperature offset compensation
- Altitude/pressure compensation
- Error recovery and device reset capabilities

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

3. Build the project:
   ```bash
   idf.py build
   ```

4. Flash to your ESP32-C6:
   ```bash
   idf.py -p PORT flash
   ```

## Zigbee Configuration

The device operates as a Zigbee End Device (ZED) with the following clusters:
- Basic Cluster
- Identify Cluster
- Carbon Dioxide Measurement Cluster
- Temperature Measurement Cluster
- Humidity Measurement Cluster

Device specifications:
- Profile ID: Home Automation (0x0104)
- Device ID: Temperature Sensor (0x0302)
- Endpoint: Configurable (default in app_defs.h)

## Sensor Configuration

The SCD30 sensor is configured with:
- Default measurement interval: Configurable (typical 5 seconds)
- Temperature offset compensation
- Optional altitude/pressure compensation
- Automatic error recovery and sensor reset capabilities

## Project Structure

```
├── main/
│   ├── app_defs.h           # Application definitions and constants
│   ├── i2c_handler.c/h      # I2C communication handling
│   ├── main.c              # Main application entry point
│   ├── scd30_driver.c/h    # SCD30 sensor driver
│   └── zigbee_handler.c/h  # Zigbee communication handling
```

## Usage

1. Power up the device
2. The device will automatically attempt to join available Zigbee networks
3. Once connected, the device will:
   - Begin taking measurements
   - Report values to the Zigbee coordinator
   - Handle any configuration commands from the network

## Error Handling

The device includes several error recovery mechanisms:
- I2C bus recovery
- Sensor initialization retries
- Measurement validation
- Automatic sensor reset on consecutive errors
- Network reconnection handling

## Troubleshooting

1. Device not joining network:
   - Verify Zigbee coordinator is operating
   - Check if network is permitting new devices
   - Monitor device logs for join attempts

2. Invalid sensor readings:
   - Check I2C connections
   - Verify sensor power supply
   - Monitor logs for sensor initialization errors

3. Communication issues:
   - Verify I2C pin configurations
   - Check for proper sensor addressing
   - Monitor for I2C bus errors in logs

## Contributing

Please submit issues and pull requests with any improvements or bug fixes.

## To-Do

1. More robust network handling
2. Power management
3. Add display

## License
TBD