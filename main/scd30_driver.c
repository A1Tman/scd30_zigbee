/**
 * @file scd30_driver.c
 * @brief Implementation of SCD30 sensor driver
 */

#include "scd30_driver.h"
#include "i2c_handler.h"
#include "zigbee_handler.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <driver/gpio.h>
#include "math.h"

//#define ENABLE_RAW_DEBUG

static const char *TAG = "SCD30_DRIVER";

/* Static variables */
static TaskHandle_t scd30_task_handle = NULL;
static bool task_running = false;

/* CRC calculation for SCD30 communication */
static uint8_t scd30_crc8(const uint8_t *data, size_t len)
{
    // This CRC-8 calculation is based on the SCD30 datasheet and Adafruit's implementation.
    const uint8_t POLYNOMIAL = 0x31;
    uint8_t crc = 0xFF;

    for (size_t j = 0; j < len; ++j) {
        crc ^= data[j];

        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}


/* Send command to SCD30 */
static esp_err_t scd30_send_command(uint16_t command, const uint16_t *data, size_t words)
{
    uint8_t buf[20];  // Maximum command length (command + data + CRC)
    size_t idx = 0;

    // Add command bytes
    buf[idx++] = command >> 8;
    buf[idx++] = command & 0xFF;

    // Add data bytes if present
    if (words > 0) {
        for (size_t i = 0; i < words; i++) {
            buf[idx++] = data[i] >> 8;  // High byte of data
            buf[idx++] = data[i] & 0xFF;  // Low byte of data
            // Append CRC for each word
            uint8_t crc = scd30_crc8(&buf[idx - 2], 2);
            buf[idx++] = crc;
        }
    }

    // Add delay before sending command
    vTaskDelay(pdMS_TO_TICKS(10));
    
    esp_err_t ret = i2c_handler_write(buf, idx);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send command 0x%04x: %s", command, esp_err_to_name(ret));
    }
    
    // Add delay after command
    vTaskDelay(pdMS_TO_TICKS(20));
    
    return ret;
}

/* Read data from SCD30 - Ready status*/
static esp_err_t scd30_check_communication(void)
{
    bool data_ready;
    esp_err_t ret = scd30_get_data_ready_status(&data_ready);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with SCD30: %s", esp_err_to_name(ret));
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    return ESP_OK;
}

/* Read data from SCD30 */
static esp_err_t scd30_read_data(uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_handler_read(data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from sensor: %s", esp_err_to_name(ret));
        return ret;
    }

    // Log the raw data read from the sensor if enabled
    ESP_LOGI(TAG, "Raw data read from sensor: ");
        for (size_t i = 0; i < len; i++) {
        ESP_LOGI(TAG, "0x%02x ", data[i]);
        }

    // Verify CRC for each word (each word is 2 data bytes followed by 1 CRC byte)
    for (size_t i = 0; i < len; i += 3) {
        uint8_t expected_crc = scd30_crc8(&data[i], 2);
        if (expected_crc != data[i + 2]) {
            ESP_LOGE(TAG, "CRC check failed at position %d. Expected 0x%02x, got 0x%02x", i, expected_crc, data[i + 2]);
            return ESP_ERR_INVALID_CRC;
        }
    }

    return ret;
}

esp_err_t scd30_init(void)
{
    ESP_LOGI(TAG, "Initializing SCD30 sensor");
    esp_err_t ret;
    int retry_count = 0;
    i2c_master_dev_handle_t dev_handle = NULL;

    // First, add the SCD30 device to the I2C bus
    ret = i2c_handler_add_device(SCD30_SENSOR_ADDR, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SCD30 to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    while (retry_count < SCD30_INIT_RETRY_COUNT) {
        if (retry_count > 0) {
            ESP_LOGI(TAG, "Retry attempt %d of %d", retry_count + 1, SCD30_INIT_RETRY_COUNT);
            vTaskDelay(pdMS_TO_TICKS(SCD30_INIT_RETRY_DELAY_MS));
        }

        // Add initial delay after power-up
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Check if we can communicate with the sensor
        ret = scd30_check_communication();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Cannot communicate with SCD30 (attempt %d): %s", 
                     retry_count + 1, esp_err_to_name(ret));
            retry_count++;
            continue;
        }
        
        // Perform soft reset first
        ret = scd30_reset();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to reset sensor: %s", esp_err_to_name(ret));
            // Continue anyway as this might fail on first boot
        }
        
        // Wait for sensor to initialize after reset
        vTaskDelay(pdMS_TO_TICKS(SCD30_INIT_RETRY_DELAY_MS));
        
        // Stop any ongoing measurement
        ret = scd30_stop_measurement();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to stop measurements: %s", esp_err_to_name(ret));
            // Continue as this might fail if no measurements are running
        }
        vTaskDelay(pdMS_TO_TICKS(500));
                
        // Set temperature offset
        ESP_LOGI(TAG, "Attempting to set temperature offset to %.2f°C", SCD30_SW_TEMP_OFFSET);
        ret = scd30_set_temperature_offset(SCD30_SW_TEMP_OFFSET);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set temperature offset (attempt %d): %s", 
                     retry_count + 1, esp_err_to_name(ret));
            retry_count++;
            continue;
        }

        // Verify the offset
        float current_offset;
        ret = scd30_get_temperature_offset(&current_offset);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Verified temperature offset is set to %.2f°C", current_offset);
        } else {
            ESP_LOGW(TAG, "Failed to verify temperature offset");
            retry_count++;
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Apply compensation based on configuration
        #if SCD30_USE_PRESSURE_COMP
        ret = scd30_set_pressure_compensation(DEFAULT_PRESSURE_MBAR);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set pressure compensation: %s", esp_err_to_name(ret));
            // Continue anyway as this is not critical
        } else {
            ESP_LOGI(TAG, "Pressure compensation enabled at %u mbar", DEFAULT_PRESSURE_MBAR);
        }
        #else
        ret = scd30_set_altitude_compensation(SCD30_DEFAULT_ALTITUDE);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set altitude compensation: %s", esp_err_to_name(ret));
            // Continue anyway as this is not critical
        } else {
            ESP_LOGI(TAG, "Altitude compensation enabled at %u meters", SCD30_DEFAULT_ALTITUDE);
        }
        #endif

        // Set measurement interval
        ret = scd30_set_measurement_interval(SCD30_MEASUREMENT_INTERVAL);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set measurement interval (attempt %d): %s", 
                     retry_count + 1, esp_err_to_name(ret));
            retry_count++;
            continue;
        }

        // If we got here, initialization was successful
        ESP_LOGI(TAG, "SCD30 initialized successfully after %d attempt(s)", retry_count + 1);
        
        // Adding delay to allow sensor stabilization
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2 seconds delay to stabilize the sensor before probing

        return ESP_OK;
    }

    // If we got here, all retry attempts failed
    ESP_LOGE(TAG, "Failed to initialize SCD30 after %d attempts", SCD30_INIT_RETRY_COUNT);
    i2c_master_bus_rm_device(dev_handle);  // Clean up if initialization failed
    return ESP_ERR_INVALID_RESPONSE;
}


esp_err_t scd30_start_continuous_measurement(uint16_t ambient_pressure_mbar)
{
    uint16_t pressure_data = ambient_pressure_mbar;
    return scd30_send_command(SCD30_CMD_START_MEASURE, &pressure_data, 1);
}

esp_err_t scd30_stop_measurement(void)
{
    return scd30_send_command(SCD30_CMD_STOP_MEASURE, NULL, 0);
}

esp_err_t scd30_set_measurement_interval(uint16_t interval_sec)
{
    if (interval_sec < 2 || interval_sec > 1800) {
        ESP_LOGE(TAG, "Invalid measurement interval");
        return ESP_ERR_INVALID_ARG;
    }
    
    return scd30_send_command(SCD30_CMD_SET_INTERVAL, &interval_sec, 1);
}

esp_err_t scd30_get_data_ready_status(bool *data_ready)
{
    uint8_t ready_buf[3];
    esp_err_t ret;

    ret = scd30_send_command(SCD30_CMD_GET_DATA_READY, NULL, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(3));  // Wait for processing

    ret = scd30_read_data(ready_buf, sizeof(ready_buf));
    if (ret == ESP_OK) {
        *data_ready = (ready_buf[0] << 8 | ready_buf[1]) == 1;
    }

    return ret;
}

esp_err_t scd30_read_measurement(scd30_measurement_t *measurement)
{
    if (measurement == NULL) {
        return ESP_ERR_INVALID_ARG; 
    }

    esp_err_t ret;
    bool data_ready = false;

    // Check if data is ready
    ret = scd30_get_data_ready_status(&data_ready);
    if (ret != ESP_OK) {
        return ret;
    }

    if (!data_ready) {
        ESP_LOGW(TAG, "Measurement data not ready yet.");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[18];  // 6 bytes per value (including CRC)

    // Send read measurement command
    ret = scd30_send_command(SCD30_CMD_READ_MEASUREMENT, NULL, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(5));  // Increased from 3ms to 5ms for more reliable operation

    // Read measurement data
    ret = scd30_read_data(data, sizeof(data));
    if (ret != ESP_OK) {
        return ret;
    }

    // Convert data to float values
    uint32_t co2_raw = ((uint32_t)data[0] << 24) |
                       ((uint32_t)data[1] << 16) |
                       ((uint32_t)data[3] << 8) |
                       data[4];
    measurement->co2_ppm = *(float*)&co2_raw;

    uint32_t temp_raw = ((uint32_t)data[6] << 24) |
                        ((uint32_t)data[7] << 16) |
                        ((uint32_t)data[9] << 8) |
                        data[10];
    float raw_temp = *(float*)&temp_raw;
    
    // Software temperature compensation
    // Note: Using software compensation instead of SCD30's built-in offset
    // due to I2C transaction queue limitations in the current setup
    measurement->temperature = raw_temp - SCD30_SW_TEMP_OFFSET;  // Subtract 2.5°C to match reference

    uint32_t hum_raw = ((uint32_t)data[12] << 24) |
                       ((uint32_t)data[13] << 16) |
                       ((uint32_t)data[15] << 8) |
                       data[16];
    measurement->humidity = *(float*)&hum_raw;
   
    vTaskDelay(pdMS_TO_TICKS(20));  // Give sensor time to process

    return ESP_OK;
}

esp_err_t scd30_reset(void)
{
    return scd30_send_command(SCD30_CMD_RESET, NULL, 0);
}

/* SCD30 measurement task */
static void scd30_measurement_task(void *pvParameters)
{
    scd30_measurement_t measurement;
    bool data_ready;
    esp_err_t ret;
    uint8_t consecutive_errors = 0;

    // Start continuous measurement with ambient pressure compensation
    ret = scd30_start_continuous_measurement(SCD30_AMBIENT_PRESSURE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start continuous measurement");
        vTaskDelete(NULL);
        return;
    }

    while (task_running) {
        // Check if new data is available
        ret = scd30_get_data_ready_status(&data_ready);
        if (ret == ESP_OK && data_ready) {
            // Read measurement
            ret = scd30_read_measurement(&measurement);
            if (ret == ESP_OK) {
                // Validate measurements are within reasonable ranges
                if (measurement.co2_ppm >= SCD30_CO2_MIN && measurement.co2_ppm <= SCD30_CO2_MAX &&
                    measurement.temperature >= SCD30_TEMP_MIN && measurement.temperature <= SCD30_TEMP_MAX &&
                    measurement.humidity >= SCD30_HUM_MIN && measurement.humidity <= SCD30_HUM_MAX) {
                    
                    ESP_LOGI(TAG, "CO2: %.1f ppm, Temperature: %.2f°C, Humidity: %.1f%%",
                            measurement.co2_ppm, measurement.temperature, measurement.humidity);

                    // Update Zigbee with raw values
                    zigbee_handler_update_measurements(measurement.co2_ppm, 
                                                     measurement.temperature, 
                                                     measurement.humidity);
                    
                    consecutive_errors = 0;
                } else {
                    ESP_LOGW(TAG, "Measurements out of valid range");
                    consecutive_errors++;
                }
            } else {
                ESP_LOGW(TAG, "Failed to read measurements: %s", esp_err_to_name(ret));
                consecutive_errors++;
            }

            // Check if we need to reset the sensor
            if (consecutive_errors >= SCD30_MAX_CONSECUTIVE_ERRORS) {
                ESP_LOGE(TAG, "Too many consecutive errors, attempting sensor reset");
                scd30_reset();
                vTaskDelay(pdMS_TO_TICKS(SCD30_RECOVERY_DELAY_MS));
                scd30_start_continuous_measurement(SCD30_AMBIENT_PRESSURE);
                consecutive_errors = 0; // Reset error counter after a successful read
            }

            vTaskDelay(pdMS_TO_TICKS(5000));
        } else if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to check data ready status: %s", esp_err_to_name(ret));
            consecutive_errors++;
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    scd30_stop_measurement();
    vTaskDelete(NULL);
}

esp_err_t scd30_start_task(uint8_t task_priority)
{
    if (scd30_task_handle != NULL) {
        ESP_LOGW(TAG, "Task already running");
        return ESP_ERR_INVALID_STATE;
    }

    task_running = true;
    BaseType_t result = xTaskCreate(
        scd30_measurement_task,
        "SCD30_task",
        8192,
        NULL,
        task_priority,
        &scd30_task_handle
    );

    if (result != pdPASS) {
        task_running = false;
        ESP_LOGE(TAG, "Failed to create SCD30 task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t scd30_stop_task(void)
{
    if (scd30_task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    task_running = false;
    // Task will clean up and delete itself
    return ESP_OK;
}

esp_err_t validate_scd30_readings(float co2_ppm, float temperature, float humidity) {
    if (co2_ppm < SCD30_CO2_MIN || co2_ppm > SCD30_CO2_MAX) {
        ESP_LOGW(TAG, "Invalid CO2 reading: %f ppm", co2_ppm);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (temperature < SCD30_TEMP_MIN || temperature > SCD30_TEMP_MAX) {
        ESP_LOGW(TAG, "Invalid temperature reading: %f°C", temperature);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (humidity < SCD30_HUM_MIN || humidity > SCD30_HUM_MAX) {
        ESP_LOGW(TAG, "Invalid humidity reading: %f%%", humidity);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t scd30_get_temperature_offset(float *offset_celsius)
{
    if (offset_celsius == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[3];
    esp_err_t ret;

    ret = scd30_send_command(SCD30_CMD_TEMP_OFFSET, NULL, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(3));

    ret = scd30_read_data(data, sizeof(data));
    if (ret != ESP_OK) {
        return ret;
    }

    uint16_t offset_word = (data[0] << 8) | data[1];
    int16_t offset_ticks;
    
    // Handle two's complement for negative values
    if (offset_word & 0x8000) {
        // Value is negative
        offset_ticks = (int16_t)(offset_word - 65536);
    } else {
        offset_ticks = (int16_t)offset_word;
    }
    
    *offset_celsius = offset_ticks / 100.0f;

    ESP_LOGI(TAG, "Current temperature offset: %.2f°C (word: 0x%04x, ticks: %d)", 
             *offset_celsius, offset_word, offset_ticks);
    return ESP_OK;
}

esp_err_t scd30_set_temperature_offset(float offset_celsius)
{
    // Convert to ticks (1 tick = 0.01°C)
    uint16_t offset_ticks = (uint16_t)(offset_celsius * 100.0f);
    
    ESP_LOGI(TAG, "Setting temperature offset to %.2f°C (%u ticks, 0x%04X)", 
             offset_celsius, offset_ticks, offset_ticks);
             
    return scd30_send_command(SCD30_CMD_TEMP_OFFSET, &offset_ticks, 1);
}

esp_err_t scd30_set_altitude_compensation(uint16_t altitude_meters)
{
    // Static variable to track the last set altitude value
    static uint16_t prev_altitude = 0xFFFF; // Initialized to an invalid value to ensure it's logged the first time

    // Only log if the altitude value changes
    if (altitude_meters != prev_altitude) {
        ESP_LOGI(TAG, "Setting altitude compensation to %u meters", altitude_meters);
        prev_altitude = altitude_meters;
    }

    // Send command to set altitude compensation
    return scd30_send_command(SCD30_CMD_ALTI_COMP, &altitude_meters, 1);
}

esp_err_t scd30_set_pressure_compensation(uint16_t pressure_mbar)
{
    // Static variable to track the last set pressure value
    static uint16_t prev_pressure = 0xFFFF; // Initialized to an invalid value to ensure it's logged the first time

    // Only log if the pressure value changes
    if (pressure_mbar != prev_pressure) {
        ESP_LOGI(TAG, "Setting pressure compensation to %u mbar", pressure_mbar);
        prev_pressure = pressure_mbar;
    }

    // Send command to set pressure compensation
    return scd30_send_command(SCD30_CMD_SET_PRESSURE, &pressure_mbar, 1);
}
