/**
 * @file scd30_driver.c
 * @brief Implementation of SCD30 sensor driver
 */

 #include "scd30_driver.h"
 #include "i2c_handler.h"
 #include "zigbee_handler.h"
 #include "app_defs.h" 
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include <string.h>
 #include <driver/gpio.h>
 #include "math.h"
 
 //#define ENABLE_RAW_DEBUG
 
 static const char *TAG = "SCD30_DRIVER";
 
 /* Constants */
 #define SCD30_MAX_COMMAND_SIZE      20
 #define SCD30_READ_DELAY_MS         4
 #define SCD30_DATA_READY_DELAY_MS   3
 #define SCD30_BOOT_TIME_MS          2000
 #define SCD30_RESET_DELAY_MS        30
 
 /* Static variables */
 static TaskHandle_t scd30_task_handle = NULL;
 static bool task_running = false;
 
 /* CRC calculation for SCD30 communication */
static uint8_t scd30_crc8(const uint8_t *data, size_t len)
{
    /* CRC-8 formula from the SCD30 datasheet.  Implementation mirrors the
     * Adafruit library to ensure interoperability.  Test data 0xBE 0xEF should
     * yield 0x92.
     */
    const uint8_t POLYNOMIAL = 0x31;
    uint8_t crc = 0xFF;

    for (size_t j = 0; j < len; j++) {
        crc ^= data[j];
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }

    return crc;
}
 
 /* Send command to SCD30 */
 static esp_err_t scd30_send_command(uint16_t command, const uint16_t *data, size_t words)
 {
     uint8_t buf[SCD30_MAX_COMMAND_SIZE];
     size_t idx = 0;
 
     // Add command bytes
     buf[idx++] = command >> 8;
     buf[idx++] = command & 0xFF;
 
     // Add data bytes if present
     if (words > 0 && data != NULL) {
         for (size_t i = 0; i < words; i++) {
             buf[idx++] = data[i] >> 8;  // High byte of data
             buf[idx++] = data[i] & 0xFF;  // Low byte of data
             // Append CRC for each word
             uint8_t crc = scd30_crc8(&buf[idx - 2], 2);
             buf[idx++] = crc;
         }
     }
    
     esp_err_t ret = i2c_handler_write(buf, idx);
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to send command 0x%04x: %s", command, esp_err_to_name(ret));
     }   
     
     return ret;
 }
 
 /* Write a command and then read data from the sensor */
static esp_err_t scd30_write_then_read(uint16_t command, uint8_t *data, size_t len)
{
    uint8_t cmd[2];
    cmd[0] = command >> 8;
    cmd[1] = command & 0xFF;

    esp_err_t ret = i2c_handler_write(cmd, sizeof(cmd));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send command 0x%04x: %s", command, esp_err_to_name(ret));
        i2c_handler_recover();
        return ret;
    }

    // Datasheet specifies ~4ms delay between write and read
    if (command == SCD30_CMD_GET_DATA_READY) {
        vTaskDelay(pdMS_TO_TICKS(SCD30_DATA_READY_DELAY_MS + 2));
    } else {
        vTaskDelay(pdMS_TO_TICKS(SCD30_READ_DELAY_MS + 1));
    }

    for (int attempt = 0; attempt < 2; attempt++) {
        ret = i2c_handler_read(data, len);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read response for 0x%04x: %s", command, esp_err_to_name(ret));
            i2c_handler_recover();
            if (attempt == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            return ret;
        }

        bool crc_ok = true;
        for (size_t i = 0; i < len; i += 3) {
            uint8_t expected_crc = scd30_crc8(&data[i], 2);
            if (expected_crc != data[i + 2]) {
                crc_ok = false;
                ESP_LOGE(TAG, "CRC check failed at position %d. Expected 0x%02x, got 0x%02x",
                         i, expected_crc, data[i + 2]);
                ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, len, ESP_LOG_DEBUG);
                i2c_handler_recover();
                break;
            }
        }

        if (crc_ok) {
            return ESP_OK;
        }

        if (attempt == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    return ESP_ERR_INVALID_CRC;
}
 
 /* Check communication with SCD30 */
 static esp_err_t scd30_check_communication(void)
 {
     bool data_ready;
     esp_err_t ret = scd30_get_data_ready_status(&data_ready);
     
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to communicate with SCD30: %s", esp_err_to_name(ret));
         return ESP_ERR_INVALID_RESPONSE;
     }
     ESP_LOGI(TAG, "SCD30 communication check successful.");
     return ESP_OK;
 }
 
 esp_err_t scd30_init(void)
 {
     ESP_LOGI(TAG, "Initializing SCD30 sensor");
     esp_err_t ret;
 
     /* The SCD30 device is already added to the bus in i2c_handler_init(). */
     i2c_master_dev_handle_t dev_handle = i2c_handler_get_device();
     if (dev_handle == NULL) {
         ESP_LOGE(TAG, "I2C device handle is not available");
         return ESP_ERR_INVALID_STATE;
     }
 
     /* Soft reset the sensor */
     ret = scd30_reset();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to reset sensor: %s", esp_err_to_name(ret));
         return ret;
     }
     
     // Wait for sensor boot-up (datasheet specifies < 2s)
     ESP_LOGI(TAG, "Waiting for sensor boot-up...");
     vTaskDelay(pdMS_TO_TICKS(SCD30_BOOT_TIME_MS));
 
     /* Start continuous measurement. First I2C transaction after reset may fail. */
     ret = scd30_start_continuous_measurement(SCD30_AMBIENT_PRESSURE);
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "First measurement start failed (expected), retrying...");
         vTaskDelay(pdMS_TO_TICKS(100));
         ret = scd30_start_continuous_measurement(SCD30_AMBIENT_PRESSURE);
         if (ret != ESP_OK) {
             ESP_LOGE(TAG, "Unable to start measurements: %s", esp_err_to_name(ret));
             return ret;
         }
     }
 
     /* Set measurement interval to 2 seconds */
     ret = scd30_set_measurement_interval(SCD30_MEASUREMENT_INTERVAL);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to set measurement interval: %s", esp_err_to_name(ret));
         return ret;
     }
 
     // Check communication after initialization
     ret = scd30_check_communication();
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "SCD30 communication check failed after initialization: %s", 
                  esp_err_to_name(ret));
         // Not returning error here, as basic init might still be okay
     }
 
     ESP_LOGI(TAG, "SCD30 initialization complete");
     return ESP_OK;
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
         ESP_LOGE(TAG, "Invalid measurement interval: %d (must be 2-1800)", interval_sec);
         return ESP_ERR_INVALID_ARG;
     }
     
     return scd30_send_command(SCD30_CMD_SET_INTERVAL, &interval_sec, 1);
 }
 
 esp_err_t scd30_get_data_ready_status(bool *data_ready)
 {
     if (data_ready == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     uint8_t ready_buf[3];
     esp_err_t ret = scd30_write_then_read(SCD30_CMD_GET_DATA_READY,
                                           ready_buf, sizeof(ready_buf));
     if (ret == ESP_OK) {
         *data_ready = ((ready_buf[0] << 8) | ready_buf[1]) == 1;
     }
 
     return ret;
 }
 
 esp_err_t scd30_read_measurement(scd30_measurement_t *measurement,
                                  bool skip_ready_check)
 {
     if (measurement == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     esp_err_t ret;
 
     if (!skip_ready_check) {
         bool data_ready = false;
         ret = scd30_get_data_ready_status(&data_ready);
         if (ret != ESP_OK) {
             return ret;
         }
 
         if (!data_ready) {
             ESP_LOGD(TAG, "Measurement data not ready yet.");
             return ESP_ERR_INVALID_STATE;
         }
     }
 
     uint8_t data[18];  // 6 values × 3 bytes each (2 data + 1 CRC)
 
     // Send command and then read measurement data
     ret = scd30_write_then_read(SCD30_CMD_READ_MEASUREMENT, data, sizeof(data));
     if (ret != ESP_OK) {
         return ret;
     }
 
    // Convert data to IEEE-754 floats as described in the SCD30 datasheet.
    // The sensor transmits each 32‑bit value in big-endian order with a
    // CRC appended after every 16‑bit word: MSB, LSB, CRC, MSB, LSB, CRC.
     
     // CO2: bytes 0,1,3,4 (skip bytes 2,5 - CRC)
    uint32_t co2_raw = ((uint32_t)data[0] << 24) |
                       ((uint32_t)data[1] << 16) |
                       ((uint32_t)data[3] << 8) |
                       ((uint32_t)data[4]);
    memcpy(&measurement->co2_ppm, &co2_raw, sizeof(float));
 
     // Temperature: bytes 6,7,9,10 (skip bytes 8,11 - CRC)
    uint32_t temp_raw = ((uint32_t)data[6] << 24) |
                        ((uint32_t)data[7] << 16) |
                        ((uint32_t)data[9] << 8) |
                        ((uint32_t)data[10]);
    float temperature;
    memcpy(&temperature, &temp_raw, sizeof(float));
     
     // Apply software temperature compensation
     // Note: Using software compensation instead of SCD30's built-in offset
     // due to I2C transaction queue limitations in the current setup
     measurement->temperature = temperature - SCD30_SW_TEMP_OFFSET;
 
     // Humidity: bytes 12,13,15,16 (skip bytes 14,17 - CRC)
    uint32_t hum_raw = ((uint32_t)data[12] << 24) |
                       ((uint32_t)data[13] << 16) |
                       ((uint32_t)data[15] << 8) |
                       ((uint32_t)data[16]);
    memcpy(&measurement->humidity, &hum_raw, sizeof(float));
 
     return ESP_OK;
 }
 
 esp_err_t scd30_reset(void)
 {
     esp_err_t ret = scd30_send_command(SCD30_CMD_RESET, NULL, 0);
     if (ret == ESP_OK) {
         // Give sensor time to reset
         vTaskDelay(pdMS_TO_TICKS(SCD30_RESET_DELAY_MS));
     }
     return ret;
 }
 
 esp_err_t validate_scd30_readings(float co2_ppm, float temperature, float humidity) 
 {
     if (co2_ppm < SCD30_CO2_MIN || co2_ppm > SCD30_CO2_MAX) {
         ESP_LOGW(TAG, "Invalid CO2 reading: %.1f ppm", co2_ppm);
         return ESP_ERR_INVALID_RESPONSE;
     }
     if (temperature < SCD30_TEMP_MIN || temperature > SCD30_TEMP_MAX) {
         ESP_LOGW(TAG, "Invalid temperature reading: %.1f°C", temperature);
         return ESP_ERR_INVALID_RESPONSE;
     }
     if (humidity < SCD30_HUM_MIN || humidity > SCD30_HUM_MAX) {
         ESP_LOGW(TAG, "Invalid humidity reading: %.1f%%", humidity);
         return ESP_ERR_INVALID_RESPONSE;
     }
     return ESP_OK;
 }
 
 /* SCD30 measurement task */
 static void scd30_measurement_task(void *pvParameters)
 {
     scd30_measurement_t measurement;
     esp_err_t ret;
     uint8_t consecutive_errors = 0;
     const uint8_t MAX_RETRIES = 3;
 
     ESP_LOGI(TAG, "SCD30 measurement task started");
 
     // Start continuous measurement with ambient pressure compensation
     for (int i = 0; i < MAX_RETRIES; i++) {
         ret = scd30_start_continuous_measurement(SCD30_AMBIENT_PRESSURE);
         if (ret == ESP_OK) {
             break;
         }
         ESP_LOGW(TAG, "Failed to start continuous measurement (attempt %d/%d)", 
                  i + 1, MAX_RETRIES);
         vTaskDelay(pdMS_TO_TICKS(100));
     }
     
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to start continuous measurement after %d attempts", MAX_RETRIES);
         task_running = false;
         vTaskDelete(NULL);
         return;
     }
 
     // Give the sensor time to warm up before reading data
     ESP_LOGI(TAG, "Waiting for sensor warm-up...");
     vTaskDelay(pdMS_TO_TICKS(SCD30_WARMUP_TIME_MS));
 
     while (task_running) {
         ret = scd30_read_measurement(&measurement, false);
         
         if (ret == ESP_OK) {
             // Validate measurements
             ret = validate_scd30_readings(measurement.co2_ppm, 
                                         measurement.temperature, 
                                         measurement.humidity);
             if (ret == ESP_OK) {
                 ESP_LOGI(TAG, "CO2: %.1f ppm, Temperature: %.2f°C, Humidity: %.1f%%",
                         measurement.co2_ppm, measurement.temperature, measurement.humidity);
 
                 // Update Zigbee with validated values
                 zigbee_handler_update_measurements(measurement.co2_ppm, 
                                                  measurement.temperature, 
                                                  measurement.humidity);
                 
                 consecutive_errors = 0;
             } else {
                 ESP_LOGW(TAG, "Measurements out of valid range");
                 consecutive_errors++;
             }
         } else if (ret == ESP_ERR_INVALID_STATE) {
             /* Data not ready yet - this is normal */
             ESP_LOGD(TAG, "Data not ready, waiting...");
             vTaskDelay(pdMS_TO_TICKS(500));
             continue;
         } else {
             ESP_LOGW(TAG, "Failed to read measurements: %s", esp_err_to_name(ret));
             consecutive_errors++;
         }
 
         // Check if we need to recover from errors
         if (consecutive_errors >= SCD30_MAX_CONSECUTIVE_ERRORS) {
             ESP_LOGE(TAG, "Too many consecutive errors (%d), attempting sensor recovery", 
                      consecutive_errors);
             
             // Try to recover
             ret = scd30_reset();
             if (ret == ESP_OK) {
                 vTaskDelay(pdMS_TO_TICKS(SCD30_BOOT_TIME_MS));
                 
                 // Restart measurements
                 for (int i = 0; i < MAX_RETRIES; i++) {
                     ret = scd30_start_continuous_measurement(SCD30_AMBIENT_PRESSURE);
                     if (ret == ESP_OK) {
                         ESP_LOGI(TAG, "Sensor recovery successful");
                         consecutive_errors = 0;
                         vTaskDelay(pdMS_TO_TICKS(SCD30_WARMUP_TIME_MS));
                         break;
                     }
                     vTaskDelay(pdMS_TO_TICKS(100));
                 }
             }
             
             if (ret != ESP_OK) {
                 ESP_LOGE(TAG, "Sensor recovery failed, stopping task");
                 break;
             }
         }
 
         // Wait before next measurement (adjust based on measurement interval)
         vTaskDelay(pdMS_TO_TICKS(SCD30_MEASUREMENT_INTERVAL * 1000));
     }
 
     // Clean up
     scd30_stop_measurement();
     ESP_LOGI(TAG, "SCD30 measurement task stopped");
     task_running = false;
     scd30_task_handle = NULL;
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
         scd30_task_handle = NULL;
         ESP_LOGE(TAG, "Failed to create SCD30 task");
         return ESP_FAIL;
     }
 
     ESP_LOGI(TAG, "SCD30 task created successfully");
     return ESP_OK;
 }
 
 esp_err_t scd30_stop_task(void)
 {
     if (scd30_task_handle == NULL) {
         return ESP_ERR_INVALID_STATE;
     }
 
     ESP_LOGI(TAG, "Stopping SCD30 task...");
     task_running = false;
     
     // Wait a bit for task to finish cleanly (it will clean up and delete itself)
     vTaskDelay(pdMS_TO_TICKS(SCD30_MEASUREMENT_INTERVAL * 1000 + 500));
     
     // Ensure handle is cleared
     scd30_task_handle = NULL;
     
     return ESP_OK;
 }
 
 esp_err_t scd30_get_temperature_offset(float *offset_celsius)
 {
     if (offset_celsius == NULL) {
         return ESP_ERR_INVALID_ARG;
     }
 
     uint8_t data[3];
     esp_err_t ret = scd30_write_then_read(SCD30_CMD_TEMP_OFFSET, data, sizeof(data));
     if (ret != ESP_OK) {
         return ret;
     }
 
     uint16_t offset_word = (data[0] << 8) | data[1];
     
     // Convert from ticks to degrees (1 tick = 0.01°C)
     *offset_celsius = offset_word / 100.0f;
 
     ESP_LOGI(TAG, "Current temperature offset: %.2f°C (raw: 0x%04x)", 
              *offset_celsius, offset_word);
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
     static uint16_t prev_altitude = 0xFFFF;
 
     // Only log if the altitude value changes
     if (altitude_meters != prev_altitude) {
         ESP_LOGI(TAG, "Setting altitude compensation to %u meters", altitude_meters);
         prev_altitude = altitude_meters;
     }
 
     return scd30_send_command(SCD30_CMD_ALTI_COMP, &altitude_meters, 1);
 }
 
 esp_err_t scd30_set_pressure_compensation(uint16_t pressure_mbar)
 {
     // Static variable to track the last set pressure value
     static uint16_t prev_pressure = 0xFFFF;
 
     // Only log if the pressure value changes
     if (pressure_mbar != prev_pressure) {
         ESP_LOGI(TAG, "Setting pressure compensation to %u mbar", pressure_mbar);
         prev_pressure = pressure_mbar;
     }
 
     return scd30_send_command(SCD30_CMD_SET_PRESSURE, &pressure_mbar, 1);
 }