/**
 * @file scd30_driver.c
 * @brief Implementation of SCD30 sensor driver
 */

 #include "scd30_driver.h"
 #include "i2c_handler.h"
 #include "zigbee_handler.h"
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/queue.h"
 #include "freertos/task.h"
 #include "nvs.h"
 #include <math.h>
 #include <stdio.h>
 #include <string.h>
 #include <driver/gpio.h>
 
   static const char *TAG = "SCD30_DRIVER";
 
 /* Static variables */
 static TaskHandle_t scd30_task_handle = NULL;
 static QueueHandle_t scd30_command_queue = NULL;
 static volatile bool task_running = false;
 static scd30_runtime_config_t scd30_config;
 static scd30_runtime_config_t scd30_last_zigbee_attr_snapshot;
 static uint16_t scd30_last_force_recalibration_attr = 0;
 static bool scd30_config_loaded = false;
 static bool scd30_last_zigbee_attr_snapshot_valid = false;

#define SCD30_DEBUG_MAX_LOG_BYTES 18
#define SCD30_CONFIG_NVS_NAMESPACE "scd30_cfg"
#define SCD30_CONFIG_NVS_KEY       "settings"
#define SCD30_COMMAND_QUEUE_LEN    4

typedef enum {
    SCD30_COMMAND_APPLY_CONFIG = 0,
    SCD30_COMMAND_FORCE_RECALIBRATION = 1,
} scd30_command_type_t;

typedef struct {
    scd30_command_type_t type;
    scd30_runtime_config_t config;
    uint16_t target_ppm;
} scd30_command_t;

static void scd30_set_default_config(scd30_runtime_config_t *config);
static esp_err_t scd30_load_cached_config(void);
static esp_err_t scd30_save_config_to_nvs(const scd30_runtime_config_t *config);
static esp_err_t scd30_ensure_runtime_support(void);
static const char *scd30_compensation_mode_to_string(uint8_t mode);
static uint16_t scd30_get_start_pressure_arg(const scd30_runtime_config_t *config);
static uint16_t scd30_get_effective_altitude(const scd30_runtime_config_t *config);
static esp_err_t scd30_apply_static_config(const scd30_runtime_config_t *config, bool verify_temp_offset);
static esp_err_t scd30_restart_measurement_with_config(const scd30_runtime_config_t *config);
static esp_err_t scd30_queue_command(const scd30_command_t *command);
static void scd30_process_pending_commands(TickType_t *measurement_start_tick,
                                           bool *stabilization_wait_logged,
                                           bool *stabilization_complete_logged,
                                           uint8_t *consecutive_errors);
static void scd30_sync_config_from_zigbee_attributes(TickType_t *measurement_start_tick,
                                                     bool *stabilization_wait_logged,
                                                     bool *stabilization_complete_logged,
                                                     uint8_t *consecutive_errors);
 
 /* CRC calculation for SCD30 communication */
 static uint8_t scd30_crc8(const uint8_t *data, size_t len)
 {
     // This CRC-8 calculation is based on the SCD30 datasheet
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

 static void scd30_log_raw_data(const char *context, const uint8_t *data, size_t len)
 {
     char hex_buf[(SCD30_DEBUG_MAX_LOG_BYTES * 3) + 1] = {0};
     size_t bytes_to_log = len;
     size_t offset = 0;

     if (bytes_to_log > SCD30_DEBUG_MAX_LOG_BYTES) {
         bytes_to_log = SCD30_DEBUG_MAX_LOG_BYTES;
     }

     for (size_t i = 0; i < bytes_to_log && offset < sizeof(hex_buf); ++i) {
         int written = snprintf(hex_buf + offset, sizeof(hex_buf) - offset,
                                i + 1 < bytes_to_log ? "%02x " : "%02x", data[i]);
         if (written < 0) {
             return;
         }
         offset += (size_t)written;
     }

     ESP_LOGD(TAG, "%s raw data (%u bytes)%s: %s",
              context,
              (unsigned int)len,
              len > bytes_to_log ? " [truncated]" : "",
              hex_buf);
 }

 static void scd30_set_default_config(scd30_runtime_config_t *config)
 {
     memset(config, 0, sizeof(*config));
     config->version = SCD30_CONFIG_VERSION;
     config->temp_offset_x100 = (int16_t)lroundf(SCD30_HW_TEMP_OFFSET * 100.0f);
     config->pressure_comp_mbar = SCD30_AMBIENT_PRESSURE;
     config->altitude_comp_m = 0;
     config->auto_calibration = 0;
     config->compensation_mode = SCD30_COMPENSATION_MODE_PRESSURE;
 }

 static esp_err_t scd30_load_cached_config(void)
 {
     if (scd30_config_loaded) {
         return ESP_OK;
     }

     scd30_set_default_config(&scd30_config);

     nvs_handle_t nvs_handle;
     esp_err_t err = nvs_open(SCD30_CONFIG_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
     if (err != ESP_OK) {
         ESP_LOGW(TAG, "Failed to open NVS for SCD30 config, using defaults: %s", esp_err_to_name(err));
         scd30_config_loaded = true;
         return ESP_OK;
     }

     size_t required_size = sizeof(scd30_config);
     err = nvs_get_blob(nvs_handle, SCD30_CONFIG_NVS_KEY, &scd30_config, &required_size);
     nvs_close(nvs_handle);

     if (err != ESP_OK || required_size != sizeof(scd30_config) ||
         scd30_config.version != SCD30_CONFIG_VERSION) {
         scd30_set_default_config(&scd30_config);
         if (err != ESP_ERR_NVS_NOT_FOUND) {
             ESP_LOGW(TAG, "Invalid or missing stored SCD30 config, using defaults");
         }
     }

     scd30_last_zigbee_attr_snapshot = scd30_config;
     scd30_last_zigbee_attr_snapshot_valid = true;
     scd30_config_loaded = true;
     return ESP_OK;
 }

 static esp_err_t scd30_save_config_to_nvs(const scd30_runtime_config_t *config)
 {
     nvs_handle_t nvs_handle;
     esp_err_t err = nvs_open(SCD30_CONFIG_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
     if (err != ESP_OK) {
         return err;
     }

     err = nvs_set_blob(nvs_handle, SCD30_CONFIG_NVS_KEY, config, sizeof(*config));
     if (err == ESP_OK) {
         err = nvs_commit(nvs_handle);
     }

     nvs_close(nvs_handle);
     return err;
 }

 static esp_err_t scd30_ensure_runtime_support(void)
 {
     esp_err_t ret = scd30_load_cached_config();
     if (ret != ESP_OK) {
         return ret;
     }

     if (scd30_command_queue == NULL) {
         scd30_command_queue = xQueueCreate(SCD30_COMMAND_QUEUE_LEN, sizeof(scd30_command_t));
         if (scd30_command_queue == NULL) {
             ESP_LOGE(TAG, "Failed to create SCD30 command queue");
             return ESP_ERR_NO_MEM;
         }
     }

     return ESP_OK;
 }

 static const char *scd30_compensation_mode_to_string(uint8_t mode)
 {
     switch ((scd30_compensation_mode_t)mode) {
         case SCD30_COMPENSATION_MODE_PRESSURE:
             return "pressure";
         case SCD30_COMPENSATION_MODE_ALTITUDE:
             return "altitude";
         case SCD30_COMPENSATION_MODE_NONE:
         default:
             return "none";
     }
 }

 static uint16_t scd30_get_start_pressure_arg(const scd30_runtime_config_t *config)
 {
     return config->compensation_mode == SCD30_COMPENSATION_MODE_PRESSURE ?
         config->pressure_comp_mbar : 0;
 }

 static uint16_t scd30_get_effective_altitude(const scd30_runtime_config_t *config)
 {
     return config->compensation_mode == SCD30_COMPENSATION_MODE_ALTITUDE ?
         config->altitude_comp_m : 0;
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
 
     vTaskDelay(pdMS_TO_TICKS(10));
    
     esp_err_t ret = i2c_handler_write(buf, idx);
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to send command 0x%04x: %s", command, esp_err_to_name(ret));
     }   
 
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
 static esp_err_t scd30_read_data(uint8_t *data, size_t len, const char *context)
 {
     esp_err_t ret = i2c_handler_read(data, len);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to read from sensor: %s", esp_err_to_name(ret));
         return ret;
     }

     scd30_log_raw_data(context, data, len);
 
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

 static esp_err_t scd30_apply_static_config(const scd30_runtime_config_t *config, bool verify_temp_offset)
 {
     float offset_celsius = config->temp_offset_x100 / 100.0f;
     uint16_t effective_altitude = scd30_get_effective_altitude(config);

     ESP_LOGI(TAG,
              "Applying SCD30 config: temp offset %.2f°C, ASC %s, compensation mode %s",
              offset_celsius,
              config->auto_calibration ? "ENABLED" : "DISABLED",
              scd30_compensation_mode_to_string(config->compensation_mode));

     esp_err_t ret = scd30_set_temperature_offset(offset_celsius);
     if (ret != ESP_OK) {
         return ret;
     }

     if (verify_temp_offset) {
         float current_offset = 0.0f;
         ret = scd30_get_temperature_offset(&current_offset);
         if (ret != ESP_OK) {
             return ret;
         }
         ESP_LOGI(TAG, "Verified temperature offset is set to %.2f°C", current_offset);
     }

     ret = scd30_set_auto_calibration(config->auto_calibration != 0);
     if (ret != ESP_OK) {
         return ret;
     }

     ret = scd30_set_altitude_compensation(effective_altitude);
     if (ret != ESP_OK) {
         return ret;
     }

     if (config->compensation_mode == SCD30_COMPENSATION_MODE_PRESSURE) {
         ESP_LOGI(TAG, "Stored pressure compensation: %u mbar", config->pressure_comp_mbar);
     } else if (config->compensation_mode == SCD30_COMPENSATION_MODE_ALTITUDE) {
         ESP_LOGI(TAG, "Stored altitude compensation: %u meters", effective_altitude);
     }

     ret = scd30_set_measurement_interval(SCD30_MEASUREMENT_INTERVAL);
     if (ret != ESP_OK) {
         return ret;
     }

     return ESP_OK;
 }

 static esp_err_t scd30_restart_measurement_with_config(const scd30_runtime_config_t *config)
 {
     esp_err_t ret = scd30_stop_measurement();
     if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to stop measurement before config apply: %s", esp_err_to_name(ret));
     }

     vTaskDelay(pdMS_TO_TICKS(500));

     ret = scd30_apply_static_config(config, false);
     if (ret != ESP_OK) {
         return ret;
     }

     ret = scd30_start_continuous_measurement(scd30_get_start_pressure_arg(config));
     if (ret != ESP_OK) {
         return ret;
     }

     return ESP_OK;
 }

 static esp_err_t scd30_queue_command(const scd30_command_t *command)
 {
     if (scd30_command_queue == NULL) {
         return ESP_ERR_INVALID_STATE;
     }

     if (xQueueSend(scd30_command_queue, command, 0) != pdTRUE) {
         ESP_LOGW(TAG, "SCD30 command queue is full");
         return ESP_ERR_TIMEOUT;
     }

     return ESP_OK;
 }

 static void scd30_process_pending_commands(TickType_t *measurement_start_tick,
                                            bool *stabilization_wait_logged,
                                            bool *stabilization_complete_logged,
                                            uint8_t *consecutive_errors)
 {
     if (scd30_command_queue == NULL) {
         return;
     }

     scd30_command_t command;
     while (xQueueReceive(scd30_command_queue, &command, 0) == pdTRUE) {
         esp_err_t ret = ESP_OK;

         switch (command.type) {
             case SCD30_COMMAND_APPLY_CONFIG:
                 ret = scd30_restart_measurement_with_config(&command.config);
                 if (ret == ESP_OK) {
                     scd30_config = command.config;
                     *measurement_start_tick = xTaskGetTickCount();
                     *stabilization_wait_logged = false;
                     *stabilization_complete_logged = false;
                     *consecutive_errors = 0;
                     ESP_LOGI(TAG, "Updated SCD30 config applied; restarting startup stabilization window");
                 } else {
                     ESP_LOGE(TAG, "Failed to apply updated SCD30 config: %s", esp_err_to_name(ret));
                 }
                 break;

             case SCD30_COMMAND_FORCE_RECALIBRATION:
                 ret = scd30_force_recalibration(command.target_ppm);
                 if (ret != ESP_OK) {
                     ESP_LOGE(TAG, "Forced recalibration command failed: %s", esp_err_to_name(ret));
                 }
                 break;

             default:
                 ESP_LOGW(TAG, "Unknown queued SCD30 command type: %d", command.type);
                 break;
         }
     }
 }

 static void scd30_sync_config_from_zigbee_attributes(TickType_t *measurement_start_tick,
                                                      bool *stabilization_wait_logged,
                                                      bool *stabilization_complete_logged,
                                                      uint8_t *consecutive_errors)
 {
     if (!esp_zb_lock_acquire(pdMS_TO_TICKS(50))) {
         return;
     }

     scd30_runtime_config_t attr_config = scd30_config;
     esp_zb_zcl_attr_t *attr = NULL;
     uint16_t force_recalibration_ppm = 0;

     attr = esp_zb_zcl_get_attribute(HA_CUSTOM_CO2_ENDPOINT, CO2_CONTROL_CLUSTER_ID,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, CO2_CONTROL_ATTR_AUTO_CALIBRATE_ID);
     if (attr && attr->data_p) {
         attr_config.auto_calibration = (*(uint8_t *)attr->data_p) ? 1 : 0;
     }

     attr = esp_zb_zcl_get_attribute(HA_CUSTOM_CO2_ENDPOINT, CO2_CONTROL_CLUSTER_ID,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, CO2_CONTROL_ATTR_TEMP_OFFSET_ID);
     if (attr && attr->data_p) {
         attr_config.temp_offset_x100 = *(int16_t *)attr->data_p;
     }

     attr = esp_zb_zcl_get_attribute(HA_CUSTOM_CO2_ENDPOINT, CO2_CONTROL_CLUSTER_ID,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, CO2_CONTROL_ATTR_PRESSURE_COMP_ID);
     if (attr && attr->data_p) {
         attr_config.pressure_comp_mbar = *(uint16_t *)attr->data_p;
     }

     attr = esp_zb_zcl_get_attribute(HA_CUSTOM_CO2_ENDPOINT, CO2_CONTROL_CLUSTER_ID,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, CO2_CONTROL_ATTR_ALTITUDE_COMP_ID);
     if (attr && attr->data_p) {
         attr_config.altitude_comp_m = *(uint16_t *)attr->data_p;
     }

     attr = esp_zb_zcl_get_attribute(HA_CUSTOM_CO2_ENDPOINT, CO2_CONTROL_CLUSTER_ID,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, CO2_CONTROL_ATTR_FORCE_RECAL_ID);
     if (attr && attr->data_p) {
         force_recalibration_ppm = *(uint16_t *)attr->data_p;
     }

     esp_zb_lock_release();

     bool pressure_changed = !scd30_last_zigbee_attr_snapshot_valid ||
         attr_config.pressure_comp_mbar != scd30_last_zigbee_attr_snapshot.pressure_comp_mbar;
     bool altitude_changed = !scd30_last_zigbee_attr_snapshot_valid ||
         attr_config.altitude_comp_m != scd30_last_zigbee_attr_snapshot.altitude_comp_m;

     if (pressure_changed) {
         attr_config.compensation_mode = attr_config.pressure_comp_mbar == 0 ?
             SCD30_COMPENSATION_MODE_NONE : SCD30_COMPENSATION_MODE_PRESSURE;
     } else if (altitude_changed) {
         attr_config.compensation_mode = attr_config.altitude_comp_m == 0 ?
             SCD30_COMPENSATION_MODE_NONE : SCD30_COMPENSATION_MODE_ALTITUDE;
     } else {
         attr_config.compensation_mode = scd30_config.compensation_mode;
     }
     attr_config.version = SCD30_CONFIG_VERSION;

     scd30_last_zigbee_attr_snapshot = attr_config;
     scd30_last_zigbee_attr_snapshot_valid = true;

     bool config_changed = memcmp(&attr_config, &scd30_config, sizeof(attr_config)) != 0;

     if (!config_changed) {
         if (force_recalibration_ppm == 0 || force_recalibration_ppm == scd30_last_force_recalibration_attr) {
             return;
         }
     }

     if (force_recalibration_ppm != 0 && force_recalibration_ppm != scd30_last_force_recalibration_attr) {
         scd30_last_force_recalibration_attr = force_recalibration_ppm;

         if (force_recalibration_ppm < 400 || force_recalibration_ppm > 2000) {
             ESP_LOGW(TAG, "Ignoring out-of-range forced recalibration request from Zigbee attributes: %u ppm",
                      force_recalibration_ppm);
         } else if (scd30_force_recalibration(force_recalibration_ppm) == ESP_OK) {
             ESP_LOGI(TAG, "Applied forced recalibration request from Zigbee attributes: %u ppm",
                      force_recalibration_ppm);
         } else {
             ESP_LOGE(TAG, "Failed to apply forced recalibration request from Zigbee attributes");
         }

         if (esp_zb_lock_acquire(pdMS_TO_TICKS(50))) {
             uint16_t clear_value = 0;
             esp_zb_zcl_set_attribute_val(HA_CUSTOM_CO2_ENDPOINT, CO2_CONTROL_CLUSTER_ID,
                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                          CO2_CONTROL_ATTR_FORCE_RECAL_ID,
                                          &clear_value, false);
             esp_zb_lock_release();
         }
     } else if (force_recalibration_ppm == 0) {
         scd30_last_force_recalibration_attr = 0;
     }

     if (!config_changed) {
         return;
     }

     ESP_LOGI(TAG,
              "Detected updated Zigbee control attributes: temp offset %.2f°C, ASC %s, pressure %u mbar, altitude %u m",
              attr_config.temp_offset_x100 / 100.0f,
              attr_config.auto_calibration ? "ENABLED" : "DISABLED",
              attr_config.pressure_comp_mbar,
              attr_config.altitude_comp_m);

     if (scd30_save_config_to_nvs(&attr_config) != ESP_OK) {
         ESP_LOGW(TAG, "Failed to persist config detected from Zigbee attributes");
     }

     if (scd30_restart_measurement_with_config(&attr_config) == ESP_OK) {
         scd30_config = attr_config;
         *measurement_start_tick = xTaskGetTickCount();
         *stabilization_wait_logged = false;
         *stabilization_complete_logged = false;
         *consecutive_errors = 0;
         ESP_LOGI(TAG, "Applied Zigbee attribute changes from local attribute table");
     } else {
         ESP_LOGE(TAG, "Failed to apply config derived from Zigbee attributes");
     }
 }
 
 esp_err_t scd30_force_recalibration(uint16_t target_ppm)
 {
     // Validate input range (typical outdoor air is ~400 ppm)
     if (target_ppm < 300 || target_ppm > 2000) {
         ESP_LOGE(TAG, "Invalid recalibration target: %u ppm (valid range: 300-2000)", target_ppm);
         return ESP_ERR_INVALID_ARG;
     }
     
     ESP_LOGI(TAG, "Starting forced recalibration to %u ppm", target_ppm);
     ESP_LOGW(TAG, "Ensure sensor has been exposed to %u ppm CO2 for at least 2 minutes", target_ppm);
     
     // Send forced recalibration command with target value
     esp_err_t ret = scd30_send_command(SCD30_CMD_FORCE_RECALIBRATION, &target_ppm, 1);
     
     if (ret == ESP_OK) {
         ESP_LOGI(TAG, "Forced recalibration to %u ppm initiated successfully", target_ppm);
         ESP_LOGI(TAG, "Recalibration process may take several minutes to complete");
     } else {
         ESP_LOGE(TAG, "Failed to initiate forced recalibration: %s", esp_err_to_name(ret));
     }
     
     return ret;
 }

esp_err_t scd30_init(void)
{
    ESP_LOGI(TAG, "Initializing SCD30 sensor");
    esp_err_t ret;
    int retry_count = 0;
    scd30_runtime_config_t config;

    // The SCD30 device was already registered on the I2C bus by i2c_handler_init().
    // Do not add it again here — a second registration would create a leaked handle
    // since all I2C operations use the handle stored in i2c_handler.

    ret = scd30_ensure_runtime_support();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SCD30 runtime support: %s", esp_err_to_name(ret));
        return ret;
    }

    config = scd30_config;

    while (retry_count < SCD30_INIT_RETRY_COUNT) {
        if (retry_count > 0) {
            ESP_LOGI(TAG, "Retry attempt %d of %d", retry_count + 1, SCD30_INIT_RETRY_COUNT);
            vTaskDelay(pdMS_TO_TICKS(SCD30_INIT_RETRY_DELAY_MS));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Initial delay after power-up

        // Check communication
        ret = scd30_check_communication();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Cannot communicate with SCD30 (attempt %d): %s",
                     retry_count + 1, esp_err_to_name(ret));
            retry_count++;
            continue;
        }

        // Soft reset
        scd30_reset();
        vTaskDelay(pdMS_TO_TICKS(SCD30_INIT_RETRY_DELAY_MS));

        // Stop any ongoing measurement
        scd30_stop_measurement();
        vTaskDelay(pdMS_TO_TICKS(500));

        ret = scd30_apply_static_config(&config, true);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to apply stored SCD30 config (attempt %d): %s",
                     retry_count + 1, esp_err_to_name(ret));
            retry_count++;
            continue;
        }

        if (config.compensation_mode == SCD30_COMPENSATION_MODE_PRESSURE) {
            ESP_LOGI(TAG, "Pressure compensation will be applied when measurements start at %u mbar",
                     config.pressure_comp_mbar);
        } else if (config.compensation_mode == SCD30_COMPENSATION_MODE_ALTITUDE) {
            ESP_LOGI(TAG, "Altitude compensation configured for %u meters", config.altitude_comp_m);
        } else {
            ESP_LOGI(TAG, "Pressure and altitude compensation disabled");
        }

        ESP_LOGI(TAG, "SCD30 initialized successfully after %d attempt(s)", retry_count + 1);
        ESP_LOGI(TAG, "SCD30 configuration complete; startup stabilization will continue in the measurement task");

        return ESP_OK;
    }

    // If all retry attempts failed
    ESP_LOGE(TAG, "Failed to initialize SCD30 after %d attempts", SCD30_INIT_RETRY_COUNT);
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
 
     ret = scd30_read_data(ready_buf, sizeof(ready_buf), "data-ready status");
     if (ret == ESP_OK) {
         *data_ready = (ready_buf[0] << 8 | ready_buf[1]) == 1;
     }
 
     return ret;
 }
 
 static float parse_scd30_float(const uint8_t *data) {
    uint32_t raw = ((uint32_t)data[0] << 24) |
                   ((uint32_t)data[1] << 16) |
                   ((uint32_t)data[3] << 8)  |
                   ((uint32_t)data[4]);
    float value;
    memcpy(&value, &raw, sizeof(float));
    return value;
}

 esp_err_t scd30_read_measurement(scd30_measurement_t *measurement, bool skip_ready_check)
{
    if (measurement == NULL) {
        return ESP_ERR_INVALID_ARG; 
    }

    esp_err_t ret;

    if (!skip_ready_check) {
        bool data_ready = false;
        ret = scd30_get_data_ready_status(&data_ready);
        if (ret != ESP_OK) return ret;
        if (!data_ready) return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[18];  // 3 floats, each 4 bytes + 1 CRC per 2 bytes

    ret = scd30_send_command(SCD30_CMD_READ_MEASUREMENT, NULL, 0);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(15));

    ret = scd30_read_data(data, sizeof(data), "measurement");
    if (ret != ESP_OK) return ret;

    // CRC checks
    for (int i = 0; i < 18; i += 6) {
        if (scd30_crc8(&data[i], 2) != data[i+2] ||
            scd30_crc8(&data[i+3], 2) != data[i+5]) {
            ESP_LOGW(TAG, "CRC check failed for measurement block at offset %d", i);
            return ESP_ERR_INVALID_CRC;
        }
    }

    measurement->co2_ppm     = parse_scd30_float(&data[0]);
    measurement->temperature = parse_scd30_float(&data[6]);
    measurement->humidity    = parse_scd30_float(&data[12]);

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
     scd30_runtime_config_t active_config;
     bool data_ready;
     esp_err_t ret;
     uint8_t consecutive_errors = 0;
     TickType_t measurement_start_tick = xTaskGetTickCount();
     bool stabilization_wait_logged = false;
     bool stabilization_complete_logged = false;

     if (scd30_get_config(&active_config) != ESP_OK) {
         ESP_LOGW(TAG, "Failed to load cached SCD30 config for measurement task, using defaults");
         scd30_set_default_config(&active_config);
     }
 
     // Start continuous measurement with the persisted compensation mode.
     ret = scd30_start_continuous_measurement(scd30_get_start_pressure_arg(&active_config));
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to start continuous measurement");
         vTaskDelete(NULL);
         return;
     }

     ESP_LOGI(TAG, "Continuous measurement started; deferring publishes for %u ms while readings stabilize",
              SCD30_STARTUP_STABILIZATION_MS);
 
     while (task_running) {
         scd30_process_pending_commands(&measurement_start_tick,
                                        &stabilization_wait_logged,
                                        &stabilization_complete_logged,
                                        &consecutive_errors);
         scd30_sync_config_from_zigbee_attributes(&measurement_start_tick,
                                                  &stabilization_wait_logged,
                                                  &stabilization_complete_logged,
                                                  &consecutive_errors);

         // Check if new data is available
         ret = scd30_get_data_ready_status(&data_ready);
         if (ret == ESP_OK && data_ready) {
             // Read measurement
             ret = scd30_read_measurement(&measurement, true);
             if (ret == ESP_OK) {
                 // Validate measurements are within reasonable ranges
                  if (measurement.co2_ppm >= SCD30_CO2_MIN && measurement.co2_ppm <= SCD30_CO2_MAX &&
                      measurement.temperature >= SCD30_TEMP_MIN && measurement.temperature <= SCD30_TEMP_MAX &&
                      measurement.humidity >= SCD30_HUM_MIN && measurement.humidity <= SCD30_HUM_MAX) {
                      bool in_startup_stabilization =
                          (xTaskGetTickCount() - measurement_start_tick) <
                          pdMS_TO_TICKS(SCD30_STARTUP_STABILIZATION_MS);

                      ESP_LOGI(TAG, "CO2: %.1f ppm, Temperature: %.2f°C, Humidity: %.1f%%",
                              measurement.co2_ppm, measurement.temperature, measurement.humidity);

                      if (in_startup_stabilization) {
                          if (!stabilization_wait_logged) {
                              ESP_LOGI(TAG, "Valid measurements received during startup stabilization; postponing Zigbee updates");
                              stabilization_wait_logged = true;
                          }
                      } else {
                          if (!stabilization_complete_logged) {
                              ESP_LOGI(TAG, "Startup stabilization complete; publishing measurements");
                              stabilization_complete_logged = true;
                          }

                          zigbee_handler_update_measurements(measurement.co2_ppm,
                                                           measurement.temperature,
                                                           measurement.humidity);
                      }

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
                  if (scd30_get_config(&active_config) != ESP_OK) {
                      scd30_set_default_config(&active_config);
                  }
                  if (scd30_restart_measurement_with_config(&active_config) != ESP_OK) {
                      ESP_LOGE(TAG, "Failed to reapply SCD30 config after reset");
                  }
                  measurement_start_tick = xTaskGetTickCount();
                  stabilization_wait_logged = false;
                  stabilization_complete_logged = false;
                  ESP_LOGI(TAG, "Continuous measurement restarted; deferring publishes for %u ms while readings stabilize",
                           SCD30_STARTUP_STABILIZATION_MS);
                  consecutive_errors = 0; // Reset error counter after sensor reset attempt
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
     scd30_task_handle = NULL;
     vTaskDelete(NULL);
 }
 
 esp_err_t scd30_start_task(uint8_t task_priority)
 {
     if (scd30_task_handle != NULL) {
         ESP_LOGW(TAG, "Task already running");
         return ESP_ERR_INVALID_STATE;
     }

     esp_err_t ret = scd30_ensure_runtime_support();
     if (ret != ESP_OK) {
         return ret;
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
 
     ret = scd30_read_data(data, sizeof(data), "temperature offset");
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
     // Convert to ticks (1 tick = 0.01°C).
     // Cast via int16_t first: casting a negative float directly to uint16_t
     // is implementation-defined behaviour in C.
     int16_t signed_ticks = (int16_t)(offset_celsius * 100.0f);
     uint16_t offset_ticks = (uint16_t)signed_ticks;
     
     ESP_LOGI(TAG, "Setting temperature offset to %.2f°C (%d ticks, 0x%04X)",
              offset_celsius, signed_ticks, offset_ticks);
              
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
     static uint16_t prev_pressure = 0xFFFF;

     if (pressure_mbar != prev_pressure) {
         if (pressure_mbar == 0) {
             ESP_LOGI(TAG, "Disabling pressure compensation");
         } else {
             ESP_LOGI(TAG, "Setting pressure compensation to %u mbar", pressure_mbar);
         }
         prev_pressure = pressure_mbar;
     }

     // For SCD30, ambient pressure compensation is supplied as the argument to the
     // "start continuous measurement" command. Writing the command again while the
     // sensor is measuring updates the compensation value.
     return scd30_start_continuous_measurement(pressure_mbar);
}

esp_err_t scd30_set_auto_calibration(bool enable)
{
    uint16_t value = enable ? 1 : 0;
    ESP_LOGI(TAG, "Setting auto calibration to %s", enable ? "ENABLED" : "DISABLED");
    return scd30_send_command(SCD30_CMD_AUTO_SELF_CALIBRATION, &value, 1);
}

esp_err_t scd30_get_config(scd30_runtime_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = scd30_load_cached_config();
    if (ret != ESP_OK) {
        return ret;
    }

    *config = scd30_config;
    return ESP_OK;
}

esp_err_t scd30_update_temperature_offset(float offset_celsius)
{
    if (offset_celsius < -10.0f || offset_celsius > 10.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = scd30_ensure_runtime_support();
    if (ret != ESP_OK) {
        return ret;
    }

    scd30_runtime_config_t updated = scd30_config;
    updated.temp_offset_x100 = (int16_t)lroundf(offset_celsius * 100.0f);
    updated.version = SCD30_CONFIG_VERSION;

    ret = scd30_save_config_to_nvs(&updated);
    if (ret != ESP_OK) {
        return ret;
    }

    scd30_config = updated;

    if (scd30_task_handle == NULL) {
        return ESP_OK;
    }

    scd30_command_t command = {
        .type = SCD30_COMMAND_APPLY_CONFIG,
        .config = updated,
    };
    return scd30_queue_command(&command);
}

esp_err_t scd30_update_auto_calibration(bool enable)
{
    esp_err_t ret = scd30_ensure_runtime_support();
    if (ret != ESP_OK) {
        return ret;
    }

    scd30_runtime_config_t updated = scd30_config;
    updated.auto_calibration = enable ? 1 : 0;
    updated.version = SCD30_CONFIG_VERSION;

    ret = scd30_save_config_to_nvs(&updated);
    if (ret != ESP_OK) {
        return ret;
    }

    scd30_config = updated;

    if (scd30_task_handle == NULL) {
        return ESP_OK;
    }

    scd30_command_t command = {
        .type = SCD30_COMMAND_APPLY_CONFIG,
        .config = updated,
    };
    return scd30_queue_command(&command);
}

esp_err_t scd30_update_pressure_compensation(uint16_t pressure_mbar)
{
    if (pressure_mbar != 0 && (pressure_mbar < 700 || pressure_mbar > 1400)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = scd30_ensure_runtime_support();
    if (ret != ESP_OK) {
        return ret;
    }

    scd30_runtime_config_t updated = scd30_config;
    updated.pressure_comp_mbar = pressure_mbar;
    updated.compensation_mode = pressure_mbar == 0 ? SCD30_COMPENSATION_MODE_NONE
                                                   : SCD30_COMPENSATION_MODE_PRESSURE;
    updated.version = SCD30_CONFIG_VERSION;

    ret = scd30_save_config_to_nvs(&updated);
    if (ret != ESP_OK) {
        return ret;
    }

    scd30_config = updated;

    if (scd30_task_handle == NULL) {
        return ESP_OK;
    }

    scd30_command_t command = {
        .type = SCD30_COMMAND_APPLY_CONFIG,
        .config = updated,
    };
    return scd30_queue_command(&command);
}

esp_err_t scd30_update_altitude_compensation(uint16_t altitude_meters)
{
    if (altitude_meters > 8848) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = scd30_ensure_runtime_support();
    if (ret != ESP_OK) {
        return ret;
    }

    scd30_runtime_config_t updated = scd30_config;
    updated.altitude_comp_m = altitude_meters;
    updated.compensation_mode = altitude_meters == 0 ? SCD30_COMPENSATION_MODE_NONE
                                                     : SCD30_COMPENSATION_MODE_ALTITUDE;
    updated.version = SCD30_CONFIG_VERSION;

    ret = scd30_save_config_to_nvs(&updated);
    if (ret != ESP_OK) {
        return ret;
    }

    scd30_config = updated;

    if (scd30_task_handle == NULL) {
        return ESP_OK;
    }

    scd30_command_t command = {
        .type = SCD30_COMMAND_APPLY_CONFIG,
        .config = updated,
    };
    return scd30_queue_command(&command);
}

esp_err_t scd30_request_force_recalibration(uint16_t target_ppm)
{
    esp_err_t ret = scd30_ensure_runtime_support();
    if (ret != ESP_OK) {
        return ret;
    }

    if (scd30_task_handle == NULL) {
        return scd30_force_recalibration(target_ppm);
    }

    scd30_command_t command = {
        .type = SCD30_COMMAND_FORCE_RECALIBRATION,
        .target_ppm = target_ppm,
    };
    return scd30_queue_command(&command);
}
