/**
 * @file main.c
 * @brief Main application for ESP32 Zigbee SCD30 sensor
 */

#include "main.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "i2c_handler.h"
#include "zigbee_handler.h"
#include "scd30_driver.h"
#include "app_defs.h"
#include "zcl_utility.h"

static const char *TAG = "MAIN";

// Event group to signal when Zigbee is ready
EventGroupHandle_t system_events;
#define ZIGBEE_CONNECTED_BIT BIT0

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile end device source code.
#endif

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

// Initialize I2C and SCD30 sensor
static esp_err_t init_sensor_system(void)
{
    esp_err_t ret;
    int scd30_retry_count = 0;

    // Initialize I2C
    ret = i2c_handler_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize SCD30 sensor with retries
    while (scd30_retry_count < SCD30_INIT_RETRY_COUNT) {
        ret = scd30_init();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "SCD30 initialized successfully after %d attempts", 
                     scd30_retry_count + 1);
            break;
        }

        scd30_retry_count++;
        
        if (scd30_retry_count < SCD30_INIT_RETRY_COUNT) {
            ESP_LOGW(TAG, "SCD30 initialization attempt %d failed: %s. Retrying...", 
                     scd30_retry_count, esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(SCD30_INIT_RETRY_DELAY_MS));
        } else {
            ESP_LOGE(TAG, "Failed to initialize SCD30 after %d attempts: %s", 
                     SCD30_INIT_RETRY_COUNT, esp_err_to_name(ret));
            return ret;
        }
    }

    return ESP_OK;
}

// Callback function for Zigbee connection status
void zigbee_connection_callback(bool connected)
{
    if (connected) {
        xEventGroupSetBits(system_events, ZIGBEE_CONNECTED_BIT);
    } else {
        xEventGroupClearBits(system_events, ZIGBEE_CONNECTED_BIT);
    }
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // Create the event group
    system_events = xEventGroupCreate();
    if (system_events == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }

    // Initialize NVS
    ESP_LOGI(TAG, "Initializing NVS...");
    ESP_ERROR_CHECK(init_nvs());
    ESP_LOGI(TAG, "NVS initialized successfully");

    // Register connection callback before initializing Zigbee
    zigbee_handler_set_connection_callback(zigbee_connection_callback);

    // Create Zigbee task
    BaseType_t xReturned = xTaskCreate(
        esp_zb_task,          // Task function
        "Zigbee_main",        // Task name
        4096,                 // Stack size
        NULL,                 // Parameters
        5,                    // Priority
        NULL                  // Task handle
    );
    
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Zigbee task");
        return;
    }

    ESP_ERROR_CHECK(zigbee_handler_start());

    // Wait for Zigbee connection
    EventBits_t bits = xEventGroupWaitBits(system_events,
                                          ZIGBEE_CONNECTED_BIT,
                                          pdFALSE,
                                          pdTRUE,
                                          pdMS_TO_TICKS(120000));

    if ((bits & ZIGBEE_CONNECTED_BIT) == 0) {
        ESP_LOGE(TAG, "Failed to connect to Zigbee network within timeout");
        return;
    }

    ESP_LOGI(TAG, "Zigbee connected, initializing sensor system");

    // Now initialize I2C and sensors
    ESP_ERROR_CHECK(init_sensor_system());

    // Scan the I2C bus
    i2c_scan();

    // Create sensor task only after everything is initialized
    ESP_LOGI(TAG, "Starting SCD30 task...");
    ESP_ERROR_CHECK(scd30_start_task(SCD30_TASK_PRIORITY));
    ESP_LOGI(TAG, "SCD30 task started");

    ESP_LOGI(TAG, "Application startup complete");
}