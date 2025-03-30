/**
 * @file main.c
 * @brief Main application for ESP32 Zigbee SCD30 sensor
 */

#include "main.h"
#include "app_defs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "i2c_handler.h"
#include "zigbee_handler.h"
#include "scd30_driver.h"
#include "zcl_utility.h"
#include "troubleshooting.h"
#include <inttypes.h> // Include for PRIu32 macros

static const char *TAG = "MAIN";

// Constants for timeouts and task configuration
#define ZIGBEE_TASK_STACK_SIZE       4096
#define ZIGBEE_CONNECTION_TIMEOUT_MS 120000  // 2 minutes
#define CONNECTION_CHECK_INTERVAL_MS 2000    // Check connection every 2 seconds
#define MAIN_TASK_DELAY_MS           500     // Check interval for factory reset

// Event group to signal when Zigbee is ready
EventGroupHandle_t system_events = NULL;

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile end device source code.
#endif

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs to be erased due to: %s", esp_err_to_name(ret));
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "NVS erase failed: %s", esp_err_to_name(ret));
            return ret;
        }
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

// Clean up application resources
static void app_cleanup(void)
{
    ESP_LOGI(TAG, "Cleaning up application resources");
    
    // Stop SCD30 task
    scd30_stop_task();
    vTaskDelay(pdMS_TO_TICKS(100)); // Give the task time to stop
    
    // Clean up I2C resources
    i2c_handler_cleanup();  // This returns void
    
    // Clean up Zigbee resources
    esp_err_t ret = zigbee_handler_cleanup();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Zigbee cleanup encountered issues: %s", esp_err_to_name(ret));
    }
    
    // Delete the event group
    if (system_events != NULL) {
        vEventGroupDelete(system_events);
        system_events = NULL;
    }
    
    ESP_LOGI(TAG, "Cleanup complete");
}

// Callback function for Zigbee connection status
void zigbee_connection_callback(bool connected)
{
    if (connected) {
        ESP_LOGI(TAG, "Connection callback: Connected to Zigbee network");
        ESP_LOGI(TAG, "Network details - Short addr: 0x%04x, PAN ID: 0x%04x, Channel: %d",
                 esp_zb_get_short_address(), esp_zb_get_pan_id(), esp_zb_get_current_channel());
        xEventGroupSetBits(system_events, ZIGBEE_CONNECTED_BIT);
    } else {
        ESP_LOGW(TAG, "Connection callback: Disconnected from Zigbee network");
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

    // Initialize factory reset button handler before anything else
    ESP_ERROR_CHECK(troubleshooting_init());

    // Initialize NVS
    ESP_LOGI(TAG, "Initializing NVS...");
    ESP_ERROR_CHECK(init_nvs());
    ESP_LOGI(TAG, "NVS initialized successfully");

    // Register connection callback before initializing Zigbee
    zigbee_handler_set_connection_callback(zigbee_connection_callback);

    // Create Zigbee task with named constants
    BaseType_t xReturned = xTaskCreate(
        esp_zb_task,              // Task function
        "Zigbee_main",            // Task name
        ZIGBEE_TASK_STACK_SIZE,   // Stack size
        NULL,                     // Parameters
        ZIGBEE_TASK_PRIORITY,     // Priority
        NULL                      // Task handle
    );
    
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Zigbee task");
        return;
    }

    // Check if we should perform a clean start
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
    }
    
    uint8_t has_connected_before = 0;
    size_t required_size = sizeof(has_connected_before);
    err = nvs_get_blob(nvs_handle, "has_connected", &has_connected_before, &required_size);
    
    bool clean_start_performed = false;
    
    // Perform clean start only if device has never connected before
    if (err == ESP_ERR_NVS_NOT_FOUND || has_connected_before == 0) {
        ESP_LOGI(TAG, "No previous connection detected, performing clean start");
        ESP_ERROR_CHECK(zigbee_handler_clean_start());
        clean_start_performed = true;
    } else {
        ESP_LOGI(TAG, "Previous connection detected, attempting normal start");
        ESP_ERROR_CHECK(zigbee_handler_start());
    }
    
    nvs_close(nvs_handle);

    // Improved connection waiting with progressive backoff
    bool connected = false;
    uint32_t total_wait_time = 0;
    uint8_t connection_attempts = 0;
    const uint8_t MAX_CONNECTION_ATTEMPTS = 3;

    while (!connected && connection_attempts < MAX_CONNECTION_ATTEMPTS) {
        ESP_LOGI(TAG, "Waiting for Zigbee connection (attempt %d/%d)...", 
                 connection_attempts + 1, MAX_CONNECTION_ATTEMPTS);
        
        // Wait for connection with timeout
        uint32_t attempt_timeout = ZIGBEE_CONNECTION_TIMEOUT_MS;
        uint32_t attempt_wait_time = 0;
        
        while (!connected && attempt_wait_time < attempt_timeout) {
            EventBits_t bits = xEventGroupWaitBits(system_events,
                                                 ZIGBEE_CONNECTED_BIT | FACTORY_RESET_REQUESTED_BIT,
                                                 pdFALSE,
                                                 pdFALSE,  // Wait for either bit
                                                 pdMS_TO_TICKS(CONNECTION_CHECK_INTERVAL_MS));

            attempt_wait_time += CONNECTION_CHECK_INTERVAL_MS;
            total_wait_time += CONNECTION_CHECK_INTERVAL_MS;

            if (bits & FACTORY_RESET_REQUESTED_BIT) {
                ESP_LOGI(TAG, "Factory reset requested, performing reset...");
                
                // Use our cleanup function instead of doing it manually here
                app_cleanup();
                
                // Perform factory reset
                esp_zb_factory_reset();
                
                // Clear NVS - this will also clear the "has_connected" flag
                nvs_flash_erase();
                nvs_flash_init(); // Re-initialize NVS
                
                // Explicitly clear the has_connected flag to ensure clean start on next boot
                nvs_handle_t nvs_handle;
                if (nvs_open("storage", NVS_READWRITE, &nvs_handle) == ESP_OK) {
                    uint8_t has_connected = 0;
                    nvs_set_blob(nvs_handle, "has_connected", &has_connected, sizeof(has_connected));
                    nvs_commit(nvs_handle);
                    nvs_close(nvs_handle);
                    ESP_LOGI(TAG, "Reset connection status in NVS");
                }
                
                // Restart the device
                ESP_LOGI(TAG, "Factory reset complete, restarting device");
                esp_restart();
            }

            if (bits & ZIGBEE_CONNECTED_BIT) {
                connected = true;
                ESP_LOGI(TAG, "Connected to Zigbee network after %u ms", (unsigned int)total_wait_time);
                
                // Save the connection status to NVS
                nvs_handle_t nvs_handle;
                if (nvs_open("storage", NVS_READWRITE, &nvs_handle) == ESP_OK) {
                    uint8_t has_connected = 1;
                    nvs_set_blob(nvs_handle, "has_connected", &has_connected, sizeof(has_connected));
                    nvs_commit(nvs_handle);
                    nvs_close(nvs_handle);
                    ESP_LOGI(TAG, "Saved connection status to NVS");
                }
                
                break;
            } else {
                ESP_LOGW(TAG, "Waiting for Zigbee connection... (%u/%u ms)", 
                        (unsigned int)attempt_wait_time, (unsigned int)attempt_timeout);
                
                // Periodically check if we should attempt a network rejoin
                if (attempt_wait_time % 30000 == 0 && attempt_wait_time > 0) { // Every 30 seconds
                    ESP_LOGI(TAG, "Attempting to force rejoin...");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                }
            }
        }
        
        if (!connected) {
            connection_attempts++;
            
            if (connection_attempts < MAX_CONNECTION_ATTEMPTS) {
                ESP_LOGW(TAG, "Connection attempt %d failed, trying again...", connection_attempts);
                
                // If we've tried once normally, and once with a clean start, try changing channels
                if (connection_attempts == 2) {
                    ESP_LOGI(TAG, "Trying different channel mask...");
                    // Try a different channel - for example channel 25
                    uint32_t channel_mask = (1 << 25);
                    esp_zb_set_primary_network_channel_set(channel_mask);
                } 
                // On the second attempt, try a clean start if we haven't done one already
                else if (connection_attempts == 1 && !clean_start_performed) {
                    ESP_LOGI(TAG, "Performing clean start to reset Zigbee stack state...");
                    zigbee_handler_cleanup();
                    esp_err_t err = zigbee_handler_clean_start();
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Clean start failed: %s", esp_err_to_name(err));
                    }
                    clean_start_performed = true;
                }
                
                vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds before next attempt
            }
        }
    }

    if (!connected) {
        ESP_LOGE(TAG, "Failed to connect to Zigbee network after %d attempts", MAX_CONNECTION_ATTEMPTS);
        
        // Reset the has_connected flag to force a clean start on next boot if we restart
        nvs_handle_t nvs_handle;
        if (nvs_open("storage", NVS_READWRITE, &nvs_handle) == ESP_OK) {
            uint8_t has_connected = 0;
            nvs_set_blob(nvs_handle, "has_connected", &has_connected, sizeof(has_connected));
            nvs_commit(nvs_handle);
            nvs_close(nvs_handle);
            ESP_LOGI(TAG, "Reset connection status in NVS for clean start on next boot");
        }
        
        ESP_LOGI(TAG, "Initializing sensor system anyway to allow local operation");
        
        // We'll initialize the sensor even without Zigbee - this allows local operation
        // and will be ready when network connectivity returns
        ESP_ERROR_CHECK(init_sensor_system());
        
        // Scan the I2C bus
        i2c_scan();
        
        // Start the sensor task anyway
        ESP_LOGI(TAG, "Starting SCD30 task...");
        ESP_ERROR_CHECK(scd30_start_task(SCD30_TASK_PRIORITY));
        
        // Continue to main loop, which will keep trying to reconnect
    } else {
        ESP_LOGI(TAG, "Zigbee connected, initializing sensor system");
        
        // Now initialize I2C and sensors
        ESP_ERROR_CHECK(init_sensor_system());
        
        // Scan the I2C bus
        i2c_scan();
        
        // Create sensor task only after everything is initialized
        ESP_LOGI(TAG, "Starting SCD30 task...");
        ESP_ERROR_CHECK(scd30_start_task(SCD30_TASK_PRIORITY));
    }

    ESP_LOGI(TAG, "Application startup complete");

    // Track consecutive reconnection failures
    uint8_t reconnect_failures = 0;
    const uint8_t MAX_RECONNECT_FAILURES = 5;

    // Improved main loop with better monitoring and recovery
    while (1) {
        // Use a shorter timeout to prevent watchdog triggering
        EventBits_t bits = xEventGroupWaitBits(system_events,
                                            ZIGBEE_CONNECTED_BIT | FACTORY_RESET_REQUESTED_BIT,
                                            pdFALSE,  // Don't clear bits automatically
                                            pdFALSE,  // Wait for any bit
                                            pdMS_TO_TICKS(5000));  // 5-second check interval
                                            
        // Check for factory reset first
        if (bits & FACTORY_RESET_REQUESTED_BIT) {
            ESP_LOGI(TAG, "Factory reset requested, performing reset...");
            
            // Stop sensor task first
            scd30_stop_task();
            vTaskDelay(pdMS_TO_TICKS(100)); // Short delay to allow task to exit
            
            // Perform factory reset
            esp_zb_factory_reset();
            
            // Clear NVS - this will also clear the "has_connected" flag
            nvs_flash_erase();
            nvs_flash_init(); // Re-initialize NVS
            
            // Explicitly clear the has_connected flag to ensure clean start on next boot
            nvs_handle_t nvs_handle;
            if (nvs_open("storage", NVS_READWRITE, &nvs_handle) == ESP_OK) {
                uint8_t has_connected = 0;
                nvs_set_blob(nvs_handle, "has_connected", &has_connected, sizeof(has_connected));
                nvs_commit(nvs_handle);
                nvs_close(nvs_handle);
                ESP_LOGI(TAG, "Reset connection status in NVS");
            }
            
            // Clear the bit
            xEventGroupClearBits(system_events, FACTORY_RESET_REQUESTED_BIT);
            
            // Restart the device
            ESP_LOGI(TAG, "Factory reset complete, restarting device");
            esp_restart();
        }
        
        // Check connection status
        bool is_connected_now = (bits & ZIGBEE_CONNECTED_BIT);
        static bool was_connected = false;
        static uint32_t last_reconnect_attempt = 0;
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Connection lost scenario
        if (was_connected && !is_connected_now) {
            ESP_LOGW(TAG, "Zigbee connection lost, initiating reconnection...");
            
            // Only attempt reconnect if enough time has passed since last attempt
            if (current_time - last_reconnect_attempt > 30000) { // 30 seconds
                last_reconnect_attempt = current_time;
                
                // First check if Zigbee stack is still responsive
                if (esp_zb_is_started()) {
                    ESP_LOGI(TAG, "Attempting to rejoin network...");
                    
                    // Signal that we need to reconnect via the zigbee_handler
                    esp_err_t err = zigbee_handler_reconnect();
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Reconnection attempt failed: %s", esp_err_to_name(err));
                        reconnect_failures++;
                        
                        if (reconnect_failures >= MAX_RECONNECT_FAILURES) {
                            ESP_LOGE(TAG, "Multiple reconnection failures (%d/%d), restarting device", 
                                    reconnect_failures, MAX_RECONNECT_FAILURES);
                            reconnect_failures = 0;
                            
                            // Reset the has_connected flag to force a clean start on next boot
                            nvs_handle_t nvs_handle;
                            if (nvs_open("storage", NVS_READWRITE, &nvs_handle) == ESP_OK) {
                                uint8_t has_connected = 0;
                                nvs_set_blob(nvs_handle, "has_connected", &has_connected, sizeof(has_connected));
                                nvs_commit(nvs_handle);
                                nvs_close(nvs_handle);
                                ESP_LOGI(TAG, "Reset connection status in NVS for clean start on next boot");
                            }
                            
                            esp_restart();
                        } else {
                            ESP_LOGW(TAG, "Will retry reconnection later (failure %d/%d)", 
                                    reconnect_failures, MAX_RECONNECT_FAILURES);
                        }
                    } else {
                        ESP_LOGI(TAG, "Reconnection procedure initiated");
                    }
                } else {
                    ESP_LOGE(TAG, "Zigbee stack is not started, restarting device");
                    
                    // Reset the has_connected flag to force a clean start on next boot
                    nvs_handle_t nvs_handle;
                    if (nvs_open("storage", NVS_READWRITE, &nvs_handle) == ESP_OK) {
                        uint8_t has_connected = 0;
                        nvs_set_blob(nvs_handle, "has_connected", &has_connected, sizeof(has_connected));
                        nvs_commit(nvs_handle);
                        nvs_close(nvs_handle);
                        ESP_LOGI(TAG, "Reset connection status in NVS for clean start on next boot");
                    }
                    
                    esp_restart();
                }
            }
        }
        // Connection never established or long-term disconnection scenario
        else if (!is_connected_now) {
            // Only attempt reconnect if enough time has passed since last attempt
            if (current_time - last_reconnect_attempt > 60000) { // 1 minute
                last_reconnect_attempt = current_time;
                
                ESP_LOGW(TAG, "Still disconnected from Zigbee network, attempting to rejoin...");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
        }
        // Just connected (transition from disconnected to connected)
        else if (!was_connected && is_connected_now) {
            ESP_LOGI(TAG, "Zigbee connection established/restored");
            reconnect_failures = 0; // Reset failure counter on successful connection
            
            // Log connection details
            ESP_LOGI(TAG, "Connected to network - Short addr: 0x%04x, PAN ID: 0x%04x, Channel: %d",
                     esp_zb_get_short_address(), esp_zb_get_pan_id(), esp_zb_get_current_channel());
        }
        // Still connected
        else if (is_connected_now) {
            // Periodically log that we're still connected (every ~5 minutes)
            static uint32_t last_connection_log = 0;
            if (current_time - last_connection_log > 300000) { // 5 minutes
                last_connection_log = current_time;
                ESP_LOGI(TAG, "Zigbee connection stable - Short addr: 0x%04x, Channel: %d", 
                         esp_zb_get_short_address(), esp_zb_get_current_channel());
            }
        }
        
        // Update tracking state
        was_connected = is_connected_now;
        
        // Short delay to ensure other tasks get CPU time
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
