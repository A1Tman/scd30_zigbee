/**
 * @file zigbee_handler.h
 * @brief Zigbee communication handler for ESP32
 * 
 * Handles Zigbee network initialization, message handling, and attribute management.
 */

#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl_utility.h"
#include "esp_zigbee_cluster.h"
#include "zb_vendor.h"
#include "esp_zigbee_core.h"

/* Zigbee Network Configuration */
#define MAX_CHILDREN                   1     /*!< Maximum number of connected devices */
#define INSTALLCODE_POLICY_ENABLE      false  /*!< Install code policy for security */
#define ED_AGING_TIMEOUT               ESP_ZB_ED_AGING_TIMEOUT_64MIN /*!< End device aging timeout */
#define ED_KEEP_ALIVE                  3000   /*!< Keep alive time in milliseconds */
#define HA_CUSTOM_CO2_ENDPOINT         12     /*!< Zigbee endpoint for SCD30 sensor */
#define ESP_ZB_PRIMARY_CHANNEL_MASK    ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /*!< Primary channel mask */

/* Network Steering & Initializing Configuration */
#define ESP_ZB_NETWORK_INIT_TIMEOUT    90000  /*!< Timeout for Zigbee network initialization in ms */
#define STEERING_MAX_ATTEMPTS          100    /*!< Maximum number of steering attempts */
#define STEERING_RETRY_DELAY_MS        5000   /*!< Delay between steering attempts in ms */
#define ED_SCAN_DURATION              3     /*!< Scan duration 0-7, higher is longer */

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x09""ESPRESSIF"      /*!< Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x07"CONFIG_IDF_TARGET /*!< Customized model identifier */
#define ESP_ZB_ZCL_VERSION                 0x08    // ZCL version 8
#define ESP_ZB_POWER_SOURCE                0x04     /*!< DC-source */
                                                    // Available options:
                                                    // 0x01 = Mains (single phase)
                                                    // 0x02 = Mains (3 phase)
                                                    // 0x03 = Battery
                                                    // 0x04 = DC source
                                                    // 0x05 = Emergency mains constantly powered
                                                    // 0x06 = Emergency mains and transfer switch */

/* Connection callback type */
typedef void (*zigbee_connection_callback_t)(bool connected);

/* Configuration Macros */
#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                               \
    {                                                               \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                         \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                                \
    {                                                               \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,       \
    }

// Function declarations of Core Zigbee initialization and control
/**
 * @brief Initialize Zigbee stack and configuration
 * @return ESP_OK if successful, otherwise error code
 */
void esp_zb_task(void *pvParameters);

/**
 * @brief Start Zigbee network operations
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t zigbee_handler_start(void);

/**
 * @brief Initialize power save functionality
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t zigbee_handler_power_save_init(void);

//Main public interface functions
/**
 * @brief Update sensor measurements in Zigbee network
 * @param co2_ppm CO2 measurement in PPM
 * @param temperature Temperature in degrees Celsius
 * @param humidity Relative humidity percentage
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t zigbee_handler_update_measurements(float co2_ppm, float temperature, float humidity);

/**
 * @brief Check if device is connected to Zigbee network
 * @return true if connected, false otherwise
 */
bool zigbee_handler_is_connected(void);

/**
 * @brief Register connection status callback
 * @param callback Function to be called when connection status changes
 */
void zigbee_handler_register_connection_callback(zigbee_connection_callback_t callback);

/**
 * @brief Set the connection status callback for the Zigbee handler.
 *
 * This function allows you to register a callback function that will be called 
 * whenever the connection status changes. The callback function should accept 
 * a boolean parameter indicating whether the device is connected (`true`) or 
 * disconnected (`false`).
 *
 * @param callback Pointer to a function that takes a single `bool` parameter.
 *                 - `true`: Device is connected.
 *                 - `false`: Device is disconnected.
 *
 * @return None.
 */
void zigbee_handler_set_connection_callback(void (*callback)(bool connected));


//Required callback handlers (part of the Zigbee stack interface)
/**
 * @brief Handle Zigbee stack signals
 * @param signal_struct Pointer to signal structure
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);
esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);
esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message);
esp_err_t zigbee_handler_configure_reporting(void);

// Task-related functions that need to be accessible
esp_err_t deferred_driver_init(void);
void SCD30_task(void *pvParameters);
void status_management(esp_zb_zcl_status_t status, uint16_t cluster_id, uint16_t attr_id);

/**
 * @brief Perform a clean start of the Zigbee stack (erases storage)
 * @return ESP_OK on success, or error code
 */
esp_err_t zigbee_handler_clean_start(void);

/**
 * @brief Cleanup Zigbee resources before shutdown
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t zigbee_handler_cleanup(void);

/**
 * @brief Attempt to reconnect to the Zigbee network
 * @return ESP_OK if reconnection procedure started, error code otherwise
 */
esp_err_t zigbee_handler_reconnect(void);