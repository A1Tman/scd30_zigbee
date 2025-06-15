/**
 * @file zigbee_handler.c
 * @brief Implementation of Zigbee communication handler
 */

#include "zigbee_handler.h"
#include "zcl/esp_zigbee_zcl_carbon_dioxide_measurement.h"
#include "zcl/esp_zigbee_zcl_humidity_meas.h"
#include "zcl/esp_zigbee_zcl_temperature_meas.h"
#include "esp_check.h"
#include "esp_pm.h"
#include "stdbool.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "app_defs.h"
#include "main.h"
#include "i2c_handler.h"
#include "scd30_driver.h"
#include "string.h"
#include "esp_partition.h"
#include <inttypes.h>

static const char *TAG = "ZIGBEE_HANDLER";
static const char *TAG_DEFERRED = "DEFERRED_DRIVER";

/********************* Functions **************************/
/* Static variables */
static uint8_t steering_attempts = 0;
static bool is_connected = false;

/* External variables */
extern EventGroupHandle_t system_events;

/* Forward declarations */
esp_err_t handle_read_attr_response(const esp_zb_zcl_cmd_read_attr_resp_message_t *message);
esp_err_t handle_write_attr_response(const esp_zb_zcl_cmd_write_attr_resp_message_t *message);
esp_err_t handle_attr_report(const esp_zb_zcl_report_attr_message_t *message);
esp_err_t handle_default_response(const esp_zb_zcl_cmd_default_resp_message_t *message);
void handle_status(esp_zb_zcl_status_t status, uint16_t cluster_id, uint16_t attr_id);
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);
static zigbee_connection_callback_t connection_callback = NULL;
static void configure_reporting_alarm_handler(uint8_t param);

/* Core Zigbee functions */
void esp_zb_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing Zigbee stack as End Device");
    
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* Create and initialize clusters */
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(NULL);
    if (!esp_zb_basic_cluster) {
        ESP_LOGE(TAG, "Failed to create basic cluster");
        vTaskDelete(NULL);
        return;
    }
    
    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = 0
    };
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);
    if (!esp_zb_identify_cluster) {
        ESP_LOGE(TAG, "Failed to create identify cluster");
        vTaskDelete(NULL);
        return;
    }

    // --- UPDATED CO₂ CLUSTER CONFIGURATION ---
    // The cluster is defined as a scaled float value.
    // Here the default is 400 ppm scaled down (i.e. 400/1e6) and the maximum is 10000 ppm scaled down.
    esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2_cfg = {
        .measured_value = 400.0f / 1e6f,
        .min_measured_value = 0.0f,
        .max_measured_value = 10000.0f / 1e6f,
    };

    esp_zb_attribute_list_t *esp_zb_co2_cluster = esp_zb_carbon_dioxide_measurement_cluster_create(&co2_cfg);
    if (!esp_zb_co2_cluster) {
        ESP_LOGE(TAG, "Failed to create CO2 cluster");
        vTaskDelete(NULL);
        return;
    }

    esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
        .measured_value = 2000,
        .min_value = -4000,
        .max_value = 7000,
    };
    esp_zb_attribute_list_t *esp_zb_temp_cluster = esp_zb_temperature_meas_cluster_create(&temp_cfg);
    if (!esp_zb_temp_cluster) {
        ESP_LOGE(TAG, "Failed to create temperature cluster");
        vTaskDelete(NULL);
        return;
    }

    esp_zb_humidity_meas_cluster_cfg_t humidity_cfg = {
        .measured_value = 5000,
        .min_value = 0,
        .max_value = 10000,
    };
    esp_zb_attribute_list_t *esp_zb_humidity_cluster = esp_zb_humidity_meas_cluster_create(&humidity_cfg);
    if (!esp_zb_humidity_cluster) {
        ESP_LOGE(TAG, "Failed to create humidity cluster");
        vTaskDelete(NULL);
        return;
    }

    /* Create cluster list and add clusters */
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    if (!esp_zb_cluster_list) {
        ESP_LOGE(TAG, "Failed to create cluster list");
        vTaskDelete(NULL);
        return;
    }
    
    /* Add clusters */
    // Basic clusters
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER);
    // Configuration clusters
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(esp_zb_cluster_list, esp_zb_co2_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temp_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list, esp_zb_humidity_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Create endpoint list */
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    if (!esp_zb_ep_list) {
        ESP_LOGE(TAG, "Failed to create endpoint list");
        vTaskDelete(NULL);
        return;
    }

    /* Set endpoint config */
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_CUSTOM_CO2_ENDPOINT,   // Use your custom CO₂ endpoint (12)
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,             // Home Automation profile
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,       // Custom CO₂ sensor device type (make sure this is defined properly)
        .app_device_version = 0
    };

    /* Add endpoint to list and register device */
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);
    esp_zb_device_register(esp_zb_ep_list);
    
    /* Register Zigbee action handler */
    esp_zb_core_action_handler_register(zb_action_handler);
    
    /* Set channel mask and log it */
    uint32_t channel_mask = ESP_ZB_PRIMARY_CHANNEL_MASK;
    esp_zb_set_primary_network_channel_set(channel_mask);
    ESP_LOGI(TAG, "Setting channel mask: 0x%08lx", channel_mask);
    
    ESP_LOGI(TAG, "All Zigbee clusters created and registered");
    ESP_LOGI(TAG, "Device configured as: Custom CO2 Sensor (Device ID: 0x%04x)", endpoint_config.app_device_id);
    
    // Task main loop
    while (1) {
        esp_zb_cli_main_loop_iteration();
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms delay to prevent watchdog triggers
    }

    // Should never reach here
    vTaskDelete(NULL);
}

esp_err_t zigbee_handler_start(void)
{
    esp_err_t err = esp_zb_start(false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Zigbee stack: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Zigbee stack started, waiting for commissioning");
    steering_attempts = 0;
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    static uint8_t steering_attempts = 0;
    
    if (mode_mask == ESP_ZB_BDB_MODE_NETWORK_STEERING) {
        // First check if we're already connected
        if (zigbee_handler_is_connected()) {
            ESP_LOGI(TAG, "Already connected to network, stopping commissioning attempts");
            steering_attempts = 0;  // Reset for future use if needed
            return;  // Don't proceed with channel changing or scheduling
        }

        // Simplified channel selection - only use common Zigbee channels
        uint8_t channels[] = {15, 11, 20, 25};  // Most common Zigbee channels
        uint8_t channel;
        uint32_t delay_ms;

        // Use a simpler channel selection strategy
        if (steering_attempts < 4) {
            channel = channels[steering_attempts];
            // Give more time on channel 15 (most common)
            delay_ms = (channel == 15) ? 20000 : 10000;
        } else {
            // After trying all channels once, cycle through them again
            channel = channels[steering_attempts % 4];
            delay_ms = 15000;  // Standard delay for subsequent attempts
        }
        
        uint32_t channel_mask = (1 << channel);
        
        ESP_LOGI(TAG, "Commissioning attempt %d: Trying channel %d (mask: 0x%08x), next attempt in %" PRIu32 " ms", 
                 steering_attempts + 1, channel, (unsigned int)channel_mask, delay_ms);
                 
        esp_zb_set_primary_network_channel_set(channel_mask);
        steering_attempts++;
        
        // Limit total attempts to prevent infinite retries
        if (steering_attempts < 12) {  // Max 3 cycles through all channels
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, delay_ms);
        } else {
            ESP_LOGW(TAG, "Maximum commissioning attempts reached");
            steering_attempts = 0;  // Reset for potential manual retry
        }
        return;
    }

    // Start commissioning if mode is not network steering
    esp_err_t err = esp_zb_bdb_start_top_level_commissioning(mode_mask);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Commissioning failed to start (status: %s)", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Started commissioning mode: 0x%x", mode_mask);
    }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    static bool commissioning_in_progress = false;
    
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialization signal received");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION | 
                                                 ESP_ZB_BDB_MODE_NETWORK_FORMATION | 
                                                 ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        ESP_LOGI(TAG, "Device startup signal received");
        if (!commissioning_in_progress) {
            ESP_LOGI(TAG, "Starting commissioning sequence...");
            commissioning_in_progress = true;
            // Start network steering without forcing the trust center address.
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, 
                                    ESP_ZB_BDB_MODE_NETWORK_STEERING, 
                                    STEERING_RETRY_DELAY_MS);
        } else {
            ESP_LOGI(TAG, "Commissioning already in progress");
        }
        break;
            
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            is_connected = true;
            steering_attempts = 0;
            ESP_LOGI(TAG, "Successfully joined network:");
            ESP_LOGI(TAG, "  Channel: %d", esp_zb_get_current_channel());
            ESP_LOGI(TAG, "  PAN ID: 0x%04x", esp_zb_get_pan_id());
            ESP_LOGI(TAG, "  Short addr: 0x%04x", esp_zb_get_short_address());
            
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "  Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0]);
            
            xEventGroupSetBits(system_events, ZIGBEE_CONNECTED_BIT);
            
            if (connection_callback) {
                connection_callback(true);
            }
            
            // Reporting configuration disabled - let coordinator poll instead
            ESP_LOGI(TAG, "Attribute reporting disabled - coordinator will poll for updates");
        } else {
            ESP_LOGW(TAG, "Network steering failed (status: %s, attempt: %d)", 
                     esp_err_to_name(err_status), steering_attempts);
            uint8_t current_channel = esp_zb_get_current_channel();
            ESP_LOGW(TAG, "Last attempted channel: %d", current_channel);
            if (steering_attempts >= 8) {
                ESP_LOGW(TAG, "Multiple steering failures, taking a longer cooldown...");
                steering_attempts = 0;
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                       ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
            } else {
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                       ESP_ZB_BDB_MODE_NETWORK_STEERING, STEERING_RETRY_DELAY_MS);
            }
        }
        break;
    
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        ESP_LOGI(TAG, "Device announcement signal received");
        if (is_connected) {
            xEventGroupSetBits(system_events, ZIGBEE_CONNECTED_BIT);
        }
        break;

    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        ESP_LOGI(TAG, "Network join permit status: %s", err_status == ESP_OK ? "Permitted" : "Not Permitted");
        break;

    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        ESP_LOGI(TAG, "Production configuration ready");
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGW(TAG, "Device left the network");
        is_connected = false;
        xEventGroupClearBits(system_events, ZIGBEE_CONNECTED_BIT);
        if (connection_callback) {
            connection_callback(false);
        }
        commissioning_in_progress = false;
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;
    
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        signal_struct->esp_err_status = ESP_FAIL;  // Prevent sleep
        ESP_LOGD(TAG, "Sleep prevented - device busy");
        break;

    default:
        ESP_LOGI(TAG, "Unhandled signal: %s (0x%x), status: %s", 
                 esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

/* Primary handlers */
/**
 * @brief Handler for Zigbee core actions
 * @param callback_id Type of callback received
 * @param message Pointer to callback-specific message data
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

/**
 * @brief Wrapper function for delayed attribute reporting configuration
 * NOTE: This function is disabled as we're letting the coordinator poll instead
 */
static void configure_reporting_alarm_handler(uint8_t param)
{
    ESP_LOGI(TAG, "Attribute reporting configuration disabled - coordinator will poll");
    // Function body commented out to disable automatic reporting
    /*
    ESP_LOGI(TAG, "Executing delayed attribute reporting configuration");
    
    // Keeping it simple, just calling the configuration function
    esp_err_t ret = zigbee_handler_configure_reporting();
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Reporting configuration failed: %s", esp_err_to_name(ret));
        
        // Using a simple retry mechanism to avoid stack issues
        static uint8_t retry_count = 0;
        if (retry_count < 3) {  // Limit to 3 retries to be safe            
            ESP_LOGI(TAG, "Scheduling retry %d in 10 seconds", retry_count + 1);
            esp_zb_scheduler_alarm(&configure_reporting_alarm_handler, 0, 650);
            retry_count++;
        } else {
            ESP_LOGW(TAG, "Max retries reached, giving up on reporting configuration.");
            retry_count = 0;
        }
    } else {
        ESP_LOGI(TAG, "Attribute reporting configured successfully");
    }
    */
}

/**
 * @brief Handle attribute operations
 */
esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)", message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)",
             message->info.dst_endpoint, message->info.cluster, message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_CUSTOM_CO2_ENDPOINT) {
        switch (message->info.cluster) {
        default:
            ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)", message->info.cluster, message->attribute.id);
        }
    }
    return ret;
}

/* Secondary handlers */
/**
 * @brief Handle read attribute responses
 */
esp_err_t handle_read_attr_response(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_ERR_INVALID_ARG, TAG, "Empty read response message");
    if (message->info.dst_endpoint == HA_CUSTOM_CO2_ENDPOINT) {
        esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
        while (variable != NULL) {
            handle_status(variable->status, message->info.cluster, variable->attribute.id);
            variable = variable->next;
        }
    } else {
        ESP_LOGW(TAG, "Read response for unexpected endpoint: %d", message->info.dst_endpoint);
    }
    return ESP_OK;
}

/**
 * @brief Handle write attribute responses
 */
esp_err_t handle_write_attr_response(const esp_zb_zcl_cmd_write_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_ERR_INVALID_ARG, TAG, "Empty write response message");
    if (message->info.dst_endpoint == HA_CUSTOM_CO2_ENDPOINT) {
        handle_status(message->info.status, message->info.cluster, 0);
    } else {
        ESP_LOGW(TAG, "Write response for unexpected endpoint: %d", message->info.dst_endpoint);
    }
    return ESP_OK;
}

/**
 * @brief Handle Zigbee status codes
 */
void handle_status(esp_zb_zcl_status_t status, uint16_t cluster_id, uint16_t attr_id)
{
    switch (status) {
        case ESP_ZB_ZCL_STATUS_SUCCESS:
            ESP_LOGI(TAG, "Attribute 0x%04x in cluster 0x%04x updated successfully", attr_id, cluster_id);
            break;
        case ESP_ZB_ZCL_STATUS_INVALID_VALUE:
            ESP_LOGW(TAG, "Invalid value for attribute 0x%04x in cluster 0x%04x", attr_id, cluster_id);
            break;
        case ESP_ZB_ZCL_STATUS_READ_ONLY:
            ESP_LOGW(TAG, "Attempted to write read-only attribute 0x%04x in cluster 0x%04x", attr_id, cluster_id);
            break;
        case ESP_ZB_ZDP_STATUS_INSUFFICIENT_SPACE:
            ESP_LOGW(TAG, "Insufficient space for attribute 0x%04x in cluster 0x%04x", attr_id, cluster_id);
            break;
        case ESP_ZB_ZCL_STATUS_NOT_FOUND:
            ESP_LOGW(TAG, "Attribute 0x%04x not found in cluster 0x%04x", attr_id, cluster_id);
            break;
        case ESP_ZB_ZCL_STATUS_UNREPORTABLE_ATTRIB:
            ESP_LOGW(TAG, "Attribute 0x%04x in cluster 0x%04x cannot be reported", attr_id, cluster_id);
            break;
        case ESP_ZB_ZCL_STATUS_INVALID_TYPE:
            ESP_LOGW(TAG, "Invalid data type for attribute 0x%04x in cluster 0x%04x", attr_id, cluster_id);
            break;
        default:
            ESP_LOGW(TAG, "Unknown status (0x%02x) for attribute 0x%04x in cluster 0x%04x", status, attr_id, cluster_id);
            break;
    }
}

void status_management(esp_zb_zcl_status_t status, uint16_t cluster_id, uint16_t attr_id)
{
    switch(status) {
        case 0:
            ESP_LOGI(TAG, "Set attribute %i in cluster %i succeeded", attr_id, cluster_id);
            break;
        default:
            ESP_LOGW(TAG, "Set attribute %i in cluster %i failed", attr_id, cluster_id);
            break;
    }
}

/* Public API functions */
esp_err_t zigbee_handler_update_measurements(float co2_ppm, float temperature, float humidity)
{
    static bool first_measurement = true;
    if (first_measurement) {
        ESP_LOGI(TAG, "First measurement after connection, waiting for warmup...");
        vTaskDelay(pdMS_TO_TICKS(SCD30_WARMUP_TIME_MS));
        first_measurement = false;
    }
    
    esp_zb_zcl_status_t status;
    
    // Convert values for Zigbee transmission:
    float zbCO2 = co2_ppm / 1e6f;  // According to official spec, CO2 is a float value scaled by 1e6
    int16_t zbTemperature = (int16_t)(temperature * 100.0);  // Temperature in 0.01°C units
    uint16_t zbHumidity = (uint16_t)(humidity * 100.0);      // Humidity in 0.01% units
    
    // Validation and logging of the CO2 float value
    if (zbCO2 <= 0.0f || zbCO2 > 0.1f) {  // Sanity check: 0 to 100,000 ppm range
        ESP_LOGW(TAG, "CO2 value outside expected range: %f (raw: %f ppm)", 
                (double)zbCO2, (double)co2_ppm);
        // Still proceed with the value - this is just a warning
    }
    
    // Log the sensor readings with more decimal places for the scaled value:
    ESP_LOGI(TAG, "Sending measurements - CO2: %.1f ppm (scaled: %.10f), Temp: %.2f°C (%d), Humidity: %.1f%% (%u)",
             (double)co2_ppm, (double)zbCO2, (double)temperature, zbTemperature, (double)humidity, zbHumidity);

    // Set attribute values with delays and process events in between
    
    // CO2 attribute
    vTaskDelay(pdMS_TO_TICKS(200));
    status = esp_zb_zcl_set_attribute_val(HA_CUSTOM_CO2_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
        &zbCO2, false);
    status_management(status, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
        ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID);
    
    // Process any pending events
    esp_zb_cli_main_loop_iteration();
    vTaskDelay(pdMS_TO_TICKS(200));

    // Temperature attribute
    status = esp_zb_zcl_set_attribute_val(HA_CUSTOM_CO2_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        &zbTemperature, false);
    status_management(status, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);
    
    // Process any pending events
    esp_zb_cli_main_loop_iteration();
    vTaskDelay(pdMS_TO_TICKS(200));

    // Humidity attribute
    status = esp_zb_zcl_set_attribute_val(HA_CUSTOM_CO2_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        &zbHumidity, false);
    status_management(status, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID);
    
    // Process any pending events
    esp_zb_cli_main_loop_iteration();
    vTaskDelay(pdMS_TO_TICKS(200));
   
    // Final processing of any pending events
    esp_zb_cli_main_loop_iteration();

    return ESP_OK;
}

esp_err_t zigbee_handler_configure_reporting(void)
{
    ESP_LOGI(TAG, "Configuring attribute reporting");

    // Reporting intervals (in seconds)
    const uint16_t min_interval = 30;    // Minimum reporting interval: 30 sec
    const uint16_t max_interval = 300;   // Maximum reporting interval: 300 sec
    esp_err_t ret = ESP_OK;
    esp_err_t final_ret = ESP_OK;  // Track overall success/failure
    
    // According to the official header, the CO₂ measured value is a float in the range [0.0, 1.0]
    // where the value is scaled (ppm/1e6). A reportable change threshold should be defined accordingly.
    static float co2_change = 0.0001f;  // This corresponds to ~100ppm change
    esp_zb_zcl_config_report_record_t co2_record = {
        .attributeID = ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
        .attrType = 0x39, // float type (single precision)
        .min_interval = min_interval,
        .max_interval = max_interval,
        .reportable_change = &co2_change
    };

    esp_zb_zcl_config_report_cmd_t co2_report_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000, // Coordinator address
            .dst_endpoint = HA_CUSTOM_CO2_ENDPOINT,
            .src_endpoint = HA_CUSTOM_CO2_ENDPOINT
        },
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, // Try server direction first
        .record_field = &co2_record,
        .record_number = 1
    };

    // Add error handling with direction flag flexibility
    ret = esp_zb_zcl_config_report_cmd_req(&co2_report_cmd);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "CO2 reporting config failed with TO_SRV direction, trying TO_CLI: %s", esp_err_to_name(ret));
        final_ret = ret;  // Save error but continue
        
        // Try with client direction
        co2_report_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        ret = esp_zb_zcl_config_report_cmd_req(&co2_report_cmd);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure CO₂ reporting: %s (error code: %d)", esp_err_to_name(ret), ret);
            final_ret = ret;
        } else {
            ESP_LOGI(TAG, "CO₂ reporting configured successfully with TO_CLI direction");
        }
    } else {
        ESP_LOGI(TAG, "CO₂ reporting configured successfully with TO_SRV direction");
    }

    // Process any pending events before continuing
    esp_zb_cli_main_loop_iteration();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // REMOVED: Force immediate report section - this was causing crashes

    // ------------------------------
    // Temperature reporting configuration
    // ------------------------------
    // Temperature is represented as a signed 16-bit value in hundredths of °C
    static int16_t temp_change = 25;  // Report if change exceeds 0.25°C
    esp_zb_zcl_config_report_record_t temp_record = {
        .attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .attrType = ESP_ZB_ZCL_ATTR_TYPE_S16,
        .min_interval = min_interval,
        .max_interval = max_interval,
        .reportable_change = &temp_change
    };

    esp_zb_zcl_config_report_cmd_t temp_report_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = HA_CUSTOM_CO2_ENDPOINT,
            .src_endpoint = HA_CUSTOM_CO2_ENDPOINT
        },
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, // Try server direction first
        .record_field = &temp_record,
        .record_number = 1
    };

    // Try with server direction first, then client if needed
    ret = esp_zb_zcl_config_report_cmd_req(&temp_report_cmd);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Temperature reporting config failed with TO_SRV direction, trying TO_CLI: %s", esp_err_to_name(ret));
        if (final_ret == ESP_OK) final_ret = ret;  // Only update if still OK
        
        // Try with client direction
        temp_report_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        ret = esp_zb_zcl_config_report_cmd_req(&temp_report_cmd);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure temperature reporting: %s (error code: %d)", esp_err_to_name(ret), ret);
            if (final_ret == ESP_OK) final_ret = ret;
        } else {
            ESP_LOGI(TAG, "Temperature reporting configured successfully with TO_CLI direction");
        }
    } else {
        ESP_LOGI(TAG, "Temperature reporting configured successfully with TO_SRV direction");
    }

    // Process any pending events
    esp_zb_cli_main_loop_iteration();
    vTaskDelay(pdMS_TO_TICKS(500));

    // REMOVED: Force immediate report section - this was causing crashes

    // ------------------------------
    // Humidity reporting configuration
    // ------------------------------
    // Humidity is represented as an unsigned 16-bit value in hundredths of percent.
    static uint16_t humidity_change = 100;  // Report if change exceeds 1.0% (100 hundredths)
    esp_zb_zcl_config_report_record_t humidity_record = {
        .attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        .attrType = ESP_ZB_ZCL_ATTR_TYPE_U16,
        .min_interval = min_interval,
        .max_interval = max_interval,
        .reportable_change = &humidity_change
    };

    esp_zb_zcl_config_report_cmd_t humidity_report_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = HA_CUSTOM_CO2_ENDPOINT,
            .src_endpoint = HA_CUSTOM_CO2_ENDPOINT
        },
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, // Try server direction first
        .record_field = &humidity_record,
        .record_number = 1
    };

    // Try with server direction first, then client if needed
    ret = esp_zb_zcl_config_report_cmd_req(&humidity_report_cmd);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Humidity reporting config failed with TO_SRV direction, trying TO_CLI: %s", esp_err_to_name(ret));
        if (final_ret == ESP_OK) final_ret = ret;
        
        // Try with client direction
        humidity_report_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        ret = esp_zb_zcl_config_report_cmd_req(&humidity_report_cmd);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure humidity reporting: %s (error code: %d)", esp_err_to_name(ret), ret);
            if (final_ret == ESP_OK) final_ret = ret;
        } else {
            ESP_LOGI(TAG, "Humidity reporting configured successfully with TO_CLI direction");
        }
    } else {
        ESP_LOGI(TAG, "Humidity reporting configured successfully with TO_SRV direction");
    }

    // REMOVED: Force immediate report section - this was causing crashes

    // Final processing to ensure all commands are handled
    esp_zb_cli_main_loop_iteration();
    
    ESP_LOGI(TAG, "Attribute reporting configuration completed with status: %s", 
             final_ret == ESP_OK ? "Success" : "Some configurations failed");

    return final_ret;
}

bool zigbee_handler_is_connected(void)
{
    return is_connected;
}

void zigbee_handler_register_connection_callback(zigbee_connection_callback_t callback)
{
    connection_callback = callback;
}

void zigbee_handler_set_connection_callback(void (*callback)(bool connected)) {
    connection_callback = callback;
}

esp_err_t zigbee_handler_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        , .light_sleep_enable = true
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

/* Task and driver functions */
esp_err_t deferred_driver_init(void)
{
    esp_err_t ret;
    ret = i2c_handler_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_DEFERRED, "FAILED to initialize I2C");
        return ret;
    }
    ret = scd30_start_task(SCD30_TASK_PRIORITY);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_DEFERRED, "FAILED to start SCD30 task");
    }
    return ret;
}

esp_err_t zigbee_handler_clean_start(void)
{
    ESP_LOGI(TAG, "Performing clean start for Zigbee stack");
    
    const esp_partition_t *zb_storage = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, 0x81, "zb_storage");
    
    if (zb_storage) {
        ESP_LOGI(TAG, "Erasing Zigbee storage partition at 0x%" PRIx32 ", size %" PRIu32,
                 zb_storage->address, zb_storage->size);
        esp_err_t err = esp_partition_erase_range(zb_storage, 0, zb_storage->size);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase Zigbee storage: %s", esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "Zigbee storage erased successfully");
    } else {
        ESP_LOGW(TAG, "Zigbee storage partition not found");
    }
    
    const esp_partition_t *zb_fct = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, 0x81, "zb_fct");
    
    if (zb_fct) {
        ESP_LOGI(TAG, "Erasing Zigbee factory test partition");
        esp_partition_erase_range(zb_fct, 0, zb_fct->size);
    }
    
    return zigbee_handler_start();
}

/**
 * @brief Cleanup Zigbee resources before shutdown
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t zigbee_handler_cleanup(void)
{
    ESP_LOGI(TAG, "Cleaning up Zigbee resources");
    
    // Reset connection state
    is_connected = false;
    
    // Stop the Zigbee stack if it's running
    if (esp_zb_is_started()) {
        ESP_LOGI(TAG, "Stopping Zigbee scheduler");
        
        // Cancel specific alarms we know about
        esp_zb_scheduler_alarm_cancel(configure_reporting_alarm_handler, 0);
        esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, 
                                     ESP_ZB_BDB_MODE_NETWORK_STEERING);
    }
    
    ESP_LOGI(TAG, "Zigbee resources cleaned up successfully");
    return ESP_OK;
}

/**
 * @brief Attempt to reconnect to the Zigbee network
 * @return ESP_OK if reconnection procedure started, error code otherwise
 */
esp_err_t zigbee_handler_reconnect(void)
{
    ESP_LOGI(TAG, "Initiating Zigbee reconnection");
    
    // Reset steering attempts counter
    steering_attempts = 0;
    
    if (!esp_zb_is_started()) {
        ESP_LOGE(TAG, "Zigbee stack not started");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Initiate network steering (reconnection)
    ESP_LOGI(TAG, "Starting network steering to reconnect");
    
    // Use channel 15 for first reconnection attempt
    uint32_t channel_mask = (1 << 15);
    esp_zb_set_primary_network_channel_set(channel_mask);
    ESP_LOGI(TAG, "Setting initial reconnection channel mask: 0x%08lx (channel 15)", channel_mask);
    
    esp_err_t err = esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start network steering: %s", esp_err_to_name(err));
        return err;
    }
    
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Compatibility wrappers matching the Q_sensor API                            */
/* -------------------------------------------------------------------------- */

void zigbee_setup(void)
{
    ESP_LOGI(TAG, "zigbee_setup() wrapper called");
    zigbee_handler_start();
}

void update_attributes(attribute_t attribute)
{
    if (attribute == ATTRIBUTE_ALL || attribute == ATTRIBUTE_SCD) {
        scd30_measurement_t m;
        if (scd30_read_measurement(&m, false) == ESP_OK) {
            zigbee_handler_update_measurements(m.co2_ppm, m.temperature, m.humidity);
        } else {
            ESP_LOGW(TAG, "update_attributes: failed to read measurement");
        }
    }
}

void send_bin_cfg_option(int endpoint, bool value) {
    ESP_LOGI(TAG, "send_bin_cfg_option stub called (endpoint %d, value %d)", endpoint, value);
}

void send_zone_1_state(uint8_t bit_index, uint8_t value) {
    ESP_LOGI(TAG, "send_zone_1_state stub called (bit %d, value %d)", bit_index, value);
}

void force_update_task(void) {
    ESP_LOGI(TAG, "force_update_task stub executed");
    update_attributes(ATTRIBUTE_ALL);
}

void force_update(void) {
    ESP_LOGI(TAG, "force_update stub executed");
    update_attributes(ATTRIBUTE_ALL);
}

void read_server_time(void) {
    ESP_LOGI(TAG, "read_server_time stub called");
}
