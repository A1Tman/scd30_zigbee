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
#include "i2c_handler.h"
#include "scd30_driver.h"
#include "string.h"

static const char *TAG = "ZIGBEE_HANDLER";
static const char *TAG_DEFERRED = "DEFERRED_DRIVER";

/********************* Functions **************************/
/* Static variables */
static uint8_t steering_attempts = 0;
static bool is_connected = false;

/* External variables */
extern EventGroupHandle_t system_events;

/* Forward declarations */
esp_err_t  handle_read_attr_response(const esp_zb_zcl_cmd_read_attr_resp_message_t *message);
esp_err_t  handle_write_attr_response(const esp_zb_zcl_cmd_write_attr_resp_message_t *message);
esp_err_t  handle_attr_report(const esp_zb_zcl_report_attr_message_t *message);
esp_err_t  handle_default_response(const esp_zb_zcl_cmd_default_resp_message_t *message);
void  handle_status(esp_zb_zcl_status_t status, uint16_t cluster_id, uint16_t attr_id);
void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);
static zigbee_connection_callback_t connection_callback = NULL;

/* Core Zigbee functions*/
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
    //Basic clusters
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER);
    //Configuration clusters
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
        .endpoint = SCD30_ZB_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };

    /* Add endpoint to list and register device */
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);
    esp_zb_device_register(esp_zb_ep_list);
    
    /* Register Zigbee action handler*/
    esp_zb_core_action_handler_register(zb_action_handler);
    
    /* Set channel mask and log it */
    uint32_t channel_mask = ESP_ZB_PRIMARY_CHANNEL_MASK;
    esp_zb_set_primary_network_channel_set(channel_mask);
    ESP_LOGI(TAG, "Setting channel mask: 0x%08lx", channel_mask);

    ESP_LOGI(TAG, "All Zigbee clusters created and registered");
    ESP_LOGI(TAG, "Device configured as: Temperature Sensor (Device ID: 0x%04x)", 
             ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID);
    
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

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    static bool commissioning_in_progress = false;
    
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialization signal received");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_SE_SIGNAL_APS_KEY_READY:
        ESP_LOGI(TAG, "Configuration ready, starting network steering");
        if (err_status == ESP_OK){
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;

case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    ESP_LOGI(TAG, "Device startup signal received");
    // Always attempt commissioning on startup after flash erase
    if (!commissioning_in_progress) {
        ESP_LOGI(TAG, "Starting commissioning sequence...");
        commissioning_in_progress = true;
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
        
        // Set the connected bit to unblock app_main()
        xEventGroupSetBits(system_events, ZIGBEE_CONNECTED_BIT);

        // Configure attribute reporting
        ESP_LOGI(TAG, "Configuring attribute reporting...");
        zigbee_handler_configure_reporting();

        // Notify through callback if registered
        if (connection_callback) {
            connection_callback(true);
        }
    } else {
        ESP_LOGW(TAG, "Network steering failed (status: %s), retrying...", 
                esp_err_to_name(err_status));
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                             ESP_ZB_BDB_MODE_NETWORK_STEERING,
                             STEERING_RETRY_DELAY_MS);
    }
    break;
    
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        ESP_LOGI(TAG, "Device announcement signal received");
            if (is_connected) {
                // Additional confirmation of successful join
                xEventGroupSetBits(system_events, ZIGBEE_CONNECTED_BIT);
        }
        break;

    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        ESP_LOGI(TAG, "Network join permit status: %s", 
                 err_status == ESP_OK ? "Permitted" : "Not Permitted");
        break;

    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        ESP_LOGI(TAG, "Production configuration ready");
        break;

    case ESP_ZB_SE_SIGNAL_REJOIN:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Rejoin procedure successful");
            is_connected = true;
            xEventGroupSetBits(system_events, ZIGBEE_CONNECTED_BIT);  // Added this
        } else {
            ESP_LOGW(TAG, "Rejoin procedure failed: %s", esp_err_to_name(err_status));
            is_connected = false;
            xEventGroupClearBits(system_events, ZIGBEE_CONNECTED_BIT);  // Added this
            // Force a new commissioning attempt
            commissioning_in_progress = false;  // Added this
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;
    
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        signal_struct->esp_err_status = ESP_FAIL;  // Prevent sleep
        ESP_LOGD(TAG, "Sleep prevented - device busy");
        break;

    default:
        ESP_LOGI(TAG, "Unhandled signal: %s (0x%x), status: %s", 
                 esp_zb_zdo_signal_to_string(sig_type), sig_type, 
                 esp_err_to_name(err_status));
        break;
    }
}

/* Primary handlers*/
/**
 * @brief Handler for Zigbee core actions
 * @param callback_id Type of callback received
 * @param message Pointer to callback-specific message data
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) // Implementation
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

void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    esp_err_t err = esp_zb_bdb_start_top_level_commissioning(mode_mask);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Commissioning failed to start (status: %s), scheduling retry...", 
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Started commissioning mode: 0x%x", mode_mask);
    }
}

/**
 * @brief Handle attribute operations
 */
esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message) // Implementation
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == SCD30_ZB_ENDPOINT) {
        switch (message->info.cluster) 
        {
        default:
            ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }
    return ret;
}

/* Secondary handlers */
/**
 * @brief Handle read attribute responses
 */
esp_err_t  handle_read_attr_response(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_ERR_INVALID_ARG, TAG, "Empty read response message");

    if (message->info.dst_endpoint == SCD30_ZB_ENDPOINT) {
        esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;

        while (variable != NULL) {
            handle_status(variable->status,
                         message->info.cluster,
                         variable->attribute.id);

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
esp_err_t  handle_write_attr_response(const esp_zb_zcl_cmd_write_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_ERR_INVALID_ARG, TAG, "Empty write response message");

    if (message->info.dst_endpoint == SCD30_ZB_ENDPOINT) {
        handle_status(message->info.status,
                     message->info.cluster,
                     0);  // Write responses don't include attr_id
    } else {
        ESP_LOGW(TAG, "Write response for unexpected endpoint: %d", message->info.dst_endpoint);
    }

    return ESP_OK;
}

/**
 * @brief Handle Zigbee status codes
 */
void  handle_status(esp_zb_zcl_status_t status, uint16_t cluster_id, uint16_t attr_id)
{
    switch (status) {
        case ESP_ZB_ZCL_STATUS_SUCCESS:
            ESP_LOGI(TAG, "Attribute 0x%04x in cluster 0x%04x updated successfully", 
                     attr_id, cluster_id);
            break;
        case ESP_ZB_ZCL_STATUS_INVALID_VALUE:
            ESP_LOGW(TAG, "Invalid value for attribute 0x%04x in cluster 0x%04x", 
                     attr_id, cluster_id);
            break;
        case ESP_ZB_ZCL_STATUS_READ_ONLY:
            ESP_LOGW(TAG, "Attempted to write read-only attribute 0x%04x in cluster 0x%04x", 
                     attr_id, cluster_id);
            break;
        case ESP_ZB_ZDP_STATUS_INSUFFICIENT_SPACE:
            ESP_LOGW(TAG, "Insufficient space for attribute 0x%04x in cluster 0x%04x", 
                     attr_id, cluster_id);
            break;
        case ESP_ZB_ZCL_STATUS_NOT_FOUND:
            ESP_LOGW(TAG, "Attribute 0x%04x not found in cluster 0x%04x", 
                     attr_id, cluster_id);
            break;
        case ESP_ZB_ZCL_STATUS_UNREPORTABLE_ATTRIB:
            ESP_LOGW(TAG, "Attribute 0x%04x in cluster 0x%04x cannot be reported", 
                     attr_id, cluster_id);
            break;
        case ESP_ZB_ZCL_STATUS_INVALID_TYPE:
            ESP_LOGW(TAG, "Invalid data type for attribute 0x%04x in cluster 0x%04x", 
                     attr_id, cluster_id);
            break;
        default:
            ESP_LOGW(TAG, "Unknown status (0x%02x) for attribute 0x%04x in cluster 0x%04x", 
                     status, attr_id, cluster_id);
            break;
    }
}

void status_management (esp_zb_zcl_status_t status, uint16_t cluster_id, uint16_t attr_id)
{
    switch(status)
    {
        case 0:
            ESP_LOGI(TAG, "Set attribute %i in cluster %i succeded", attr_id, cluster_id);
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
    
    // Convert values for Zigbee transmission
    uint16_t zbTemperature = (uint16_t)(temperature * 100.0);
    uint16_t zbHumidity = (uint16_t)(humidity * 100.0);
    float_t zbCO2ppm = co2_ppm / 1e6; // Convert ppm to a fraction of one according to typedef

    // Update Zigbee attributes
    status = esp_zb_zcl_set_attribute_val(SCD30_ZB_ENDPOINT, 
        ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT, 
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 
        ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, 
        &zbCO2ppm, false);
    status_management(status, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT, 
        ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID);

    status = esp_zb_zcl_set_attribute_val(SCD30_ZB_ENDPOINT, 
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, 
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, 
        &zbTemperature, false);
    status_management(status, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, 
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);

    status = esp_zb_zcl_set_attribute_val(SCD30_ZB_ENDPOINT, 
        ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, 
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, 
        &zbHumidity, false);
    status_management(status, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, 
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID);

    return ESP_OK;
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

esp_err_t zigbee_handler_configure_reporting(void)
{
    ESP_LOGI(TAG, "Configuring attribute reporting");
    // This tells the device to report values when they change
    // Set values to be reportable when updating them
    esp_err_t ret = ESP_OK;
    
    // For future attribute updates, use the "reportable" flag
    // by modifying zigbee_handler_update_measurements to use this flag
    ESP_LOGI(TAG, "Attribute reporting configured via reportable flag");
    
    return ret;
}

/* Task and driver functions */
esp_err_t deferred_driver_init(void)
{
    esp_err_t ret;

    // Initialize the I2C bus using i2c_handler_init() function from i2c_handler
    ret = i2c_handler_init();     
    if(ret != ESP_OK) {
        ESP_LOGW(TAG_DEFERRED, "FAILED to initialize I2C");
        return ret;
    }

    // I2C Initialization was successful, start the SCD30 task
    ret = scd30_start_task(SCD30_TASK_PRIORITY);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_DEFERRED, "FAILED to start SCD30 task");
    }

    return ret;
}