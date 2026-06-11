/**
 * @file zigbee_handler.c
 * @brief Implementation of Zigbee communication handler
 */

#include "zigbee_handler.h"
#include "zcl/esp_zigbee_zcl_carbon_dioxide_measurement.h"
#include "zcl/esp_zigbee_zcl_humidity_meas.h"
#include "zcl/esp_zigbee_zcl_temperature_meas.h"
#include "esp_check.h"
#include "stdbool.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "app_defs.h"
#include "i2c_handler.h"
#include "scd30_driver.h"
#include "string.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "ZIGBEE_HANDLER";
static const int64_t STEERING_STALE_TIMEOUT_MS = 12000;
#define ZIGBEE_SIGNAL_NLME_STATUS_INDICATION 0x32

/********************* Functions **************************/
/* Static variables */
static volatile uint8_t steering_attempts = 0;
static volatile bool is_connected = false;
static volatile bool steering_in_flight = false;
static volatile bool commissioning_in_progress = false;
static volatile uint32_t pending_channel_mask = 0;
static volatile int64_t steering_started_at_ms = 0;
static volatile uint8_t steering_watchdog_token = 0;

/* External variables */
extern EventGroupHandle_t system_events;

/* Forward declarations */
static void handle_status(esp_zb_zcl_status_t status, uint16_t cluster_id, uint16_t attr_id);
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);
static zigbee_connection_callback_t connection_callback = NULL;
static esp_err_t schedule_reconnect_request(uint32_t channel_mask);
static bool steering_is_stale(void);
static void clear_stale_steering_state(const char *reason);
static void steering_watchdog_alarm_handler(uint8_t token);

static esp_err_t handle_co2_control_attribute(const esp_zb_zcl_set_attr_value_message_t *message);
static esp_err_t mirror_co2_control_attribute(uint16_t attr_id, const void *value, size_t value_size);
static esp_err_t handle_auto_calibrate_attr(const esp_zb_zcl_set_attr_value_message_t *message);
static esp_err_t handle_temp_offset_attr(const esp_zb_zcl_set_attr_value_message_t *message);
static esp_err_t handle_pressure_comp_attr(const esp_zb_zcl_set_attr_value_message_t *message);
static esp_err_t handle_altitude_comp_attr(const esp_zb_zcl_set_attr_value_message_t *message);
static esp_err_t handle_force_recalibration_attr(const esp_zb_zcl_set_attr_value_message_t *message);
#if ENABLE_MAINTENANCE_ZIGBEE_CONTROLS
static esp_err_t handle_restart_measurement_attr(const esp_zb_zcl_set_attr_value_message_t *message);
static esp_err_t handle_debug_command_attr(const esp_zb_zcl_set_attr_value_message_t *message);
#endif

/* Core Zigbee functions */
void esp_zb_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing Zigbee stack as End Device");
    
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* Create and initialize clusters */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_VERSION,
        .power_source = ESP_ZB_POWER_SOURCE,
    };
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
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

    /* Manufacturer specific CO2 control cluster */
    scd30_runtime_config_t sensor_config;
    if (scd30_get_config(&sensor_config) != ESP_OK) {
        memset(&sensor_config, 0, sizeof(sensor_config));
        sensor_config.temp_offset_x100 = 250;
        sensor_config.pressure_comp_mbar = 1013;
    }

    bool auto_calibrate = sensor_config.auto_calibration != 0;
    int16_t temp_offset_x100 = sensor_config.temp_offset_x100;
    uint16_t pressure_comp_mbar = sensor_config.pressure_comp_mbar;
    uint16_t altitude_comp_m = sensor_config.altitude_comp_m;
    uint16_t force_recalibration_ppm = 0;  // Default no forced recalibration
#if ENABLE_MAINTENANCE_ZIGBEE_CONTROLS
    bool restart_measurement = false;      // Default no restart
    uint8_t debug_command = 0;             // Default no debug command
#endif
    
    esp_zb_attribute_list_t *esp_zb_co2_ctrl_cluster = esp_zb_zcl_attr_list_create(CO2_CONTROL_CLUSTER_ID);
    if (!esp_zb_co2_ctrl_cluster) {
        ESP_LOGE(TAG, "Failed to create CO2 control cluster");
        vTaskDelete(NULL);
        return;
    }
    esp_zb_custom_cluster_add_custom_attr(esp_zb_co2_ctrl_cluster,
        CO2_CONTROL_ATTR_AUTO_CALIBRATE_ID,
        ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
        &auto_calibrate);

    esp_zb_custom_cluster_add_custom_attr(esp_zb_co2_ctrl_cluster,
            CO2_CONTROL_ATTR_TEMP_OFFSET_ID,
            ESP_ZB_ZCL_ATTR_TYPE_S16,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            &temp_offset_x100);

    esp_zb_custom_cluster_add_custom_attr(esp_zb_co2_ctrl_cluster,
            CO2_CONTROL_ATTR_PRESSURE_COMP_ID,
            ESP_ZB_ZCL_ATTR_TYPE_U16,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            &pressure_comp_mbar);

    esp_zb_custom_cluster_add_custom_attr(esp_zb_co2_ctrl_cluster,
            CO2_CONTROL_ATTR_ALTITUDE_COMP_ID,
            ESP_ZB_ZCL_ATTR_TYPE_U16,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            &altitude_comp_m);

    esp_zb_custom_cluster_add_custom_attr(esp_zb_co2_ctrl_cluster,
            CO2_CONTROL_ATTR_FORCE_RECAL_ID,
            ESP_ZB_ZCL_ATTR_TYPE_U16,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            &force_recalibration_ppm);

#if ENABLE_MAINTENANCE_ZIGBEE_CONTROLS
    esp_zb_custom_cluster_add_custom_attr(esp_zb_co2_ctrl_cluster,
            CO2_CONTROL_ATTR_RESTART_MEASURE_ID,
            ESP_ZB_ZCL_ATTR_TYPE_BOOL,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            &restart_measurement);

    esp_zb_custom_cluster_add_custom_attr(esp_zb_co2_ctrl_cluster,
            CO2_CONTROL_ATTR_DEBUG_COMMAND_ID,
            ESP_ZB_ZCL_ATTR_TYPE_U8,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            &debug_command);
#else
    ESP_LOGI(TAG, "Maintenance-only Zigbee controls are disabled in this build");
#endif

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
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_co2_ctrl_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Create endpoint list */
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    if (!esp_zb_ep_list) {
        ESP_LOGE(TAG, "Failed to create endpoint list");
        vTaskDelete(NULL);
        return;
    }

    /* Set endpoint config */
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_CUSTOM_CO2_ENDPOINT,   // Use custom CO₂ endpoint (12)
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

    // Start the stack from this task, after esp_zb_init() and endpoint
    // registration are complete, so no other task can observe a half-initialized
    // stack (esp_zb_start used to be called from app_main, racing this setup).
    if (zigbee_handler_start() != ESP_OK) {
        ESP_LOGE(TAG, "Zigbee stack failed to start; terminating Zigbee task");
        vTaskDelete(NULL);
        return;
    }

    // Task main loop
    esp_zb_stack_main_loop();

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

    commissioning_in_progress = false;
    steering_in_flight = false;
    pending_channel_mask = 0;
    steering_started_at_ms = 0;
    ESP_LOGI(TAG, "Zigbee stack started, waiting for commissioning");
    steering_attempts = 0;
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    if (mode_mask == ESP_ZB_BDB_MODE_NETWORK_STEERING) {
        if (zigbee_handler_is_connected()) {
            ESP_LOGI(TAG, "Already connected to network, stopping commissioning attempts");
            steering_attempts = 0;
            return;
        }

        uint32_t channel_mask = pending_channel_mask;
        if (channel_mask != 0) {
            pending_channel_mask = 0;
            if ((channel_mask & (channel_mask - 1)) == 0) {
                uint8_t channel = 0;
                while (((channel_mask >> channel) & 0x1u) == 0u) {
                    channel++;
                }
                ESP_LOGI(TAG, "Commissioning attempt %d: forced channel %d (mask: 0x%08x)",
                         steering_attempts + 1, channel, (unsigned int)channel_mask);
            } else {
                ESP_LOGI(TAG, "Commissioning attempt %d: forced channel mask 0x%08x",
                         steering_attempts + 1, (unsigned int)channel_mask);
            }
        } else {
            // Rotate through the most common Zigbee channels on each attempt.
            static const uint8_t channels[] = {15, 11, 20, 25};
            uint8_t channel = channels[steering_attempts % 4];
            channel_mask = (1u << channel);

            ESP_LOGI(TAG, "Commissioning attempt %d: channel %d (mask: 0x%08x)",
                     steering_attempts + 1, channel, (unsigned int)channel_mask);
        }

        if (steering_in_flight) {
            ESP_LOGW(TAG, "Steering already in flight, skipping duplicate attempt");
            return;
        }

        esp_zb_set_primary_network_channel_set(channel_mask);
        steering_attempts++;
        steering_in_flight = true;
        steering_started_at_ms = esp_timer_get_time() / 1000;
        steering_watchdog_token++;
        esp_zb_scheduler_alarm(steering_watchdog_alarm_handler,
                               steering_watchdog_token,
                               STEERING_STALE_TIMEOUT_MS + 1000);

        esp_err_t err = esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        if (err != ESP_OK) {
            steering_in_flight = false;
            steering_started_at_ms = 0;
            ESP_LOGW(TAG, "Failed to start network steering: %s", esp_err_to_name(err));
        }
        return;
    }

    // For non-steering modes (e.g. INITIALIZATION passed from other callers)
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
    
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialization signal received");
        // End devices cannot form networks; only run initialization here.
        // Network steering is started from the DEVICE_FIRST_START / DEVICE_REBOOT handler.
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        ESP_LOGI(TAG, "Device startup signal received");
        if (err_status != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack on startup signal: %s",
                     esp_err_to_name(err_status));
            break;
        }

        if (commissioning_in_progress && steering_is_stale()) {
            clear_stale_steering_state("startup signal arrived while steering was stale");
            commissioning_in_progress = false;
        }

        if (esp_zb_bdb_is_factory_new()) {
            if (!commissioning_in_progress) {
                ESP_LOGI(TAG, "Factory-new device detected; starting commissioning sequence...");
                commissioning_in_progress = true;
                // Start network steering without forcing the trust center address.
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                        ESP_ZB_BDB_MODE_NETWORK_STEERING,
                                        STEERING_RETRY_DELAY_MS);
            } else {
                ESP_LOGI(TAG, "Commissioning already in progress");
            }
        } else {
            ESP_LOGI(TAG, "Known network detected; waiting for fast rejoin before steering fallback");
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
            
            commissioning_in_progress = false;
            steering_in_flight = false;
            steering_started_at_ms = 0;
            xEventGroupSetBits(system_events, ZIGBEE_CONNECTED_BIT);

            if (connection_callback) {
                connection_callback(true);
            }

            // Reporting configuration disabled - let coordinator poll instead
            ESP_LOGI(TAG, "Attribute reporting disabled - coordinator will poll for updates");
        } else {
            // Clear connected state so the retry callback and main loop can act on it
            is_connected = false;
            steering_in_flight = false;
            steering_started_at_ms = 0;
            xEventGroupClearBits(system_events, ZIGBEE_CONNECTED_BIT);
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

    case ZIGBEE_SIGNAL_NLME_STATUS_INDICATION:
        if (err_status == ESP_OK) {
            ESP_LOGD(TAG, "NLME status indication received");
        } else {
            ESP_LOGW(TAG, "NLME status indication: %s", esp_err_to_name(err_status));
        }
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
        steering_in_flight = false;
        steering_started_at_ms = 0;
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;
    
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        // Informational signal: stack reports it has no pending work. We never
        // call esp_zb_sleep_now(), so the device simply stays awake.
        ESP_LOGD(TAG, "CAN_SLEEP signal received; remaining awake");
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
        ESP_LOGD(TAG, "Ignoring unhandled Zigbee action callback (0x%x)", callback_id);
        break;
    }
    return ret;
}

/**
 * @brief Enhanced attribute handler for CO2 control cluster
 * @param message Zigbee attribute set message
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, 
                        "Received message: error status(%d)", message->info.status);
                        
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)",
             message->info.dst_endpoint, message->info.cluster, message->attribute.id, message->attribute.data.size);

    if (message->info.dst_endpoint == HA_CUSTOM_CO2_ENDPOINT) {
        switch (message->info.cluster) {
        case CO2_CONTROL_CLUSTER_ID:
            ret = handle_co2_control_attribute(message);
            break;
        default:
            ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)", message->info.cluster, message->attribute.id);
        }
    }
    return ret;
}

/**
 * @brief Handle CO2 control cluster attribute writes
 * @param message Zigbee attribute set message  
 * @return ESP_OK if successful, otherwise error code
 */
static esp_err_t handle_co2_control_attribute(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    
    if (message->attribute.data.size == 0 || message->attribute.data.value == NULL) {
        ESP_LOGW(TAG, "Empty or NULL attribute data for attribute 0x%04x", message->attribute.id);
        return ESP_ERR_INVALID_ARG;
    }

    switch (message->attribute.id) {
        case CO2_CONTROL_ATTR_AUTO_CALIBRATE_ID:
            ret = handle_auto_calibrate_attr(message);
            break;
            
        case CO2_CONTROL_ATTR_TEMP_OFFSET_ID:
            ret = handle_temp_offset_attr(message);
            break;
            
        case CO2_CONTROL_ATTR_PRESSURE_COMP_ID:
            ret = handle_pressure_comp_attr(message);
            break;
            
        case CO2_CONTROL_ATTR_ALTITUDE_COMP_ID:
            ret = handle_altitude_comp_attr(message);
            break;
            
        case CO2_CONTROL_ATTR_FORCE_RECAL_ID:
            ret = handle_force_recalibration_attr(message);
            break;

#if ENABLE_MAINTENANCE_ZIGBEE_CONTROLS
        case CO2_CONTROL_ATTR_RESTART_MEASURE_ID:
            ret = handle_restart_measurement_attr(message);
            break;

        case CO2_CONTROL_ATTR_DEBUG_COMMAND_ID:
            ret = handle_debug_command_attr(message);
            break;
#endif

        default:
            ESP_LOGW(TAG, "Unknown CO2 control attribute: 0x%04x", message->attribute.id);
            ret = ESP_ERR_NOT_SUPPORTED;
    }
    
    return ret;
}

static esp_err_t mirror_co2_control_attribute(uint16_t attr_id, const void *value, size_t value_size)
{
    esp_zb_zcl_attr_t *attr = esp_zb_zcl_get_attribute(
        HA_CUSTOM_CO2_ENDPOINT,
        CO2_CONTROL_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        attr_id
    );

    if (attr == NULL || attr->data_p == NULL) {
        ESP_LOGW(TAG, "Unable to mirror CO2 control attribute 0x%04x into local Zigbee attribute table", attr_id);
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(attr->data_p, value, value_size);
    return ESP_OK;
}

/**
 * @brief Handle auto calibration attribute
 */
static esp_err_t handle_auto_calibrate_attr(const esp_zb_zcl_set_attr_value_message_t *message)
{
    if (message->attribute.data.size < 1) {
        ESP_LOGW(TAG, "Invalid data size for auto_calibrate: %d", message->attribute.data.size);
        return ESP_ERR_INVALID_ARG;
    }
    
    bool enable = *((uint8_t *)message->attribute.data.value) ? true : false;
    esp_err_t ret = scd30_update_auto_calibration(enable);
    
    if (ret == ESP_OK) {
        uint8_t attr_value = enable ? 1 : 0;
        esp_err_t mirror_ret = mirror_co2_control_attribute(
            CO2_CONTROL_ATTR_AUTO_CALIBRATE_ID,
            &attr_value,
            sizeof(attr_value)
        );
        if (mirror_ret != ESP_OK) {
            ESP_LOGW(TAG, "Auto calibration was applied but local attribute mirror failed: %s",
                     esp_err_to_name(mirror_ret));
        }
        ESP_LOGI(TAG, "Auto calibration update queued: %s", enable ? "ENABLED" : "DISABLED");
    } else {
        ESP_LOGE(TAG, "Failed to set auto calibration: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Handle temperature offset attribute (value * 100)
 */
static esp_err_t handle_temp_offset_attr(const esp_zb_zcl_set_attr_value_message_t *message)
{
    if (message->attribute.data.size < 2) {
        ESP_LOGW(TAG, "Invalid data size for temp_offset: %d", message->attribute.data.size);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Parse int16 value (little-endian)
    int16_t offset_x100;
    memcpy(&offset_x100, message->attribute.data.value, sizeof(int16_t));
    
    // Convert to float (divide by 100)
    float offset_celsius = offset_x100 / 100.0f;
    
    if (offset_celsius < SCD30_TEMP_OFFSET_MIN_C || offset_celsius > SCD30_TEMP_OFFSET_MAX_C) {
        ESP_LOGW(TAG, "Temperature offset out of range: %.2f°C", offset_celsius);
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = scd30_update_temperature_offset(offset_celsius);
    
    if (ret == ESP_OK) {
        esp_err_t mirror_ret = mirror_co2_control_attribute(
            CO2_CONTROL_ATTR_TEMP_OFFSET_ID,
            &offset_x100,
            sizeof(offset_x100)
        );
        if (mirror_ret != ESP_OK) {
            ESP_LOGW(TAG, "Temperature offset was applied but local attribute mirror failed: %s",
                     esp_err_to_name(mirror_ret));
        }
        ESP_LOGI(TAG, "Temperature offset update queued: %.2f°C", offset_celsius);
    } else {
        ESP_LOGE(TAG, "Failed to set temperature offset: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Handle pressure compensation attribute
 */
static esp_err_t handle_pressure_comp_attr(const esp_zb_zcl_set_attr_value_message_t *message)
{
    if (message->attribute.data.size < 2) {
        ESP_LOGW(TAG, "Invalid data size for pressure_comp: %d", message->attribute.data.size);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Parse uint16 value (little-endian)
    uint16_t pressure_mbar;
    memcpy(&pressure_mbar, message->attribute.data.value, sizeof(uint16_t));
    
    // A value of 0 disables pressure compensation. Otherwise expect a realistic
    // atmospheric pressure range.
    if (pressure_mbar != 0 && (pressure_mbar < 700 || pressure_mbar > 1400)) {
        ESP_LOGW(TAG, "Pressure compensation out of range: %u mbar", pressure_mbar);
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = scd30_update_pressure_compensation(pressure_mbar);
    
    if (ret == ESP_OK) {
        esp_err_t mirror_ret = mirror_co2_control_attribute(
            CO2_CONTROL_ATTR_PRESSURE_COMP_ID,
            &pressure_mbar,
            sizeof(pressure_mbar)
        );
        if (mirror_ret != ESP_OK) {
            ESP_LOGW(TAG, "Pressure compensation was applied but local attribute mirror failed: %s",
                     esp_err_to_name(mirror_ret));
        }
        if (pressure_mbar == 0) {
            ESP_LOGI(TAG, "Pressure compensation disable queued");
        } else {
            ESP_LOGI(TAG, "Pressure compensation update queued: %u mbar", pressure_mbar);
        }
    } else {
        ESP_LOGE(TAG, "Failed to set pressure compensation: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Handle altitude compensation attribute
 */
static esp_err_t handle_altitude_comp_attr(const esp_zb_zcl_set_attr_value_message_t *message)
{
    if (message->attribute.data.size < 2) {
        ESP_LOGW(TAG, "Invalid data size for altitude_comp: %d", message->attribute.data.size);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Parse uint16 value (little-endian)
    uint16_t altitude_m;
    memcpy(&altitude_m, message->attribute.data.value, sizeof(uint16_t));
    
    // Note: Setting altitude to 0 disables altitude compensation
    if (altitude_m > 8848) {  // Mount Everest height as upper limit
        ESP_LOGW(TAG, "Altitude compensation out of range: %u meters", altitude_m);
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = scd30_update_altitude_compensation(altitude_m);
    
    if (ret == ESP_OK) {
        esp_err_t mirror_ret = mirror_co2_control_attribute(
            CO2_CONTROL_ATTR_ALTITUDE_COMP_ID,
            &altitude_m,
            sizeof(altitude_m)
        );
        if (mirror_ret != ESP_OK) {
            ESP_LOGW(TAG, "Altitude compensation was applied but local attribute mirror failed: %s",
                     esp_err_to_name(mirror_ret));
        }
        if (altitude_m == 0) {
            ESP_LOGI(TAG, "Altitude compensation disable queued");
        } else {
            ESP_LOGI(TAG, "Altitude compensation update queued: %u meters", altitude_m);
        }
    } else {
        ESP_LOGE(TAG, "Failed to set altitude compensation: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Handle forced recalibration attribute
 */
static esp_err_t handle_force_recalibration_attr(const esp_zb_zcl_set_attr_value_message_t *message)
{
    if (message->attribute.data.size < 2) {
        ESP_LOGW(TAG, "Invalid data size for force_recalibration: %d", message->attribute.data.size);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Parse uint16 value (little-endian)
    uint16_t target_ppm;
    memcpy(&target_ppm, message->attribute.data.value, sizeof(uint16_t));
    
    // A write of 0 cancels/clears a pending recalibration without triggering one
    if (target_ppm == 0) {
        ESP_LOGI(TAG, "Force recalibration cancelled (target = 0)");
        return ESP_OK;
    }

    if (target_ppm < SCD30_FRC_TARGET_PPM_MIN || target_ppm > SCD30_FRC_TARGET_PPM_MAX) {
        ESP_LOGW(TAG, "Force recalibration value out of range: %u ppm", target_ppm);
        return ESP_ERR_INVALID_ARG;
    }

    // Only validate here. The stack has already stored the value in the
    // attribute table; scd30_sync_config_from_zigbee_attributes (sensor task)
    // picks it up from there, applies the FRC exactly once and clears the
    // attribute back to 0. Triggering it here as well would send the FRC
    // command to the sensor twice.
    ESP_LOGI(TAG, "Forced recalibration to %u ppm accepted; sensor task will apply it", target_ppm);

    return ESP_OK;
}

/**
 * @brief Handle restart measurement attribute
 */
#if ENABLE_MAINTENANCE_ZIGBEE_CONTROLS
static esp_err_t handle_restart_measurement_attr(const esp_zb_zcl_set_attr_value_message_t *message)
{
    if (message->attribute.data.size < 1) {
        ESP_LOGW(TAG, "Invalid data size for restart_measurement: %d", message->attribute.data.size);
        return ESP_ERR_INVALID_ARG;
    }
    
    bool restart = *((uint8_t *)message->attribute.data.value) ? true : false;
    
    if (!restart) {
        ESP_LOGI(TAG, "Restart measurement set to false - no action taken");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Restarting SCD30 measurements");
    
    // Stop current measurement
    esp_err_t ret = scd30_stop_measurement();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to stop measurement: %s", esp_err_to_name(ret));
    }

    // NOTE: Do not vTaskDelay here — this runs in the Zigbee stack callback
    // context and blocking would prevent the stack from servicing keep-alives.
    // scd30_send_command already enforces the required inter-command gap (30 ms).

    // Restart with default pressure
    ret = scd30_start_continuous_measurement(SCD30_AMBIENT_PRESSURE);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SCD30 measurements restarted successfully");
    } else {
        ESP_LOGE(TAG, "Failed to restart measurements: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief Handle debug command attribute
 */
static esp_err_t handle_debug_command_attr(const esp_zb_zcl_set_attr_value_message_t *message)
{
    if (message->attribute.data.size < 1) {
        ESP_LOGW(TAG, "Invalid data size for debug_command: %d", message->attribute.data.size);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t debug_opcode = *((uint8_t *)message->attribute.data.value);
    
    ESP_LOGI(TAG, "Debug command received: 0x%02x", debug_opcode);
    
    esp_err_t ret = ESP_OK;
    
    switch (debug_opcode) {
        case 0x01:  // Soft reset
            ESP_LOGI(TAG, "Debug: Performing soft reset");
            ret = scd30_reset();
            break;
            
        case 0x02:  // Stop measurement
            ESP_LOGI(TAG, "Debug: Stopping measurements");
            ret = scd30_stop_measurement();
            break;
            
        case 0x03:  // Start measurement
            ESP_LOGI(TAG, "Debug: Starting measurements");
            ret = scd30_start_continuous_measurement(SCD30_AMBIENT_PRESSURE);
            break;
            
        case 0x04:  // Force measurement read
            ESP_LOGI(TAG, "Debug: Forcing measurement read");
            scd30_measurement_t measurement;
            ret = scd30_read_measurement(&measurement, true);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Debug measurement: CO2=%.1f ppm, T=%.2f°C, H=%.1f%%",
                        measurement.co2_ppm, measurement.temperature, measurement.humidity);
            }
            break;
            
        case 0x00:  // No operation
            ESP_LOGI(TAG, "Debug: No operation");
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown debug command: 0x%02x", debug_opcode);
            ret = ESP_ERR_INVALID_ARG;
    }
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Debug command 0x%02x executed successfully", debug_opcode);
    } else {
        ESP_LOGE(TAG, "Debug command 0x%02x failed: %s", debug_opcode, esp_err_to_name(ret));
    }

    return ret;
}
#endif

/* Secondary handlers */
/**
 * @brief Handle Zigbee status codes
 */
static void handle_status(esp_zb_zcl_status_t status, uint16_t cluster_id, uint16_t attr_id)
{
    switch (status) {
        case ESP_ZB_ZCL_STATUS_SUCCESS:
            ESP_LOGD(TAG, "Attribute 0x%04x in cluster 0x%04x updated successfully", attr_id, cluster_id);
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

/* Public API functions */
esp_err_t zigbee_handler_update_measurements(float co2_ppm, float temperature, float humidity)
{
    esp_zb_zcl_status_t status;
    
    // Convert values for Zigbee transmission:
    float zbCO2 = co2_ppm / 1e6f;  // According to official spec, CO2 is a float value scaled by 1e6
    int16_t zbTemperature = (int16_t)roundf(temperature * 100.0f);  // Temperature in 0.01°C units
    uint16_t zbHumidity = (uint16_t)roundf(humidity * 100.0f);      // Humidity in 0.01% units
    
    // Validation and logging of the CO2 float value
    if (zbCO2 <= 0.0f || zbCO2 > 0.01f) {  // Sanity check: 0 to 10,000 ppm (SCD30 maximum)
        ESP_LOGW(TAG, "CO2 value outside expected range: %f (raw: %f ppm)", 
                (double)zbCO2, (double)co2_ppm);
        // Still proceed with the value - this is just a warning
    }
    
    // Log the sensor readings with more decimal places for the scaled value:
    ESP_LOGD(TAG, "Sending measurements - CO2: %.1f ppm (scaled: %.10f), Temp: %.2f°C (%d), Humidity: %.1f%% (%u)",
             (double)co2_ppm, (double)zbCO2, (double)temperature, zbTemperature, (double)humidity, zbHumidity);

    // Acquire the Zigbee stack lock before calling any Zigbee SDK API from this
    // (non-Zigbee) task context.  The SDK docs state this is mandatory for all
    // call sites that are not Zigbee callbacks.
    if (!esp_zb_lock_acquire(pdMS_TO_TICKS(200))) {
        ESP_LOGW(TAG, "Failed to acquire Zigbee lock for attribute update, skipping");
        return ESP_ERR_TIMEOUT;
    }

    // CO2 attribute
    status = esp_zb_zcl_set_attribute_val(HA_CUSTOM_CO2_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
        &zbCO2, false);
    handle_status(status, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
        ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID);

    // Temperature attribute
    status = esp_zb_zcl_set_attribute_val(HA_CUSTOM_CO2_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        &zbTemperature, false);
    handle_status(status, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);

    // Humidity attribute
    status = esp_zb_zcl_set_attribute_val(HA_CUSTOM_CO2_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        &zbHumidity, false);
    handle_status(status, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID);

    esp_zb_lock_release();

    return ESP_OK;
}

bool zigbee_handler_is_connected(void)
{
    return is_connected;
}

void zigbee_handler_set_connection_callback(zigbee_connection_callback_t callback)
{
    connection_callback = callback;
}

/**
 * @brief Attempt to reconnect to the Zigbee network
 * @return ESP_OK if reconnection procedure started, error code otherwise
 */
esp_err_t zigbee_handler_reconnect(void)
{
    ESP_LOGI(TAG, "Initiating Zigbee reconnection");

    return schedule_reconnect_request(0);
}

esp_err_t zigbee_handler_reconnect_with_mask(uint32_t channel_mask)
{
    ESP_LOGI(TAG, "Initiating Zigbee reconnection with mask 0x%08lx",
             (unsigned long)channel_mask);

    return schedule_reconnect_request(channel_mask);
}

static esp_err_t schedule_reconnect_request(uint32_t channel_mask)
{
    if (!esp_zb_is_started()) {
        ESP_LOGE(TAG, "Zigbee stack not started");
        return ESP_ERR_INVALID_STATE;
    }

    if (steering_in_flight && steering_is_stale()) {
        clear_stale_steering_state("reconnect requested after steering timeout");
    }

    if (steering_in_flight) {
        ESP_LOGW(TAG, "Steering already in flight, reconnect request ignored");
        return ESP_OK;
    }

    if (!esp_zb_lock_acquire(pdMS_TO_TICKS(200))) {
        ESP_LOGW(TAG, "Failed to acquire Zigbee lock for reconnect scheduling");
        return ESP_ERR_TIMEOUT;
    }

    // Reset steering attempts counter so reconnects start from a known state.
    // Done under the Zigbee lock so it cannot race the increment in the
    // commissioning callback running in stack context.
    steering_attempts = 0;
    pending_channel_mask = channel_mask;
    // Route through the commissioning callback so channel rotation and
    // the steering_in_flight guard are applied consistently.
    esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                           ESP_ZB_BDB_MODE_NETWORK_STEERING, 0);
    esp_zb_lock_release();
    ESP_LOGI(TAG, "Reconnection scheduled via commissioning callback");

    return ESP_OK;
}

static bool steering_is_stale(void)
{
    if (!steering_in_flight || steering_started_at_ms == 0) {
        return false;
    }

    int64_t now_ms = esp_timer_get_time() / 1000;
    return (now_ms - steering_started_at_ms) >= STEERING_STALE_TIMEOUT_MS;
}

static void clear_stale_steering_state(const char *reason)
{
    ESP_LOGW(TAG, "Clearing stale steering state: %s", reason);
    commissioning_in_progress = false;
    steering_in_flight = false;
    steering_started_at_ms = 0;
}

static void steering_watchdog_alarm_handler(uint8_t token)
{
    if (token != steering_watchdog_token) {
        return;
    }

    if (!steering_is_stale()) {
        return;
    }

    clear_stale_steering_state("steering result timeout expired");
    ESP_LOGW(TAG, "Restarting commissioning after stale steering timeout");
    esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                           ESP_ZB_BDB_MODE_NETWORK_STEERING, 0);
}


