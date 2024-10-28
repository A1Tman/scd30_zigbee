*Sensor commands*
The command set of the SCD30 is defined as follows. All commands are available via Modbus and I2C.
- Trigger continuous measurement with optional ambient pressure compensation
- Stop continuous measurement
- Set measurement interval
- Get data ready status
- Read measurement
- (De-)Activate continuous calculation of reference value for automatic self-calibration (ASC)
- Set external reference value for forced recalibration (FRC)
- Set temperature offset for onboard RH/T sensor
- Altitude compensation
- Read firmware version
- Soft reset

Dependencies
esp-zigbee-sdk
zcl:

Working snippets and important typedefs:
/**
 * @brief Structure of device descriptor on a endpoint
 */
typedef struct esp_zb_endpoint_config_s {
    uint8_t    endpoint;                        /*!< Endpoint */
    uint16_t   app_profile_id;                  /*!< Application profile identifier */
    uint16_t   app_device_id;                   /*!< Application device identifier */
    uint32_t   app_device_version: 4;           /*!< Application device version */
} ESP_ZB_PACKED_STRUCT
esp_zb_endpoint_config_t;

/******************* endpoint data model *******************/

/**
 * @brief Type to represent ZCL endpoint definition structure
 * @note  The esp_zb_zcl_reporting_info_t defines see @ref esp_zb_zcl_reporting_info_s
 * @note  The esp_zb_af_simple_desc_1_1_t defines see @ref esp_zb_af_simple_desc_1_1_s
 */
typedef struct esp_zb_endpoint_s {
    uint8_t ep_id;                              /*!< Endpoint ID */
    uint16_t profile_id;                        /*!< Application profile, which implemented on endpoint */
    esp_zb_callback_t device_handler;           /*!< endpoint specific callback, handles all commands for this endpoint. If set, it will be called to handle callback,like esp_zb_add_read_attr_resp_cb */
    esp_zb_callback_t identify_handler;         /*!< Identify notification callback. If set, it will be called on identification start and stop indicating start event with a non-zero argument*/
    uint8_t reserved_size;                      /*!< Unused parameter (reserved for future use) */
    void *reserved_ptr;                         /*!< Unused parameter (reserved for future use) */
    uint8_t cluster_count;                      /*!< Number of supported clusters */
    union {
        esp_zb_zcl_cluster_t *cluster_desc_list;    /*!< Supported clusters arranged in array */
        esp_zb_cluster_list_t *cluster_list;        /*!< Supported clusters arranged in list */
    } ESP_ZB_PACKED_STRUCT;                         /*!< Cluster data model */
    esp_zb_af_simple_desc_1_1_t *simple_desc;   /*!< Simple descriptor */
#if defined ZB_ENABLE_ZLL
    uint8_t group_id_count;                     /*!< Number of group id */
#endif /* defined ZB_ENABLE_ZLL */
    uint8_t rep_info_count;                     /*!< Number of reporting info slots */
    esp_zb_zcl_reporting_info_t *reporting_info; /*!< Attributes reporting information */
    uint8_t cvc_alarm_count;          /*!< Number of continuous value change alarm slots */
    esp_zb_zcl_cvc_alarm_variables_t *cvc_alarm_info;   /*!< pointer to the cvc alarm structure */
} ESP_ZB_PACKED_STRUCT
esp_zb_endpoint_t;

*/****************** standard clusters *********************/*
/**
 * @brief Zigbee standard mandatory attribute for basic cluster.
 *
 */
typedef struct esp_zb_basic_cluster_cfg_s {
    uint8_t  zcl_version;                       /*!<  ZCL version */
    uint8_t  power_source;                      /*!<  The sources of power available to the device */
} esp_zb_basic_cluster_cfg_t;

/**
 * @brief Zigbee default attribute for Commissioning cluster.
 *
 */
typedef struct esp_zb_commissioning_cluster_cfg_s {
    uint16_t           short_address;               /**< Short Address */
    esp_zb_ieee_addr_t extended_panid;              /**< Extended Panid */
    uint16_t           panid;                       /**< Panid */
    uint32_t           channel_mask;                /**< Channel Mask */
    uint8_t            protocol_version;            /**< Protocol Version */
    uint8_t            stack_profile;               /**< Stack Profile */
    uint8_t            startup_control;             /**< Startup Control */
    esp_zb_ieee_addr_t trust_center_address;        /**< Trust Center Address */
    uint8_t            network_key[16];             /**< Network Key */
    bool               use_insecure_join;           /**< Use Insecure Join */
    uint8_t            preconfigured_link_key[16];  /**< Preconfigured Link Key */
    uint8_t            network_key_seq_num;         /**< Network Key Seq Num */
    uint8_t            network_key_type;            /**< Network Key Type */
    uint16_t           network_manager_address;     /**< Network Manager Address */
} esp_zb_commissioning_cluster_cfg_t;

/**
 * @brief Zigbee standard mandatory attribute for temperature measurement cluster.
 *
 */
typedef struct esp_zb_temperature_meas_cluster_cfg_s {
    int16_t measured_value;                     /*!<  The attribute indicates the temperature in degrees Celsius */
    int16_t min_value;                          /*!<  The attribute indicates minimum value of the measured value */
    int16_t max_value;                          /*!<  The attribute indicates maximum value of the measured value */
} esp_zb_temperature_meas_cluster_cfg_t;

/**
 * @brief Zigbee standard mandatory attribute for humidity measurement cluster.
 *
 */
typedef struct esp_zb_humidity_meas_cluster_cfg_s {
    uint16_t measured_value;                     /*!<  The attribute indicates the humidity in 100*percent */
    uint16_t min_value;                          /*!<  The attribute indicates minimum value of the measured value */
    uint16_t max_value;                          /*!<  The attribute indicates maximum value of the measured value */
} esp_zb_humidity_meas_cluster_cfg_t;


/**
 * @brief Zigbee standard mandatory attribute for occupancy sensing cluster
 *
 */
typedef struct esp_zb_occupancy_sensing_cluster_cfg_s {
    uint8_t occupancy;                          /*!<  Bit 0 specifies the sensed occupancy as follows: 1 = occupied, 0 = unoccupied. */
    uint32_t sensor_type;                       /*!<  The attribute specifies the type of the occupancy sensor */
    uint8_t sensor_type_bitmap;                 /*!<  The attribute specifies the types of the occupancy sensor */
} esp_zb_occupancy_sensing_cluster_cfg_t;

/**
 * @brief Zigbee standard mandatory attribute for carbon dioxide measurement cluster
 *
 */
typedef struct esp_zb_carbon_dioxide_measurement_cluster_cfg_s {
    float_t measured_value;     /*!<  This attribute represents the carbon dioxide concentration as a fraction of one */
    float_t min_measured_value; /*!<  This attribute indicates the minimum value of measuredvalue that is capable of being measured */
    float_t max_measured_value; /*!<  This attribute indicates the maximum value of measuredvalue that is capable of being measured */
} esp_zb_carbon_dioxide_measurement_cluster_cfg_t;


/********************* Standard Clusters ENDPOINT **********************/

/**
 * @brief Zigbee HA standard temperature sensor clusters.
 *
 */
typedef struct esp_zb_temperature_sensor_cfg_s {
    esp_zb_basic_cluster_cfg_t basic_cfg;                /*!<  Basic cluster configuration, @ref esp_zb_basic_cluster_cfg_s */
    esp_zb_identify_cluster_cfg_t identify_cfg;          /*!<  Identify cluster configuration, @ref esp_zb_identify_cluster_cfg_s */
    esp_zb_temperature_meas_cluster_cfg_t temp_meas_cfg; /*!<  Temperature measurement cluster configuration, @ref esp_zb_temperature_meas_cluster_cfg_s */
} esp_zb_temperature_sensor_cfg_t;

/**
 * @brief Zigbee HA standard configuration tool clusters.
 *
 */
typedef struct esp_zb_configuration_tool_cfg_s {
    esp_zb_basic_cluster_cfg_t basic_cfg;       /*!<  Basic cluster configuration, @ref esp_zb_basic_cluster_cfg_s */
    esp_zb_identify_cluster_cfg_t identify_cfg; /*!<  Identify cluster configuration, @ref esp_zb_identify_cluster_cfg_s */
} esp_zb_configuration_tool_cfg_t;

/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "esp_zigbee_type.h"

/**
 * @brief  Create an empty endpoint list.
 *
 * @note This endpoint list is ready to add endpoint refer @ref esp_zb_ep_list_add_ep.
 * @return pointer to  @ref esp_zb_ep_list_s
 *
 */
esp_zb_ep_list_t *esp_zb_ep_list_create(void);

/**
 * @brief Add an endpoint (which includes cluster list) in a endpoint list.
 *
 * @param[in] ep_list A pointer to endpoint list @ref esp_zb_ep_list_s
 * @param[in] cluster_list  An cluster list which wants to add to endpoint
 * @param[in] endpoint_config  A specific endpoint config @ref esp_zb_endpoint_config_s
 * @anchor esp_zb_ep_list_add_ep
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if endpoint list not initialized
 *
 */
esp_err_t esp_zb_ep_list_add_ep(esp_zb_ep_list_t *ep_list, esp_zb_cluster_list_t *cluster_list, esp_zb_endpoint_config_t endpoint_config);

/**
 * @brief Add a gateway endpoint to the endpoint list.
 * @param[in] ep_list         A pointer to the endpoint list where the @p cluster_list will be added, @ref esp_zb_ep_list_s.
 * @param[in] cluster_list    A pointer to @ref esp_zb_cluster_list_s indicating the gateway's clusters.
 * @param[in] endpoint_config The specific endpoint configuration, @ref esp_zb_endpoint_config_s.
 *
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_NO_MEM if the number of gateways reaches the limit.
 *      - ESP_ERR_INVALID_ARG if the endpoint list is not initialized.
 */
esp_err_t esp_zb_ep_list_add_gateway_ep(esp_zb_ep_list_t *ep_list, esp_zb_cluster_list_t *cluster_list, esp_zb_endpoint_config_t endpoint_config);

/**
 * @brief Get endpoint (cluster list) from a endpoint list.
 *
 * @param[in] ep_list A pointer to endpoint list @ref esp_zb_ep_list_s
 * @param[in] ep_id The endpoint id for cluster list
 *
 * @return
 *      - pointer to  @ref esp_zb_cluster_list_s, if the endpoint is found in the endpoint list
 *      - ``NULL`` if endpoint is not found
 *
 */
esp_zb_cluster_list_t *esp_zb_ep_list_get_ep(const esp_zb_ep_list_t *ep_list, uint8_t ep_id);

#ifdef __cplusplus
}
#endif

/********************* ESP_ZIGBEE_CORE **************************/


#define ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK 0x07FFF800U /*!< channel 11-26 for compatibility with 2.4GHZ*/

#ifdef CONFIG_ZB_ZED
#define ESP_ZB_SLEEP_MAXIMUM_THRESHOLD_MS 86400000U /*! Maximum sleep threshold*/
#endif                                              /** CONFIG_ZB_ZED */



/**
 * @name End Device (ED) timeout request
 * @anchor nwk_requested_timeout
 */
typedef enum {
    ESP_ZB_ED_AGING_TIMEOUT_10SEC    = 0U, /*!< 10 second timeout */
    ESP_ZB_ED_AGING_TIMEOUT_2MIN     = 1U, /*!< 2 minutes */
    ESP_ZB_ED_AGING_TIMEOUT_4MIN     = 2U, /*!< 4 minutes */
    ESP_ZB_ED_AGING_TIMEOUT_8MIN     = 3U, /*!< 8 minutes */
    ESP_ZB_ED_AGING_TIMEOUT_16MIN    = 4U, /*!< 16 minutes */
    ESP_ZB_ED_AGING_TIMEOUT_32MIN    = 5U, /*!< 32 minutes */
    ESP_ZB_ED_AGING_TIMEOUT_64MIN    = 6U, /*!< 64 minutes */
    ESP_ZB_ED_AGING_TIMEOUT_128MIN   = 7U, /*!< 128 minutes */
    ESP_ZB_ED_AGING_TIMEOUT_256MIN   = 8U, /*!< 256 minutes */
    ESP_ZB_ED_AGING_TIMEOUT_512MIN   = 9U, /*!< 512 minutes */
    ESP_ZB_ED_AGING_TIMEOUT_1024MIN  = 10U, /*!< 1024 minutes */
    ESP_ZB_ED_AGING_TIMEOUT_2048MIN  = 11U, /*!< 2048 minutes */
    ESP_ZB_ED_AGING_TIMEOUT_4096MIN  = 12U, /*!< 4096 minutes */
    ESP_ZB_ED_AGING_TIMEOUT_8192MIN  = 13U, /*!< 8192 minutes */
    ESP_ZB_ED_AGING_TIMEOUT_16384MIN = 14U, /*!< 16384 minutes */
} esp_zb_aging_timeout_t;



/**
 * @brief Enum of the Zigbee core action callback id
 *
 * @note
 *    1. If one endpoint possesses the same custom cluster identifier in both client and server roles,
 *       any request or response command for the custom cluster will only trigger the
 *       ``ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID`` callback.
 *    2. The callback ids without ``CMD`` in their names would provide messages of the following structure:
 *       @code{c}
 *       typedef struct xxx_message_s {
 *           esp_zb_device_cb_common_info_t info;
 *           ...
 *       } xxx_message_t;
 *       @endcode
 *       While the callback ids with ``CMD`` in their names would provide messages of the following structure:
 *       @code{c}
 *       typedef struct xxx_message_s {
 *           esp_zb_zcl_cmd_info_t info;
 *           ...
 *       } xxx_message_t;
 *       @endcode
 *
 */
typedef enum esp_zb_core_action_callback_id_s {
    ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID                    = 0x0000,   /*!< Set attribute value, refer to esp_zb_zcl_set_attr_value_message_t */
    ESP_ZB_CORE_SCENES_STORE_SCENE_CB_ID                = 0x0001,   /*!< Store scene, refer to esp_zb_zcl_store_scene_message_t */
    ESP_ZB_CORE_SCENES_RECALL_SCENE_CB_ID               = 0x0002,   /*!< Recall scene, refer to esp_zb_zcl_recall_scene_message_t */
    ESP_ZB_CORE_IAS_ZONE_ENROLL_RESPONSE_VALUE_CB_ID    = 0x0003,   /*!< IAS Zone enroll response, refer to esp_zb_zcl_ias_zone_enroll_response_message_t */
    ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID                 = 0x0004,   /*!< Upgrade OTA, refer to esp_zb_zcl_ota_upgrade_value_message_t */
    ESP_ZB_CORE_OTA_UPGRADE_SRV_STATUS_CB_ID            = 0x0005,   /*!< OTA Server status, refer to esp_zb_zcl_ota_upgrade_server_status_message_t */
    ESP_ZB_CORE_OTA_UPGRADE_SRV_QUERY_IMAGE_CB_ID       = 0x0006,   /*!< OTA Server query image, refer to esp_zb_zcl_ota_upgrade_server_query_image_message_t */
    ESP_ZB_CORE_THERMOSTAT_VALUE_CB_ID                  = 0x0007,   /*!< Thermostat value, refer to esp_zb_zcl_thermostat_value_message_t */
    ESP_ZB_CORE_METERING_GET_PROFILE_CB_ID              = 0x0008,   /*!< Metering get profile, refer to esp_zb_zcl_metering_get_profile_message_t */
    ESP_ZB_CORE_METERING_GET_PROFILE_RESP_CB_ID         = 0x0009,   /*!< Metering get profile response, refer to esp_zb_zcl_metering_get_profile_resp_message_t */
    ESP_ZB_CORE_METERING_REQ_FAST_POLL_MODE_CB_ID       = 0x000a,   /*!< Metering request fast poll mode, refer to esp_zb_zcl_metering_request_fast_poll_mode_message_t */
    ESP_ZB_CORE_METERING_REQ_FAST_POLL_MODE_RESP_CB_ID  = 0x000b,   /*!< Metering request fast poll mode response, refer to esp_zb_zcl_metering_request_fast_poll_mode_resp_message_t */
    ESP_ZB_CORE_METERING_GET_SNAPSHOT_CB_ID             = 0x000c,   /*!< Metering get snapshot, refer to esp_zb_zcl_metering_get_snapshot_message_t */
    ESP_ZB_CORE_METERING_PUBLISH_SNAPSHOT_CB_ID         = 0x000d,   /*!< Metering publish snapshot, refer to esp_zb_zcl_metering_publish_snapshot_message_t */
    ESP_ZB_CORE_METERING_GET_SAMPLED_DATA_CB_ID         = 0x000e,   /*!< Metering get sampled data, refer to esp_zb_zcl_metering_get_sampled_data_message_t */
    ESP_ZB_CORE_METERING_GET_SAMPLED_DATA_RESP_CB_ID    = 0x000f,   /*!< Metering get sampled data response, refer to esp_zb_zcl_metering_get_sampled_data_resp_message_t */
    ESP_ZB_CORE_DOOR_LOCK_LOCK_DOOR_CB_ID               = 0x0010,   /*!< Lock/unlock door request, refer to esp_zb_zcl_door_lock_lock_door_message_t */
    ESP_ZB_CORE_DOOR_LOCK_LOCK_DOOR_RESP_CB_ID          = 0x0011,   /*!< Lock/unlock door response, refer to esp_zb_zcl_door_lock_lock_door_resp_message_t */
    ESP_ZB_CORE_IDENTIFY_EFFECT_CB_ID                   = 0x0012,   /*!< Identify triggers effect request, refer to esp_zb_zcl_identify_effect_message_t */
    ESP_ZB_CORE_BASIC_RESET_TO_FACTORY_RESET_CB_ID      = 0x0013,   /*!< Reset all clusters of endpoint to factory default, refer to esp_zb_zcl_basic_reset_factory_default_message_t  */
    ESP_ZB_CORE_PRICE_GET_CURRENT_PRICE_CB_ID           = 0x0014,   /*!< Price get current price, refer to esp_zb_zcl_price_get_current_price_message_t */
    ESP_ZB_CORE_PRICE_GET_SCHEDULED_PRICES_CB_ID        = 0x0015,   /*!< Price get scheduled prices, refer to esp_zb_zcl_price_get_scheduled_prices_message_t */
    ESP_ZB_CORE_PRICE_GET_TIER_LABELS_CB_ID             = 0x0016,   /*!< Price get tier labels, refer to esp_zb_zcl_price_get_tier_labels_message_t */
    ESP_ZB_CORE_PRICE_PUBLISH_PRICE_CB_ID               = 0x0017,   /*!< Price publish price, refer to esp_zb_zcl_price_publish_price_message_t */
    ESP_ZB_CORE_PRICE_PUBLISH_TIER_LABELS_CB_ID         = 0x0018,   /*!< Price publish tier labels, refer to esp_zb_zcl_price_publish_tier_labels_message_t */
    ESP_ZB_CORE_PRICE_PRICE_ACK_CB_ID                   = 0x0019,   /*!< Price price acknowledgement, refer to esp_zb_zcl_price_ack_message_t */
    ESP_ZB_CORE_COMM_RESTART_DEVICE_CB_ID               = 0x001a,   /*!< Commissioning restart device, refer to esp_zigbee_zcl_commissioning_restart_device_message_t */
    ESP_ZB_CORE_COMM_OPERATE_STARTUP_PARAMS_CB_ID       = 0x001b,   /*!< Commissioning operate startup parameters, refer to esp_zigbee_zcl_commissioning_operate_startup_parameters_message_t */
    ESP_ZB_CORE_COMM_COMMAND_RESP_CB_ID                 = 0x001c,   /*!< Commissioning command response, refer to esp_zigbee_zcl_commissioning_command_response_message_t */
    ESP_ZB_CORE_IAS_WD_START_WARNING_CB_ID              = 0x001d,   /*!< IAS WD cluster Start Warning command, refer to esp_zb_zcl_ias_wd_start_warning_message_t */
    ESP_ZB_CORE_IAS_WD_SQUAWK_CB_ID                     = 0x001e,   /*!< IAS WD cluster Squawk command, refer to esp_zb_zcl_ias_wd_squawk_message_t */
    ESP_ZB_CORE_IAS_ACE_ARM_CB_ID                       = 0x001f,   /*!< IAS ACE cluster Arm command, refer to esp_zb_zcl_ias_ace_arm_message_t */
    ESP_ZB_CORE_IAS_ACE_BYPASS_CB_ID                    = 0x0020,   /*!< IAS ACE cluster Bypass command, refer to esp_zb_zcl_ias_ace_bypass_message_t */
    ESP_ZB_CORE_IAS_ACE_EMERGENCY_CB_ID                 = 0x0021,   /*!< IAS ACE cluster Emergency command, refer to esp_zb_zcl_ias_ace_emergency_message_t */
    ESP_ZB_CORE_IAS_ACE_FIRE_CB_ID                      = 0x0022,   /*!< IAS ACE cluster Fire command, refer to esp_zb_zcl_ias_ace_fire_message_t */
    ESP_ZB_CORE_IAS_ACE_PANIC_CB_ID                     = 0x0023,   /*!< IAS ACE cluster Panic command, refer to esp_zb_zcl_ias_ace_panic_message_t */
    ESP_ZB_CORE_IAS_ACE_GET_PANEL_STATUS_CB_ID          = 0x0024,   /*!< IAS ACE cluster Get Panel Status command, refer to esp_zb_zcl_ias_ace_get_panel_status_message_t */
    ESP_ZB_CORE_IAS_ACE_GET_BYPASSED_ZONE_LIST_CB_ID    = 0x0025,   /*!< IAS ACE cluster Get Bypass Zone List command, refer to esp_zb_zcl_ias_ace_get_bypassed_zone_list_message_t */
    ESP_ZB_CORE_IAS_ACE_GET_ZONE_STATUS_CB_ID           = 0x0026,   /*!< IAS ACE cluster Get Zone Status command, refer to esp_zb_zcl_ias_ace_get_zone_status_message_t */
    ESP_ZB_CORE_IAS_ACE_ARM_RESP_CB_ID                  = 0x0027,   /*!< IAS ACE cluster Arm command response, refer to esp_zb_zcl_ias_ace_arm_response_message_t */
    ESP_ZB_CORE_IAS_ACE_GET_ZONE_ID_MAP_RESP_CB_ID      = 0x0028,   /*!< IAS ACE cluster Get Zone ID MAP command response, refer to esp_zb_zcl_ias_ace_get_zone_id_map_response_message_t */
    ESP_ZB_CORE_IAS_ACE_GET_ZONE_INFO_RESP_CB_ID        = 0x0029,   /*!< IAS ACE cluster Get Zone Information command response, refer to esp_zb_zcl_ias_ace_get_zone_info_response_message_t */
    ESP_ZB_CORE_IAS_ACE_ZONE_STATUS_CHANGED_CB_ID       = 0x002a,   /*!< IAS ACE cluster Zone Status Changed command, refer to esp_zb_zcl_ias_ace_zone_status_changed_message_t */
    ESP_ZB_CORE_IAS_ACE_PANEL_STATUS_CHANGED_CB_ID      = 0x002b,   /*!< IAS ACE cluster Panel Status Changed command, refer to esp_zb_zcl_ias_ace_panel_status_changed_message_t */
    ESP_ZB_CORE_IAS_ACE_GET_PANEL_STATUS_RESP_CB_ID     = 0x002c,   /*!< IAS ACE cluster Get Panel Status command response, refer to esp_zb_zcl_ias_ace_get_panel_status_response_message_t */
    ESP_ZB_CORE_IAS_ACE_SET_BYPASSED_ZONE_LIST_CB_ID    = 0x002d,   /*!< IAS ACE cluster Set Bypassed Zone List command, refer to esp_zb_zcl_ias_ace_set_bypassed_zone_list_message_t */
    ESP_ZB_CORE_IAS_ACE_BYPASS_RESP_CB_ID               = 0x002e,   /*!< IAS ACE cluster Bypass command response, refer to esp_zb_zcl_ias_ace_bypass_response_message_t */
    ESP_ZB_CORE_IAS_ACE_GET_ZONE_STATUS_RESP_CB_ID      = 0x002f,   /*!< IAS ACE cluster Get Zone Status command response, refer to esp_zb_zcl_ias_ace_get_zone_status_response_message_t */
    ESP_ZB_CORE_WINDOW_COVERING_MOVEMENT_CB_ID          = 0x0030,   /*!< Window covering movement command, refer to esp_zb_zcl_window_covering_movement_message_t */
    ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID      = 0x0031,   /*!< OTA upgrade query image response message, refer to esp_zb_zcl_ota_upgrade_query_image_resp_message_t */
    ESP_ZB_CORE_THERMOSTAT_WEEKLY_SCHEDULE_SET_CB_ID    = 0x0032,   /*!< Thermostat weekly schedule stable set, refer to esp_zb_zcl_thermostat_weekly_schedule_set_message_t */
    ESP_ZB_CORE_DRLC_LOAD_CONTORL_EVENT_CB_ID           = 0x0040,   /*!< Demand response and load control cluster LoadControlEvent message, refer to esp_zb_zcl_drlc_load_control_event_message_t */
    ESP_ZB_CORE_DRLC_CANCEL_LOAD_CONTROL_EVENT_CB_ID    = 0x0041,   /*!< Demand response and load control cluster CancelLoadControlEvent message, refer to esp_zb_zcl_drlc_cancel_load_control_event_message_t */
    ESP_ZB_CORE_DRLC_CANCEL_ALL_LOAD_CONTROL_EVENTS_CB_ID = 0x0042, /*!< Demand response and load control cluster CancelAllLoadControlEvents message, refer to esp_zb_zcl_drlc_cancel_all_load_control_events_message_t */
    ESP_ZB_CORE_DRLC_REPORT_EVENT_STATUS_CB_ID          = 0x0043,   /*!< Demand response and load control cluster ReportEventStatus message, refer to esp_zb_zcl_drlc_report_event_status_message_t */
    ESP_ZB_CORE_DRLC_GET_SCHEDULED_EVENTS_CB_ID         = 0x0044,   /*!< Demand response and load control cluster GetScheduledEvents message, refer to esp_zb_zcl_drlc_get_scheduled_events_message_t */
    ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID                = 0x1000,   /*!< Read attribute response, refer to esp_zb_zcl_cmd_read_attr_resp_message_t */
    ESP_ZB_CORE_CMD_WRITE_ATTR_RESP_CB_ID               = 0x1001,   /*!< Write attribute response, refer to esp_zb_zcl_cmd_write_attr_resp_message_t */
    ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID            = 0x1002,   /*!< Configure report response, refer to esp_zb_zcl_cmd_config_report_resp_message_t */
    ESP_ZB_CORE_CMD_READ_REPORT_CFG_RESP_CB_ID          = 0x1003,   /*!< Read report configuration response, refer to esp_zb_zcl_cmd_read_report_config_resp_message_t */
    ESP_ZB_CORE_CMD_DISC_ATTR_RESP_CB_ID                = 0x1004,   /*!< Discover attributes response, refer to esp_zb_zcl_cmd_discover_attributes_resp_message_t */
    ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID                  = 0x1005,   /*!< Default response, refer to esp_zb_zcl_cmd_default_resp_message_t */
    ESP_ZB_CORE_CMD_OPERATE_GROUP_RESP_CB_ID            = 0x1010,   /*!< Group add group response, refer to esp_zb_zcl_groups_operate_group_resp_message_t */
    ESP_ZB_CORE_CMD_VIEW_GROUP_RESP_CB_ID               = 0x1011,   /*!< Group view response, refer to esp_zb_zcl_groups_view_group_resp_message_t */
    ESP_ZB_CORE_CMD_GET_GROUP_MEMBERSHIP_RESP_CB_ID     = 0x1012,   /*!< Group get membership response, refer to esp_zb_zcl_groups_get_group_membership_resp_message_t */
    ESP_ZB_CORE_CMD_OPERATE_SCENE_RESP_CB_ID            = 0x1020,   /*!< Scenes operate response, refer to esp_zb_zcl_scenes_operate_scene_resp_message_t */
    ESP_ZB_CORE_CMD_VIEW_SCENE_RESP_CB_ID               = 0x1021,   /*!< Scenes view response, refer to esp_zb_zcl_scenes_view_scene_resp_message_t */
    ESP_ZB_CORE_CMD_GET_SCENE_MEMBERSHIP_RESP_CB_ID     = 0x1022,   /*!< Scenes get membership response, refer to esp_zb_zcl_scenes_get_scene_membership_resp_message_t */
    ESP_ZB_CORE_CMD_IAS_ZONE_ZONE_ENROLL_REQUEST_ID     = 0x1030,   /*!< IAS Zone enroll request, refer to esp_zb_zcl_ias_zone_enroll_request_message_t */
    ESP_ZB_CORE_CMD_IAS_ZONE_ZONE_STATUS_CHANGE_NOT_ID  = 0x1031,   /*!< IAS Zone status change notification, refer to esp_zb_zcl_ias_zone_status_change_notification_message_t */
    ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID            = 0x1040,   /*!< Custom Cluster request, refer to esp_zb_zcl_custom_cluster_command_message_t */
    ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_RESP_CB_ID           = 0x1041,   /*!< Custom Cluster response, refer to esp_zb_zcl_custom_cluster_command_message_t */
    ESP_ZB_CORE_CMD_PRIVILEGE_COMMAND_REQ_CB_ID         = 0x1050,   /*!< Custom Cluster request, refer to esp_zb_zcl_privilege_command_message_t */
    ESP_ZB_CORE_CMD_PRIVILEGE_COMMAND_RESP_CB_ID        = 0x1051,   /*!< Custom Cluster response, refer to esp_zb_zcl_privilege_command_message_t */
    ESP_ZB_CORE_CMD_TOUCHLINK_GET_GROUP_ID_RESP_CB_ID   = 0x1060,   /*!< Touchlink commissioning cluster get group id response, refer to esp_zb_touchlink_get_group_identifiers_resp_message_t */
    ESP_ZB_CORE_CMD_TOUCHLINK_GET_ENDPOINT_LIST_RESP_CB_ID = 0x1061,   /*!< Touchlink commissioning cluster get endpoint list response, refer to esp_zb_zcl_touchlink_get_endpoint_list_resp_message_t */
    ESP_ZB_CORE_CMD_THERMOSTAT_GET_WEEKLY_SCHEDULE_RESP_CB_ID = 0x1070, /*!< Thermostat cluster get weekly schedule response, refer to esp_zb_zcl_thermostat_get_weekly_schedule_resp_message_t */
    ESP_ZB_CORE_CMD_GREEN_POWER_RECV_CB_ID                 = 0x1F00,   /*!< Green power cluster command receiving, refer to esp_zb_zcl_cmd_green_power_recv_message_t */
    ESP_ZB_CORE_REPORT_ATTR_CB_ID                          = 0x2000,   /*!< Attribute Report, refer to esp_zb_zcl_report_attr_message_t */
} esp_zb_core_action_callback_id_t;


/**
 * @brief The Zigbee End device configuration.
 *
 */
typedef struct {
    uint8_t ed_timeout; /*!< Set End Device Timeout, refer to esp_zb_aging_timeout_t */
    uint32_t keep_alive; /*!< Set Keep alive Timeout, in milliseconds, with a maximum value of 65,000,000,000.*/
} esp_zb_zed_cfg_t;

/**
 * @brief The Zigbee device configuration.
 * @note  For esp_zb_role please refer defined by esp_zb_nwk_device_type_t.
 */
typedef struct esp_zb_cfg_s {
    esp_zb_nwk_device_type_t esp_zb_role; /*!< The nwk device type */
    bool install_code_policy;             /*!< Allow install code security policy or not */
    union {
        esp_zb_zczr_cfg_t zczr_cfg; /*!< The Zigbee zc/zr device configuration */
        esp_zb_zed_cfg_t zed_cfg;   /*!< The Zigbee zed device configuration */
    } nwk_cfg;                      /*!< Union of the network configuration */
} esp_zb_cfg_t;

/**
 * @brief  Zigbee stack initialization.
 *
 * @note To be called inside the application's main cycle at start.
 * @note Default is no NVRAM erase from start up, user could call factory reset for erase NVRAM and other action please refer esp_zb_factory_reset().
 * @note Make sure to use correct corresponding nwk_cfg with your device type @ref esp_zb_cfg_s.
 * @anchor esp_zb_init
 * @param[in] nwk_cfg   The pointer to the initialization Zigbee configuration
 *
 */
void esp_zb_init(esp_zb_cfg_t *nwk_cfg);

/**
 * @brief Set the BDB primary channel mask.
 *
 * Beacon request will be sent on these channels for the BDB energy scan.
 *
 * @note  This function should be run AFTER @ref esp_zb_init is called and before @ref esp_zb_start. These masks define the allowable channels on which the device may attempt to
 * form or join a network at startup time. If function is not called, by default it will scan all channels or read from `zb_fct` NVRAM zone if available. Please refer to tools/mfg_tool.
 * @param[in] channel_mask  Valid channel mask is from 0x00000800 (only channel 11) to 0x07FFF800 (all channels from 11 to 26)
 * @return  - ESP_OK on success
            - ESP_ERR_INVALID_ARG if the channel mask is out of range
 */
esp_err_t esp_zb_set_primary_network_channel_set(uint32_t channel_mask);

/**
 * @brief Get the BDB primary channel mask
 *
 * @return A 32-bit channel mask
 */
uint32_t esp_zb_get_primary_network_channel_set(void);

/**
 * @brief   Set the BDB secondary channel mask.
 *
 * Beacon request will be sent on these channels for the BDB energy scan, if no network found after energy scan on the primary channels.
 *
 * @param[in] channel_mask Valid channel mask is from 0x00000800 (only channel 11) to 0x07FFF800 (all channels from 11 to 26)
 * @return  - ESP_OK on success
            - ESP_ERR_INVALID_ARG if the channel mask is out of range
 */
esp_err_t esp_zb_set_secondary_network_channel_set(uint32_t channel_mask);

/**
 * @brief Get the BDB secondary channel mask
 *
 * @return A 32-bit channel mask
 */
uint32_t esp_zb_get_secondary_network_channel_set(void);

/**
 * @brief   Set the 2.4G channel mask.
 *
 * @param[in] channel_mask Valid channel mask is from 0x00000800 (only channel 11) to 0x07FFF800 (all channels from 11 to 26)
 * @return  - ESP_OK on success
            - ESP_ERR_INVALID_ARG if the channel mask is out of range
 */
esp_err_t esp_zb_set_channel_mask(uint32_t channel_mask);

/**
 * @brief Get the 2.4 channel mask
 *
 * @return A 32-bit channel mask
 */
uint32_t esp_zb_get_channel_mask(void);

/**
 * @brief   Check if device is factory new.
 *
 * @return - True factory new.
 *
 */
bool esp_zb_bdb_is_factory_new(void);

/**
 * @brief Get the scan duration time
 *
 * @return Scan duration is in beacon intervals (15.36 ms)
 */
uint8_t esp_zb_bdb_get_scan_duration(void);

/**
 * @brief Set the scan duration time
 *
 * @param[in] duration  The scan duration time is in beacon intervals, defined as ((1 << duration) + 1) * 15.36 ms
 */
void esp_zb_bdb_set_scan_duration(uint8_t duration);

/**
 * @brief Open Zigbee network
 *
 * @param[in] permit_duration Zigbee network open time
 * @return
 *      - ESP_OK: on success
 *      - ESP_ERR_NO_MEM: not memory
 *      - ESP_FAILED: on failed
 */
esp_err_t esp_zb_bdb_open_network(uint8_t permit_duration);

/**
 * @brief Close Zigbee network
 *
 * @return
 *      - ESP_OK: on success
 *      - ESP_FAIL: on failure
 */
esp_err_t esp_zb_bdb_close_network(void);

/**
 * @brief Check if device has joined network or not
 *
 * @return
 *      - true: device is joined
 *      - false: device is not joined
 */
bool esp_zb_bdb_dev_joined(void);

/**
 * @brief   Set the tx power.
 * @param[in]  power 8-bit of power value in dB, ranging from IEEE802154_TXPOWER_VALUE_MIN to IEEE802154_TXPOWER_VALUE_MAX
 */
void esp_zb_set_tx_power(int8_t power);

/**
 * @brief   Get the tx power.
 * @param[in]  power 8-bit of power pointer value in dB
 */
void esp_zb_get_tx_power(int8_t *power);

/**
 * @brief  Start top level commissioning procedure with specified mode mask.
 *
 * @param[in] mode_mask commissioning modes refer to esp_zb_bdb_commissioning_mode
 *
 * @return - ESP_OK on success
 *
 */
esp_err_t esp_zb_bdb_start_top_level_commissioning(uint8_t mode_mask);

/**
 * @brief Perform `local reset` procedure
 * @note This only takes effect when the device is on a network. The device will leave the current network and
 *       clear all Zigbee persistent data, except the outgoing NWK frame counter. It will be in nearly the same
 *       state as when it left the factory. A `ZB_ZDO_SIGNAL_LEAVE` signal with `ESP_ZB_NWK_LEAVE_TYPE_RESET`
 *       will be triggered to indicate the end of the procedure.
 */
void esp_zb_bdb_reset_via_local_action(void);

/**
 *  @brief Perform "factory reset" procedure
 *  @note The device will completely erase the `zb_storage` partition and then restart
 */
void esp_zb_factory_reset(void);

/**
 * @brief Reset all endpoints to factory default
 *
 * @note If @p cb is not set or @p cb return NULL, the default attribute value will be set to zero
 * @param[in] reset_report Whether reset report of clusters or not
 * @param[in] cb The user can utilize the callback to set default attribute value
 *
 * @return
 *      - ESP_OK: on success
 *      - ESP_FAIL: on failed
 */
esp_err_t esp_zb_zcl_reset_all_endpoints_to_factory_default(bool reset_report, esp_zb_zcl_reset_default_attr_callback_t cb);

/**
 * @brief Reset endpoint to factory default
 *
 * @note If @p cb is not set or @p cb return NULL, the default attribute value will be set to zero
 * @param[in] endpoint      The endpoint identifier which will be reset
 * @param[in] reset_report  Whether reset report of clusters or not
 * @param[in] cb The user can utilize the callback to set default attribute value
 *
 * @return
 *      - ESP_OK: on success
 *      - ESP_FAIL: on failed
 */
esp_err_t esp_zb_zcl_reset_endpoint_to_factory_default(uint8_t endpoint, bool reset_report, esp_zb_zcl_reset_default_attr_callback_t cb);

/**
 * @brief Reset the NVRAM and ZCL data to factory default
 *
 */
void esp_zb_zcl_reset_nvram_to_factory_default(void);

/**
 * @brief   Start Zigbee function.
 *
 * @param[in] autostart - true    autostart mode
 *                      - false   no-autostart mode
 *
 * @note Autostart mode: It initializes, load some parameters from NVRAM and proceed with startup.
 * Startup means either Formation (for ZC), rejoin or discovery/association join.  After startup
 * complete, @ref esp_zb_app_signal_handler is called, so application will know when to do
 * some useful things.
 *
 * @note No-autostart mode: It initializes scheduler and buffers pool, but not MAC and upper layers.
 * Notifies the application that Zigbee framework (scheduler, buffer pool, etc.) has started, but no
 * join/rejoin/formation/BDB initialization has been done yet.
 * Typically esp_zb_start with no_autostart mode is used when application wants to do something before
 * starting joining the network.
 *
 * For example, you can use this function if it is needed to enable leds, timers
 * or any other devices on periphery to work with them before starting working in a network. It's
 * also useful if you want to run something locally during joining.
 *
 * @note Precondition: stack must be initialized by @ref esp_zb_init call. @ref esp_zb_init sets default IB
 * parameters, so caller has a chance to change some of them. Note that NVRAM and product config will be
 * loaded after esp_zb_start() call.
 *
 * @note Zigbee stack is not looped in this routine. Instead, it schedules callback and returns.  Caller
 * must run  esp_zb_stack_main_loop() after this routine.
 *
 * @note Application should later call Zigbee commissioning initiation - for instance,
 * esp_zb_bdb_start_top_level_commissioning().
 * @anchor esp_zb_start
 * @return - ESP_OK on success
 *
 */
esp_err_t esp_zb_start(bool autostart);













/***************************************************************************************/
*Alternative and more comprehensive handler*
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, 
                       "Received message: error status(%d)", message->info.status);
    
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", 
             message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);

    if (message->info.dst_endpoint == SCD30_ZB_ENDPOINT) {
        switch (message->info.cluster) {
        case ESP_ZB_ZCL_CLUSTER_ID_BASIC:
            switch (message->attribute.id) {
                case ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID:
                    ESP_LOGI(TAG, "Basic ZCL Version: %d", message->attribute.data.value ? *(uint8_t*)message->attribute.data.value : 0);
                    break;
                case ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID:
                    ESP_LOGI(TAG, "Power Source: %d", message->attribute.data.value ? *(uint8_t*)message->attribute.data.value : 0);
                    break;
                case ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID:
                    ESP_LOGI(TAG, "Model Identifier updated");
                    break;
                case ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID:
                    ESP_LOGI(TAG, "Manufacturer Name updated");
                    break;
                default:
                    ESP_LOGI(TAG, "Unhandled Basic cluster attribute: 0x%x", message->attribute.id);
                    break;
            }
            break;
            
        case ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG:
            switch (message->attribute.id) {
                case ESP_ZB_ZCL_ATTR_POWER_CONFIG_MAINS_VOLTAGE_ID:
                    ESP_LOGI(TAG, "Mains Voltage: %d", message->attribute.data.value ? *(uint16_t*)message->attribute.data.value : 0);
                    break;
                case ESP_ZB_ZCL_ATTR_POWER_CONFIG_MAINS_FREQUENCY_ID:
                    ESP_LOGI(TAG, "Mains Frequency: %d", message->attribute.data.value ? *(uint8_t*)message->attribute.data.value : 0);
                    break;
                default:
                    ESP_LOGI(TAG, "Unhandled Power Config cluster attribute: 0x%x", message->attribute.id);
                    break;
            }
            break;
            
        case ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT:
            switch (message->attribute.id) {
                case ESP_ZB_ZCL_ATTR_CO2_MEASUREMENT_VALUE_ID:
                    ESP_LOGI(TAG, "CO2 Measurement: %d ppm", 
                            message->attribute.data.value ? *(uint16_t*)message->attribute.data.value : 0);
                    break;
                case ESP_ZB_ZCL_ATTR_CO2_MEASUREMENT_MIN_VALUE_ID:
                    ESP_LOGI(TAG, "CO2 Min Value: %d ppm", 
                            message->attribute.data.value ? *(uint16_t*)message->attribute.data.value : 0);
                    break;
                case ESP_ZB_ZCL_ATTR_CO2_MEASUREMENT_MAX_VALUE_ID:
                    ESP_LOGI(TAG, "CO2 Max Value: %d ppm", 
                            message->attribute.data.value ? *(uint16_t*)message->attribute.data.value : 0);
                    break;
                default:
                    ESP_LOGI(TAG, "Unhandled CO2 measurement attribute: 0x%x", message->attribute.id);
                    break;
            }
            break;
            
        case ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT:
            switch (message->attribute.id) {
                case ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID:
                    ESP_LOGI(TAG, "Temperature: %.2f°C", 
                            message->attribute.data.value ? *(int16_t*)message->attribute.data.value / 100.0f : 0);
                    break;
                case ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID:
                    ESP_LOGI(TAG, "Min Temperature: %.2f°C", 
                            message->attribute.data.value ? *(int16_t*)message->attribute.data.value / 100.0f : 0);
                    break;
                case ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID:
                    ESP_LOGI(TAG, "Max Temperature: %.2f°C", 
                            message->attribute.data.value ? *(int16_t*)message->attribute.data.value / 100.0f : 0);
                    break;
                default:
                    ESP_LOGI(TAG, "Unhandled Temperature measurement attribute: 0x%x", message->attribute.id);
                    break;
            }
            break;
            
        case ESP_ZB_ZCL_CLUSTER_ID_HUMIDITY_MEASUREMENT:
            switch (message->attribute.id) {
                case ESP_ZB_ZCL_ATTR_HUMIDITY_MEASUREMENT_VALUE_ID:
                    ESP_LOGI(TAG, "Humidity: %.2f%%", 
                            message->attribute.data.value ? *(uint16_t*)message->attribute.data.value / 100.0f : 0);
                    break;
                case ESP_ZB_ZCL_ATTR_HUMIDITY_MEASUREMENT_MIN_VALUE_ID:
                    ESP_LOGI(TAG, "Min Humidity: %.2f%%", 
                            message->attribute.data.value ? *(uint16_t*)message->attribute.data.value / 100.0f : 0);
                    break;
                case ESP_ZB_ZCL_ATTR_HUMIDITY_MEASUREMENT_MAX_VALUE_ID:
                    ESP_LOGI(TAG, "Max Humidity: %.2f%%", 
                            message->attribute.data.value ? *(uint16_t*)message->attribute.data.value / 100.0f : 0);
                    break;
                default:
                    ESP_LOGI(TAG, "Unhandled Humidity measurement attribute: 0x%x", message->attribute.id);
                    break;
            }
            break;

        default:
            ESP_LOGW(TAG, "Unhandled cluster: 0x%x, attribute: 0x%x", 
                     message->info.cluster, message->attribute.id);
            break;
        }
    }
    
    return ret;
}# scd30_zigbee
