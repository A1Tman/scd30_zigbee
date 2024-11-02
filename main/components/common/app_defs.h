#pragma once

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_timer.h"

/* System Events */
extern EventGroupHandle_t system_events;

/* System Event Group Bits */
#define ZIGBEE_CONNECTED_BIT       BIT0    /*!< Event bit indicating Zigbee connection status */
#define FACTORY_RESET_REQUESTED_BIT BIT1    /*!< Event bit for factory reset request */
#define REJOIN_REQUESTED_BIT       BIT2    /*!< Event bit for network rejoin request */
#define DIAG_INFO_REQUESTED_BIT    BIT3    /*!< Event bit for diagnostic info request */
#define DEBUG_TOGGLE_REQUESTED_BIT  BIT4    /*!< Event bit for debug mode toggle request */

/* Button Configuration */
#define BOOT_BUTTON_GPIO           GPIO_NUM_9    /*!< ESP32-C6 BOOT button GPIO number */

/* Button Press Patterns */
#define FACTORY_RESET_PRESS_COUNT  4             /*!< Number of presses for factory reset */
#define DIAG_INFO_PRESS_COUNT      2             /*!< Number of presses for diagnostic info */
#define DEBUG_TOGGLE_PRESS_COUNT   8             /*!< Number of presses for debug toggle */

/* Button Timing Configuration */
#define FACTORY_RESET_TIMEOUT_MS   3000          /*!< Time window for multiple presses (3 seconds) */
#define LONG_PRESS_TIME_MS         3000          /*!< Duration for long press detection (3 seconds) */
#define DEBOUNCE_TIME_MS          100             /*!< Button debounce time (100ms) */

/*
 * Task Priority Structure
 * ESP-IDF/FreeRTOS: 0-25 (configMAX_PRIORITIES-1)
 * Higher number = Higher priority
 * Avoid using priorities above 20 (reserved for system tasks)
 */

/* System Level (18-20) - Rarely used, system critical */
#define SYSTEM_CRITICAL_PRIORITY    19  /*!< System critical tasks, use sparingly */

/* Communication Level (13-17) */
#define NETWORK_HIGH_PRIORITY       17  /*!< Critical network response tasks */
#define ZIGBEE_TASK_PRIORITY       15  /*!< Zigbee main task - elevated for reliable comms */
#define NETWORK_LOW_PRIORITY       13  /*!< Non-critical network tasks */

/* Sensor/IO Level (8-12) */
#define SENSOR_HIGH_PRIORITY       12  /*!< Time-critical sensor readings */
#define SENSOR_MEDIUM_PRIORITY     10  /*!< Standard sensor operations */
#define SCD30_TASK_PRIORITY        9  /*!< CO2 sensor readings - not time critical */
#define SENSOR_LOW_PRIORITY         8  /*!< Background sensor tasks */

/* Application Level (4-7) */
#define APP_HIGH_PRIORITY           7  /*!< Important application logic */
#define APP_NORMAL_PRIORITY         5  /*!< Standard application tasks */
#define APP_LOW_PRIORITY           4  /*!< Background application tasks */

/* Background Level (1-3) */
#define BACKGROUND_HIGH_PRIORITY    3  /*!< Higher priority background work */
#define BACKGROUND_NORMAL_PRIORITY  2  /*!< Normal background work */
#define BACKGROUND_LOW_PRIORITY     1  /*!< Lowest priority background work */

/* System Reserved */
#define IDLE_PRIORITY              0  /*!< Reserved for FreeRTOS idle task */

/* SCD30 Configuration */
#define SCD30_INIT_RETRY_COUNT        3    /*!< Number of times to retry SCD30 initialization */
#define SCD30_INIT_RETRY_DELAY_MS  1000    /*!< Delay between initialization retries in ms */