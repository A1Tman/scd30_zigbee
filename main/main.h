/**
 * @file main.h
 * @brief Main header file for ESP32 Zigbee SCD30 sensor application
 * 
 * @copyright 2021-2022 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once
#ifndef MAIN_H
#define MAIN_H

#include "esp_err.h"
#include "stdbool.h"

/**
 * @brief Enumeration used by the Q_sensor project to select which attributes
 *        to update when sending measurements over Zigbee.
 */
typedef enum {
    ATTRIBUTE_ALL,  /*!< Update all available attributes */
    ATTRIBUTE_SCD,  /*!< Update SCD30 related attributes only */
} attribute_t;

/**
 * @brief Initialize non-volatile storage
 * @return ESP_OK if successful, otherwise error code
 */
static esp_err_t init_nvs(void);

/**
 * @brief Initialize sensor system (I2C bus and SCD30 sensor)
 * @return ESP_OK if successful, otherwise error code
 */
static esp_err_t init_sensor_system(void);

// Function declarations
void zigbee_connection_callback(bool connected);

#endif /* MAIN_H */
