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

// Function declarations
void zigbee_connection_callback(bool connected);

#endif /* MAIN_H */
