#pragma once

#include "esp_err.h"
#include "app_defs.h"

/**
 * @brief Initialize factory reset and troubleshooting buttons
 * 
 * @return esp_err_t ESP_OK if successful
 */
esp_err_t troubleshooting_init(void);