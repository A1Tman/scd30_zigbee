/**
 * @file i2c_handler.h
 * @brief I2C communication handler for ESP32
 * 
 * Handles I2C bus initialization, communication, and cleanup operations
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/* I2C Bus Configuration */
#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0       /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          50000  /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0       /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0       /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       2000

/* SCD30 Specific Configuration*/
#define SCD30_SENSOR_ADDR           0x61    /*!< SCD30 I2C address */
#define SCD30_RESET_BIT             0xD304  /*!< SCD30 reset command */

/* Timing Configuration */
#define SENSOR_READ_INTERVAL_MS     5000    /*!< Sensor reading interval */
#define TIMER_WAKEUP_TIME_US        (55 * 1000 * 1000)  /*!< Timer wakeup time in microseconds */

/**
 * @brief Initialize I2C master interface
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t i2c_handler_init(void);

/**
 * @brief Cleanup I2C resources
 */
void i2c_handler_cleanup(void);

/**
 * @brief Get the I2C device handle
 * @return I2C device handle
 */
i2c_master_dev_handle_t i2c_handler_get_device(void);

/**
 * @brief Check if I2C is initialized
 * @return true if initialized, false otherwise
 */
bool i2c_handler_is_initialized(void);

/**
 * @brief Write data to I2C device
 * @param data Pointer to data buffer to write
 * @param len Length of data to write
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t i2c_handler_write(const uint8_t* data, size_t len);

/**
 * @brief Read data from I2C device
 * @param data Pointer to data buffer for reading
 * @param len Length of data to read
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t i2c_handler_read(uint8_t* data, size_t len);

/**
 * @brief Write then read data from I2C device (combined operation)
 * @param write_data Pointer to data buffer to write
 * @param write_len Length of data to write
 * @param read_data Pointer to data buffer for reading
 * @param read_len Length of data to read
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t i2c_handler_write_read(const uint8_t* write_data, size_t write_len, 
                                uint8_t* read_data, size_t read_len);

/**
 * @brief Add I2C device to the bus
 * @param device_addr Device I2C address
 * @param dev_handle Pointer to store the device handle
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t i2c_handler_add_device(uint8_t device_addr, i2c_master_dev_handle_t* dev_handle);                                

/**
 * @brief Probe the I2C device to check if it's responsive
 * @return ESP_OK if device responds, otherwise error code
 */
esp_err_t i2c_handler_probe_device(uint8_t address);

/**
 * @brief Scan the I2C bus for connected devices and log their addresses
 */
void i2c_scan(void);
