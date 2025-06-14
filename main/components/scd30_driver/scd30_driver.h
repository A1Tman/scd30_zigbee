/**
 * @file scd30_driver.h
 * @brief Driver for Sensirion SCD30 CO2, Temperature, and Humidity sensor
 */

#pragma once
#ifndef SCD30_DRIVER_H
#define SCD30_DRIVER_H

#include <stdbool.h>
#include "esp_err.h"
#include "zigbee_handler.h"

/* SCD30 I2C Configuration */
#define SCD30_SENSOR_ADDR           0x61    /*!< SCD30 I2C address */
#define SCD30_INIT_RETRY_COUNT      3
#define SCD30_INIT_RETRY_DELAY_MS   1000

/* SCD30 Data Quality Configuration */
#define SCD30_HISTORY_SIZE          3       /*!< Number of readings to average */
#define SCD30_TEMP_DELTA_THRESHOLD  5.0f    /*!< Maximum temperature change between readings (°C) */
#define SCD30_CO2_DELTA_THRESHOLD   500.0f  /*!< Maximum CO2 change between readings (ppm) */

/* SCD30 Error Recovery Configuration */
#define SCD30_MAX_CONSECUTIVE_ERRORS 5      /*!< Maximum consecutive errors before reset */
#define SCD30_RECOVERY_DELAY_MS     2000    /*!< Delay after reset in ms */

/* SCD30 Commands */
#define SCD30_CMD_START_MEASURE     0x0010  /*!< Start continuous measurement */
#define SCD30_CMD_STOP_MEASURE      0x0104  /*!< Stop measurements */
#define SCD30_CMD_SET_INTERVAL      0x4600  /*!< Set measurement interval */
#define SCD30_CMD_GET_DATA_READY    0x0202  /*!< Get data ready status */
#define SCD30_CMD_READ_MEASUREMENT  0x0300  /*!< Read measurement values */
#define SCD30_CMD_SET_PRESSURE      0x5110  /*!< Set ambient pressure compensation */
#define SCD30_CMD_ALTI_COMP        0x5102  /*!< Set altitude compensation */
#define SCD30_CMD_TEMP_OFFSET      0x5403  /*!< Set/Get temperature offset */
#define SCD30_CMD_RESET            0xD304  /*!< Soft reset */
#define SCD30_CMD_AUTO_SELF_CALIBRATION 0x5306 /*!< (De)Activate self-calibration (ASC), Format: uint16 “1”: Activate ASC, “0”: Deactivate continuous ASC*/

/* Configuration Values */
#define SCD30_MEASUREMENT_INTERVAL  5       /*!< Measurement interval in seconds */
#define SCD30_AMBIENT_PRESSURE     1013    /*!< Default ambient pressure in mbar */
#define SCD30_DEFAULT_ALTITUDE     500     /*!< Default altitude in meters */
#define SCD30_HW_TEMP_OFFSET      2.5f    /*!< Hardware temperature offset in °C */
#define SCD30_SW_TEMP_OFFSET      4.0f   /*!< Software temperature offset in °C */

// Measurement ranges
#define SCD30_CO2_MIN 400
#define SCD30_CO2_MAX 10000
#define SCD30_TEMP_MIN -40
#define SCD30_TEMP_MAX 125
#define SCD30_HUM_MIN 0
#define SCD30_HUM_MAX 100

/**
 * @brief Measurement data structure
 */
typedef struct {
    float co2_ppm;        /*!< CO2 concentration in ppm */
    float temperature;    /*!< Temperature in degrees Celsius */
    float humidity;       /*!< Relative humidity in percent */
} scd30_measurement_t;

/**
 * @brief Initialize the SCD30 sensor
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t scd30_init(void);

/**
 * @brief Start continuous measurements
 * @param ambient_pressure_mbar Ambient pressure in mbar (0 = disabled)
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t scd30_start_continuous_measurement(uint16_t ambient_pressure_mbar);

/**
 * @brief Stop continuous measurements
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t scd30_stop_measurement(void);

/**
 * @brief Set measurement interval
 * @param interval_sec Interval in seconds (2-1800)
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t scd30_set_measurement_interval(uint16_t interval_sec);

/**
 * @brief Check if new measurement data is available
 * @param data_ready Pointer to store result
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t scd30_get_data_ready_status(bool *data_ready);

/**
 * @brief Read measurement data from sensor
 * @param measurement Pointer to measurement structure
 * @return ESP_OK if successful, otherwise error code
 */
/**
 * @brief Read measurement data from sensor
 *
 * This function can optionally skip the internal check for the sensor's
 * data-ready status when the caller has already verified that new data is
 * available.
 *
 * @param measurement       Pointer to measurement structure to fill
 * @param skip_ready_check  If true, do not query the sensor for the data
 *                          ready status before reading the measurement.
 *                          When false, the function will perform the check
 *                          internally.
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t scd30_read_measurement(scd30_measurement_t *measurement,
                                 bool skip_ready_check);

/**
 * @brief Soft reset the sensor
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t scd30_reset(void);

/**
 * @brief Start the SCD30 measurement task
 * @param task_priority Priority for the measurement task
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t scd30_start_task(uint8_t task_priority);

/**
 * @brief Stop the SCD30 measurement task
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t scd30_stop_task(void);

/**
 * @brief Validates the sensor readings for CO2, temperature, and humidity.
 *
 * This function checks whether the readings obtained from the SCD30 sensor
 * are within valid ranges. The ranges are defined as follows:
 * - CO2: 0 - 10000 ppm
 * - Temperature: -40°C to 125°C
 * - Humidity: 0% to 100%
 *
 * @param co2_ppm The CO2 concentration in parts per million (ppm).
 * @param temperature The temperature in degrees Celsius.
 * @param humidity The relative humidity in percentage.
 * @return 
 * - ESP_OK: If all readings are within valid ranges.
 * - ESP_ERR_INVALID_RESPONSE: If any of the readings fall outside of the valid range.
 */
esp_err_t validate_scd30_readings(float co2_ppm, float temperature, float humidity);

/**
 * @brief Set temperature offset for compensation
 * @param offset_celsius Temperature offset in degrees Celsius (0.01°C resolution)
 * @return ESP_OK if successful, otherwise error code
 */
esp_err_t scd30_set_temperature_offset(float offset_celsius);

esp_err_t scd30_get_temperature_offset(float *offset_celsius);

esp_err_t scd30_set_altitude_compensation(uint16_t altitude_meters);

esp_err_t scd30_set_pressure_compensation(uint16_t pressure_mbar);

#endif /* SCD30_DRIVER_H */
