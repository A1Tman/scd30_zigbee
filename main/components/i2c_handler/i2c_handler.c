/**
 * @file i2c_handler.c
 * @brief Implementation of I2C communication handler
 */

#include "i2c_handler.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char *TAG = "I2C_HANDLER";

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t i2c_dev_handle = NULL;
static bool is_initialized = false;
static esp_err_t i2c_handler_recover_bus(void);

static esp_err_t i2c_handler_recover_bus(void)
{
    ESP_LOGW(TAG, "Attempting I2C bus recovery...");
    
    // Configure SCL and SDA as GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2C_MASTER_SCL_IO) | (1ULL << I2C_MASTER_SDA_IO),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Toggle SCL 9 times to recover any stuck device
    for(int i = 0; i < 9; i++) {
        gpio_set_level(I2C_MASTER_SCL_IO, 1);
        esp_rom_delay_us(5);
        gpio_set_level(I2C_MASTER_SCL_IO, 0);
        esp_rom_delay_us(5);
    }

    // Generate STOP condition
    gpio_set_level(I2C_MASTER_SDA_IO, 0);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_MASTER_SDA_IO, 1);
    esp_rom_delay_us(5);

    // Reset pins back to I2C
    gpio_reset_pin(I2C_MASTER_SCL_IO);
    gpio_reset_pin(I2C_MASTER_SDA_IO);

    ESP_LOGI(TAG, "I2C bus recovery completed");
    return ESP_OK;
}

esp_err_t i2c_handler_init(void)
{
    ESP_LOGI(TAG, "Starting I2C initialization...");
    ESP_LOGI(TAG, "SDA Pin: %d, SCL Pin: %d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    // Attempting to recover the bus in case it's stuck
    i2c_handler_recover_bus();

    esp_err_t ret;
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 1,
        .flags.enable_internal_pullup = true,
    };

    // Initializing the I2C bus
    ret = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C bus initialized successfully");

    // Configuring the SCD30 device
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SCD30_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    // Adding the SCD30 device to the bus
    ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SCD30 device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(i2c_bus_handle);
        i2c_bus_handle = NULL;
        return ret;
    }
    ESP_LOGI(TAG, "SCD30 device added to I2C bus");
  
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set the initialization flag before probing so the probe can run
    is_initialized = true;

    // Attempting to probe the SCD30 device
    ret = i2c_handler_probe_device(SCD30_SENSOR_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Device probe failed for address 0x%02x, but continuing anyway", SCD30_SENSOR_ADDR);
    }

    return ESP_OK;
}

esp_err_t i2c_handler_add_device(uint8_t device_addr, i2c_master_dev_handle_t* dev_handle)
{
    if (!is_initialized) {
        ESP_LOGE(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device (0x%x): %s", device_addr, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Successfully added device with address 0x%x", device_addr);
    return ESP_OK;
}

void i2c_handler_cleanup(void)
{
    if (i2c_dev_handle != NULL) {
        i2c_master_bus_rm_device(i2c_dev_handle);
        i2c_dev_handle = NULL;
    }
    if (i2c_bus_handle != NULL) {
        i2c_del_master_bus(i2c_bus_handle);
        i2c_bus_handle = NULL;
    }
    is_initialized = false;
    ESP_LOGI(TAG, "I2C resources cleaned up");
}

i2c_master_dev_handle_t i2c_handler_get_device(void)
{
    return i2c_dev_handle;
}

bool i2c_handler_is_initialized(void)
{
    return is_initialized;
}

esp_err_t i2c_handler_write(const uint8_t* data, size_t len)
{
    if (!is_initialized || !i2c_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = i2c_master_transmit(i2c_dev_handle, data, len, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t i2c_handler_read(uint8_t* data, size_t len)
{
    if (!is_initialized || !i2c_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = i2c_master_receive(i2c_dev_handle, data, len, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t i2c_handler_write_read(const uint8_t* write_data, size_t write_len, uint8_t* read_data, size_t read_len)
{
    if (!is_initialized || !i2c_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = i2c_master_transmit_receive(i2c_dev_handle, 
                                              write_data, write_len,
                                              read_data, read_len,
                                              I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write-read failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t i2c_handler_probe_device(uint8_t address)
{
    if (!is_initialized || !i2c_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t probe_cmd[] = {0x02, 0x02};  // Get data ready command
    uint8_t probe_resp[3];

    ESP_LOGI(TAG, "Probing device at address 0x%02x...", address);
    
    esp_err_t ret = i2c_master_transmit(i2c_dev_handle, probe_cmd, sizeof(probe_cmd), I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Probe transmit failed for address 0x%02x: %s", address, esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(5));  // Wait a bit before reading

    ret = i2c_master_receive(i2c_dev_handle, probe_resp, sizeof(probe_resp), I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Probe receive failed for address 0x%02x: %s", address, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Probe successful for address 0x%02x", address);
    return ESP_OK;
}

void i2c_scan() {
    // Use the defined address for SCD30 from the header
    uint8_t address = SCD30_SENSOR_ADDR; 
    esp_err_t ret = i2c_handler_probe_device(address);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "I2C device found at address 0x%02x", address);
    } else {
        ESP_LOGW(TAG, "No I2C device found at address 0x%02x", address);
    }
}
