#include "troubleshooting.h"
#include "app_defs.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "scd30_driver.h"
#include "zigbee_handler.h"

static const char *TAG = "TROUBLESHOOTING";

// Static variables for button handling — shared between ISR and task context,
// so all must be volatile to prevent the compiler from caching their values.
static volatile int64_t press_start_time = 0;
static volatile bool button_pressed = false;
static volatile int64_t last_press_time = 0;
// Counts button events that xEventGroupSetBitsFromISR failed to deliver
// (timer service queue full). Logged from task context since the ISR cannot.
static volatile uint32_t dropped_button_events = 0;

// Forward declarations
static void handle_button_events(void);
static void button_events_task(void *pvParameters);

// Button event handler task
static void handle_button_events(void) {
    if (system_events == NULL) {
        ESP_LOGE(TAG, "System events handle is NULL");
        vTaskDelay(pdMS_TO_TICKS(1000));  // Add delay to prevent tight loop
        return;
    }

    static uint32_t reported_dropped_events = 0;
    uint32_t dropped = dropped_button_events;
    if (dropped != reported_dropped_events) {
        ESP_LOGW(TAG, "%u button event(s) were dropped in the ISR (timer queue full)",
                 (unsigned int)(dropped - reported_dropped_events));
        reported_dropped_events = dropped;
    }

    EventBits_t bits = xEventGroupGetBits(system_events);

    if (bits & REJOIN_REQUESTED_BIT) {
        ESP_LOGI(TAG, "Rejoin requested via long press");
        esp_err_t err = zigbee_handler_reconnect();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to schedule Zigbee reconnect: %s", esp_err_to_name(err));
        }
        xEventGroupClearBits(system_events, REJOIN_REQUESTED_BIT);
        vTaskDelay(pdMS_TO_TICKS(100));  // Add small delay after operation
    }

    if (bits & OUTDOOR_RECAL_REQUESTED_BIT) {
        ESP_LOGW(TAG, "Outdoor recalibration requested via very long press");
        ESP_LOGW(TAG, "Use only after the sensor has been in clean outdoor/reference air for at least 2 minutes");

        esp_err_t err = scd30_request_force_recalibration(OUTDOOR_RECAL_TARGET_PPM);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to queue outdoor recalibration to %u ppm: %s",
                     OUTDOOR_RECAL_TARGET_PPM, esp_err_to_name(err));
        }

        xEventGroupClearBits(system_events, OUTDOOR_RECAL_REQUESTED_BIT);
        vTaskDelay(pdMS_TO_TICKS(100));  // Add small delay after operation
    }
}

static void button_events_task(void *pvParameters) {
    while (1) {
        handle_button_events();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void IRAM_ATTR boot_button_isr_handler(void* arg) {
    int64_t current_time = esp_timer_get_time() / 1000;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if ((current_time - last_press_time) > DEBOUNCE_TIME_MS) {
        if (gpio_get_level(BOOT_BUTTON_GPIO) == 0) {  // Button pressed
            press_start_time = current_time;
            button_pressed = true;
        } else {  // Button released
            if (button_pressed) {
                int64_t press_duration = current_time - press_start_time;

                if (press_duration >= OUTDOOR_RECAL_PRESS_TIME_MS) {
                    if (xEventGroupSetBitsFromISR(system_events,
                            OUTDOOR_RECAL_REQUESTED_BIT,
                            &xHigherPriorityTaskWoken) == pdFAIL) {
                        dropped_button_events++;
                    }
                } else if (press_duration >= REJOIN_PRESS_TIME_MS) {
                    if (xEventGroupSetBitsFromISR(system_events,
                            REJOIN_REQUESTED_BIT,
                            &xHigherPriorityTaskWoken) == pdFAIL) {
                        dropped_button_events++;
                    }
                }
            }
            button_pressed = false;
        }
        last_press_time = current_time;
    }

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t troubleshooting_init(void) {
    esp_err_t ret;
    
    // Configure GPIO for BOOT button
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,  // Changed to catch both edges
    };
    
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BOOT button GPIO");
        return ret;
    }

    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service");
        return ret;
    }

    // Add ISR handler
    ret = gpio_isr_handler_add(BOOT_BUTTON_GPIO, boot_button_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BOOT button ISR handler");
        return ret;
    }

    // Create button events task
    BaseType_t xReturned = xTaskCreate(
        button_events_task,
        "button_events",
        8192,
        NULL,
        APP_NORMAL_PRIORITY,
        NULL
    );
    
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button events task");
        return ESP_FAIL;
    }

    return ESP_OK;
}
