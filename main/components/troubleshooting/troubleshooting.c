#include "troubleshooting.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/timers.h"
#include "zigbee_handler.h"

static const char *TAG = "TROUBLESHOOTING";

// Static variables for button handling
static int64_t press_start_time = 0;
static bool button_pressed = false;
static int boot_button_press_count = 0;
static int64_t last_press_time = 0;
static TimerHandle_t reset_timer = NULL;

// Forward declarations
static void reset_press_count(TimerHandle_t timer);
static void handle_button_events(void);
static void button_events_task(void *pvParameters);

// Button event handler task
static void handle_button_events(void) {
    if (system_events == NULL) {
        ESP_LOGE(TAG, "System events handle is NULL");
        vTaskDelay(pdMS_TO_TICKS(1000));  // Add delay to prevent tight loop
        return;
    }

    EventBits_t bits = xEventGroupGetBits(system_events);
    
    if (bits & REJOIN_REQUESTED_BIT) {
        ESP_LOGI(TAG, "Rejoin requested via long press");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        xEventGroupClearBits(system_events, REJOIN_REQUESTED_BIT);
        vTaskDelay(pdMS_TO_TICKS(100));  // Add small delay after operation
    }
    
    if (bits & DIAG_INFO_REQUESTED_BIT) {
        ESP_LOGI(TAG, "=== Diagnostic Information ===");
        ESP_LOGI(TAG, "Channel: %d", esp_zb_get_current_channel());
        ESP_LOGI(TAG, "PAN ID: 0x%04x", esp_zb_get_pan_id());
        ESP_LOGI(TAG, "Short addr: 0x%04x", esp_zb_get_short_address());
        ESP_LOGI(TAG, "Connected: %s", zigbee_handler_is_connected() ? "Yes" : "No");
        xEventGroupClearBits(system_events, DIAG_INFO_REQUESTED_BIT);
        vTaskDelay(pdMS_TO_TICKS(100));  // Add small delay after operation
    }
    
    if (bits & DEBUG_TOGGLE_REQUESTED_BIT) {
        static bool debug_enabled = false;
        debug_enabled = !debug_enabled;
        esp_log_level_set("*", debug_enabled ? ESP_LOG_DEBUG : ESP_LOG_INFO);
        ESP_LOGI(TAG, "Debug logging %s", debug_enabled ? "enabled" : "disabled");
        xEventGroupClearBits(system_events, DEBUG_TOGGLE_REQUESTED_BIT);
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
                
                if (press_duration >= LONG_PRESS_TIME_MS) {
                    xEventGroupSetBitsFromISR(system_events, 
                                            REJOIN_REQUESTED_BIT,
                                            &xHigherPriorityTaskWoken);
                    boot_button_press_count = 0;  // Reset count after long press
                } else {
                    boot_button_press_count++;
                    
                    // Start/restart the timer on first press
                    if (boot_button_press_count == 1) {
                        xTimerStartFromISR(reset_timer, &xHigherPriorityTaskWoken);
                    }
                    
                    ESP_EARLY_LOGI(TAG, "Press count: %d", boot_button_press_count);  // Debug logging

                    if (boot_button_press_count == DIAG_INFO_PRESS_COUNT) {
                        xEventGroupSetBitsFromISR(system_events, 
                                                DIAG_INFO_REQUESTED_BIT,
                                                &xHigherPriorityTaskWoken);
                    } else if (boot_button_press_count == FACTORY_RESET_PRESS_COUNT) {
                        xEventGroupSetBitsFromISR(system_events, 
                                                FACTORY_RESET_REQUESTED_BIT,
                                                &xHigherPriorityTaskWoken);
                    } else if (boot_button_press_count == DEBUG_TOGGLE_PRESS_COUNT) {
                        xEventGroupSetBitsFromISR(system_events, 
                                                DEBUG_TOGGLE_REQUESTED_BIT,
                                                &xHigherPriorityTaskWoken);
                    }

                    // Reset count and stop timer if we've hit any of our target counts
                    if (boot_button_press_count >= FACTORY_RESET_PRESS_COUNT) {
                        boot_button_press_count = 0;
                        xTimerStopFromISR(reset_timer, &xHigherPriorityTaskWoken);
                    }
                }
            }
            button_pressed = false;
        }
        last_press_time = current_time;
    }
}

// Modify the timer callback
static void reset_press_count(TimerHandle_t timer) {
    if (boot_button_press_count > 0 && boot_button_press_count < FACTORY_RESET_PRESS_COUNT) {
        ESP_LOGW(TAG, "Button press sequence timeout, count was: %d", boot_button_press_count);
        boot_button_press_count = 0;
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

    // Create timer for reset window
    reset_timer = xTimerCreate("reset_timer", 
                              pdMS_TO_TICKS(FACTORY_RESET_TIMEOUT_MS),
                              pdFALSE,
                              NULL,
                              reset_press_count);
    if (reset_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create reset timer");
        return ESP_ERR_NO_MEM;
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
        5,
        NULL
    );
    
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button events task");
        return ESP_FAIL;
    }

    return ESP_OK;
}
