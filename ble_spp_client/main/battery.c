#include "battery.h"
#include "hw_config.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BATTERY";
static bool is_charging = false;


#define BATTERY_VOLTAGE_DIVIDER_RATIO 2.0f  // Two 100kΩ resistors in series

esp_err_t battery_init(void) {
    esp_err_t ret;

    // Configure GPIO for charging status
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BATTERY_CHARGING_STATUS_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return ret;
    }   
    return ESP_OK;
}

static void battery_monitor_task(void *pvParameters) {
    int last_state = -1;
    int stable_count = 0;
    const int DEBOUNCE_COUNT = 3;
    
    while (1) {
        int current_state = gpio_get_level(BATTERY_CHARGING_STATUS_PIN);
        
        if (current_state == last_state) {
            stable_count++;
            if (stable_count >= DEBOUNCE_COUNT && is_charging != (current_state == 0)) {
                is_charging = (current_state == 0);
                ESP_LOGI(TAG, "Battery status: %s",
                         is_charging ? "Charging" : "Not Charging");
            }
        } else {
            stable_count = 0;
            last_state = current_state;
        }

        // Update voltage reading when charging
        if (is_charging) {
            ESP_LOGI(TAG, "Battery status: %s",
                     is_charging ? "Charging" : "Not Charging");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void battery_start_monitoring(void) {
    xTaskCreate(battery_monitor_task, "batt_monitor", 2048, NULL, 5, NULL);
}

bool battery_is_charging(void) {
    return is_charging;
}
