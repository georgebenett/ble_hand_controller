#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "ble_spp_client.h"
#include "adc.h"
#include "lcd.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#define TAG "MAIN"
#define SLEEP_PIN GPIO_NUM_4
#define SLEEP_TIMEOUT_MS 2000
#define INACTIVITY_TIMEOUT_MS 50000  // 50 seconds inactivity timeout

extern bool is_connect;

static lv_obj_t *adc_label = NULL;
static char adc_str[32];

static void update_display_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        float voltage = get_latest_voltage();
        int32_t rpm = get_latest_rpm();

        snprintf(adc_str, sizeof(adc_str), "%.2fv\n%ld", voltage, rpm);

        if (adc_label != NULL) {
            lv_label_set_text(adc_label, adc_str);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
    }
}

static void init_sleep_gpio(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SLEEP_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Enable wake up from GPIO (active low)
    ESP_ERROR_CHECK(gpio_wakeup_enable(SLEEP_PIN, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
}

static void check_sleep_conditions(void *pvParameters)
{
    TickType_t pin_low_time = 0;
    bool pin_was_low = false;

    while (1) {
        int pin_level = gpio_get_level(SLEEP_PIN);

        if (pin_level == 0) {  // Button is pressed (active low)
            if (!pin_was_low) {
                pin_was_low = true;
                pin_low_time = xTaskGetTickCount();
            } else {
                // Calculate how long the button has been held
                uint32_t elapsed_ms = (xTaskGetTickCount() - pin_low_time) * portTICK_PERIOD_MS;

                // Show progress only while holding (up to SLEEP_TIMEOUT_MS)
                if (elapsed_ms < SLEEP_TIMEOUT_MS) {
                    uint8_t progress = (elapsed_ms * 100) / SLEEP_TIMEOUT_MS;
                    lcd_show_loading_bar(progress);
                }

                // Check if button has been held for timeout period
                if (elapsed_ms >= SLEEP_TIMEOUT_MS) {
                    ESP_LOGI(TAG, "Entering light sleep mode");
                    lcd_show_loading_bar(100);  // Show full progress

                    // Wait for button release before sleeping
                    while (gpio_get_level(SLEEP_PIN) == 0) {
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }

                    lcd_hide_loading_bar();  // Hide the loading bar before sleep
                    lcd_reset_loading_bar();  // Reset the pointer

                    // Configure wakeup on button press (active low)
                    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(1ULL << SLEEP_PIN, ESP_GPIO_WAKEUP_GPIO_LOW));

                    // Enter light sleep
                    esp_deep_sleep_start();

                    // After wakeup, log and restart
                    ESP_LOGI(TAG, "Waking up from light sleep - performing restart");
                    vTaskDelay(pdMS_TO_TICKS(100));

                    // Perform software reset
                    esp_restart();
                }
            }
        } else {
            if (pin_was_low) {
                lcd_hide_loading_bar();  // Hide the loading bar when button is released
                lcd_reset_loading_bar();  // Reset the pointer
            }
            pin_was_low = false;
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // Back to original 20ms delay
    }
}

static void check_inactivity_sleep(void)
{
    // Get inactivity time from LVGL and adjust for tick rate
    uint32_t inactivity_time = lv_disp_get_inactive_time(NULL) * portTICK_PERIOD_MS;

    // Check if we should go to sleep (if inactive and not connected)
    if (inactivity_time > INACTIVITY_TIMEOUT_MS && !is_connect) {
        ESP_LOGI(TAG, "System inactive for %lu ms and no BLE connection. Entering deep sleep.",
                 inactivity_time);

        // Configure wakeup on button press (active low)
        ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(1ULL << SLEEP_PIN,
                                                         ESP_GPIO_WAKEUP_GPIO_LOW));

        // Enter deep sleep
        esp_deep_sleep_start();
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Application");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize sleep GPIO
    init_sleep_gpio();

    // Initialize ADC and start tasks
    ESP_ERROR_CHECK(adc_init());
    adc_start_task();

    // Wait for ADC calibration
    while (!adc_is_calibrated()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Initialize BLE
    spp_client_demo_init();
    ESP_LOGI(TAG, "BLE Initialization complete");

    // Initialize LCD and LVGL
    lcd_init();

    // Create and configure display label
    adc_label = lcd_create_label("0");

    // Start display tasks
    lcd_start_tasks();

    // Create display update task
    xTaskCreate(update_display_task, "update_display", 4096, NULL, 4, NULL);

    // Create sleep monitoring task
    xTaskCreate(check_sleep_conditions, "sleep_monitor", 2048, NULL, 3, NULL);

    // Main task can now sleep
    while (1) {
        check_inactivity_sleep();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

