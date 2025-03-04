#include "sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lcd.h"
#include "ui/ui.h"
#include "lvgl.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"
#include "esp_task_wdt.h"
#include "hw_config.h"
#include "ble_spp_client.h"
#include "viber.h"


#define TAG "SLEEP"

static TickType_t last_activity_time;
static TickType_t last_reset_time = 0;
#define RESET_DEBOUNCE_TIME_MS 2000



static void sleep_monitor_task(void *pvParameters) {
    while (1) {
        sleep_check_inactivity(is_connect);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void sleep_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HALL_SENSOR_VDD_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    last_activity_time = xTaskGetTickCount();

    xTaskCreate(sleep_monitor_task, "sleep_monitor", 2048, NULL, 4, NULL);
}

void sleep_reset_inactivity_timer(void)
{
    TickType_t current_time = xTaskGetTickCount();

    // Only reset if enough time has passed since last reset
    if ((current_time - last_reset_time) * portTICK_PERIOD_MS >= RESET_DEBOUNCE_TIME_MS) {
        last_activity_time = current_time;
        last_reset_time = current_time;
    }
}

void sleep_check_inactivity(bool is_ble_connected)
{
    TickType_t current_time = xTaskGetTickCount();
    TickType_t elapsed_time = (current_time - last_activity_time) * portTICK_PERIOD_MS;

    // Check if we should go to sleep (if inactive and not connected)
    if (elapsed_time > INACTIVITY_TIMEOUT_MS && !is_ble_connected) {
        enter_deep_sleep();
    }
}

void enter_deep_sleep(void) {
    // Disable task watchdog (only if it was enabled)
    #if CONFIG_ESP_TASK_WDT_EN
        esp_task_wdt_deinit();
    #endif

    // Turn off hall sensor
    gpio_set_level(HALL_SENSOR_VDD_PIN, 0);

    // Configure accelerometer INT1 pin for wake-up
    rtc_gpio_init(LIS3DHTR_INT1_PIN);
    rtc_gpio_set_direction(LIS3DHTR_INT1_PIN, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_dis(LIS3DHTR_INT1_PIN);
    rtc_gpio_pulldown_en(LIS3DHTR_INT1_PIN);
    rtc_gpio_hold_en(LIS3DHTR_INT1_PIN);

    // Enable wake-up on high level (accelerometer interrupt)
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(LIS3DHTR_INT1_PIN, 1));

    // Enter deep sleep
    ESP_LOGI(TAG, "Entering deep sleep");

    // Play sleep pattern
    viber_play_pattern(VIBER_PATTERN_DOUBLE_SHORT);
    vTaskDelay(pdMS_TO_TICKS(250));  // Wait for vibration to complete

    esp_deep_sleep_start();
}