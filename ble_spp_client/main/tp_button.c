#include "tp_button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "viber.h"
#include "sleep.h"

static const char *TAG = "TP Button";

static QueueHandle_t que_touch = NULL;
typedef struct touch_msg {
    touch_pad_intr_mask_t intr_mask;
    uint32_t pad_num;
    uint32_t pad_status;
    uint32_t pad_val;
} touch_event_t;

static const touch_pad_t button[TP_BUTTON_NUM] = {
    TOUCH_PAD_NUM9,
    TOUCH_PAD_NUM12,
};

static const float button_threshold[TP_BUTTON_NUM] = {
    0.01,
    0.01
};

static int active_buttons = 0;
static bool buttons_active = false;

static void touchsensor_interrupt_cb(void *arg)
{
    int task_awoken = pdFALSE;
    touch_event_t evt;

    evt.intr_mask = touch_pad_read_intr_status_mask();
    evt.pad_status = touch_pad_get_status();
    evt.pad_num = touch_pad_get_current_meas_channel();

    if (evt.intr_mask & TOUCH_PAD_INTR_MASK_ACTIVE) {
        active_buttons++;
    } else if (evt.intr_mask & TOUCH_PAD_INTR_MASK_INACTIVE) {
        if (active_buttons > 0) {
            active_buttons--;
        }
    }

    xQueueSendFromISR(que_touch, &evt, &task_awoken);
    if (task_awoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void tp_set_thresholds(void)
{
    uint32_t touch_value;
    for (int i = 0; i < TP_BUTTON_NUM; i++) {
        // Add a small delay before reading
        vTaskDelay(pdMS_TO_TICKS(20));

        touch_pad_read_benchmark(button[i], &touch_value);
        touch_pad_set_thresh(button[i], touch_value * button_threshold[i]);

        // Only log the final calibrated values
        if (touch_value < 4000000) {  // Only log reasonable values
            ESP_LOGI(TAG, "Touch pad [%d] calibrated - base: %"PRIu32", threshold: %"PRIu32,
                     button[i], touch_value, (uint32_t)(touch_value * button_threshold[i]));
        }
    }
}

static void tp_read_task(void *pvParameter)
{
    touch_event_t evt = {0};

    vTaskDelay(50 / portTICK_PERIOD_MS);
    tp_set_thresholds();

    while (1) {
        if (xQueueReceive(que_touch, &evt, portMAX_DELAY)) {
            if (evt.intr_mask & TOUCH_PAD_INTR_MASK_ACTIVE) {
                // Activate immediately when both buttons are pressed
                if (active_buttons == TP_BUTTON_NUM) {
                    buttons_active = true;
                    viber_play_pattern(VIBER_PATTERN_SINGLE_SHORT);
                    sleep_reset_inactivity_timer();
                }
            } else if (evt.intr_mask & TOUCH_PAD_INTR_MASK_INACTIVE) {
                buttons_active = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t tp_button_init(void)
{
    ESP_LOGI(TAG, "Initializing touch pad buttons");

    que_touch = xQueueCreate(TP_BUTTON_NUM, sizeof(touch_event_t));
    if (que_touch == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(touch_pad_init());
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize touch pads without logging initial values
    for (int i = 0; i < TP_BUTTON_NUM; i++) {
        ESP_ERROR_CHECK(touch_pad_config(button[i]));

        uint32_t touch_value;
        int retry_count = 0;
        const int MAX_RETRIES = 5;

        while (retry_count < MAX_RETRIES) {
            if (touch_pad_read_benchmark(button[i], &touch_value) == ESP_OK && touch_value > 0) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            retry_count++;
        }

        if (retry_count >= MAX_RETRIES) {
            ESP_LOGE(TAG, "Failed to initialize touch pad [%d]", button[i]);
            return ESP_FAIL;
        }
    }

    // Filter setting with more aggressive noise reduction
    touch_filter_config_t filter_info = {
        .mode = TOUCH_PAD_FILTER_IIR_32,          // Increased from IIR_16
        .debounce_cnt = 2,                        // Increased from 1
        .noise_thr = 3,                           // Added noise threshold
        .jitter_step = 4,
        .smh_lvl = TOUCH_PAD_SMOOTH_IIR_4,        // Increased from IIR_2
    };
    ESP_ERROR_CHECK(touch_pad_filter_set_config(&filter_info));
    ESP_ERROR_CHECK(touch_pad_filter_enable());

    // Add delay before setting thresholds
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set and verify thresholds
    tp_set_thresholds();

    // Register touch interrupt
    ESP_ERROR_CHECK(touch_pad_isr_register(touchsensor_interrupt_cb, NULL, TOUCH_PAD_INTR_MASK_ALL));
    ESP_ERROR_CHECK(touch_pad_intr_enable(TOUCH_PAD_INTR_MASK_ACTIVE |
                                        TOUCH_PAD_INTR_MASK_INACTIVE |
                                        TOUCH_PAD_INTR_MASK_TIMEOUT));

    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    ESP_ERROR_CHECK(touch_pad_fsm_start());

    xTaskCreate(&tp_read_task, "tp_read_task", 4096, NULL, 5, NULL);

    return ESP_OK;
}

bool tp_button_is_active(void) {
    return buttons_active;
}