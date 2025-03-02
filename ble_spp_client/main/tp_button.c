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

static void touchsensor_interrupt_cb(void *arg)
{
    int task_awoken = pdFALSE;
    touch_event_t evt;

    evt.intr_mask = touch_pad_read_intr_status_mask();
    evt.pad_status = touch_pad_get_status();
    evt.pad_num = touch_pad_get_current_meas_channel();

    if (evt.intr_mask & TOUCH_PAD_INTR_MASK_ACTIVE) {
        sleep_reset_inactivity_timer();
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
        touch_pad_read_benchmark(button[i], &touch_value);
        touch_pad_set_thresh(button[i], touch_value * button_threshold[i]);
        ESP_LOGI(TAG, "touch pad [%d] base %"PRIu32", thresh %"PRIu32,
                 button[i], touch_value, (uint32_t)(touch_value * button_threshold[i]));
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
                ESP_LOGI(TAG, "TouchPad[%"PRIu32"] is pressed", evt.pad_num);
                viber_play_pattern(VIBER_PATTERN_SINGLE_SHORT);
            }
            if (evt.intr_mask & TOUCH_PAD_INTR_MASK_INACTIVE) {
                ESP_LOGI(TAG, "TouchPad[%"PRIu32"] is released", evt.pad_num);
            }
            if (evt.intr_mask & TOUCH_PAD_INTR_MASK_TIMEOUT) {
                ESP_LOGI(TAG, "TouchPad[%"PRIu32"] timeout", evt.pad_num);
                touch_pad_timeout_resume();
            }
        }
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

    // Initialize touch pads
    for (int i = 0; i < TP_BUTTON_NUM; i++) {
        ESP_ERROR_CHECK(touch_pad_config(button[i]));
    }

    // Filter setting
    touch_filter_config_t filter_info = {
        .mode = TOUCH_PAD_FILTER_IIR_16,
        .debounce_cnt = 1,
        .noise_thr = 0,
        .jitter_step = 4,
        .smh_lvl = TOUCH_PAD_SMOOTH_IIR_2,
    };
    ESP_ERROR_CHECK(touch_pad_filter_set_config(&filter_info));
    ESP_ERROR_CHECK(touch_pad_filter_enable());

    // Register touch interrupt
    ESP_ERROR_CHECK(touch_pad_isr_register(touchsensor_interrupt_cb, NULL, TOUCH_PAD_INTR_MASK_ALL));
    ESP_ERROR_CHECK(touch_pad_intr_enable(TOUCH_PAD_INTR_MASK_ACTIVE |
                                        TOUCH_PAD_INTR_MASK_INACTIVE |
                                        TOUCH_PAD_INTR_MASK_TIMEOUT));

    // Start FSM timer
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    ESP_ERROR_CHECK(touch_pad_fsm_start());

    // Create touch pad reading task
    xTaskCreate(&tp_read_task, "tp_read_task", 4096, NULL, 5, NULL);

    return ESP_OK;
}