#include "tp_button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "viber.h"
#include "sleep.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "TP Button";

// Function declarations
static esp_err_t save_calibration_to_nvs(uint32_t *values);
static esp_err_t load_calibration_from_nvs(uint32_t *values);
static void tp_set_thresholds(void);
static void touchsensor_interrupt_cb(void *arg);
static void tp_read_task(void *pvParameter);

static QueueHandle_t que_touch = NULL;
typedef struct touch_msg {
    touch_pad_intr_mask_t intr_mask;
    uint32_t pad_num;
    uint32_t pad_status;
    uint32_t pad_val;
    bool should_vibrate;  // New field to indicate vibration needed
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
    touch_event_t evt = {0};

    evt.intr_mask = touch_pad_read_intr_status_mask();
    evt.pad_status = touch_pad_get_status();
    evt.pad_num = touch_pad_get_current_meas_channel();

    if (evt.intr_mask & TOUCH_PAD_INTR_MASK_ACTIVE) {
        active_buttons++;
        if (active_buttons >= TP_BUTTON_NUM) {
            buttons_active = true;
            evt.should_vibrate = true;  // Request vibration
        }
    } else if (evt.intr_mask & TOUCH_PAD_INTR_MASK_INACTIVE) {
        buttons_active = false;
        if (active_buttons > 0) {
            active_buttons--;
        }
        if (active_buttons == 0) {
            evt.should_vibrate = true;  // Request vibration
        }
    }

    if (active_buttons > TP_BUTTON_NUM) {
        active_buttons = TP_BUTTON_NUM;
    }

    xQueueSendFromISR(que_touch, &evt, &task_awoken);
    if (task_awoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void tp_set_thresholds(void)
{
    uint32_t touch_values[TP_BUTTON_NUM];
    uint32_t samples[TP_BUTTON_NUM][TP_CALIBRATION_SAMPLES];

#if CALIBRATE_TP
    ESP_LOGI(TAG, "Force calibration flag set, performing calibration");
    nvs_handle_t nvs_handle;
    if (nvs_open(TP_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) == ESP_OK) {
        nvs_erase_key(nvs_handle, TP_NVS_KEY_CALIBRATED);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
#else
    if (load_calibration_from_nvs(touch_values) == ESP_OK) {
        for (int i = 0; i < TP_BUTTON_NUM; i++) {
            touch_pad_set_thresh(button[i], touch_values[i] * button_threshold[i]);
            ESP_LOGI(TAG, "Touch pad [%d] loaded calibration - base: %"PRIu32", threshold: %"PRIu32,
                     button[i], touch_values[i], (uint32_t)(touch_values[i] * button_threshold[i]));
        }
        return;
    }
#endif

    // Perform calibration with multiple samples
    ESP_LOGI(TAG, "Starting touch pad calibration process");
    ESP_LOGI(TAG, "Please follow the calibration instructions:");

    for (int sample = 0; sample < TP_CALIBRATION_SAMPLES; sample++) {
        ESP_LOGI(TAG, "Sample %d/%d - Press and hold both buttons for %d ms",
                 sample + 1, TP_CALIBRATION_SAMPLES, TP_CALIBRATION_HOLD_TIME_MS);

        vTaskDelay(pdMS_TO_TICKS(TP_CALIBRATION_DELAY_MS));

        // Take readings for each button
        for (int i = 0; i < TP_BUTTON_NUM; i++) {
            touch_pad_read_benchmark(button[i], &samples[i][sample]);
            ESP_LOGI(TAG, "Button %d reading: %"PRIu32, button[i], samples[i][sample]);
        }

        // Indicate completion of this sample
        viber_play_pattern(VIBER_PATTERN_SINGLE_SHORT);
    }

    // Calculate average values
    for (int i = 0; i < TP_BUTTON_NUM; i++) {
        uint32_t sum = 0;
        for (int sample = 0; sample < TP_CALIBRATION_SAMPLES; sample++) {
            sum += samples[i][sample];
        }
        touch_values[i] = sum / TP_CALIBRATION_SAMPLES;

        // Set thresholds using averaged values
        touch_pad_set_thresh(button[i], touch_values[i] * button_threshold[i]);
        ESP_LOGI(TAG, "Touch pad [%d] calibration complete - base: %"PRIu32", threshold: %"PRIu32,
                 button[i], touch_values[i], (uint32_t)(touch_values[i] * button_threshold[i]));
    }

    // Save the calibration
    save_calibration_to_nvs(touch_values);

    // Indicate calibration completion
    viber_play_pattern(VIBER_PATTERN_DOUBLE_SHORT);
    ESP_LOGI(TAG, "Calibration complete!");
}

static void tp_read_task(void *pvParameter)
{
    touch_event_t evt = {0};

    vTaskDelay(50 / portTICK_PERIOD_MS);
    tp_set_thresholds();

    while (1) {
        if (xQueueReceive(que_touch, &evt, portMAX_DELAY)) {
            if (evt.should_vibrate) {
                viber_play_pattern(VIBER_PATTERN_SINGLE_SHORT);
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

bool tp_button_is_active(void)
{
    return buttons_active && (active_buttons == TP_BUTTON_NUM);
}

static esp_err_t save_calibration_to_nvs(uint32_t *values) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(TP_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    // Save calibration values
    err = nvs_set_blob(nvs_handle, TP_NVS_KEY_VALUES, values, sizeof(uint32_t) * TP_BUTTON_NUM);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }

    // Set calibration flag
    err = nvs_set_u8(nvs_handle, TP_NVS_KEY_CALIBRATED, 1);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return err;
}

static esp_err_t load_calibration_from_nvs(uint32_t *values) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(TP_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    // Check if calibration exists
    uint8_t is_calibrated = 0;
    err = nvs_get_u8(nvs_handle, TP_NVS_KEY_CALIBRATED, &is_calibrated);
    if (err != ESP_OK || !is_calibrated) {
        nvs_close(nvs_handle);
        return ESP_ERR_NOT_FOUND;
    }

    // Read calibration values
    size_t required_size = sizeof(uint32_t) * TP_BUTTON_NUM;
    err = nvs_get_blob(nvs_handle, TP_NVS_KEY_VALUES, values, &required_size);
    nvs_close(nvs_handle);

    return err;
}

esp_err_t tp_button_force_calibration(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(TP_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_erase_key(nvs_handle, TP_NVS_KEY_CALIBRATED);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
    tp_set_thresholds();
    return ESP_OK;
}