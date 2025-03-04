#include "lis3dhtr.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hw_config.h"
#include <math.h>
#include "viber.h"
#include "sleep.h"

static const char *TAG = "LIS3DHTR";

// Add at the top of the file
static SemaphoreHandle_t i2c_mutex = NULL;

// I2C initialization
static esp_err_t i2c_master_init(void)
{
    esp_err_t ret;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;

    // Try to install the driver - if it fails with ESP_ERR_INVALID_STATE,
    // it means the driver is already installed
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "I2C driver already installed");
        return ESP_OK;
    }

    return ret;
}

// Write to register
static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, LIS3DHTR_I2C_ADDR,
                                             (uint8_t[]){reg, value}, 2,
                                             pdMS_TO_TICKS(10));

    xSemaphoreGive(i2c_mutex);
    return ret;
}

// Read from register
static esp_err_t read_register(uint8_t reg, uint8_t *data)
{
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, LIS3DHTR_I2C_ADDR,
                                                &reg, 1, data, 1,
                                                pdMS_TO_TICKS(10));

    xSemaphoreGive(i2c_mutex);
    return ret;
}

esp_err_t lis3dhtr_init(void)
{
    esp_err_t ret;
    const int MAX_RETRIES = 3;
    int retry_count = 0;

    // Create mutex for I2C access
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize I2C with retries
    while (retry_count < MAX_RETRIES) {
        ret = i2c_master_init();
        if (ret == ESP_OK) break;
        ESP_LOGW(TAG, "I2C init attempt %d failed: %d", retry_count + 1, ret);
        vTaskDelay(pdMS_TO_TICKS(100));
        retry_count++;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C after %d attempts", MAX_RETRIES);
        return ESP_ERR_NOT_FOUND;
    }

    // Reset retry counter
    retry_count = 0;

    // Configure INT1 pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LIS3DHTR_INT1_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Add longer delay after I2C initialization
    vTaskDelay(pdMS_TO_TICKS(200));

    // Test communication with retries
    while (retry_count < MAX_RETRIES) {
        uint8_t who_am_i;
        ret = read_register(LIS3DHTR_REG_WHO_AM_I, &who_am_i);

        if (ret == ESP_OK && who_am_i == LIS3DHTR_WHO_AM_I_VALUE) {
            break;
        }

        ESP_LOGW(TAG, "WHO_AM_I read attempt %d failed. ret=%d, value=0x%02x",
                 retry_count + 1, ret, who_am_i);
        vTaskDelay(pdMS_TO_TICKS(100));
        retry_count++;
    }

    if (retry_count >= MAX_RETRIES) {
        ESP_LOGE(TAG, "Failed to verify WHO_AM_I after %d attempts", MAX_RETRIES);
        return ESP_ERR_NOT_FOUND;
    }

    // Rest of initialization with retries for each write
    const uint8_t init_regs[][2] = {
        {LIS3DHTR_REG_CTRL1, 0x47},
        {LIS3DHTR_REG_CTRL4, 0x00},
        {LIS3DHTR_REG_CTRL2, 0x09},
        {LIS3DHTR_REG_CTRL3, 0x40},
        {LIS3DHTR_REG_INT1_CFG, 0x2A},
        {LIS3DHTR_REG_INT1_THS, 0x40},
        {LIS3DHTR_REG_INT1_DURATION, 0x00}
    };

    for (int i = 0; i < sizeof(init_regs)/sizeof(init_regs[0]); i++) {
        retry_count = 0;
        while (retry_count < MAX_RETRIES) {
            ret = write_register(init_regs[i][0], init_regs[i][1]);
            if (ret == ESP_OK) break;
            ESP_LOGW(TAG, "Register 0x%02x write attempt %d failed", init_regs[i][0], retry_count + 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            retry_count++;
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write register 0x%02x", init_regs[i][0]);
            return ret;
        }
    }

    ESP_LOGI(TAG, "LIS3DHTR initialized successfully");
    return ESP_OK;
}

static esp_err_t read_registers(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, LIS3DHTR_I2C_ADDR,
                                                &reg, 1, data, len,
                                                pdMS_TO_TICKS(10));

    xSemaphoreGive(i2c_mutex);
    return ret;
}

esp_err_t lis3dhtr_read_acc(lis3dhtr_data_t *data)
{
    esp_err_t ret;
    uint8_t raw_data[6];
    const int NUM_SAMPLES = 8;  // Increased number of samples to average
    float x_sum = 0, y_sum = 0, z_sum = 0;

    for(int i = 0; i < NUM_SAMPLES; i++) {
        ret = read_registers(LIS3DHTR_REG_OUT_X_L, raw_data, 6);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read accelerometer data: %d", ret);
            return ret;
        }

        int16_t x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
        int16_t y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
        int16_t z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

        // Use SENSITIVITY_2G since we're in ±2g mode
        x_sum += x * SENSITIVITY_2G;
        y_sum += y * SENSITIVITY_2G;
        z_sum += z * SENSITIVITY_2G;

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    data->x = x_sum / NUM_SAMPLES;
    data->y = y_sum / NUM_SAMPLES;
    data->z = z_sum / NUM_SAMPLES;

    return ESP_OK;
}

static bool check_for_shake(lis3dhtr_data_t *data) {
    static TickType_t last_shake_time = 0;
    static uint8_t shake_count = 0;
    static float filtered_acceleration = 0.0f;

    // Calculate total acceleration magnitude
    float magnitude = sqrtf(data->x * data->x + data->y * data->y + data->z * data->z);
    float net_acceleration = fabsf(magnitude - 1.0f);

    // Apply stronger low-pass filter (90% previous, 10% new)
    filtered_acceleration = 0.9f * filtered_acceleration + 0.1f * net_acceleration;

    TickType_t current_time = xTaskGetTickCount();

    // Only proceed if filtered acceleration is significant
    if (filtered_acceleration < 0.1f) {
        shake_count = 0;
        return false;
    }

    // Check if we're in cooldown period
    if ((current_time - last_shake_time) * portTICK_PERIOD_MS < SHAKE_COOLDOWN_MS) {
        shake_count = 0;
        return false;
    }

    // Only log when filtered acceleration is significant
    if (filtered_acceleration > SHAKE_RESET_THRESHOLD_G) {
        ESP_LOGI(TAG, "Filtered acceleration: %.2f, count=%d", filtered_acceleration, shake_count);
    }

    if (filtered_acceleration > SHAKE_THRESHOLD_G) {
        shake_count++;
        if (shake_count >= SHAKE_CONSECUTIVE_SAMPLES) {
            last_shake_time = current_time;
            shake_count = 0;
            return true;
        }
    } else {
        shake_count = 0;
    }

    return false;
}

void lis3dhtr_task(void *pvParameters)
{
    lis3dhtr_data_t data;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        if (lis3dhtr_read_acc(&data) == ESP_OK) {
            if (check_for_shake(&data)) {
                sleep_reset_inactivity_timer();
                viber_play_pattern(VIBER_PATTERN_SINGLE_SHORT);
            }
        } else {
            ESP_LOGW(TAG, "Failed to read accelerometer");
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
    }
}

