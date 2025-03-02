#include "lis3dhtr.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hw_config.h"

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
    int retry_count = 0;
    const int MAX_RETRIES = 3;

    // Create mutex for I2C access
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return ESP_ERR_NO_MEM;
    }

    do {
        // Initialize I2C
        ret = i2c_master_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize I2C (attempt %d): %d", retry_count + 1, ret);
            vTaskDelay(pdMS_TO_TICKS(100));
            retry_count++;
            continue;
        }

        // Add a delay after I2C initialization
        vTaskDelay(pdMS_TO_TICKS(100));

        // Test communication by reading WHO_AM_I register with retries
        uint8_t who_am_i;
        int who_am_i_retries = 3;
        bool who_am_i_success = false;

        while (who_am_i_retries--) {
            ret = read_register(LIS3DHTR_REG_WHO_AM_I, &who_am_i);
            if (ret == ESP_OK && who_am_i == LIS3DHTR_WHO_AM_I_VALUE) {
                who_am_i_success = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (!who_am_i_success) {
            ESP_LOGW(TAG, "Failed to verify WHO_AM_I (attempt %d)", retry_count + 1);
            retry_count++;
            continue;
        }

        // Initialize accelerometer with verification
        // Enable all axes, normal mode
        ret = write_register(LIS3DHTR_REG_CTRL1, 0x97);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to write CTRL1 (attempt %d)", retry_count + 1);
            retry_count++;
            continue;
        }

        // Verify CTRL1 write
        uint8_t ctrl1_verify;
        ret = read_register(LIS3DHTR_REG_CTRL1, &ctrl1_verify);
        if (ret != ESP_OK || ctrl1_verify != 0x97) {
            ESP_LOGW(TAG, "CTRL1 verification failed (attempt %d)", retry_count + 1);
            retry_count++;
            continue;
        }

        // High-resolution mode + ±4g range
        ret = write_register(LIS3DHTR_REG_CTRL4, 0x18);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to write CTRL4 (attempt %d)", retry_count + 1);
            retry_count++;
            continue;
        }

        // Verify CTRL4 write
        uint8_t ctrl4_verify;
        ret = read_register(LIS3DHTR_REG_CTRL4, &ctrl4_verify);
        if (ret != ESP_OK || ctrl4_verify != 0x18) {
            ESP_LOGW(TAG, "CTRL4 verification failed (attempt %d)", retry_count + 1);
            retry_count++;
            continue;
        }

        ESP_LOGI(TAG, "Accelerometer initialized successfully");
        return ESP_OK;

    } while (retry_count < MAX_RETRIES);

    ESP_LOGE(TAG, "Failed to initialize accelerometer after %d attempts", MAX_RETRIES);
    return ESP_ERR_NOT_FOUND;
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
    const int NUM_SAMPLES = 5;  // Number of samples to average
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

        // Use SENSITIVITY_4G since we changed to ±4g range
        x_sum += x * SENSITIVITY_4G;
        y_sum += y * SENSITIVITY_4G;
        z_sum += z * SENSITIVITY_4G;

        vTaskDelay(pdMS_TO_TICKS(2));  // Small delay between readings
    }

    data->x = x_sum / NUM_SAMPLES;
    data->y = y_sum / NUM_SAMPLES;
    data->z = z_sum / NUM_SAMPLES;

    return ESP_OK;
}

void lis3dhtr_task(void *pvParameters)
{
    lis3dhtr_data_t data;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        lis3dhtr_read_acc(&data);

        /*if (ret == ESP_OK) {
            ESP_LOGI(TAG, "X: %.2f g, Y: %.2f g, Z: %.2f g", data.x, data.y, data.z);
        } else {
            ESP_LOGW(TAG, "Failed to read accelerometer: %d", ret);
            // Add a longer delay on error to avoid spamming
            vTaskDelay(pdMS_TO_TICKS(1000));
        }*/

        // Use vTaskDelayUntil for consistent timing
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
    }
}

