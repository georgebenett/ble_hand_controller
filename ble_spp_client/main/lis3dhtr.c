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

    // Create mutex for I2C access
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize I2C
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize I2C: %d - continuing without accelerometer", ret);
        return ESP_ERR_NOT_FOUND;
    }

    // Configure INT1 pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LIS3DHTR_INT1_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %d", ret);
        return ret;
    }

    // Add a small delay after I2C initialization
    vTaskDelay(pdMS_TO_TICKS(100));

    // Test communication by reading WHO_AM_I register
    uint8_t who_am_i;
    ret = read_register(LIS3DHTR_REG_WHO_AM_I, &who_am_i);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read WHO_AM_I register: %d - accelerometer not found", ret);
        return ESP_ERR_NOT_FOUND;
    }
    
    if (who_am_i != LIS3DHTR_WHO_AM_I_VALUE) {
        ESP_LOGW(TAG, "Invalid WHO_AM_I value: 0x%02x - accelerometer not found", who_am_i);
        return ESP_ERR_NOT_FOUND;
    }

    // Initialize accelerometer
    // Enable all axes, normal mode
    ret = write_register(LIS3DHTR_REG_CTRL1, 0x97); // ODR=100Hz, all axes enabled
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CTRL1: %d", ret);
        return ret;
    }

    // High-resolution mode (0x08) + range setting:
    // ±2g  = 0x00
    // ±4g  = 0x10
    // ±8g  = 0x20
    // ±16g = 0x30
    ret = write_register(LIS3DHTR_REG_CTRL4, 0x18); // 0x08 | 0x10 for ±4g range
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CTRL4: %d", ret);
        return ret;
    }

    // Configure wake-up detection
    ret = write_register(LIS3DHTR_REG_CTRL3, 0x40);  // AOI1 interrupt on INT1
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CTRL3: %d", ret);
        return ret;
    }

    // Configure interrupt 1 source
    ret = write_register(LIS3DHTR_REG_INT1_CFG, 0x2A);  // Enable XYZ high events
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write INT1_CFG: %d", ret);
        return ret;
    }

    // Set interrupt threshold (adjust this value based on sensitivity needed)
    ret = write_register(LIS3DHTR_REG_INT1_THS, 0x40);  // Threshold about 1g
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write INT1_THS: %d", ret);
        return ret;
    }

    // Set interrupt duration
    ret = write_register(LIS3DHTR_REG_INT1_DURATION, 0x00);  // Minimum duration
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write INT1_DURATION: %d", ret);
        return ret;
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
        esp_err_t ret = lis3dhtr_read_acc(&data);
        
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

