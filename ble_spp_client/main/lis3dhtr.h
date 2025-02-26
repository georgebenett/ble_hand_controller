#ifndef LIS3DHTR_H
#define LIS3DHTR_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "hw_config.h"
#include <stdbool.h>

// LIS3DHTR I2C Address (can be 0x18 or 0x19 depending on SA0/SDO pin)
#define LIS3DHTR_I2C_ADDR          0x19

// Register addresses
#define LIS3DHTR_REG_STATUS        0x27
#define LIS3DHTR_REG_OUT_X_L       0x28
#define LIS3DHTR_REG_OUT_X_H       0x29
#define LIS3DHTR_REG_OUT_Y_L       0x2A
#define LIS3DHTR_REG_OUT_Y_H       0x2B
#define LIS3DHTR_REG_OUT_Z_L       0x2C
#define LIS3DHTR_REG_OUT_Z_H       0x2D
#define LIS3DHTR_REG_CTRL1         0x20
#define LIS3DHTR_REG_CTRL2         0x21
#define LIS3DHTR_REG_CTRL3         0x22
#define LIS3DHTR_REG_CTRL4         0x23
#define LIS3DHTR_REG_CTRL5         0x24
#define LIS3DHTR_REG_CTRL6         0x25
#define LIS3DHTR_REG_WHO_AM_I      0x0F
#define LIS3DHTR_WHO_AM_I_VALUE    0x33
#define LIS3DHTR_REG_INT1_CFG      0x30
#define LIS3DHTR_REG_INT1_SRC      0x31
#define LIS3DHTR_REG_INT1_THS      0x32
#define LIS3DHTR_REG_INT1_DURATION 0x33

// Sensitivity values for different scales (in g/digit)
#define SENSITIVITY_2G  0.000061f  // 0.061 mg/digit
#define SENSITIVITY_4G  0.000122f  // 0.122 mg/digit
#define SENSITIVITY_8G  0.000244f  // 0.244 mg/digit
#define SENSITIVITY_16G 0.000732f  // 0.732 mg/digit


// Data structure for accelerometer readings
typedef struct {
    float x;
    float y;
    float z;
} lis3dhtr_data_t;

// Function declarations
esp_err_t lis3dhtr_init(void);
esp_err_t lis3dhtr_read_acc(lis3dhtr_data_t *data);
esp_err_t lis3dhtr_set_data_rate(uint8_t rate);
esp_err_t lis3dhtr_enable_int1(bool enable);
void lis3dhtr_task(void *pvParameters);

#endif // LIS3DHTR_H 