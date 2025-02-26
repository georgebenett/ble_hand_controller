#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#define CORE_0 0
#define CORE_1 1

/* ADC Configuration */
#define THROTTLE_PIN             ADC_CHANNEL_9 // adc1_ch9->gpio_num_10
#define HALL_SENSOR_VDD_PIN      GPIO_NUM_14

/* Battery Configuration */
#define BATTERY_VOLTAGE_PIN      GPIO_NUM_4
#define BATTERY_ADC_CHANNEL      ADC_CHANNEL_3  // For GPIO 4
#define BATTERY_CHARGING_PIN     GPIO_NUM_3
#define BATTERY_CHARGING_STATUS_PIN GPIO_NUM_45

/* Viber configuration */
#define VIBER_PIN                GPIO_NUM_46

/* Button Configuration */
#define MAIN_BUTTON_GPIO GPIO_NUM_8

/* I2C Configuration */
#define I2C_MASTER_NUM          I2C_NUM_1
#define I2C_MASTER_SDA_IO       GPIO_NUM_47
#define I2C_MASTER_SCL_IO       GPIO_NUM_48
#define I2C_MASTER_FREQ_HZ      100000
#define LIS3DHTR_INT1_PIN       GPIO_NUM_21

#endif // HW_CONFIG_H
