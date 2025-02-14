
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "lvgl.h"

// LCD Configuration
#define LCD_HOST    SPI2_HOST
#define TOUCH_HOST  I2C_NUM_0

#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define LCD_CS         (GPIO_NUM_40)
#define LCD_PCLK       (GPIO_NUM_39)
#define LCD_DATA0      (GPIO_NUM_36)
#define LCD_DATA1      (GPIO_NUM_38)
#define LCD_DATA2      (GPIO_NUM_41)
#define LCD_DATA3      (GPIO_NUM_42)
#define LCD_RST        (GPIO_NUM_35)
#define LCD_BK_LIGHT   (-1)
#define LCD_EN         (GPIO_NUM_18)

// The pixel number in horizontal and vertical
#define LCD_H_RES              368
#define LCD_V_RES              448

#if CONFIG_LV_COLOR_DEPTH == 32
#define LCD_BIT_PER_PIXEL       (24)
#elif CONFIG_LV_COLOR_DEPTH == 16
#define LCD_BIT_PER_PIXEL       (16)
#endif

// Touch configuration
#define USE_TOUCH               1
#if USE_TOUCH
#define PIN_NUM_TOUCH_SCL      (GPIO_NUM_6)
#define PIN_NUM_TOUCH_SDA      (GPIO_NUM_7)
#define PIN_NUM_TOUCH_RST      (GPIO_NUM_17)
#define PIN_NUM_TOUCH_INT      (GPIO_NUM_16)
#endif

// LVGL specific configuration
#define LVGL_BUF_HEIGHT        (LCD_V_RES / 4)
#define LVGL_TICK_PERIOD_MS    2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE   (4 * 1024)
#define LVGL_TASK_PRIORITY     2

// Public functions
esp_err_t display_init(void);
bool display_lock(int timeout_ms);
void display_unlock(void);

