#pragma once

#include <stdio.h>
#include "esp_err.h"
#include "driver/touch_pad.h"
#include <stdbool.h>

#define CALIBRATE_TP 0  // Set to 1 to force touch pad calibration
#define TP_CALIBRATION_SAMPLES 5  // Number of press-and-hold samples to take
#define TP_CALIBRATION_HOLD_TIME_MS 1000  // How long to hold each press
#define TP_CALIBRATION_DELAY_MS 500  // Delay between samples

#define TP_BUTTON_NUM    2
#define TP_BUTTON_WATERPROOF_ENABLE 1
#define TP_BUTTON_DENOISE_ENABLE    1
#define TP_BUTTON_CHANGE_CONFIG     0

#define TP_NVS_NAMESPACE       "tp_button"
#define TP_NVS_KEY_CALIBRATED "tp_cal"
#define TP_NVS_KEY_VALUES     "tp_values"

esp_err_t tp_button_init(void);
bool tp_button_is_active(void);
esp_err_t tp_button_force_calibration(void);
esp_err_t tp_button_get_calibration(uint32_t *values);
