#pragma once

#include <stdio.h>
#include "esp_err.h"
#include "driver/touch_pad.h"
#include <stdbool.h>

#define TP_BUTTON_NUM    2
#define TP_BUTTON_WATERPROOF_ENABLE 1
#define TP_BUTTON_DENOISE_ENABLE    1
#define TP_BUTTON_CHANGE_CONFIG     0

esp_err_t tp_button_init(void);
bool tp_button_is_active(void);