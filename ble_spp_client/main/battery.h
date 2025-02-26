#ifndef BATTERY_H
#define BATTERY_H

#include <stdbool.h>
#include "esp_err.h"

// Function declarations
esp_err_t battery_init(void);
void battery_start_monitoring(void);
bool battery_is_charging(void);
float battery_get_voltage(void);

#endif // BATTERY_H 