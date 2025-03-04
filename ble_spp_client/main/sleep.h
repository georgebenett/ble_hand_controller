#ifndef SLEEP_H
#define SLEEP_H

#include <stdbool.h>
#include "esp_sleep.h"


#define INACTIVITY_TIMEOUT_MS 300000  // 5 minutes

void sleep_init(void);
void sleep_reset_inactivity_timer(void);
void sleep_check_inactivity(bool is_ble_connected);
void enter_deep_sleep(void);

#endif // SLEEP_H