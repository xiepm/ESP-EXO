#pragma once
#include <stdbool.h>
#include "esp_err.h"
/* exo_v2.1 GPIO42 controls motor electronics power, not motor enable.
 * Boot owner: app_main only. No CAN, movement commands, or NVS writes. */
esp_err_t exo_board_power_init(void);
esp_err_t exo_board_power_prepare_feedback(void);
bool exo_board_power_commanded_on(void); /* commanded GPIO, not measured voltage */
