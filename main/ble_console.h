#ifndef __BLE_CONSOLE_H__
#define __BLE_CONSOLE_H__

#include "esp_err.h"
#include "exo_types.h"
#include "rs01_driver.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ble_console_init(void);
void ble_console_push_feedback(exo_actuator_id_t actuator,
                               const rs01_feedback_snapshot_t *feedback);
bool ble_console_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLE_CONSOLE_H__ */
