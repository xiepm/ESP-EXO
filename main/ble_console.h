#ifndef __BLE_CONSOLE_H__
#define __BLE_CONSOLE_H__

#include "robstride_control.h"
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ble_console_init(void);
void ble_console_push_feedback(const motor_feedback_t *fb);
bool ble_console_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLE_CONSOLE_H__ */