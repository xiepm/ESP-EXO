#ifndef __BLE_CONSOLE_H__
#define __BLE_CONSOLE_H__

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ble_console_init(void);
bool ble_console_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLE_CONSOLE_H__ */
