#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Starts the programming-UART-only engineering debug console. */
esp_err_t debug_console_init(void);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_CONSOLE_H */
