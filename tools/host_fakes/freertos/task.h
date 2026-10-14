#ifndef HOST_FAKE_FREERTOS_TASK_H
#define HOST_FAKE_FREERTOS_TASK_H
#include "freertos/FreeRTOS.h"
static inline void vTaskDelay(TickType_t ticks) { (void)ticks; }
#endif
