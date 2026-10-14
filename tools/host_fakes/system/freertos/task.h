#ifndef HOST_SYSTEM_TASK_H
#define HOST_SYSTEM_TASK_H
#include "freertos/FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks);
void xTaskNotifyGive(TaskHandle_t task);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
TaskHandle_t xTaskCreateStaticPinnedToCore(void (*fn)(void *), const char *name,
    uint32_t stack_size, void *arg, unsigned priority, StackType_t *stack,
    StaticTask_t *buffer, BaseType_t core);
#endif
