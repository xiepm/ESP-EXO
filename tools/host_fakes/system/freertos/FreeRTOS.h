#ifndef HOST_SYSTEM_FREERTOS_H
#define HOST_SYSTEM_FREERTOS_H
#include <stdint.h>
typedef unsigned TickType_t;
typedef int BaseType_t;
typedef uint32_t StackType_t;
typedef int StaticTask_t;
typedef int StaticSemaphore_t;
typedef void *TaskHandle_t;
typedef void *SemaphoreHandle_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#endif
