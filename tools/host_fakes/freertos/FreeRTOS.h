#ifndef HOST_FAKE_FREERTOS_H
#define HOST_FAKE_FREERTOS_H
#include <stdint.h>
typedef unsigned TickType_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#endif
