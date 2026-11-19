#pragma once
#include "freertos/FreeRTOS.h"
#define pdPASS 1
void vTaskDelay(TickType_t);
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t*,TickType_t);
int xTaskCreate(void(*)(void*),const char*,unsigned,void*,unsigned,TaskHandle_t*);
