#pragma once
#include <stdint.h>
#include "esp_err.h"
#define GPIO_NUM_42 42
#define GPIO_MODE_OUTPUT 1
#define GPIO_PULLUP_DISABLE 0
#define GPIO_PULLDOWN_ENABLE 1
#define GPIO_INTR_DISABLE 0
typedef struct { uint64_t pin_bit_mask; int mode,pull_up_en,pull_down_en,intr_type; } gpio_config_t;
esp_err_t gpio_set_level(int pin,int value);
esp_err_t gpio_config(const gpio_config_t *cfg);
