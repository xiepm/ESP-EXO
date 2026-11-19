#include "exo_board_power.h"
#include "driver/gpio.h"
#include <assert.h>
#include <stdio.h>
static int events[8], count;
static esp_err_t level_error,config_error;
esp_err_t gpio_set_level(int pin,int value) {
    assert(pin==42);events[count++]=value;return level_error;
}
esp_err_t gpio_config(const gpio_config_t *c) {
    assert(c->pin_bit_mask==(1ULL<<42) && c->mode==GPIO_MODE_OUTPUT);
    assert(!c->pull_up_en && c->pull_down_en && !c->intr_type);
    events[count++]=2;return config_error;
}
int main(void) {
    assert(exo_board_power_prepare_feedback()==ESP_ERR_INVALID_STATE && !count);
    assert(exo_board_power_init()==ESP_OK);
    assert(count==2 && events[0]==0 && events[1]==2 && !exo_board_power_commanded_on());
    assert(exo_board_power_prepare_feedback()==ESP_OK);
    assert(count==3 && events[2]==1 && exo_board_power_commanded_on());
    assert(exo_board_power_prepare_feedback()==ESP_OK && count==3);
    count=0;config_error=ESP_FAIL;
    assert(exo_board_power_init()==ESP_FAIL);
    assert(exo_board_power_prepare_feedback()==ESP_ERR_INVALID_STATE && count==2);
    config_error=ESP_OK;count=0;assert(exo_board_power_init()==ESP_OK);
    level_error=ESP_FAIL;
    assert(exo_board_power_prepare_feedback()==ESP_FAIL && !exo_board_power_commanded_on());
    puts("PASS board power LOW-before-output, explicit ON, idempotency, failure stays unready");
}
