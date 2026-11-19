#include "exo_board_power.h"
#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif
#include "driver/gpio.h"

static bool s_initialized;
static bool s_on;

esp_err_t exo_board_power_init(void) {
    s_initialized = false;
    s_on = false;
#ifdef CONFIG_EXO_BOARD_V21
    /* Like the known-working board code: set LOW before making GPIO an output. */
    esp_err_t err = gpio_set_level(GPIO_NUM_42, 0);
    if (err != ESP_OK) return err;
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << GPIO_NUM_42,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&cfg);
    if (err != ESP_OK) return err;
#endif
    s_initialized = true;
    return ESP_OK;
}

esp_err_t exo_board_power_prepare_feedback(void) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
#ifdef CONFIG_EXO_BOARD_V21
    if (s_on) return ESP_OK;
    esp_err_t err = gpio_set_level(GPIO_NUM_42, 1);
    if (err != ESP_OK) return err;
    s_on = true;
#endif
    return ESP_OK;
}

bool exo_board_power_commanded_on(void) { return s_on; }
