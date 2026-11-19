#include "exo_sensors.h"
#include "exo_attitude.h"

#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define I2C_TIMEOUT_MS 30
#define I2C_LOCK_MS 100
#define I2C_SPEED_HZ 100000
#define QMI_STARTUP_US 200000
#define SAMPLE_PERIOD_MS 10

static const char *TAG = "exo_sensors";
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_qmi;
static i2c_master_dev_handle_t s_qmc;
static SemaphoreHandle_t s_bus_lock;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static exo_sensor_status_t s_status;
static int64_t s_qmi_start_us;
static int64_t s_qmc_start_us;

static void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms) ? pdMS_TO_TICKS(ms) : 1);
}

static void delay_at_least_ms(uint32_t ms)
{
    const int64_t deadline = esp_timer_get_time() + (int64_t)ms * 1000;
    int64_t remaining;
    /* vTaskDelay counts tick boundaries, so one call can wake too early. */
    while ((remaining = deadline - esp_timer_get_time()) > 0) {
        delay_ms((uint32_t)((remaining + 999) / 1000));
    }
}

/* All *_locked functions require s_bus_lock; a SyncSample sequence must remain
 * atomic with respect to diagnostics and the second device. */
static esp_err_t read_locked(i2c_master_dev_handle_t dev, uint8_t reg,
                             uint8_t *data, size_t length)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, length, I2C_TIMEOUT_MS);
}

static esp_err_t write_locked(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value)
{
    const uint8_t bytes[] = {reg, value};
    return i2c_master_transmit(dev, bytes, sizeof(bytes), I2C_TIMEOUT_MS);
}

static esp_err_t verify_locked(i2c_master_dev_handle_t dev, uint8_t reg,
                               uint8_t value, uint8_t mask)
{
    uint8_t actual;
    esp_err_t err = read_locked(dev, reg, &actual, 1);
    if (err == ESP_OK && (actual & mask) != (value & mask)) {
        ESP_LOGE(TAG, "readback reg=0x%02x got=0x%02x expected=0x%02x mask=0x%02x",
                 reg, actual, value, mask);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return err;
}

static esp_err_t add_device(uint8_t address, i2c_master_dev_handle_t *device)
{
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_SPEED_HZ,
    };
    return i2c_master_bus_add_device(s_bus, &cfg, device);
}

static esp_err_t wait_qmi_command_locked(bool done)
{
    const int64_t deadline = esp_timer_get_time() + 100000;
    do {
        uint8_t status;
        esp_err_t err = read_locked(s_qmi, 0x2d, &status, 1);
        if (err != ESP_OK) {
            return err;
        }
        if (((status & 0x80) != 0) == done) {
            return ESP_OK;
        }
        delay_ms(1);
    } while (esp_timer_get_time() < deadline);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t qmi_config_readback_locked(void)
{
    static const uint8_t config[][2] = {
        {0x02, 0x50}, {0x03, 0x25}, {0x04, 0x55},
        {0x06, 0x77}, {0x08, 0x83}, {0x09, 0x80},
    };
    for (size_t i = 0; i < sizeof(config) / sizeof(config[0]); ++i) {
        esp_err_t err = verify_locked(s_qmi, config[i][0], config[i][1], 0xff);
        if (err != ESP_OK) {
            return err;
        }
    }
    return verify_locked(s_qmi, 0x00, 0x05, 0xff);
}

static esp_err_t qmi_init_locked(exo_sensor_device_status_t *state, bool *fallback)
{
    uint8_t address = EXO_QMI_ADDRESS;
    esp_err_t err = i2c_master_probe(s_bus, address, I2C_TIMEOUT_MS);
    if (err == ESP_ERR_NOT_FOUND) {
        address = 0x6a;
        err = i2c_master_probe(s_bus, address, I2C_TIMEOUT_MS);
        if (err == ESP_OK) {
            *fallback = true;
            ESP_LOGW(TAG, "QMI responds at 0x6a; schematic SA0=GND expects 0x6b. Check assembly/SA0.");
        }
    }
    if (err != ESP_OK) {
        return err;
    }
    state->address = address;
    state->present = true;
    if ((err = add_device(address, &s_qmi)) != ESP_OK ||
        (err = read_locked(s_qmi, 0x00, &state->chip_id, 1)) != ESP_OK) {
        return err;
    }
    if (state->chip_id != 0x05 && state->chip_id != 0x00) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    /* On this fixed QMI address, a zero ID after an MCU-only reset may be a
     * transient chip state. Do not skip its documented soft reset. A nonzero
     * foreign ID is still rejected, and BOTH reset status and ID must match
     * below before publishing any configuration or samples. */
    if ((err = write_locked(s_qmi, 0x60, 0xb0)) != ESP_OK) {
        return err;
    }
    const int64_t reset_start = esp_timer_get_time();
    delay_at_least_ms(20); /* QMI Rev D reset maximum is 15 ms. */
    ESP_LOGI(TAG, "QMI reset wait=%" PRId64 "us", esp_timer_get_time() - reset_start);
    if ((err = verify_locked(s_qmi, 0x4d, 0x80, 0xff)) != ESP_OK ||
        (err = read_locked(s_qmi, 0x00, &state->chip_id, 1)) != ESP_OK ||
        (err = read_locked(s_qmi, 0x01, &state->revision, 1)) != ESP_OK) {
        return err;
    }
    if (state->chip_id != 0x05) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    /* Configuration registers MUST be written individually, never as a burst. */
    static const uint8_t config[][2] = {
        {0x08, 0x00}, {0x02, 0x40}, {0x03, 0x25}, {0x04, 0x55},
        {0x06, 0x77}, {0x09, 0x80}, {0x0b, 0x01},
    };
    for (size_t i = 0; i < sizeof(config) / sizeof(config[0]); ++i) {
        if ((err = write_locked(s_qmi, config[i][0], config[i][1])) != ESP_OK) {
            return err;
        }
    }
    /* SyncSample requires disabling internal AHB clock gating using CTRL9.
     * CTRL8.bit7 selects status-register handshake instead of consuming INT1. */
    if ((err = write_locked(s_qmi, 0x0a, 0x12)) != ESP_OK ||
        (err = wait_qmi_command_locked(true)) != ESP_OK ||
        (err = write_locked(s_qmi, 0x0a, 0x00)) != ESP_OK ||
        (err = wait_qmi_command_locked(false)) != ESP_OK ||
        (err = write_locked(s_qmi, 0x02, 0x50)) != ESP_OK ||
        (err = write_locked(s_qmi, 0x08, 0x83)) != ESP_OK ||
        (err = qmi_config_readback_locked()) != ESP_OK) {
        return err;
    }
    s_qmi_start_us = esp_timer_get_time() + QMI_STARTUP_US;
    state->configured = true;
    return ESP_OK;
}

static esp_err_t qmc_config_readback_locked(void)
{
    esp_err_t err = verify_locked(s_qmc, 0x00, 0x80, 0xff);
    if (err == ESP_OK) err = verify_locked(s_qmc, 0x29, 0x06, 0x3f);
    if (err == ESP_OK) err = verify_locked(s_qmc, 0x0b, 0x08, 0xff);
    if (err == ESP_OK) err = verify_locked(s_qmc, 0x0a, 0xcd, 0xff);
    return err;
}

static esp_err_t qmc_init_locked(exo_sensor_device_status_t *state)
{
    esp_err_t err = i2c_master_probe(s_bus, EXO_QMC_ADDRESS, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    state->present = true;
    if ((err = add_device(EXO_QMC_ADDRESS, &s_qmc)) != ESP_OK ||
        (err = read_locked(s_qmc, 0x00, &state->chip_id, 1)) != ESP_OK) {
        return err;
    }
    if (state->chip_id != 0x80) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if ((err = write_locked(s_qmc, 0x0b, 0x80)) != ESP_OK) {
        return err;
    }
    /* Rev A only promises auto-clear for SELF_TEST, not SOFT_RST. This chip
     * retains bit7 after reset, and combining its release with CTRL2=0x08
     * loses the range setting (readback=0x00). Release reset separately and
     * allow it to settle before applying measurement configuration.
     * These waits are conservative; the datasheet gives no reset maximum. */
    delay_at_least_ms(50);
    if ((err = write_locked(s_qmc, 0x0b, 0x00)) != ESP_OK) return err;
    delay_at_least_ms(20);
    const int64_t deadline = esp_timer_get_time() + 100000;
    bool ready = false;
    do {
        uint8_t id = 0, ctrl2 = 0x80;
        err = read_locked(s_qmc, 0x00, &id, 1);
        if (err == ESP_OK) err = read_locked(s_qmc, 0x0b, &ctrl2, 1);
        if (err == ESP_OK && id == 0x80 && ctrl2 == 0x00) {
            ready = true;
            break;
        }
        delay_ms(5);
    } while (esp_timer_get_time() < deadline);
    if (!ready) return ESP_ERR_TIMEOUT;
    /* Enter Suspend before changing mode, including after an MCU-only reset. */
    static const uint8_t config[][2] = {
        {0x0a, 0x00}, {0x29, 0x06}, {0x0b, 0x08}, {0x0a, 0xcd},
    };
    for (size_t i = 0; i < sizeof(config) / sizeof(config[0]); ++i) {
        if ((err = write_locked(s_qmc, config[i][0], config[i][1])) != ESP_OK) {
            return err;
        }
        const uint8_t mask = config[i][0] == 0x29 ? 0x3f : 0xff;
        if ((err = verify_locked(s_qmc, config[i][0], config[i][1], mask)) != ESP_OK) {
            return err;
        }
    }
    if ((err = qmc_config_readback_locked()) != ESP_OK) return err;
    s_qmc_start_us = esp_timer_get_time() + 20000;
    state->configured = true;
    return ESP_OK;
}

static esp_err_t init_sensor_locked(bool qmi)
{
    exo_sensor_device_status_t state;
    portENTER_CRITICAL(&s_status_lock);
    state = qmi ? s_status.qmi : s_status.qmc;
    portEXIT_CRITICAL(&s_status_lock);
    /* Keep cumulative counters, but never publish an old sample as recovered. */
    state.present = false;
    state.configured = false;
    state.valid = false;
    state.timestamp_us = 0;
    state.chip_id = 0;
    state.revision = 0;
    state.address = qmi ? EXO_QMI_ADDRESS : EXO_QMC_ADDRESS;
    bool fallback = false;
    i2c_master_dev_handle_t *dev = qmi ? &s_qmi : &s_qmc;
    esp_err_t err = ESP_OK;
    if (*dev) {
        err = i2c_master_bus_rm_device(*dev);
        if (err == ESP_OK) *dev = NULL;
    }
    if (err == ESP_OK) {
        for (unsigned attempt = 0; attempt < (qmi ? 3U : 1U); ++attempt) {
            err = qmi ? qmi_init_locked(&state, &fallback) : qmc_init_locked(&state);
            if (err == ESP_OK || !qmi) break;
            ESP_LOGW(TAG, "QMI boot attempt=%u present=%d id=0x%02x error=%s",
                     attempt + 1, state.present, state.chip_id, esp_err_to_name(err));
            /* A foreign, nonzero ID is not permission to reset another chip. */
            if (state.chip_id != 0 && state.chip_id != 0x05) break;
            if (attempt < 2U) {
                if (*dev) {
                    const esp_err_t remove_err = i2c_master_bus_rm_device(*dev);
                    if (remove_err != ESP_OK) { err = remove_err; break; }
                    *dev = NULL;
                }
                state.present = false; state.chip_id = 0;
                delay_at_least_ms(100);
            }
        }
    }
    state.last_error = err;
    portENTER_CRITICAL(&s_status_lock);
    if (qmi) {
        s_status.qmi = state;
        s_status.qmi_address_fallback = fallback;
    } else {
        s_status.qmc = state;
    }
    portEXIT_CRITICAL(&s_status_lock);
    return err;
}

static int16_t little_i16(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static void sample_error(bool qmi, esp_err_t err, bool overflow)
{
    portENTER_CRITICAL(&s_status_lock);
    exo_sensor_device_status_t *dev = qmi ? &s_status.qmi : &s_status.qmc;
    dev->valid = false;
    dev->last_error = err;
    ++dev->errors;
    if (err == ESP_ERR_TIMEOUT) ++dev->timeouts;
    if (overflow) ++dev->overflows;
    portEXIT_CRITICAL(&s_status_lock);
}

static esp_err_t sample_qmi_locked(bool *updated, bool *overflow)
{
    *updated = false;
    *overflow = false;
    uint8_t status;
    esp_err_t err = read_locked(s_qmi, 0x2d, &status, 1);
    if (err != ESP_OK) return err;
    if (!(status & 0x03)) return ESP_OK;
    /* Reading STATUSINT when Avail=1 latches the sample. Even a prior failed
     * burst can leave Locked=1: always drain the complete frame through GZ_H. */
    if (!(status & 0x02)) esp_rom_delay_us(12);
    uint8_t bytes[12];
    err = read_locked(s_qmi, 0x35, bytes, sizeof(bytes));
    if (err != ESP_OK) return err;
    int16_t raw[6];
    float value[6];
    for (size_t i = 0; i < 6; ++i) {
        raw[i] = little_i16(bytes + i * 2);
        if (raw[i] == INT16_MIN || raw[i] == INT16_MAX) {
            *overflow = true;
            return ESP_ERR_INVALID_RESPONSE;
        }
        value[i] = raw[i] / (i < 3 ? 4096.0f : 64.0f);
    }
    portENTER_CRITICAL(&s_status_lock);
    memcpy(s_status.qmi.raw, raw, sizeof(raw));
    memcpy(s_status.qmi.value, value, sizeof(value));
    s_status.qmi.timestamp_us = esp_timer_get_time();
    ++s_status.qmi.samples;
    s_status.qmi.valid = true;
    s_status.qmi.last_error = ESP_OK;
    portEXIT_CRITICAL(&s_status_lock);
    *updated = true;
    return ESP_OK;
}

static esp_err_t sample_qmc_locked(bool *updated, bool *overflow)
{
    *updated = false;
    *overflow = false;
    uint8_t status;
    esp_err_t err = read_locked(s_qmc, 0x09, &status, 1);
    if (err != ESP_OK) return err;
    /* Reading STATUS clears both DRDY and OVFL. Inspect this one snapshot. */
    if (status & 0x02) {
        *overflow = true;
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (!(status & 0x01)) return ESP_OK;
    /* Include CHIPID for the register dump, then read all six XYZ bytes. */
    uint8_t bytes[7];
    err = read_locked(s_qmc, 0x00, bytes, sizeof(bytes));
    if (err != ESP_OK) return err;
    int16_t raw[3];
    float value[3];
    for (size_t i = 0; i < 3; ++i) {
        raw[i] = little_i16(bytes + 1 + i * 2);
        value[i] = raw[i] * (100.0f / 3750.0f); /* +/-8G, typical 3750 LSB/G. */
    }
    portENTER_CRITICAL(&s_status_lock);
    memcpy(s_status.qmc.raw, raw, sizeof(raw));
    memcpy(s_status.qmc.value, value, sizeof(value));
    memcpy(s_status.qmc_regs_00_05, bytes, sizeof(s_status.qmc_regs_00_05));
    s_status.qmc.timestamp_us = esp_timer_get_time();
    ++s_status.qmc.samples;
    s_status.qmc.valid = true;
    s_status.qmc.last_error = ESP_OK;
    portEXIT_CRITICAL(&s_status_lock);
    *updated = true;
    return ESP_OK;
}

static void sampler(void *argument)
{
    (void)argument;
    int64_t qmi_last_progress = s_qmi_start_us;
    int64_t qmc_last_progress = s_qmc_start_us;
    unsigned qmi_boot_retries = 2;
    int64_t qmi_retry_after = esp_timer_get_time() + 2000000;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        bool qmi_updated = false;
        if (xSemaphoreTake(s_bus_lock, pdMS_TO_TICKS(I2C_LOCK_MS)) == pdTRUE) {
            exo_sensor_status_t state;
            exo_sensors_get_status(&state);
            const int64_t now = esp_timer_get_time();
            /* MCU-only reset does not reset sensor power. Retry failed startup
             * after the board has settled; bounded, below control priority, no
             * motor commands. A healthy chip is never reset by this path. */
            if (!state.qmi.configured && qmi_boot_retries && now >= qmi_retry_after &&
                (state.qmi.chip_id == 0 || state.qmi.chip_id == 0x05)) {
                --qmi_boot_retries;
                (void)init_sensor_locked(true);
                qmi_retry_after = esp_timer_get_time() + 2000000;
                exo_sensors_get_status(&state);
                last_wake = xTaskGetTickCount();
            }
            /* A retry starts a new startup grace period for that device. */
            if (qmi_last_progress < s_qmi_start_us) qmi_last_progress = s_qmi_start_us;
            if (qmc_last_progress < s_qmc_start_us) qmc_last_progress = s_qmc_start_us;
            if (state.qmi.configured && now >= s_qmi_start_us) {
                bool updated, overflow;
                esp_err_t err = sample_qmi_locked(&updated, &overflow);
                if (err != ESP_OK) sample_error(true, err, overflow);
                else if (updated) {
                    qmi_last_progress = now;
                    qmi_updated = true;
                }
            }
            if (state.qmc.configured && now >= s_qmc_start_us) {
                bool updated, overflow;
                esp_err_t err = sample_qmc_locked(&updated, &overflow);
                if (err != ESP_OK) sample_error(false, err, overflow);
                else if (updated) qmc_last_progress = now;
            }
            if (state.qmi.configured && now - qmi_last_progress > EXO_SENSOR_FRESH_US) {
                sample_error(true, ESP_ERR_TIMEOUT, false);
                qmi_last_progress = now;
            }
            if (state.qmc.configured && now - qmc_last_progress > EXO_SENSOR_FRESH_US) {
                sample_error(false, ESP_ERR_TIMEOUT, false);
                qmc_last_progress = now;
            }
            xSemaphoreGive(s_bus_lock);
        }
        if (qmi_updated) {
            exo_sensor_status_t sample;
            exo_sensors_get_status(&sample);
            /* Fusion runs outside the I2C mutex and sees fresh QMC data from
             * this pass. Missing/stale magnetic data permits IMU-only fusion. */
            exo_attitude_update(&sample);
        }
        /* Fixed 100Hz polling decimates the 224.2Hz sensor ODR. SyncSample
         * keeps each complete six-axis burst coherent. */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS) ?
                                  pdMS_TO_TICKS(SAMPLE_PERIOD_MS) : 1);
    }
}

esp_err_t exo_sensors_init(void)
{
    if (s_bus || s_bus_lock) return ESP_ERR_INVALID_STATE;
    s_status.bus_hz = I2C_SPEED_HZ;
    s_status.qmi.address = EXO_QMI_ADDRESS;
    s_status.qmc.address = EXO_QMC_ADDRESS;
    s_status.qmi.last_error = ESP_ERR_INVALID_STATE;
    s_status.qmc.last_error = ESP_ERR_INVALID_STATE;
    const gpio_config_t interrupts = {
        .pin_bit_mask = (1ULL << GPIO_NUM_10) | (1ULL << GPIO_NUM_11),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&interrupts);
    if (err != ESP_OK) return err;
    s_bus_lock = xSemaphoreCreateMutex();
    if (!s_bus_lock) return ESP_ERR_NO_MEM;
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false, /* Board has external 4.7k pulls. */
    };
    if ((err = i2c_new_master_bus(&bus_cfg, &s_bus)) != ESP_OK) {
        vSemaphoreDelete(s_bus_lock);
        s_bus_lock = NULL;
        return err;
    }
    s_status.bus_ready = true;
    delay_at_least_ms(20);
    xSemaphoreTake(s_bus_lock, portMAX_DELAY); /* No sampler exists yet. */
    esp_err_t qmi_err = init_sensor_locked(true);
    esp_err_t qmc_err = init_sensor_locked(false);
    xSemaphoreGive(s_bus_lock);
    s_status.qmi.last_error = qmi_err;
    s_status.qmc.last_error = qmc_err;
    ESP_LOGI(TAG, "QMI8658A addr=0x%02x id=0x%02x rev=0x%02x config=%s (+/-8g,+/-512dps,224.2Hz,SyncSample)",
             s_status.qmi.address, s_status.qmi.chip_id, s_status.qmi.revision,
             esp_err_to_name(qmi_err));
    ESP_LOGI(TAG, "QMC5883P addr=0x2c id=0x%02x config=%s (+/-8G,ODR-code200Hz,OSR8/8)",
             s_status.qmc.chip_id, esp_err_to_name(qmc_err));
    if (xTaskCreate(sampler, "exo_sensor", 4096, NULL, 5, NULL) != pdPASS) {
        s_status.qmi.configured = false;
        s_status.qmc.configured = false;
        s_status.qmi.last_error = ESP_ERR_NO_MEM;
        s_status.qmc.last_error = ESP_ERR_NO_MEM;
        return ESP_ERR_NO_MEM;
    }
    return qmi_err != ESP_OK ? qmi_err : qmc_err;
}

esp_err_t exo_sensors_retry(char *out, size_t capacity)
{
    if (!out || !capacity) return ESP_ERR_INVALID_ARG;
    if (!s_bus || !s_bus_lock) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_bus_lock, pdMS_TO_TICKS(I2C_LOCK_MS)) != pdTRUE) return ESP_ERR_TIMEOUT;
    exo_sensor_status_t state;
    exo_sensors_get_status(&state);
    esp_err_t qmi_err = state.qmi.configured ? ESP_OK : init_sensor_locked(true);
    esp_err_t qmc_err = state.qmc.configured ? ESP_OK : init_sensor_locked(false);
    xSemaphoreGive(s_bus_lock);
    snprintf(out, capacity, "SENSOR_RETRY QMI8658A=%s QMC5883P=%s (configured devices skipped)",
             state.qmi.configured ? "SKIP" : esp_err_to_name(qmi_err),
             state.qmc.configured ? "SKIP" : esp_err_to_name(qmc_err));
    return qmi_err != ESP_OK ? qmi_err : qmc_err;
}

void exo_sensors_get_status(exo_sensor_status_t *status)
{
    if (!status) return;
    portENTER_CRITICAL(&s_status_lock);
    *status = s_status;
    portEXIT_CRITICAL(&s_status_lock);
    const int64_t now = esp_timer_get_time();
    status->qmi.valid = status->qmi.valid && status->qmi.configured &&
                        now - status->qmi.timestamp_us <= EXO_SENSOR_FRESH_US;
    status->qmc.valid = status->qmc.valid && status->qmc.configured &&
                        now - status->qmc.timestamp_us <= EXO_SENSOR_FRESH_US;
}

bool exo_sensors_healthy(void)
{
    exo_sensor_status_t state;
    exo_sensors_get_status(&state);
    return state.bus_ready && state.qmi.valid && state.qmc.valid;
}

esp_err_t exo_sensor_i2c_read(uint8_t address, uint8_t reg, uint8_t *data, size_t length)
{
    if (!data || !length || length > 32 || (size_t)reg + length > 256) return ESP_ERR_INVALID_ARG;
    if (!s_status.bus_ready) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_bus_lock, pdMS_TO_TICKS(I2C_LOCK_MS)) != pdTRUE) return ESP_ERR_TIMEOUT;
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = ESP_ERR_NOT_FOUND;
    if (address == s_status.qmi.address && s_qmi) {
        if (reg <= 0x40 && (size_t)reg + length > 0x2d) err = ESP_ERR_NOT_SUPPORTED;
        else dev = s_qmi;
    } else if (address == EXO_QMC_ADDRESS && s_qmc) {
        if (reg <= 0x09 && (size_t)reg + length > 0x01) err = ESP_ERR_NOT_SUPPORTED;
        else dev = s_qmc;
    }
    if (dev) err = read_locked(dev, reg, data, length);
    xSemaphoreGive(s_bus_lock);
    return err;
}

esp_err_t exo_sensor_i2c_scan(char *out, size_t capacity)
{
    if (!out || capacity < 32) return ESP_ERR_INVALID_ARG;
    out[0] = '\0';
    if (!s_status.bus_ready) return ESP_ERR_INVALID_STATE;
    size_t used = (size_t)snprintf(out, capacity, "I2C0 SDA=8 SCL=9 100000Hz found:");
    for (uint8_t address = 0x08; address <= 0x77; ++address) {
        if (xSemaphoreTake(s_bus_lock, pdMS_TO_TICKS(I2C_LOCK_MS)) != pdTRUE) return ESP_ERR_TIMEOUT;
        /* ESP-IDF converts milliseconds to RTOS ticks. Keep this above the
         * 10ms tick period so an absent address reports NACK, not timeout. */
        esp_err_t err = i2c_master_probe(s_bus, address, I2C_TIMEOUT_MS);
        xSemaphoreGive(s_bus_lock);
        if (err == ESP_OK && used < capacity) {
            int count = snprintf(out + used, capacity - used, " 0x%02x", address);
            if (count > 0) used += (size_t)count;
        } else if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
            if (used < capacity) snprintf(out + used, capacity - used, " ERROR@0x%02x:%s", address, esp_err_to_name(err));
            return err;
        }
        if (!(address & 0x07)) delay_ms(1); /* Let the sampler acquire the bus. */
    }
    return used < capacity ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t exo_sensors_selftest(char *out, size_t capacity)
{
    if (!out || !capacity) return ESP_ERR_INVALID_ARG;
    exo_sensor_status_t before, after;
    exo_sensors_get_status(&before);
    if (!before.bus_ready) {
        snprintf(out, capacity, "I2C_SELFTEST FAIL bus=not_initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_bus_lock, pdMS_TO_TICKS(I2C_LOCK_MS)) != pdTRUE) {
        snprintf(out, capacity, "I2C_SELFTEST FAIL bus=lock_timeout");
        return ESP_ERR_TIMEOUT;
    }
    exo_sensors_get_status(&before);
    esp_err_t qmi_err = before.qmi.configured ? qmi_config_readback_locked() : before.qmi.last_error;
    esp_err_t qmc_err = before.qmc.configured ? qmc_config_readback_locked() : before.qmc.last_error;
    /* A bad readback invalidates configuration as well as the current sample;
     * later samples must not silently clear that fault and permit arming. */
    portENTER_CRITICAL(&s_status_lock);
    if (qmi_err != ESP_OK && before.qmi.configured) {
        s_status.qmi.configured = false;
        s_status.qmi.valid = false;
        s_status.qmi.last_error = qmi_err;
        ++s_status.qmi.errors;
    }
    if (qmc_err != ESP_OK && before.qmc.configured) {
        s_status.qmc.configured = false;
        s_status.qmc.valid = false;
        s_status.qmc.last_error = qmc_err;
        ++s_status.qmc.errors;
    }
    portEXIT_CRITICAL(&s_status_lock);
    xSemaphoreGive(s_bus_lock);
    const int64_t deadline = esp_timer_get_time() + 800000;
    do {
        delay_ms(20);
        exo_sensors_get_status(&after);
        if (after.qmi.samples - before.qmi.samples >= 5 &&
            after.qmc.samples - before.qmc.samples >= 5) break;
    } while (esp_timer_get_time() < deadline);
    bool qmi_pass = qmi_err == ESP_OK && after.qmi.valid &&
                    after.qmi.samples - before.qmi.samples >= 5 &&
                    after.qmi.errors == before.qmi.errors;
    bool qmc_pass = qmc_err == ESP_OK && after.qmc.valid &&
                    after.qmc.samples - before.qmc.samples >= 5 &&
                    after.qmc.errors == before.qmc.errors;
    snprintf(out, capacity,
             "I2C_SELFTEST %s QMI8658A=%s addr=0x%02x id=0x%02x cfg=%s new=%" PRIu32 " new_errors=%" PRIu32
             " QMC5883P=%s addr=0x2c id=0x%02x cfg=%s new=%" PRIu32 " new_errors=%" PRIu32
             " (ID/config/fresh-data test; not factory MEMS self-test)",
             qmi_pass && qmc_pass ? "PASS" : "FAIL", qmi_pass ? "PASS" : "FAIL",
             after.qmi.address, after.qmi.chip_id, esp_err_to_name(qmi_err),
             after.qmi.samples - before.qmi.samples, after.qmi.errors - before.qmi.errors,
             qmc_pass ? "PASS" : "FAIL", after.qmc.chip_id, esp_err_to_name(qmc_err),
             after.qmc.samples - before.qmc.samples, after.qmc.errors - before.qmc.errors);
    return qmi_pass && qmc_pass ? ESP_OK : ESP_FAIL;
}

/* Diagnostic reads never publish raw test data or advance live sample counters.
 * For the stimulus, wait for both new data and SELF_TEST to clear, so an older
 * continuous-mode frame cannot be mistaken for the stimulus result. */
static esp_err_t qmc_test_read_locked(int64_t deadline, bool stimulus,
                                      int16_t raw[3], uint8_t *ctrl2)
{
    do {
        esp_err_t err;
        if (stimulus && (err = read_locked(s_qmc, 0x0b, ctrl2, 1)) != ESP_OK) {
            return err;
        }
        if (stimulus && (*ctrl2 & 0x40)) {
            /* STATUS clears DRDY: do not consume the only final stimulus
             * notification while SELF_TEST is still completing. */
            delay_ms(5);
            continue;
        }
        uint8_t status = 0;
        if ((err = read_locked(s_qmc, 0x09, &status, 1)) != ESP_OK) return err;
        if (status & 0x01) {
            uint8_t bytes[6];
            if ((err = read_locked(s_qmc, 0x01, bytes, sizeof(bytes))) != ESP_OK) return err;
            for (size_t i = 0; i < 3; ++i) raw[i] = little_i16(bytes + i * 2);
            return status & 0x02 ? ESP_ERR_INVALID_RESPONSE : ESP_OK;
        }
        if (status & 0x02) return ESP_ERR_INVALID_RESPONSE;
        delay_ms(5);
    } while (esp_timer_get_time() < deadline);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t qmc_test_restore_locked(void)
{
    static const uint8_t config[][2] = {
        {0x0a, 0x00}, {0x29, 0x06}, {0x0b, 0x08}, {0x0a, 0xcd},
    };
    esp_err_t first_error = ESP_OK;
    /* Attempt every restoration write even if a preceding operation failed. */
    for (size_t i = 0; i < sizeof(config) / sizeof(config[0]); ++i) {
        esp_err_t err = write_locked(s_qmc, config[i][0], config[i][1]);
        if (first_error == ESP_OK && err != ESP_OK) first_error = err;
    }
    esp_err_t err = qmc_config_readback_locked();
    return first_error != ESP_OK ? first_error : err;
}

esp_err_t exo_sensors_magtest(char *out, size_t capacity)
{
    if (!out || !capacity) return ESP_ERR_INVALID_ARG;
    if (!s_bus || !s_bus_lock) {
        snprintf(out, capacity, "QMC_MAGTEST NO_RESPONSE test=ESP_ERR_INVALID_STATE stage=bus_unavailable");
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_bus_lock, pdMS_TO_TICKS(I2C_LOCK_MS)) != pdTRUE) {
        snprintf(out, capacity, "QMC_MAGTEST NO_RESPONSE test=ESP_ERR_TIMEOUT stage=bus_lock");
        return ESP_ERR_TIMEOUT;
    }
    exo_sensor_status_t before;
    exo_sensors_get_status(&before);
    if (!s_qmc || !before.qmc.configured) {
        xSemaphoreGive(s_bus_lock);
        snprintf(out, capacity, "QMC_MAGTEST NO_RESPONSE test=ESP_ERR_INVALID_STATE stage=QMC_unconfigured");
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&s_status_lock);
    s_status.qmc.valid = false;
    s_status.qmc.configured = false;
    s_status.qmc.timestamp_us = 0;
    portEXIT_CRITICAL(&s_status_lock);

    int16_t baseline[3] = {0}, stimulus[3] = {0};
    int32_t delta[3] = {0};
    uint8_t ctrl2 = 0xff;
    bool have_baseline = false, have_stimulus = false;
    const int64_t deadline = esp_timer_get_time() + 500000;
    const char *stage = "continuous_setup";
    esp_err_t test_error = ESP_OK;
    esp_err_t restore_error = ESP_OK;
    /* Rev A section 7.3 uses +/-30G for BOTH baseline and stimulus. A raw
     * comparison must not mix the normal +/-8G range with +/-30G stimulus. */
    static const uint8_t setup[][2] = {
        {0x0a, 0x00}, {0x0b, 0x00}, {0x29, 0x06},
    };
    for (size_t i = 0; i < sizeof(setup) / sizeof(setup[0]); ++i) {
        test_error = write_locked(s_qmc, setup[i][0], setup[i][1]);
        if (test_error == ESP_OK) {
            const uint8_t mask = setup[i][0] == 0x29 ? 0x3f : 0xff;
            test_error = verify_locked(s_qmc, setup[i][0], setup[i][1], mask);
        }
        if (test_error != ESP_OK) goto restore;
    }
    uint8_t discarded_status;
    /* Discard prior normal-mode DRDY before starting the baseline. */
    test_error = read_locked(s_qmc, 0x09, &discarded_status, 1);
    if (test_error != ESP_OK) goto restore;
    test_error = write_locked(s_qmc, 0x0a, 0x03);
    if (test_error == ESP_OK) test_error = verify_locked(s_qmc, 0x0a, 0x03, 0xff);
    if (test_error != ESP_OK) goto restore;
    delay_at_least_ms(20);
    stage = "baseline";
    test_error = qmc_test_read_locked(deadline, false, baseline, &ctrl2);
    if (test_error != ESP_OK) goto restore;
    have_baseline = true;
    stage = "stimulus_trigger";
    test_error = write_locked(s_qmc, 0x0b, 0x40);
    if (test_error != ESP_OK) goto restore;
    delay_at_least_ms(5); /* Rev A section 7.3 measurement duration. */
    stage = "stimulus";
    test_error = qmc_test_read_locked(deadline, true, stimulus, &ctrl2);
    if (test_error != ESP_OK) goto restore;
    have_stimulus = true;
    delta[0] = (int32_t)baseline[0] - stimulus[0];
    delta[1] = (int32_t)stimulus[1] - baseline[1];
    delta[2] = (int32_t)stimulus[2] - baseline[2];
    stage = "complete";

restore:
    /* Restore and verify even after a timeout, overflow or transport failure.
     * The bounded acquisition wait excludes these mandatory cleanup transfers. */
    restore_error = qmc_test_restore_locked();
    s_qmc_start_us = esp_timer_get_time() + 20000;
    portENTER_CRITICAL(&s_status_lock);
    s_status.qmc.configured = restore_error == ESP_OK;
    s_status.qmc.valid = false;
    s_status.qmc.timestamp_us = 0;
    s_status.qmc.last_error = restore_error != ESP_OK ? restore_error : test_error;
    portEXIT_CRITICAL(&s_status_lock);
    xSemaphoreGive(s_bus_lock);

    /* At +/-30G the nominal noise is 2/3 LSB. This is an explicit diagnostic
     * response discriminator, NOT a manufacturer self-test acceptance limit. */
    bool response = test_error == ESP_OK && have_baseline && have_stimulus &&
                    (delta[0] >= 32 || delta[0] <= -32 ||
                     delta[1] >= 32 || delta[1] <= -32 ||
                     delta[2] >= 32 || delta[2] <= -32);
    int count = snprintf(out, capacity,
             "QMC_MAGTEST %s normal8g_valid=%u normal8g_raw=[%d,%d,%d]"
             " base30g_available=%u base30g_raw=[%d,%d,%d]"
             " stim30g_available=%u stim30g_raw=[%d,%d,%d]"
             " delta_lsb=[%" PRId32 ",%" PRId32 ",%" PRId32 "] self_test_bit=%d"
             " test=%s stage=%s restore=%s diagnostic_min_delta_lsb=32"
             " (signal response only; no manufacturer threshold or accuracy qualification)",
             response ? "RESPONSE" : "NO_RESPONSE", before.qmc.valid,
             before.qmc.raw[0], before.qmc.raw[1], before.qmc.raw[2],
             have_baseline, baseline[0], baseline[1], baseline[2],
             have_stimulus, stimulus[0], stimulus[1], stimulus[2],
             delta[0], delta[1], delta[2], ctrl2 == 0xff ? -1 : !!(ctrl2 & 0x40),
             esp_err_to_name(test_error), stage, esp_err_to_name(restore_error));
    if (test_error != ESP_OK) return test_error;
    if (restore_error != ESP_OK) return restore_error;
    return count < 0 || (size_t)count >= capacity ? ESP_ERR_NO_MEM : ESP_OK;
}
