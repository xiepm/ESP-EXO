#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* All addresses are unshifted 7-bit I2C addresses. */
#define EXO_QMI_ADDRESS 0x6b
#define EXO_QMC_ADDRESS 0x2c
#define EXO_SENSOR_FRESH_US 250000

typedef struct {
    uint8_t address;
    uint8_t chip_id;
    uint8_t revision;
    bool present;
    bool configured;
    bool valid;
    int64_t timestamp_us;
    uint32_t samples;
    uint32_t errors;
    uint32_t timeouts;
    uint32_t overflows;
    esp_err_t last_error;
    int16_t raw[6];
    /* QMI: ax/ay/az in g, gx/gy/gz in deg/s. QMC: x/y/z in uT. */
    float value[6];
} exo_sensor_device_status_t;

typedef struct {
    bool bus_ready;
    uint32_t bus_hz;
    bool qmi_address_fallback;
    exo_sensor_device_status_t qmi;
    exo_sensor_device_status_t qmc;
    /* Same QMC burst as qmc.raw: CHIPID, X_L, X_H, Y_L, Y_H, Z_L. */
    uint8_t qmc_regs_00_05[6];
} exo_sensor_status_t;

/* Initializes both chips independently, then starts the background sampler.
 * A failed chip does not prevent the other chip from running. */
esp_err_t exo_sensors_init(void);
/* Retry only unconfigured devices; caller must keep motor power off. */
esp_err_t exo_sensors_retry(char *out, size_t capacity);
void exo_sensors_get_status(exo_sensor_status_t *status);
bool exo_sensors_healthy(void);

/* Bounded live test: IDs + configuration readback + new samples from both chips. */
esp_err_t exo_sensors_selftest(char *out, size_t capacity);
/* USB diagnostic only; caller must keep motor power off. Pauses bus
 * sampling, measures QMC built-in stimulus, and restores normal configuration.
 * RESPONSE/NO_RESPONSE is diagnostic, without a manufacturer acceptance limit. */
esp_err_t exo_sensors_magtest(char *out, size_t capacity);
esp_err_t exo_sensor_i2c_scan(char *out, size_t capacity);
/* Diagnostic reads are limited to the two installed sensor addresses. Reads of
 * clear-on-read status/data registers are rejected to preserve sampler ownership. */
esp_err_t exo_sensor_i2c_read(uint8_t address, uint8_t reg, uint8_t *data, size_t length);
