/* Ported peripherals from the user's tested exo_v2.1 source, 2026-11-17.
 * Board attitude is NOT a hip encoder and never supplies motor enable gates. */
#include "exo_board_status.h"
#include "exo_board_power.h"
#include "exo_sensors.h"
#include "exo_attitude.h"
#include "exo_mag_calibration.h"
#include "exo_smbus.h"
#include "rs01_motor_profile.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

void exo_board_init(void) {
    ESP_LOGI("exo_board", "motor=%s speed_range=+/-%.1f torque_range=+/-%.1f",
             EXO_MOTOR_MODEL_NAME, (double)EXO_MOTOR_WIRE_SPEED_MAX,
             (double)EXO_MOTOR_WIRE_TORQUE_MAX);
#ifdef CONFIG_EXO_BOARD_V21
    esp_err_t cal = exo_attitude_set_mag_calibration(&EXO_BOARD_MAG_CALIBRATION);
    esp_err_t sensors = exo_sensors_init();
    esp_err_t smb = exo_smbus_init();
    ESP_LOGI("exo_board", "sensors=%s mag_cal=%s SMBus=%s (no scan)",
             esp_err_to_name(sensors), esp_err_to_name(cal), esp_err_to_name(smb));
#endif
}

static uint16_t age_ms(int64_t timestamp) {
    int64_t now = esp_timer_get_time();
    if (timestamp <= 0 || now < timestamp) return UINT16_MAX;
    int64_t age = (now - timestamp) / 1000;
    return age >= UINT16_MAX ? UINT16_MAX : (uint16_t)age;
}

void exo_board_read(uint8_t page, uint8_t *flags, uint16_t *age, float values[3]) {
    *flags = 0; *age = UINT16_MAX;
    memset(values, 0, 3 * sizeof(float));
    if (page == 0) {
        exo_attitude_status_t a;
        exo_attitude_get_status(&a);
        *age = age_ms(a.timestamp_us);
        *flags = (a.valid && *age < 250 ? 1U : 0U) |
                 (a.gyro_calibrated ? 2U : 0U) | (a.mag_fused ? 4U : 0U) |
                 (a.updates ? 8U : 0U);
        values[0] = a.roll_deg; values[1] = a.pitch_deg; values[2] = a.yaw_deg;
    } else if (page <= 3) {
        exo_sensor_status_t s;
        exo_sensors_get_status(&s);
        const exo_sensor_device_status_t *d = page == 3 ? &s.qmc : &s.qmi;
        *age = age_ms(d->timestamp_us);
        *flags = (d->valid && *age < 250 ? 1U : 0U) | (d->configured ? 8U : 0U);
        memcpy(values, d->value + (page == 2 ? 3 : 0), 3 * sizeof(float));
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(values[i])) { *flags &= ~1U; values[i] = 0; }
    }
}

void exo_board_print_status(void) {
    printf("BOARD_POWER GPIO42 command=%u (not motor enable; no voltage feedback)\n",
           exo_board_power_commanded_on() ? 1U : 0U);
    exo_sensor_status_t s; exo_attitude_status_t a; exo_smbus_status_t b;
    exo_sensors_get_status(&s); exo_attitude_get_status(&a); exo_smbus_get_status(&b);
    printf("BOARD motor=%s I2C0 SDA8/SCL9 ready=%d hz=%lu\n",
        EXO_MOTOR_MODEL_NAME, s.bus_ready, (unsigned long)s.bus_hz);
    const exo_sensor_device_status_t *devices[] = {&s.qmi, &s.qmc};
    for (unsigned i=0; i<2; ++i) {
        const exo_sensor_device_status_t *d=devices[i];
        printf("%s addr=0x%02x id=0x%02x configured=%d valid=%d samples=%lu errors=%lu timeouts=%lu age_ms=%u present=%d last_error=%s\n",
            i ? "QMC5883P" : "QMI8658A", d->address, d->chip_id, d->configured,
            d->valid, (unsigned long)d->samples, (unsigned long)d->errors,
            (unsigned long)d->timeouts, age_ms(d->timestamp_us), d->present, esp_err_to_name(d->last_error));
    }
    for (uint8_t p=0; p<4; ++p) {
        uint8_t flags; uint16_t age; float v[3]; exo_board_read(p,&flags,&age,v);
        printf("BOARD_PAGE %u flags=0x%02x age_ms=%u xyz=%.5f,%.5f,%.5f\n",
               p,flags,age,(double)v[0],(double)v[1],(double)v[2]);
    }
    printf("ATTITUDE valid=%d gyro_cal=%d progress=%lu updates=%lu gaps=%lu mag_cal=%d fused=%d\n",
        a.valid,a.gyro_calibrated,(unsigned long)a.calibration_progress_pct,
        (unsigned long)a.updates,(unsigned long)a.discontinuities,a.mag_calibrated,a.mag_fused);
    printf("SMBus I2C1 SDA15/SCL16 ready=%d transactions=%lu errors=%lu\n",
        b.initialized,(unsigned long)b.transactions,(unsigned long)b.errors);
}
