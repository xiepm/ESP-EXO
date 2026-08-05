#ifndef __IMPEDANCE_CONTROL_H__
#define __IMPEDANCE_CONTROL_H__

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start dual-motor compliant sinusoidal impedance control.
 *
 * Enables both motors (0x10 anti-phase, 0x20 in-phase) in MOVE_CONTROL_MODE
 * and begins sending MIT-style control frames at 100 Hz.
 *
 * Parameters (from demo2 reference):
 *   KP = 0.2 N·m/rad, KD = 0.10 N·m/(rad/s)
 *   Amplitude = 90° (±π/2 rad), Frequency = 0.5 Hz
 *   Startup ramp = 3.0 s (quintic)
 *
 * @return ESP_OK on success
 */
esp_err_t impedance_control_start(void);

/**
 * @brief Stop impedance control and disable both motors.
 */
esp_err_t impedance_control_stop(void);

/**
 * @return true if impedance control is currently running
 */
bool impedance_control_is_running(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMPEDANCE_CONTROL_H__ */
