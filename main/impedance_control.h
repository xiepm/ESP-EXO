#ifndef __IMPEDANCE_CONTROL_H__
#define __IMPEDANCE_CONTROL_H__

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Prepare and start dual-motor sinusoidal impedance control.
 *
 * Disables, configures, and enables logical actuators A (anti-phase) and B
 * (in-phase), captures their center positions, and then makes the controller
 * ready for calls to impedance_control_process_tick(). This function does not
 * create a task or timer.
 *
 * Parameters retained from the demo2 reference:
 *   KP = 0.2 N*m/rad, KD = 0.10 N*m/(rad/s)
 *   Amplitude = 90 degrees (+/-pi/2 rad), Frequency = 0.5 Hz
 *   Startup ramp = 3.0 s (quintic)
 *
 * @return ESP_OK on success; ESP_ERR_INVALID_STATE if start or stop is active
 */
esp_err_t impedance_control_start(void);

/**
 * @brief Execute one externally scheduled control step.
 *
 * When running, computes the retained sinusoidal reference for @p now_us and
 * attempts one motion frame for each actuator. When inactive, performs no I/O
 * and returns ESP_OK. This function contains no scheduler, delay, or logging.
 *
 * @param now_us Monotonic time in microseconds, normally esp_timer_get_time().
 * @return ESP_OK when inactive or both frames succeed; otherwise a send error
 */
esp_err_t impedance_control_process_tick(int64_t now_us);

/**
 * @brief Stop periodic output, release torque, and disable both motors.
 */
esp_err_t impedance_control_stop(void);

/**
 * @return true only after preparation succeeds and before stopping begins
 */
bool impedance_control_is_running(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMPEDANCE_CONTROL_H__ */
