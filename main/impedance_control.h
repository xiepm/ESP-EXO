#ifndef __IMPEDANCE_CONTROL_H__
#define __IMPEDANCE_CONTROL_H__

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Arms only the retained sinusoidal controller state. Motor lifecycle and
 * feedback validation belong to exo_system, not this algorithm module.
 */
typedef struct {
	float center_left_rad;
	float center_right_rad;
} impedance_control_centers_t;

esp_err_t impedance_control_arm(const impedance_control_centers_t *centers,
								int64_t start_us);

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

/* Prevents new ticks and waits for an already-entered tick to finish. */
esp_err_t impedance_control_disarm(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMPEDANCE_CONTROL_H__ */
