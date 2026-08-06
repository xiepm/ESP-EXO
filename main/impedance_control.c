#include "impedance_control.h"

#include <math.h>

#include "exo_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rs01_driver.h"

/* Parameters retained exactly from the demo2 reference. */
#define IMP_KP 0.2f
#define IMP_KD 0.10f
#define IMP_AMPLITUDE_RAD 1.570796327f /* 90 degrees */
#define IMP_FREQ_HZ 0.5f
#define IMP_RAMP_TIME_S 3.0f

static float s_center[EXO_ACTUATOR_COUNT];
static int64_t s_start_us;
static bool s_armed;
static bool s_tick_active;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* Quintic ramp from demo2: r(s) = 10s^3 - 15s^4 + 6s^5. */
static float quintic_ramp(float elapsed, float ramp_duration) {
	if (ramp_duration <= 0.0f) {
		return 1.0f;
	}
	if (elapsed <= 0.0f) {
		return 0.0f;
	}
	if (elapsed >= ramp_duration) {
		return 1.0f;
	}

	const float s = elapsed / ramp_duration;
	const float s2 = s * s;
	const float s3 = s2 * s;
	return s3 * (10.0f - 15.0f * s + 6.0f * s2);
}

esp_err_t impedance_control_arm(float center_a_rad, float center_b_rad,
								int64_t start_us) {
	if (!isfinite(center_a_rad) || !isfinite(center_b_rad) || start_us <= 0) {
		return ESP_ERR_INVALID_ARG;
	}

	portENTER_CRITICAL(&s_lock);
	if (s_armed || s_tick_active) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	s_center[EXO_ACTUATOR_A] = center_a_rad;
	s_center[EXO_ACTUATOR_B] = center_b_rad;
	s_start_us = start_us;
	s_armed = true;
	portEXIT_CRITICAL(&s_lock);
	return ESP_OK;
}

esp_err_t impedance_control_process_tick(int64_t now_us) {
	float center_a;
	float center_b;
	int64_t start_us;

	portENTER_CRITICAL(&s_lock);
	if (!s_armed) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_OK;
	}
	if (s_tick_active) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}

	s_tick_active = true;
	center_a = s_center[EXO_ACTUATOR_A];
	center_b = s_center[EXO_ACTUATOR_B];
	start_us = s_start_us;
	portEXIT_CRITICAL(&s_lock);

	const float elapsed = (float)(now_us - start_us) / 1000000.0f;
	const float omega = 2.0f * (float)M_PI * IMP_FREQ_HZ;
	const float ramp = quintic_ramp(elapsed, IMP_RAMP_TIME_S);
	const float phase = omega * elapsed;
	const float sin_val = ramp * sinf(phase);

	/* Actuator A remains anti-phase; actuator B remains in-phase. */
	const float target_a = center_a - IMP_AMPLITUDE_RAD * sin_val;
	const float target_b = center_b + IMP_AMPLITUDE_RAD * sin_val;

	/* Attempt both outputs so one actuator's TX error cannot starve the other.
	 */
	const esp_err_t err_a = rs01_try_send_motion(EXO_ACTUATOR_A, target_a, 0.0f,
												 0.0f, IMP_KP, IMP_KD);
	const esp_err_t err_b = rs01_try_send_motion(EXO_ACTUATOR_B, target_b, 0.0f,
												 0.0f, IMP_KP, IMP_KD);

	portENTER_CRITICAL(&s_lock);
	s_tick_active = false;
	portEXIT_CRITICAL(&s_lock);

	return err_a != ESP_OK ? err_a : err_b;
}

esp_err_t impedance_control_disarm(void) {
	portENTER_CRITICAL(&s_lock);
	s_armed = false;
	portEXIT_CRITICAL(&s_lock);

	for (;;) {
		portENTER_CRITICAL(&s_lock);
		const bool active = s_tick_active;
		portEXIT_CRITICAL(&s_lock);
		if (!active) {
			return ESP_OK;
		}
		vTaskDelay(1);
	}
}
