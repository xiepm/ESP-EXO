#include "impedance_control.h"

#include <math.h>

#include "exo_config.h"
#include "exo_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hip_joint.h"
#include "rs01_driver.h"
#include "sine_reference.h"

static float s_center[EXO_HIP_JOINT_COUNT];
static int64_t s_start_us;
static bool s_armed;
static bool s_tick_active;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static bool control_config_is_valid(void) {
	const rs01_limits_t *limits = rs01_get_limits();
	return sine_reference_config_is_valid(
			   &g_exo_sine_impedance_config.reference) &&
		   limits != NULL && isfinite(g_exo_sine_impedance_config.kp) &&
		   isfinite(g_exo_sine_impedance_config.kd) &&
		   g_exo_sine_impedance_config.kp >= limits->kp_min &&
		   g_exo_sine_impedance_config.kp <= limits->kp_max &&
		   g_exo_sine_impedance_config.kd >= limits->kd_min &&
		   g_exo_sine_impedance_config.kd <= limits->kd_max;
}

static esp_err_t send_joint_targets(float target_left_rad,
									float target_right_rad) {
	float target_by_actuator[EXO_ACTUATOR_COUNT] = {0};
	bool target_valid[EXO_ACTUATOR_COUNT] = {false};
	exo_actuator_id_t left_actuator = EXO_ACTUATOR_COUNT;
	exo_actuator_id_t right_actuator = EXO_ACTUATOR_COUNT;
	float left_motor_position = 0.0f;
	float right_motor_position = 0.0f;

	esp_err_t left_map_error =
		hip_joint_to_actuator_position(EXO_HIP_JOINT_LEFT, target_left_rad,
									   &left_actuator, &left_motor_position);
	esp_err_t right_map_error =
		hip_joint_to_actuator_position(EXO_HIP_JOINT_RIGHT, target_right_rad,
									   &right_actuator, &right_motor_position);
	if (left_map_error != ESP_OK) {
		return left_map_error;
	}
	if (right_map_error != ESP_OK) {
		return right_map_error;
	}
	if (left_actuator == right_actuator ||
		(unsigned)left_actuator >= (unsigned)EXO_ACTUATOR_COUNT ||
		(unsigned)right_actuator >= (unsigned)EXO_ACTUATOR_COUNT) {
		return ESP_ERR_INVALID_STATE;
	}

	target_by_actuator[left_actuator] = left_motor_position;
	target_by_actuator[right_actuator] = right_motor_position;
	target_valid[left_actuator] = true;
	target_valid[right_actuator] = true;

	esp_err_t first_error = ESP_OK;
	for (unsigned i = 0; i < (unsigned)EXO_ACTUATOR_COUNT; ++i) {
		if (!target_valid[i]) {
			return ESP_ERR_INVALID_STATE;
		}
		const esp_err_t err = rs01_try_send_motion(
			(exo_actuator_id_t)i, target_by_actuator[i], 0.0f, 0.0f,
			g_exo_sine_impedance_config.kp, g_exo_sine_impedance_config.kd);
		if (first_error == ESP_OK && err != ESP_OK) {
			first_error = err;
		}
	}
	return first_error;
}

esp_err_t impedance_control_arm(const impedance_control_centers_t *centers,
								int64_t start_us) {
	if (centers == NULL || !isfinite(centers->center_left_rad) ||
		!isfinite(centers->center_right_rad) || start_us <= 0 ||
		!control_config_is_valid()) {
		return ESP_ERR_INVALID_ARG;
	}

	portENTER_CRITICAL(&s_lock);
	if (s_armed || s_tick_active) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	s_center[EXO_HIP_JOINT_LEFT] = centers->center_left_rad;
	s_center[EXO_HIP_JOINT_RIGHT] = centers->center_right_rad;
	s_start_us = start_us;
	s_armed = true;
	portEXIT_CRITICAL(&s_lock);
	return ESP_OK;
}

esp_err_t impedance_control_process_tick(int64_t now_us) {
	float center_left;
	float center_right;
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
	center_left = s_center[EXO_HIP_JOINT_LEFT];
	center_right = s_center[EXO_HIP_JOINT_RIGHT];
	start_us = s_start_us;
	portEXIT_CRITICAL(&s_lock);

	sine_reference_sample_t reference = {0};
	esp_err_t result = ESP_ERR_INVALID_STATE;
	if (sine_reference_evaluate(&g_exo_sine_impedance_config.reference,
								now_us - start_us, &reference)) {
		/* Both human-coordinate references intentionally remain in phase. The
		 * configured joint directions reproduce the legacy A-/B+ motor targets.
		 */
		const float target_left = center_left + reference.offset_rad;
		const float target_right = center_right + reference.offset_rad;
		result = send_joint_targets(target_left, target_right);
	}

	/* Single cleanup exit: disarm must never wait forever on a failed tick. */
	portENTER_CRITICAL(&s_lock);
	s_tick_active = false;
	portEXIT_CRITICAL(&s_lock);

	return result;
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
