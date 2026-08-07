#include "hip_joint.h"

#include <math.h>
#include <string.h>

#include "exo_config.h"
#include "freertos/FreeRTOS.h"
#include "rs01_driver.h"

typedef struct {
	exo_hip_joint_id_t joint;
	exo_side_t side;
	exo_actuator_id_t actuator;
	exo_direction_t direction;
	float software_zero_motor_rad;
	uint64_t zero_source_timestamp_us;
	uint32_t zero_source_feedback_count;
	bool mapping_valid;
	bool direction_valid;
	bool zero_valid;
} hip_joint_t;

static hip_joint_t s_joints[EXO_HIP_JOINT_COUNT];
static bool s_initialized;
static portMUX_TYPE s_joint_lock = portMUX_INITIALIZER_UNLOCKED;

static bool joint_id_is_valid(exo_hip_joint_id_t joint) {
	return (unsigned)joint < (unsigned)EXO_HIP_JOINT_COUNT;
}

static bool actuator_id_is_valid(exo_actuator_id_t actuator) {
	return (unsigned)actuator < (unsigned)EXO_ACTUATOR_COUNT;
}

static bool direction_is_valid(exo_direction_t direction) {
	return direction == EXO_DIRECTION_POSITIVE ||
		   direction == EXO_DIRECTION_NEGATIVE;
}

static exo_side_t expected_side(exo_hip_joint_id_t joint) {
	return joint == EXO_HIP_JOINT_LEFT ? EXO_SIDE_LEFT : EXO_SIDE_RIGHT;
}

static bool motion_sample_is_finite(const exo_motion_sample_t *sample) {
	return isfinite(sample->position_rad) && isfinite(sample->velocity_rad_s) &&
		   isfinite(sample->torque_nm) && isfinite(sample->temperature_c);
}

static bool config_entry_is_valid(exo_hip_joint_id_t joint,
								  const exo_hip_joint_config_t *config) {
	const bool actuator_valid = actuator_id_is_valid(config->actuator);
	const bool direction_valid = direction_is_valid(config->direction);
	const bool mapping_state_valid =
		config->mapping_valid ? actuator_valid
							  : config->actuator == EXO_ACTUATOR_COUNT;
	const bool direction_state_valid =
		config->direction_valid ? direction_valid
								: config->direction == EXO_DIRECTION_UNKNOWN;

	return config->side == expected_side(joint) && mapping_state_valid &&
		   direction_state_valid &&
		   (!config->direction_valid || config->mapping_valid) &&
		   isfinite(config->software_zero_rad) &&
		   (!config->software_zero_valid || config->mapping_valid);
}

esp_err_t hip_joint_init(void) {
	portENTER_CRITICAL(&s_joint_lock);
	const bool already_initialized = s_initialized;
	portEXIT_CRITICAL(&s_joint_lock);
	if (already_initialized) {
		return ESP_OK;
	}

	hip_joint_t joints[EXO_HIP_JOINT_COUNT] = {0};
	for (unsigned i = 0; i < (unsigned)EXO_HIP_JOINT_COUNT; ++i) {
		const exo_hip_joint_id_t joint = (exo_hip_joint_id_t)i;
		const exo_hip_joint_config_t *config =
			&g_exo_hardware_config.hip_joint[i];
		if (!config_entry_is_valid(joint, config)) {
			return ESP_ERR_INVALID_ARG;
		}

		joints[i].joint = joint;
		joints[i].side = config->side;
		joints[i].actuator = config->actuator;
		joints[i].direction = config->direction;
		joints[i].software_zero_motor_rad = config->software_zero_rad;
		joints[i].mapping_valid = config->mapping_valid;
		joints[i].direction_valid = config->direction_valid;
		joints[i].zero_valid = config->software_zero_valid;
	}

	if (joints[EXO_HIP_JOINT_LEFT].mapping_valid &&
		joints[EXO_HIP_JOINT_RIGHT].mapping_valid &&
		joints[EXO_HIP_JOINT_LEFT].actuator ==
			joints[EXO_HIP_JOINT_RIGHT].actuator) {
		return ESP_ERR_INVALID_ARG;
	}

	portENTER_CRITICAL(&s_joint_lock);
	if (!s_initialized) {
		memcpy(s_joints, joints, sizeof(s_joints));
		s_initialized = true;
	}
	portEXIT_CRITICAL(&s_joint_lock);
	return ESP_OK;
}

bool hip_joint_get_snapshot(exo_hip_joint_id_t joint,
							hip_joint_snapshot_t *snapshot) {
	if (snapshot == NULL) {
		return false;
	}
	memset(snapshot, 0, sizeof(*snapshot));
	if (!joint_id_is_valid(joint)) {
		return false;
	}

	hip_joint_t state;
	portENTER_CRITICAL(&s_joint_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_joint_lock);
		return false;
	}
	state = s_joints[joint];
	portEXIT_CRITICAL(&s_joint_lock);

	snapshot->joint = state.joint;
	snapshot->side = state.side;
	snapshot->actuator = state.actuator;
	snapshot->direction = state.direction;
	snapshot->software_zero_motor_rad = state.software_zero_motor_rad;
	snapshot->zero_source_timestamp_us = state.zero_source_timestamp_us;
	snapshot->zero_source_feedback_count = state.zero_source_feedback_count;
	snapshot->mapping_valid = state.mapping_valid;
	snapshot->direction_valid = state.direction_valid;
	snapshot->zero_valid = state.zero_valid;

	if (!state.mapping_valid) {
		return true;
	}

	rs01_feedback_snapshot_t feedback;
	if (!rs01_get_feedback(state.actuator, &feedback)) {
		return true;
	}

	snapshot->raw_actuator_sample = feedback.sample;
	snapshot->feedback_timestamp_us = feedback.timestamp_us;
	snapshot->source_feedback_count = feedback.feedback_count;
	snapshot->feedback_valid = feedback.valid;
	snapshot->online = feedback.online;

	const bool sample_finite = motion_sample_is_finite(&feedback.sample);
	if (state.direction_valid && state.zero_valid && feedback.valid &&
		sample_finite) {
		const float sign = (float)state.direction;
		snapshot->joint_sample.position_rad =
			sign *
			(feedback.sample.position_rad - state.software_zero_motor_rad);
		snapshot->joint_sample.velocity_rad_s =
			sign * feedback.sample.velocity_rad_s;
		snapshot->joint_sample.torque_nm = sign * feedback.sample.torque_nm;
		snapshot->joint_sample.temperature_c = feedback.sample.temperature_c;
	}

	snapshot->joint_valid = state.mapping_valid && state.direction_valid &&
							state.zero_valid && feedback.valid &&
							feedback.online && sample_finite;
	return true;
}

esp_err_t hip_joint_set_zero_from_current(exo_hip_joint_id_t joint) {
	if (!joint_id_is_valid(joint)) {
		return ESP_ERR_INVALID_ARG;
	}

	exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
	bool mapping_valid = false;
	portENTER_CRITICAL(&s_joint_lock);
	if (s_initialized) {
		actuator = s_joints[joint].actuator;
		mapping_valid = s_joints[joint].mapping_valid;
	}
	portEXIT_CRITICAL(&s_joint_lock);
	if (!mapping_valid || !actuator_id_is_valid(actuator)) {
		return ESP_ERR_INVALID_STATE;
	}

	rs01_feedback_snapshot_t feedback;
	if (!rs01_get_feedback(actuator, &feedback) || !feedback.valid ||
		!feedback.online || !isfinite(feedback.sample.position_rad)) {
		return ESP_ERR_INVALID_STATE;
	}

	portENTER_CRITICAL(&s_joint_lock);
	s_joints[joint].software_zero_motor_rad = feedback.sample.position_rad;
	s_joints[joint].zero_source_timestamp_us = feedback.timestamp_us;
	s_joints[joint].zero_source_feedback_count = feedback.feedback_count;
	s_joints[joint].zero_valid = true;
	portEXIT_CRITICAL(&s_joint_lock);
	return ESP_OK;
}

esp_err_t hip_joint_clear_zero(exo_hip_joint_id_t joint) {
	if (!joint_id_is_valid(joint)) {
		return ESP_ERR_INVALID_ARG;
	}

	portENTER_CRITICAL(&s_joint_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_joint_lock);
		return ESP_ERR_INVALID_STATE;
	}
	s_joints[joint].software_zero_motor_rad = 0.0f;
	s_joints[joint].zero_source_timestamp_us = 0U;
	s_joints[joint].zero_source_feedback_count = 0U;
	s_joints[joint].zero_valid = false;
	portEXIT_CRITICAL(&s_joint_lock);
	return ESP_OK;
}

esp_err_t hip_joint_to_actuator_position(exo_hip_joint_id_t joint,
										 float joint_position_rad,
										 exo_actuator_id_t *actuator,
										 float *actuator_position_rad) {
	if (actuator == NULL || actuator_position_rad == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	*actuator = EXO_ACTUATOR_COUNT;
	*actuator_position_rad = 0.0f;
	if (!joint_id_is_valid(joint) || !isfinite(joint_position_rad)) {
		return ESP_ERR_INVALID_ARG;
	}

	hip_joint_t state;
	portENTER_CRITICAL(&s_joint_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_joint_lock);
		return ESP_ERR_INVALID_STATE;
	}
	state = s_joints[joint];
	portEXIT_CRITICAL(&s_joint_lock);

	if (!state.mapping_valid || !actuator_id_is_valid(state.actuator) ||
		!state.direction_valid || !direction_is_valid(state.direction) ||
		!state.zero_valid) {
		return ESP_ERR_INVALID_STATE;
	}

	const float motor_position_rad =
		state.software_zero_motor_rad +
		(float)state.direction * joint_position_rad;
	if (!isfinite(motor_position_rad)) {
		return ESP_ERR_INVALID_ARG;
	}

	*actuator = state.actuator;
	*actuator_position_rad = motor_position_rad;
	return ESP_OK;
}
