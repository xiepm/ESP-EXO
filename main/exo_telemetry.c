#include "exo_telemetry.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include "exo_types.h"
#include "freertos/FreeRTOS.h"

/* 环形队列和流配置是一个不可分割的共享状态，临界区内不做浮点计算。 */
typedef struct {
	bool initialized;
	bool streaming;
	uint8_t rate_hz;
	uint8_t source_countdown;
	uint16_t channel_mask;
	uint16_t stream_sequence;
	uint32_t next_sample_sequence;
	uint32_t last_sample_sequence;
	uint32_t notify_success_count;
	uint32_t dropped_sample_count;
	uint8_t read_index;
	uint8_t write_index;
	uint8_t count;
	exo_telemetry_sample_t ring[EXO_TELEMETRY_RING_CAPACITY];
} exo_telemetry_context_t;

static exo_telemetry_context_t s_telemetry;
static portMUX_TYPE s_telemetry_lock = portMUX_INITIALIZER_UNLOCKED;

_Static_assert(EXO_TELEMETRY_RING_CAPACITY > 0U &&
				   EXO_TELEMETRY_RING_CAPACITY <= UINT8_MAX,
			   "telemetry ring capacity must fit uint8_t indices");
_Static_assert((EXO_TELEMETRY_SOURCE_FREQUENCY_HZ % 5U) == 0U &&
				   (EXO_TELEMETRY_SOURCE_FREQUENCY_HZ % 10U) == 0U &&
				   (EXO_TELEMETRY_SOURCE_FREQUENCY_HZ % 20U) == 0U,
			   "telemetry rates must divide the 100 Hz producer");

static bool rate_is_valid(uint8_t rate_hz) {
	return rate_hz == 5U || rate_hz == 10U || rate_hz == 20U;
}

static bool channel_mask_is_valid(uint16_t channel_mask) {
	return channel_mask != 0U &&
		   (channel_mask & ~EXO_TELEMETRY_CHANNEL_MASK_ALL) == 0U;
}

/* 无效有符号值保留 INT16_MIN，所以合法量化下界从 -32767 开始。 */
static bool quantize_signed(float value, float scale, int16_t *output) {
	if (output == NULL || !isfinite(value)) {
		return false;
	}
	const float scaled = value * scale;
	if (!isfinite(scaled) || scaled < (float)(INT16_MIN + 1) ||
		scaled > (float)INT16_MAX) {
		return false;
	}
	*output = (int16_t)lroundf(scaled);
	return true;
}

static bool quantize_unit_interval(float value, uint16_t maximum,
								   uint16_t *output) {
	if (output == NULL || !isfinite(value) || value < 0.0f || value > 1.0f) {
		return false;
	}
	long scaled = lroundf(value * 10000.0f);
	if (scaled < 0L || scaled > 10000L || maximum > 10000U) {
		return false;
	}
	if (scaled > (long)maximum) {
		scaled = maximum;
	}
	*output = (uint16_t)scaled;
	return true;
}

static void set_pair_validity(exo_telemetry_sample_t *sample, uint16_t channel,
							  bool left_valid, bool right_valid) {
	if (left_valid) {
		sample->left_valid_channel_mask |= channel;
	}
	if (right_valid) {
		sample->right_valid_channel_mask |= channel;
	}
}

/* 浮点到定点的转换故意在临界区外完成，保证 100 Hz 任务关中断时间有界。 */
static void build_fixed_sample(const exo_telemetry_input_t *input,
							   exo_telemetry_sample_t *sample) {
	memset(sample, 0, sizeof(*sample));
	sample->left_position_mrad = INT16_MIN;
	sample->right_position_mrad = INT16_MIN;
	sample->left_velocity_x100_rad_s = INT16_MIN;
	sample->right_velocity_x100_rad_s = INT16_MIN;
	sample->left_motor_torque_mnm = INT16_MIN;
	sample->right_motor_torque_mnm = INT16_MIN;
	sample->left_phase_x10000 = UINT16_MAX;
	sample->right_phase_x10000 = UINT16_MAX;
	sample->left_suggestion_mnm = INT16_MIN;
	sample->right_suggestion_mnm = INT16_MIN;
	sample->confidence_x10000 = UINT16_MAX;
	sample->left_applied_mnm = INT16_MIN;
	sample->right_applied_mnm = INT16_MIN;
	if (input->timestamp_us > 0) {
		sample->timestamp_ms =
			(uint32_t)((uint64_t)input->timestamp_us / 1000U);
	}
	sample->system_state = input->system_state <= 7U ? input->system_state : 7U;
	sample->sample_flags =
		input->sample_flags & EXO_TELEMETRY_SAMPLE_INPUT_FLAGS_MASK;
	sample->activation_epoch = input->activation_epoch;
	sample->final_command = input->final_command;
	sample->final_left_valid = input->final_left_valid;
	sample->final_right_valid = input->final_right_valid;

	const bool left_position_valid =
		input->joint_position_deg.left_valid &&
		quantize_signed(exo_degrees_to_radians(input->joint_position_deg.left),
						1000.0f, &sample->left_position_mrad);
	const bool right_position_valid =
		input->joint_position_deg.right_valid &&
		quantize_signed(exo_degrees_to_radians(input->joint_position_deg.right),
						1000.0f, &sample->right_position_mrad);
	set_pair_validity(sample, EXO_TELEMETRY_CHANNEL_JOINT_POSITION,
					  left_position_valid, right_position_valid);

	const bool left_velocity_valid =
		input->joint_velocity_deg_s.left_valid &&
		quantize_signed(
			exo_degrees_to_radians(input->joint_velocity_deg_s.left), 100.0f,
			&sample->left_velocity_x100_rad_s);
	const bool right_velocity_valid =
		input->joint_velocity_deg_s.right_valid &&
		quantize_signed(
			exo_degrees_to_radians(input->joint_velocity_deg_s.right), 100.0f,
			&sample->right_velocity_x100_rad_s);
	set_pair_validity(sample, EXO_TELEMETRY_CHANNEL_JOINT_VELOCITY,
					  left_velocity_valid, right_velocity_valid);

	const bool left_torque_valid =
		input->motor_torque_nm.left_valid &&
		quantize_signed(input->motor_torque_nm.left, 1000.0f,
						&sample->left_motor_torque_mnm);
	const bool right_torque_valid =
		input->motor_torque_nm.right_valid &&
		quantize_signed(input->motor_torque_nm.right, 1000.0f,
						&sample->right_motor_torque_mnm);
	set_pair_validity(sample, EXO_TELEMETRY_CHANNEL_MOTOR_TORQUE,
					  left_torque_valid, right_torque_valid);

	const bool left_phase_valid =
		input->gait_phase_01.left_valid &&
		quantize_unit_interval(input->gait_phase_01.left, 9999U,
							   &sample->left_phase_x10000);
	const bool right_phase_valid =
		input->gait_phase_01.right_valid &&
		quantize_unit_interval(input->gait_phase_01.right, 9999U,
							   &sample->right_phase_x10000);
	set_pair_validity(sample, EXO_TELEMETRY_CHANNEL_GAIT_PHASE,
					  left_phase_valid, right_phase_valid);

	const bool left_suggestion_valid =
		input->assist_suggestion_nm.left_valid &&
		quantize_signed(input->assist_suggestion_nm.left, 1000.0f,
						&sample->left_suggestion_mnm);
	const bool right_suggestion_valid =
		input->assist_suggestion_nm.right_valid &&
		quantize_signed(input->assist_suggestion_nm.right, 1000.0f,
						&sample->right_suggestion_mnm);
	set_pair_validity(sample, EXO_TELEMETRY_CHANNEL_ASSIST_SUGGESTION,
					  left_suggestion_valid, right_suggestion_valid);

	const bool confidence_valid =
		input->gait_confidence_valid &&
		quantize_unit_interval(input->gait_confidence_01, 10000U,
							   &sample->confidence_x10000);
	/* 置信度是全局量，线上左右两槽填入同一值，便于通用 pair 解析。 */
	set_pair_validity(sample, EXO_TELEMETRY_CHANNEL_GAIT_CONFIDENCE,
					  confidence_valid, confidence_valid);

	const bool left_applied_valid =
		input->applied_assist_nm.left_valid &&
		quantize_signed(input->applied_assist_nm.left, 1000.0f,
						&sample->left_applied_mnm);
	const bool right_applied_valid =
		input->applied_assist_nm.right_valid &&
		quantize_signed(input->applied_assist_nm.right, 1000.0f,
						&sample->right_applied_mnm);
	set_pair_validity(sample, EXO_TELEMETRY_CHANNEL_APPLIED_TORQUE,
					  left_applied_valid, right_applied_valid);
}

bool exo_telemetry_init(void) {
	portENTER_CRITICAL(&s_telemetry_lock);
	if (!s_telemetry.initialized) {
		memset(&s_telemetry, 0, sizeof(s_telemetry));
		s_telemetry.initialized = true;
		s_telemetry.rate_hz = EXO_TELEMETRY_DEFAULT_RATE_HZ;
		s_telemetry.channel_mask = EXO_TELEMETRY_DEFAULT_CHANNEL_MASK;
	}
	portEXIT_CRITICAL(&s_telemetry_lock);
	return true;
}

bool exo_telemetry_start(uint8_t rate_hz, uint16_t channel_mask,
						 uint16_t *stream_sequence) {
	if (!rate_is_valid(rate_hz) || !channel_mask_is_valid(channel_mask)) {
		return false;
	}

	portENTER_CRITICAL(&s_telemetry_lock);
	if (!s_telemetry.initialized) {
		portEXIT_CRITICAL(&s_telemetry_lock);
		return false;
	}
	++s_telemetry.stream_sequence;
	if (s_telemetry.stream_sequence == 0U) {
		++s_telemetry.stream_sequence;
	}
	s_telemetry.streaming = true;
	s_telemetry.rate_hz = rate_hz;
	s_telemetry.channel_mask = channel_mask;
	s_telemetry.source_countdown = 0U;
	s_telemetry.next_sample_sequence = 1U;
	s_telemetry.last_sample_sequence = 0U;
	s_telemetry.notify_success_count = 0U;
	s_telemetry.dropped_sample_count = 0U;
	s_telemetry.read_index = 0U;
	s_telemetry.write_index = 0U;
	s_telemetry.count = 0U;
	if (stream_sequence != NULL) {
		*stream_sequence = s_telemetry.stream_sequence;
	}
	portEXIT_CRITICAL(&s_telemetry_lock);
	return true;
}

bool exo_telemetry_stop(void) {
	portENTER_CRITICAL(&s_telemetry_lock);
	const bool was_streaming = s_telemetry.initialized && s_telemetry.streaming;
	if (was_streaming) {
		s_telemetry.streaming = false;
		s_telemetry.dropped_sample_count += s_telemetry.count;
		s_telemetry.read_index = 0U;
		s_telemetry.write_index = 0U;
		s_telemetry.count = 0U;
	}
	portEXIT_CRITICAL(&s_telemetry_lock);
	return was_streaming;
}

void exo_telemetry_push(const exo_telemetry_input_t *input) {
	if (input == NULL) {
		return;
	}

	uint16_t stream_sequence;
	uint32_t sample_sequence;
	portENTER_CRITICAL(&s_telemetry_lock);
	if (!s_telemetry.initialized || !s_telemetry.streaming) {
		portEXIT_CRITICAL(&s_telemetry_lock);
		return;
	}
	if (s_telemetry.source_countdown > 0U) {
		--s_telemetry.source_countdown;
		portEXIT_CRITICAL(&s_telemetry_lock);
		return;
	}
	s_telemetry.source_countdown =
		(uint8_t)(EXO_TELEMETRY_SOURCE_FREQUENCY_HZ / s_telemetry.rate_hz - 1U);
	stream_sequence = s_telemetry.stream_sequence;
	sample_sequence = s_telemetry.next_sample_sequence++;
	/* 0 保留为“尚无样本”；六年级连续运行回绕时自动跳过。 */
	if (sample_sequence == 0U) {
		sample_sequence = s_telemetry.next_sample_sequence++;
	}
	portEXIT_CRITICAL(&s_telemetry_lock);

	exo_telemetry_sample_t sample;
	build_fixed_sample(input, &sample);
	sample.stream_sequence = stream_sequence;
	sample.sample_sequence = sample_sequence;

	portENTER_CRITICAL(&s_telemetry_lock);
	/* START/STOP 可在量化期间发生；旧 stream 样本不得混入新队列。 */
	if (!s_telemetry.streaming ||
		s_telemetry.stream_sequence != stream_sequence) {
		portEXIT_CRITICAL(&s_telemetry_lock);
		return;
	}
	if (s_telemetry.count == EXO_TELEMETRY_RING_CAPACITY) {
		s_telemetry.read_index = (uint8_t)((s_telemetry.read_index + 1U) %
										   EXO_TELEMETRY_RING_CAPACITY);
		--s_telemetry.count;
		++s_telemetry.dropped_sample_count;
	}
	s_telemetry.ring[s_telemetry.write_index] = sample;
	s_telemetry.write_index =
		(uint8_t)((s_telemetry.write_index + 1U) % EXO_TELEMETRY_RING_CAPACITY);
	++s_telemetry.count;
	s_telemetry.last_sample_sequence = sample_sequence;
	portEXIT_CRITICAL(&s_telemetry_lock);
}

bool exo_telemetry_pop(exo_telemetry_sample_t *sample) {
	if (sample == NULL) {
		return false;
	}
	portENTER_CRITICAL(&s_telemetry_lock);
	if (!s_telemetry.initialized || !s_telemetry.streaming ||
		s_telemetry.count == 0U) {
		portEXIT_CRITICAL(&s_telemetry_lock);
		return false;
	}
	*sample = s_telemetry.ring[s_telemetry.read_index];
	s_telemetry.read_index =
		(uint8_t)((s_telemetry.read_index + 1U) % EXO_TELEMETRY_RING_CAPACITY);
	--s_telemetry.count;
	portEXIT_CRITICAL(&s_telemetry_lock);
	return true;
}

void exo_telemetry_note_transport_drop(uint16_t stream_sequence) {
	portENTER_CRITICAL(&s_telemetry_lock);
	if (s_telemetry.initialized &&
		s_telemetry.stream_sequence == stream_sequence) {
		++s_telemetry.dropped_sample_count;
	}
	portEXIT_CRITICAL(&s_telemetry_lock);
}

void exo_telemetry_note_notify_success(uint16_t stream_sequence) {
	portENTER_CRITICAL(&s_telemetry_lock);
	if (s_telemetry.initialized &&
		s_telemetry.stream_sequence == stream_sequence) {
		++s_telemetry.notify_success_count;
	}
	portEXIT_CRITICAL(&s_telemetry_lock);
}

bool exo_telemetry_get_status(exo_telemetry_status_t *status) {
	if (status == NULL) {
		return false;
	}
	portENTER_CRITICAL(&s_telemetry_lock);
	*status = (exo_telemetry_status_t){
		.initialized = s_telemetry.initialized,
		.streaming = s_telemetry.streaming,
		.rate_hz = s_telemetry.rate_hz,
		.channel_mask = s_telemetry.channel_mask,
		.stream_sequence = s_telemetry.stream_sequence,
		.last_sample_sequence = s_telemetry.last_sample_sequence,
		.notify_success_count = s_telemetry.notify_success_count,
		.dropped_sample_count = s_telemetry.dropped_sample_count,
		.queued_samples = s_telemetry.count,
		.queue_capacity = EXO_TELEMETRY_RING_CAPACITY,
	};
	portEXIT_CRITICAL(&s_telemetry_lock);
	return status->initialized;
}

bool exo_telemetry_get_channel_values(const exo_telemetry_sample_t *sample,
									  uint16_t channel, uint16_t *left_value,
									  uint16_t *right_value,
									  uint8_t *valid_flags) {
	if (sample == NULL || left_value == NULL || right_value == NULL ||
		valid_flags == NULL || !channel_mask_is_valid(channel) ||
		(channel & (channel - 1U)) != 0U) {
		return false;
	}
	*valid_flags = 0U;
	if ((sample->left_valid_channel_mask & channel) != 0U) {
		*valid_flags |= EXO_TELEMETRY_VALUE_LEFT_VALID;
	}
	if ((sample->right_valid_channel_mask & channel) != 0U) {
		*valid_flags |= EXO_TELEMETRY_VALUE_RIGHT_VALID;
	}

	switch (channel) {
	case EXO_TELEMETRY_CHANNEL_JOINT_POSITION:
		*left_value = (uint16_t)sample->left_position_mrad;
		*right_value = (uint16_t)sample->right_position_mrad;
		return true;
	case EXO_TELEMETRY_CHANNEL_JOINT_VELOCITY:
		*left_value = (uint16_t)sample->left_velocity_x100_rad_s;
		*right_value = (uint16_t)sample->right_velocity_x100_rad_s;
		return true;
	case EXO_TELEMETRY_CHANNEL_MOTOR_TORQUE:
		*left_value = (uint16_t)sample->left_motor_torque_mnm;
		*right_value = (uint16_t)sample->right_motor_torque_mnm;
		return true;
	case EXO_TELEMETRY_CHANNEL_GAIT_PHASE:
		*left_value = sample->left_phase_x10000;
		*right_value = sample->right_phase_x10000;
		return true;
	case EXO_TELEMETRY_CHANNEL_ASSIST_SUGGESTION:
		*left_value = (uint16_t)sample->left_suggestion_mnm;
		*right_value = (uint16_t)sample->right_suggestion_mnm;
		return true;
	case EXO_TELEMETRY_CHANNEL_GAIT_CONFIDENCE:
		*left_value = sample->confidence_x10000;
		*right_value = sample->confidence_x10000;
		return true;
	case EXO_TELEMETRY_CHANNEL_APPLIED_TORQUE:
		*left_value = (uint16_t)sample->left_applied_mnm;
		*right_value = (uint16_t)sample->right_applied_mnm;
		return true;
	default:
		return false;
	}
}
