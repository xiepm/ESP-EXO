#include "exo_sensor_snapshot.h"

#include <limits.h>
#include <math.h>
#include <string.h>

/* 把64位时间差安全压缩到诊断字段；UINT32_MAX同时作为“不可计算”的哨兵。 */
static uint32_t saturating_age_us(int64_t captured_us, uint64_t source_us,
				  bool *time_valid) {
	if (time_valid == NULL) {
		return UINT32_MAX;
	}
	if (captured_us < 0 || source_us == 0U ||
		(uint64_t)captured_us < source_us) {
		*time_valid = false;
		return UINT32_MAX;
	}
	const uint64_t age = (uint64_t)captured_us - source_us;
	return age > UINT32_MAX ? UINT32_MAX : (uint32_t)age;
}

static uint32_t saturating_difference_us(uint64_t a, uint64_t b) {
	const uint64_t difference = a >= b ? a - b : b - a;
	return difference > UINT32_MAX ? UINT32_MAX : (uint32_t)difference;
}

/* 四个浮点反馈必须全部有限；NaN/Inf 绝不能进入滤波器的历史状态。 */
static bool joint_sample_is_finite(const exo_joint_sample_t *sample) {
	return sample != NULL && isfinite(sample->position_deg) &&
		   isfinite(sample->velocity_deg_s) && isfinite(sample->torque_nm) &&
		   isfinite(sample->temperature_c);
}

/*
 * 坐标签名描述“同一个关节角零点是否仍然成立”。反馈计数正常递增不构成坐标复位；
 * 连续位置 epoch、RAM 零位来源或零位有效性变化则必须让相位估计器重新学习。
 */
static bool coordinate_signature_changed(
	const exo_sensor_joint_snapshot_t *previous,
	const exo_sensor_joint_snapshot_t *current) {
	return previous->position_epoch != current->position_epoch ||
		   previous->zero_source_timestamp_us !=
			   current->zero_source_timestamp_us ||
		   previous->zero_source_feedback_count !=
			   current->zero_source_feedback_count ||
		   previous->zero_source_position_epoch !=
			   current->zero_source_position_epoch ||
		   previous->zero_valid != current->zero_valid;
}

static bool source_rolled_back(const exo_sensor_joint_snapshot_t *previous,
				   const exo_sensor_joint_snapshot_t *current) {
	return current->source_feedback_count < previous->source_feedback_count ||
		   current->source_timestamp_us < previous->source_timestamp_us;
}

/* 序号和epoch都把0保留为“尚无数据”。 */
static uint32_t next_nonzero(uint32_t value) {
	++value;
	return value == 0U ? 1U : value;
}

bool exo_sensor_snapshot_builder_init(
	exo_sensor_snapshot_builder_t *builder,
	const exo_sensor_snapshot_config_t *config) {
	if (builder == NULL || config == NULL ||
		config->feedback_freshness_us == 0U ||
		config->pair_skew_warning_us == 0U ||
		(config->available_sensor_mask &
		 EXO_SENSOR_CURRENT_HARDWARE_MASK) == 0U) {
		return false;
	}
	memset(builder, 0, sizeof(*builder));
	builder->config = *config;
	builder->next_sequence = 1U;
	builder->reset_epoch = 1U;
	builder->initialized = true;
	return true;
}

/* 处理一侧输入并返回本侧在当前周期有效的传感器位。 */
static uint32_t prepare_joint(
	const exo_sensor_snapshot_config_t *config, int64_t captured_us,
	const exo_sensor_joint_snapshot_t *input, bool is_left,
	exo_sensor_joint_snapshot_t *output, uint32_t *quality_flags) {
	const uint32_t kinematics_mask =
		is_left ? EXO_SENSOR_LEFT_HIP_KINEMATICS
				: EXO_SENSOR_RIGHT_HIP_KINEMATICS;
	const uint32_t torque_mask =
		is_left ? EXO_SENSOR_LEFT_MOTOR_TORQUE_ESTIMATE
				: EXO_SENSOR_RIGHT_MOTOR_TORQUE_ESTIMATE;
	const uint32_t unavailable_flag =
		is_left ? EXO_SENSOR_QUALITY_LEFT_UNAVAILABLE
				: EXO_SENSOR_QUALITY_RIGHT_UNAVAILABLE;
	const uint32_t stale_flag = is_left ? EXO_SENSOR_QUALITY_LEFT_STALE
										 : EXO_SENSOR_QUALITY_RIGHT_STALE;

	memset(output, 0, sizeof(*output));
	output->age_us = UINT32_MAX;
	if (input == NULL || !input->snapshot_available) {
		*quality_flags |= unavailable_flag;
		return 0U;
	}

	*output = *input;
	bool time_valid = true;
	output->age_us =
		saturating_age_us(captured_us, input->source_timestamp_us, &time_valid);
	if (!time_valid) {
		*quality_flags |= EXO_SENSOR_QUALITY_TIME_INVALID;
	}
	const bool finite = joint_sample_is_finite(&input->joint_sample);
	if (!finite) {
		*quality_flags |= EXO_SENSOR_QUALITY_NONFINITE_SAMPLE;
	}
	const bool fresh = output->age_us <= config->feedback_freshness_us;
	if (!fresh) {
		*quality_flags |= stale_flag;
	}
	const bool valid = input->joint_valid && input->feedback_valid &&
					   input->online && finite && fresh && time_valid;
	/* 即使调用者填入了样本，也不能宣称配置中未声明存在的硬件能力有效。 */
	return valid ? ((kinematics_mask | torque_mask) &
					config->available_sensor_mask)
			 : 0U;
}

bool exo_sensor_snapshot_build(exo_sensor_snapshot_builder_t *builder,
				   int64_t captured_us,
				   const exo_sensor_joint_snapshot_t *left,
				   const exo_sensor_joint_snapshot_t *right,
				   exo_sensor_snapshot_t *output) {
	if (builder == NULL || output == NULL || !builder->initialized) {
		return false;
	}

	memset(output, 0, sizeof(*output));
	output->sequence = builder->next_sequence;
	builder->next_sequence = next_nonzero(builder->next_sequence);
	output->captured_us = captured_us;
	output->available_sensor_mask = builder->config.available_sensor_mask;

	/*
	 * captured_us属于统一100 Hz时钟；它停滞/回退才表示整个时间基准异常。
	 * 某侧反馈尚未到达时source_timestamp_us为0，只应让该侧无效，不能让全局
	 * reset_epoch在每个控制周期都自增。
	 */
	const bool captured_time_invalid =
		captured_us <= 0 ||
		(builder->last_captured_us > 0 &&
		 captured_us <= builder->last_captured_us);
	if (captured_time_invalid) {
		output->quality_flags |= EXO_SENSOR_QUALITY_TIME_INVALID;
	}

	output->valid_sensor_mask |= prepare_joint(
		&builder->config, captured_us, left, true, &output->left,
		&output->quality_flags);
	output->valid_sensor_mask |= prepare_joint(
		&builder->config, captured_us, right, false, &output->right,
		&output->quality_flags);

	bool coordinate_reset = false;
	bool source_rollback_detected = false;
	if (left != NULL && left->snapshot_available) {
		if (builder->have_left_signature) {
			coordinate_reset |=
				coordinate_signature_changed(&builder->previous_left, left);
			source_rollback_detected |=
				source_rolled_back(&builder->previous_left, left);
		}
		builder->previous_left = *left;
		builder->have_left_signature = true;
	}
	if (right != NULL && right->snapshot_available) {
		if (builder->have_right_signature) {
			coordinate_reset |=
				coordinate_signature_changed(&builder->previous_right, right);
			source_rollback_detected |=
				source_rolled_back(&builder->previous_right, right);
		}
		builder->previous_right = *right;
		builder->have_right_signature = true;
	}
	/*
	 * 坐标、反馈源回退必须立即复位算法；统一采集时钟异常采用“进入异常”的边沿
	 * 触发一次。反馈缺失/时间戳为0虽然也置TIME_INVALID，却不会制造复位风暴。
	 */
	const bool captured_time_invalid_edge =
		captured_time_invalid && !builder->captured_time_invalid_active;
	if (coordinate_reset || source_rollback_detected ||
		captured_time_invalid_edge) {
		builder->reset_epoch = next_nonzero(builder->reset_epoch);
	}
	if (coordinate_reset) {
		output->quality_flags |= EXO_SENSOR_QUALITY_COORDINATE_RESET;
	}
	if (source_rollback_detected) {
		output->quality_flags |= EXO_SENSOR_QUALITY_SOURCE_ROLLBACK;
	}
	output->reset_epoch = builder->reset_epoch;

	if (left != NULL && right != NULL && left->snapshot_available &&
		right->snapshot_available && left->source_timestamp_us != 0U &&
		right->source_timestamp_us != 0U) {
		output->pair_skew_us = saturating_difference_us(
			left->source_timestamp_us, right->source_timestamp_us);
		if (output->pair_skew_us > builder->config.pair_skew_warning_us) {
			output->quality_flags |= EXO_SENSOR_QUALITY_PAIR_SKEW_HIGH;
		}
	}

	builder->last_captured_us = captured_us;
	builder->captured_time_invalid_active = captured_time_invalid;
	return true;
}
