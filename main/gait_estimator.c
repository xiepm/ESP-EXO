#include "gait_estimator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define EXO_GAIT_PHASE_MAX 0.999999f

static bool config_is_valid(const exo_gait_estimator_config_t *config) {
	return config != NULL && isfinite(config->filter_alpha) &&
		   config->filter_alpha > 0.0f && config->filter_alpha <= 1.0f &&
		   isfinite(config->trend_velocity_threshold_deg_s) &&
		   config->trend_velocity_threshold_deg_s > 0.0f &&
		   isfinite(config->motion_velocity_threshold_deg_s) &&
		   config->motion_velocity_threshold_deg_s > 0.0f &&
		   isfinite(config->minimum_excursion_deg) &&
		   config->minimum_excursion_deg > 0.0f &&
		   isfinite(config->minimum_period_s) &&
		   isfinite(config->maximum_period_s) &&
		   config->minimum_period_s > 0.0f &&
		   config->maximum_period_s > config->minimum_period_s &&
		   config->maximum_period_s <= 10.0f &&
		   isfinite(config->minimum_event_interval_s) &&
		   config->minimum_event_interval_s > 0.0f &&
		   config->minimum_event_interval_s < config->minimum_period_s &&
		   isfinite(config->period_ewma_alpha) &&
		   config->period_ewma_alpha > 0.0f &&
		   config->period_ewma_alpha <= 1.0f &&
		   isfinite(config->unstable_period_fraction) &&
		   config->unstable_period_fraction > 0.0f &&
		   config->unstable_period_fraction <= 1.0f &&
		   isfinite(config->bilateral_period_tolerance_fraction) &&
		   config->bilateral_period_tolerance_fraction > 0.0f &&
		   config->bilateral_period_tolerance_fraction <= 1.0f &&
		   isfinite(config->minimum_bilateral_phase_separation) &&
		   config->minimum_bilateral_phase_separation > 0.0f &&
		   config->minimum_bilateral_phase_separation < 0.5f &&
		   config->standing_timeout_us > 0U &&
		   config->standing_timeout_us <= 60000000U &&
		   config->phase_timeout_margin_us <= 10000000U &&
		   config->lock_period_count > 0U;
}

static uint32_t next_nonzero(uint32_t value) {
	++value;
	return value == 0U ? 1U : value;
}

/* 清除某侧所有周期历史；滤波器也清空，避免旧零位污染新坐标系。 */
static void clear_side(exo_gait_side_state_t *side) {
	memset(side, 0, sizeof(*side));
}

/* 进入稳定站立时保留当前位置滤波值，但丢弃所有“上一周期”记忆。 */
static void clear_side_cycle_history(exo_gait_side_state_t *side) {
	const bool filter_initialized = side->filter_initialized;
	const float position = side->filtered_position_deg;
	const float velocity = side->filtered_velocity_deg_s;
	const uint64_t source_timestamp_us = side->last_source_timestamp_us;
	const uint32_t source_feedback_count = side->last_source_feedback_count;
	clear_side(side);
	side->filter_initialized = filter_initialized;
	side->filtered_position_deg = position;
	side->filtered_velocity_deg_s = velocity;
	side->interval_min_deg = position;
	side->interval_max_deg = position;
	side->last_source_timestamp_us = source_timestamp_us;
	side->last_source_feedback_count = source_feedback_count;
}

bool exo_gait_estimator_init(exo_gait_estimator_t *estimator,
				 const exo_gait_estimator_config_t *config) {
	if (estimator == NULL || !config_is_valid(config)) {
		return false;
	}
	memset(estimator, 0, sizeof(*estimator));
	estimator->config = *config;
	estimator->state = EXO_GAIT_STATE_RESET;
	estimator->initialized = true;
	return true;
}

void exo_gait_estimator_reset(exo_gait_estimator_t *estimator,
				  uint32_t source_reset_epoch) {
	if (estimator == NULL || !estimator->initialized) {
		return;
	}
	const exo_gait_estimator_config_t config = estimator->config;
	const uint32_t output_sequence = estimator->output_sequence;
	memset(estimator, 0, sizeof(*estimator));
	estimator->config = config;
	estimator->output_sequence = output_sequence;
	estimator->last_reset_epoch = source_reset_epoch;
	estimator->state = EXO_GAIT_STATE_RESET;
	estimator->initialized = true;
}

static void initialize_output(const exo_gait_estimator_t *estimator,
				  const exo_sensor_snapshot_t *snapshot,
				  exo_gait_estimate_t *estimate) {
	memset(estimate, 0, sizeof(*estimate));
	estimate->estimator_id = EXO_GAIT_ESTIMATOR_HIP_EVENT_TBE;
	estimate->state = estimator->state;
	estimate->phase_origin = EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK;
	estimate->sequence = estimator->output_sequence;
	estimate->source_snapshot_sequence = snapshot->sequence;
	estimate->timestamp_us = snapshot->captured_us;
	estimate->source_reset_epoch = snapshot->reset_epoch;
}

/*
 * 根据速度滞环更新运动方向：速度位于 +/-threshold 中间时保持旧方向，这样噪声在
 * 零附近来回抖动不会重复制造极值事件。
 */
static int8_t trend_from_velocity(float velocity_deg_s, float threshold,
				  int8_t previous) {
	if (velocity_deg_s >= threshold) {
		return 1;
	}
	if (velocity_deg_s <= -threshold) {
		return -1;
	}
	return previous;
}

typedef struct {
	uint16_t event_flags;
	uint32_t quality_flags;
	bool motion_detected;
} side_update_result_t;

/*
 * 更新一侧滤波和事件状态。人体约定“髋屈曲为正”，因此速度从正转负对应屈曲峰，
 * 从负转正对应伸展峰。这里只报告髋角代理事件，绝不把它命名为足跟着地。
 */
static side_update_result_t update_side(
	exo_gait_side_state_t *side, const exo_gait_estimator_config_t *config,
	const exo_sensor_joint_snapshot_t *joint,
	uint32_t period_out_of_range_flag) {
	side_update_result_t result = {0};
	const float raw_position = joint->joint_sample.position_deg;
	const float raw_velocity = joint->joint_sample.velocity_deg_s;

	/*
	 * TWAI接收和100 Hz控制并不同步。同一反馈可能连续出现在多个控制快照中；若
	 * 对它反复执行低通，就会改变实际截止频率并伪造峰值时刻。只有源时间严格
	 * 前进才消费新观测，事件时间也使用该侧源时间，而非稍后的控制唤醒时间。
	 */
	const bool new_source_sample =
		joint->source_timestamp_us > side->last_source_timestamp_us;
	if (side->filter_initialized && !new_source_sample) {
		result.motion_detected =
			fabsf(side->filtered_velocity_deg_s) >=
			config->motion_velocity_threshold_deg_s;
		return result;
	}
	if (!new_source_sample || joint->source_timestamp_us > INT64_MAX) {
		return result;
	}
	const int64_t sample_time_us = (int64_t)joint->source_timestamp_us;
	side->last_source_timestamp_us = joint->source_timestamp_us;
	side->last_source_feedback_count = joint->source_feedback_count;

	if (!side->filter_initialized) {
		side->filter_initialized = true;
		side->filtered_position_deg = raw_position;
		side->filtered_velocity_deg_s = raw_velocity;
		side->interval_min_deg = raw_position;
		side->interval_max_deg = raw_position;
		side->trend = trend_from_velocity(
			raw_velocity, config->trend_velocity_threshold_deg_s, 0);
		result.motion_detected =
			fabsf(raw_velocity) >= config->motion_velocity_threshold_deg_s;
		return result;
	}

	const float alpha = config->filter_alpha;
	side->filtered_position_deg +=
		alpha * (raw_position - side->filtered_position_deg);
	side->filtered_velocity_deg_s +=
		alpha * (raw_velocity - side->filtered_velocity_deg_s);
	if (!isfinite(side->filtered_position_deg) ||
		!isfinite(side->filtered_velocity_deg_s)) {
		clear_side(side);
		return result;
	}

	result.motion_detected = fabsf(side->filtered_velocity_deg_s) >=
						 config->motion_velocity_threshold_deg_s;
	if (side->filtered_position_deg < side->interval_min_deg) {
		side->interval_min_deg = side->filtered_position_deg;
	}
	if (side->filtered_position_deg > side->interval_max_deg) {
		side->interval_max_deg = side->filtered_position_deg;
	}

	const int8_t new_trend = trend_from_velocity(
		side->filtered_velocity_deg_s,
		config->trend_velocity_threshold_deg_s, side->trend);
	if (side->trend == 0 || new_trend == side->trend) {
		side->trend = new_trend;
		return result;
	}

	const float excursion = side->interval_max_deg - side->interval_min_deg;
	const int64_t since_event_us =
		side->last_event_us > 0 ? sample_time_us - side->last_event_us
								: INT64_MAX;
	const int64_t minimum_event_us =
		(int64_t)(config->minimum_event_interval_s * 1000000.0f);
	if (excursion < config->minimum_excursion_deg ||
		since_event_us < minimum_event_us) {
		/* 只接受新的趋势，不接受噪声事件；区间继续累计以等待真实摆动。 */
		side->trend = new_trend;
		return result;
	}

	const bool flexion_peak = side->trend > 0 && new_trend < 0;
	result.event_flags = flexion_peak
						 ? EXO_GAIT_EVENT_HIP_FLEXION_PEAK
						 : EXO_GAIT_EVENT_HIP_EXTENSION_PEAK;
	side->last_event_us = sample_time_us;
	side->trend = new_trend;
	side->interval_min_deg = side->filtered_position_deg;
	side->interval_max_deg = side->filtered_position_deg;

	if (!flexion_peak) {
		return result;
	}

	if (side->last_origin_us > 0) {
		const float candidate_period_s =
			(float)(sample_time_us - side->last_origin_us) / 1000000.0f;
		if (candidate_period_s >= config->minimum_period_s &&
			candidate_period_s <= config->maximum_period_s) {
			if (side->period_s > 0.0f) {
				side->last_period_error_fraction =
					fabsf(candidate_period_s - side->period_s) /
					side->period_s;
				if (side->last_period_error_fraction >
					config->unstable_period_fraction) {
					result.quality_flags |=
						EXO_GAIT_QUALITY_PERIOD_UNSTABLE;
					/* 锁定要求连续稳定周期；突变周期不能继续累加历史票数。 */
					side->accepted_period_count = 0U;
				}
				side->period_s += config->period_ewma_alpha *
								  (candidate_period_s - side->period_s);
			} else {
				side->period_s = candidate_period_s;
				side->last_period_error_fraction = 0.0f;
			}
			if ((result.quality_flags & EXO_GAIT_QUALITY_PERIOD_UNSTABLE) ==
				0U && side->accepted_period_count < UINT8_MAX) {
				++side->accepted_period_count;
			}
			side->phase_ready =
				side->accepted_period_count >= config->lock_period_count;
			result.event_flags |= EXO_GAIT_EVENT_PHASE_WRAP;
		} else {
			/* 超出人体步行周期范围时从这个事件重新开始，不跨异常间隔外推。 */
			result.quality_flags |= period_out_of_range_flag;
			side->period_s = 0.0f;
			side->last_period_error_fraction = 0.0f;
			side->accepted_period_count = 0U;
			side->phase_ready = false;
		}
	}
	side->last_origin_us = sample_time_us;
	return result;
}

/* 把某侧已经锁定的周期转换成[0,1)相位；等待迟到事件时保持在接近1而不倒退。 */
static bool side_phase(const exo_gait_side_state_t *side,
			   const exo_gait_estimator_config_t *config, int64_t now_us,
			   float *phase, uint32_t *quality_flags,
			   uint32_t timeout_flag) {
	*phase = 0.0f;
	if (!side->phase_ready || side->period_s <= 0.0f ||
		side->last_origin_us <= 0 || now_us < side->last_origin_us) {
		return false;
	}
	const int64_t elapsed_us = now_us - side->last_origin_us;
	const int64_t expected_us = (int64_t)(side->period_s * 1000000.0f);
	if (elapsed_us > expected_us + (int64_t)config->phase_timeout_margin_us) {
		*quality_flags |= timeout_flag;
		return false;
	}
	float phase_01 = (float)elapsed_us / (float)expected_us;
	if (!isfinite(phase_01) || phase_01 < 0.0f) {
		return false;
	}
	if (phase_01 >= 1.0f) {
		phase_01 = EXO_GAIT_PHASE_MAX;
	}
	*phase = phase_01;
	return true;
}

static float side_confidence(const exo_gait_side_state_t *side,
				 const exo_gait_estimator_config_t *config,
				 bool phase_valid) {
	if (!phase_valid) {
		return 0.0f;
	}
	const float stability = 1.0f - fminf(
		1.0f, side->last_period_error_fraction /
				  config->unstable_period_fraction);
	const float lock_fraction = fminf(
		1.0f, (float)side->accepted_period_count /
				  (float)(config->lock_period_count + 1U));
	return fminf(1.0f, 0.55f + 0.25f * stability + 0.20f * lock_fraction);
}

bool exo_gait_estimator_update(exo_gait_estimator_t *estimator,
				   const exo_sensor_snapshot_t *snapshot,
				   exo_gait_estimate_t *estimate) {
	if (estimator == NULL || snapshot == NULL || estimate == NULL ||
		!estimator->initialized) {
		return false;
	}

	estimator->output_sequence = next_nonzero(estimator->output_sequence);
	initialize_output(estimator, snapshot, estimate);

	const bool time_or_sequence_rollback =
		(estimator->last_timestamp_us > 0 &&
		 snapshot->captured_us <= estimator->last_timestamp_us) ||
		(estimator->last_snapshot_sequence != 0U &&
		 snapshot->sequence <= estimator->last_snapshot_sequence);
	const bool coordinate_reset = estimator->last_reset_epoch != 0U &&
						  snapshot->reset_epoch != estimator->last_reset_epoch;
	if (time_or_sequence_rollback || coordinate_reset) {
		exo_gait_estimator_reset(estimator, snapshot->reset_epoch);
		estimator->output_sequence = estimate->sequence;
		initialize_output(estimator, snapshot, estimate);
		if (time_or_sequence_rollback) {
			estimate->quality_flags |=
				EXO_GAIT_QUALITY_TIME_OR_SEQUENCE_ROLLBACK;
		}
		if (coordinate_reset) {
			estimate->quality_flags |= EXO_GAIT_QUALITY_COORDINATE_RESET;
		}
	}

	const bool left_valid =
		(snapshot->valid_sensor_mask & EXO_SENSOR_LEFT_HIP_KINEMATICS) != 0U;
	const bool right_valid =
		(snapshot->valid_sensor_mask & EXO_SENSOR_RIGHT_HIP_KINEMATICS) != 0U;
	if (!left_valid) {
		estimate->quality_flags |= EXO_GAIT_QUALITY_LEFT_INPUT_INVALID;
		clear_side(&estimator->left);
	}
	if (!right_valid) {
		estimate->quality_flags |= EXO_GAIT_QUALITY_RIGHT_INPUT_INVALID;
		clear_side(&estimator->right);
	}
	if ((snapshot->quality_flags & EXO_SENSOR_QUALITY_PAIR_SKEW_HIGH) != 0U) {
		estimate->quality_flags |= EXO_GAIT_QUALITY_PAIR_SKEW_HIGH;
	}

	side_update_result_t left_result = {0};
	side_update_result_t right_result = {0};
	if (left_valid) {
		left_result = update_side(
			&estimator->left, &estimator->config, &snapshot->left,
			EXO_GAIT_QUALITY_LEFT_PERIOD_OUT_OF_RANGE);
		estimate->left_event_flags = left_result.event_flags;
		estimate->quality_flags |= left_result.quality_flags;
	}
	if (right_valid) {
		right_result = update_side(
			&estimator->right, &estimator->config, &snapshot->right,
			EXO_GAIT_QUALITY_RIGHT_PERIOD_OUT_OF_RANGE);
		estimate->right_event_flags = right_result.event_flags;
		estimate->quality_flags |= right_result.quality_flags;
	}
	/* 不稳定标志在下一次稳定原点事件到来前持续可见，而不是只闪现一个10 ms周期。 */
	if (estimator->left.last_period_error_fraction >
			estimator->config.unstable_period_fraction ||
		estimator->right.last_period_error_fraction >
			estimator->config.unstable_period_fraction) {
		estimate->quality_flags |= EXO_GAIT_QUALITY_PERIOD_UNSTABLE;
	}

	estimate->motion_detected =
		left_result.motion_detected || right_result.motion_detected;
	if (estimate->motion_detected) {
		estimator->last_motion_us = snapshot->captured_us;
	} else if (estimator->last_motion_us == 0) {
		estimator->last_motion_us = snapshot->captured_us;
	}

	const bool standing = left_valid && right_valid &&
		(snapshot->captured_us - estimator->last_motion_us) >=
			(int64_t)estimator->config.standing_timeout_us;
	if (standing) {
		if (estimator->state != EXO_GAIT_STATE_STANDING) {
			clear_side_cycle_history(&estimator->left);
			clear_side_cycle_history(&estimator->right);
		}
		estimator->state = EXO_GAIT_STATE_STANDING;
		estimate->quality_flags |= EXO_GAIT_QUALITY_LOW_MOTION;
	} else if (!left_valid || !right_valid) {
		estimator->state = EXO_GAIT_STATE_DEGRADED;
	} else {
		bool left_phase_valid = side_phase(
			&estimator->left, &estimator->config, snapshot->captured_us,
			&estimate->left_phase_01, &estimate->quality_flags,
			EXO_GAIT_QUALITY_LEFT_PHASE_TIMEOUT);
		bool right_phase_valid = side_phase(
			&estimator->right, &estimator->config, snapshot->captured_us,
			&estimate->right_phase_01, &estimate->quality_flags,
			EXO_GAIT_QUALITY_RIGHT_PHASE_TIMEOUT);

		if (left_phase_valid) {
			estimate->valid_flags |= EXO_GAIT_VALID_LEFT_PHASE;
			estimate->left_period_s = estimator->left.period_s;
		} else {
			estimate->quality_flags |=
				EXO_GAIT_QUALITY_LEFT_WAITING_EVENT;
		}
		if (right_phase_valid) {
			estimate->valid_flags |= EXO_GAIT_VALID_RIGHT_PHASE;
			estimate->right_period_s = estimator->right.period_s;
		} else {
			estimate->quality_flags |=
				EXO_GAIT_QUALITY_RIGHT_WAITING_EVENT;
		}

		if (left_phase_valid || right_phase_valid) {
			const float left_cadence =
				left_phase_valid ? 120.0f / estimator->left.period_s : 0.0f;
			const float right_cadence =
				right_phase_valid ? 120.0f / estimator->right.period_s : 0.0f;
			estimate->cadence_spm =
				left_phase_valid && right_phase_valid
					? 0.5f * (left_cadence + right_cadence)
					: (left_phase_valid ? left_cadence : right_cadence);
			estimate->valid_flags |= EXO_GAIT_VALID_CADENCE;

			const float left_confidence = side_confidence(
				&estimator->left, &estimator->config, left_phase_valid);
			const float right_confidence = side_confidence(
				&estimator->right, &estimator->config, right_phase_valid);
			if (left_phase_valid && right_phase_valid) {
				estimate->confidence_01 =
					fminf(left_confidence, right_confidence);

				/*
				 * 双侧判断只做宽容的可信度检查：正常左右腿大致交替，但起步、转弯
				 * 和病理步态不应被强行钉死在0.5相差。周期明显不一致或几乎同相时
				 * 保留两侧相位，置质量位并降权，交给未来策略决定是否使用。
				 */
				const float period_scale = fmaxf(
					estimator->left.period_s, estimator->right.period_s);
				const float period_difference_fraction =
					period_scale > 0.0f
						? fabsf(estimator->left.period_s -
								 estimator->right.period_s) /
							  period_scale
						: 1.0f;
				const float raw_phase_distance = fabsf(
					estimate->left_phase_01 - estimate->right_phase_01);
				const float circular_phase_distance = fminf(
					raw_phase_distance, 1.0f - raw_phase_distance);
				if (period_difference_fraction >
						estimator->config
							.bilateral_period_tolerance_fraction ||
					circular_phase_distance <
						estimator->config
							.minimum_bilateral_phase_separation) {
					estimate->quality_flags |=
						EXO_GAIT_QUALITY_BILATERAL_INCONSISTENT;
					estimate->confidence_01 *= 0.35f;
				}
			} else {
				estimate->confidence_01 =
					0.5f * (left_phase_valid ? left_confidence
									 : right_confidence);
				estimate->quality_flags |=
					EXO_GAIT_QUALITY_SINGLE_SIDE_ONLY;
			}
			if ((estimate->quality_flags &
				 EXO_GAIT_QUALITY_PAIR_SKEW_HIGH) != 0U) {
				estimate->confidence_01 *= 0.5f;
			}
			estimate->valid_flags |= EXO_GAIT_VALID_CONFIDENCE;
		}

		estimate->valid = left_phase_valid && right_phase_valid;
		estimator->state = estimate->valid ? EXO_GAIT_STATE_LOCKED
										 : EXO_GAIT_STATE_ACQUIRING;
	}

	if (estimator->state == EXO_GAIT_STATE_STANDING ||
		estimator->state == EXO_GAIT_STATE_DEGRADED) {
		estimate->valid = false;
		estimate->valid_flags = EXO_GAIT_VALID_NONE;
		estimate->left_phase_01 = 0.0f;
		estimate->right_phase_01 = 0.0f;
		estimate->left_period_s = 0.0f;
		estimate->right_period_s = 0.0f;
		estimate->cadence_spm = 0.0f;
		estimate->confidence_01 = 0.0f;
	}

	estimate->state = estimator->state;
	estimator->last_snapshot_sequence = snapshot->sequence;
	estimator->last_reset_epoch = snapshot->reset_epoch;
	estimator->last_timestamp_us = snapshot->captured_us;
	return true;
}

const char *exo_gait_estimator_state_name(exo_gait_estimator_state_t state) {
	switch (state) {
	case EXO_GAIT_STATE_RESET:
		return "RESET";
	case EXO_GAIT_STATE_STANDING:
		return "STANDING";
	case EXO_GAIT_STATE_ACQUIRING:
		return "ACQUIRING";
	case EXO_GAIT_STATE_LOCKED:
		return "LOCKED";
	case EXO_GAIT_STATE_DEGRADED:
		return "DEGRADED";
	default:
		return "UNKNOWN";
	}
}

const char *exo_gait_phase_origin_name(exo_gait_phase_origin_t origin) {
	switch (origin) {
	case EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK:
		return "HIP_FLEXION_PEAK";
	case EXO_GAIT_PHASE_ORIGIN_HIP_EXTENSION_PEAK:
		return "HIP_EXTENSION_PEAK";
	case EXO_GAIT_PHASE_ORIGIN_UNKNOWN:
	default:
		return "UNKNOWN";
	}
}
