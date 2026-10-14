#include "exo_assist.h"

#include <math.h>
#include <string.h>

#include "exo_phase_profile.h"
#include "freertos/FreeRTOS.h"

/* 固定工作区用union提供至少8字节对齐；策略不得越过descriptor声明的大小。 */
typedef union {
	uint64_t align_u64;
	double align_double;
	uint8_t bytes[EXO_ASSIST_POLICY_WORKSPACE_BYTES];
} exo_assist_policy_workspace_t;

typedef struct {
	exo_assist_policy_descriptor_t descriptor;
	const exo_assist_policy_ops_t *ops;
} exo_assist_policy_entry_t;

/*
 * PHASE_PROFILE 保存本次有效区间的起点、上一帧左右输出，以及“本轮 START 后
 * 两侧是否各自出现过新的髋屈曲峰”。它位于静态 workspace，不使用堆；步态
 * 一旦失效，框架会把它清零，恢复时重新等待新事件并渐入。
 */
typedef struct {
	int64_t start_timestamp_us;
	float previous_left_nm;
	float previous_right_nm;
	bool initialized;
	bool event_gate_primed;
	bool left_flexion_seen;
	bool right_flexion_seen;
} phase_profile_workspace_t;

_Static_assert(sizeof(phase_profile_workspace_t) <= 32U,
			   "phase profile workspace must stay within stage-10 budget");

/* 透明策略无历史状态，reset只校验静态接口参数。 */
static esp_err_t transparent_reset(void *workspace, size_t workspace_bytes,
								   const exo_assist_config_t *config) {
	(void)workspace;
	return workspace_bytes == 0U && config != NULL ? ESP_OK
												   : ESP_ERR_INVALID_ARG;
}

/*
 * 透明SHADOW只提出“跟随当前角、零速度、零力矩、零增益”。它只是可观测的接口
 * 样例，结果不会接入阶段8控制器，也不会发送到RS01。
 */
static esp_err_t transparent_step(void *workspace, size_t workspace_bytes,
								  const exo_assist_config_t *config,
								  const exo_sensor_snapshot_t *sensors,
								  const exo_gait_estimate_t *gait,
								  exo_assist_result_t *result) {
	(void)workspace;
	(void)gait;
	if (workspace_bytes != 0U || config == NULL || sensors == NULL ||
		result == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	result->suggestion.left.position_deg =
		sensors->left.joint_sample.position_deg;
	result->suggestion.right.position_deg =
		sensors->right.joint_sample.position_deg;
	result->output_kind = EXO_ASSIST_OUTPUT_PREVIEW_COMMAND;
	result->valid = true;
	return ESP_OK;
}

static const exo_assist_policy_ops_t s_transparent_ops = {
	.reset = transparent_reset,
	.step = transparent_step,
};

/* 新 START/重新获得有效步态时清空固定状态，并验证 profile 仍存在。 */
static esp_err_t phase_profile_reset(void *workspace, size_t workspace_bytes,
									 const exo_assist_config_t *config) {
	exo_phase_profile_info_t info;
	if (workspace == NULL ||
		workspace_bytes != sizeof(phase_profile_workspace_t) ||
		config == NULL ||
		!exo_phase_profile_get_info(config->profile_id, &info)) {
		return ESP_ERR_INVALID_ARG;
	}
	memset(workspace, 0, workspace_bytes);
	return ESP_OK;
}

static float quintic_ramp_01(float x) {
	if (x <= 0.0f) {
		return 0.0f;
	}
	if (x >= 1.0f) {
		return 1.0f;
	}
	const float x2 = x * x;
	const float x3 = x2 * x;
	return x3 * (10.0f + x * (-15.0f + 6.0f * x));
}

static float slew_one_tick(float previous, float target, float maximum_change_nm) {
	const float change = target - previous;
	if (change > maximum_change_nm) {
		return previous + maximum_change_nm;
	}
	if (change < -maximum_change_nm) {
		return previous - maximum_change_nm;
	}
	return target;
}

/*
 * 每侧使用 gait 中本侧的 phase 独立插值。返回值只是 torque_ff 增量建议；本函数
 * 不知道底层控制模式，也绝不发送 CAN。SHADOW 和 ACTIVE 共用完全相同的计算。
 */
static esp_err_t phase_profile_step(void *workspace, size_t workspace_bytes,
									const exo_assist_config_t *config,
									const exo_sensor_snapshot_t *sensors,
									const exo_gait_estimate_t *gait,
									exo_assist_result_t *result) {
	if (workspace == NULL ||
		workspace_bytes != sizeof(phase_profile_workspace_t) ||
		config == NULL || sensors == NULL || gait == NULL || result == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	phase_profile_workspace_t *state = (phase_profile_workspace_t *)workspace;
	/*
	 * gait estimator 在 IDLE 也连续运行，因此 START 时可能携带上一轮已经 LOCKED
	 * 的相位。不能把该历史直接解释为“本轮人体已经开始走路”。reset 后只锁存当前
	 * activation 实际观察到的新屈曲峰；LEFT/RIGHT 必须分别出现，任何一侧缺失时
	 * 都返回成功但明确给出无效、全零建议。SHADOW 与 ACTIVE 走同一闸门。
	 */
	if (!state->event_gate_primed) {
		/*
		 * reset 后的第一帧只建立时间边界，不消费事件位。这样即使 START 恰好
		 * 发生在100 Hz控制周期与反馈更新之间，也不会把 START 前最后一帧残留的
		 * 峰值误当成本轮人体运动。下一帧起，步态估计器只会重新报告新样本事件。
		 */
		state->event_gate_primed = true;
		memset(&result->suggestion, 0, sizeof(result->suggestion));
		result->output_kind = EXO_ASSIST_OUTPUT_NONE;
		result->valid = false;
		return ESP_OK;
	}
	if ((gait->left_event_flags & EXO_GAIT_EVENT_HIP_FLEXION_PEAK) != 0U) {
		state->left_flexion_seen = true;
	}
	if ((gait->right_event_flags & EXO_GAIT_EVENT_HIP_FLEXION_PEAK) != 0U) {
		state->right_flexion_seen = true;
	}
	if (!state->left_flexion_seen || !state->right_flexion_seen) {
		memset(&result->suggestion, 0, sizeof(result->suggestion));
		result->output_kind = EXO_ASSIST_OUTPUT_NONE;
		result->valid = false;
		return ESP_OK;
	}

	exo_joint_command_pair_t raw_delta;
	if (!exo_phase_profile_evaluate_with_maximum(config->profile_id, gait->phase_origin,
									gait->left_phase_01, gait->right_phase_01,
									config->left_level_percent,
									config->right_level_percent,
									(float)config->maximum_assist_torque_mnm * 0.001f,
									&raw_delta)) {
		return ESP_ERR_INVALID_ARG;
	}

	if (!state->initialized) {
		state->start_timestamp_us = sensors->captured_us;
		state->previous_left_nm = 0.0f;
		state->previous_right_nm = 0.0f;
		state->initialized = true;
	}
	if (sensors->captured_us < state->start_timestamp_us) {
		return ESP_ERR_INVALID_STATE;
	}
	const int64_t elapsed_us = sensors->captured_us - state->start_timestamp_us;
	const float ramp = quintic_ramp_01((float)elapsed_us /
									   (float)EXO_PHASE_PROFILE_START_RAMP_US);
	const float target_left = raw_delta.left.torque_ff_nm * ramp;
	const float target_right = raw_delta.right.torque_ff_nm * ramp;
	const float output_left =
		slew_one_tick(state->previous_left_nm, target_left, EXO_PHASE_PROFILE_MAX_SLEW_NM_PER_10MS);
	const float output_right =
		slew_one_tick(state->previous_right_nm, target_right, EXO_PHASE_PROFILE_MAX_SLEW_NM_PER_10MS);
	if (!isfinite(output_left) || !isfinite(output_right)) {
		return ESP_ERR_INVALID_STATE;
	}

	memset(&result->suggestion, 0, sizeof(result->suggestion));
	result->suggestion.left.torque_ff_nm = output_left;
	result->suggestion.right.torque_ff_nm = output_right;
	state->previous_left_nm = output_left;
	state->previous_right_nm = output_right;
	result->output_kind = EXO_ASSIST_OUTPUT_TORQUE_FF_DELTA;
	result->valid = true;
	return ESP_OK;
}

static const exo_assist_policy_ops_t s_phase_profile_ops = {
	.reset = phase_profile_reset,
	.step = phase_profile_step,
};

/*
 * 策略表是编译期常量。已实现 TRANSPARENT、PHASE_PROFILE、ANGLE_THRESHOLD；AAN 仍只
 * 声明未来依赖并保持 implemented=false，不能被界面误认为已有算法。
 */
/* 0.10.10：事件驱动的单次摆腿脉冲。算法返回人体坐标正力矩，
 * 由hip_joint的唯一出口乘已标定电机方向；这里不可再乘s。 */
typedef struct {
    exo_swing_pulse_leg_t left, right;
    int64_t last_us;
    uint32_t last_sequence;
    uint32_t reset_epoch;
} angle_workspace_t;
_Static_assert(sizeof(angle_workspace_t)<=EXO_ASSIST_POLICY_WORKSPACE_BYTES,
    "event pulse exceeds static workspace budget");
static esp_err_t angle_reset(void *workspace, size_t bytes,
    const exo_assist_config_t *config) {
    if (!workspace || bytes!=sizeof(angle_workspace_t) || !config)
        return ESP_ERR_INVALID_ARG;
    memset(workspace,0,bytes);
    return ESP_OK;
}
static exo_swing_pulse_input_t pulse_input(const exo_sensor_joint_snapshot_t *joint,
    int64_t now) {
    return (exo_swing_pulse_input_t){.now_us=now,
        .source_us=joint->source_timestamp_us, .feedback_count=joint->source_feedback_count,
        .angle_deg=joint->joint_sample.position_deg,
        .velocity_deg_s=joint->joint_sample.velocity_deg_s,
        .valid=joint->joint_valid && joint->mapping_valid && joint->direction_valid &&
            joint->zero_valid && joint->feedback_valid && joint->online && !joint->motor_fault &&
            joint->age_us<=EXO_PULSE_FRESHNESS_US};
}
static esp_err_t angle_step(void *workspace, size_t bytes,
    const exo_assist_config_t *config, const exo_sensor_snapshot_t *sensors,
    const exo_gait_estimate_t *gait, exo_assist_result_t *result) {
    (void)gait;
    if (!workspace || bytes!=sizeof(angle_workspace_t) || !config || !sensors || !result)
        return ESP_ERR_INVALID_ARG;
    angle_workspace_t *state=workspace;
    if (sensors->captured_us<=0 || sensors->sequence==0 ||
        sensors->left.motor_fault || sensors->right.motor_fault ||
        (sensors->quality_flags & (EXO_SENSOR_QUALITY_TIME_INVALID |
         EXO_SENSOR_QUALITY_COORDINATE_RESET | EXO_SENSOR_QUALITY_SOURCE_ROLLBACK |
         EXO_SENSOR_QUALITY_NONFINITE_SAMPLE))!=0 ||
        (state->last_us!=0 && (state->reset_epoch!=sensors->reset_epoch ||
            sensors->captured_us<=state->last_us || sensors->captured_us-state->last_us>50000 ||
            (int32_t)(sensors->sequence-state->last_sequence)<=0)))
        return ESP_ERR_INVALID_STATE;
    state->last_us=sensors->captured_us;
    state->last_sequence=sensors->sequence;
    state->reset_epoch=sensors->reset_epoch;
    const exo_swing_pulse_input_t left=pulse_input(&sensors->left,sensors->captured_us);
    const exo_swing_pulse_input_t right=pulse_input(&sensors->right,sensors->captured_us);
    if(!exo_swing_pulse_input_valid(&state->left,&left) ||
        !exo_swing_pulse_input_valid(&state->right,&right)) return ESP_ERR_INVALID_STATE;
    memset(&result->suggestion,0,sizeof(result->suggestion));
    result->suggestion.left.torque_ff_nm=exo_swing_pulse_step(&state->left,
        &config->pulse_timing,&config->pulse_window,config->angle_threshold_cdeg,
        config->left_level_percent,&left);
    result->suggestion.right.torque_ff_nm=exo_swing_pulse_step(&state->right,
        &config->pulse_timing,&config->pulse_window,config->angle_threshold_cdeg,
        config->right_level_percent,&right);
    result->output_kind=EXO_ASSIST_OUTPUT_TORQUE_FF_DELTA;
    result->valid=true;
    return ESP_OK;
}
static const exo_assist_policy_ops_t s_angle_ops={.reset=angle_reset,.step=angle_step};

static const exo_assist_policy_entry_t s_policy_table[] = {
    {
        .descriptor = {
            .strategy_id = EXO_ASSIST_STRATEGY_ANGLE_THRESHOLD,
            .strategy_version = 4U,
            .supported_mode = EXO_ASSIST_MODE_ANGLE_THRESHOLD,
            .required_sensor_mask = EXO_SENSOR_LEFT_HIP_KINEMATICS |
                                    EXO_SENSOR_RIGHT_HIP_KINEMATICS,
            .required_gait_valid_flags = EXO_GAIT_VALID_NONE,
            .minimum_gait_confidence_01 = 0.0f,
            .fixed_workspace_bytes = sizeof(angle_workspace_t),
            .implemented = true,
        },
        .ops = &s_angle_ops,
    },
	{
		.descriptor =
			{
				.strategy_id = EXO_ASSIST_STRATEGY_TRANSPARENT,
				.strategy_version = 1U,
				.supported_mode = EXO_ASSIST_MODE_TRANSPARENT,
				.required_sensor_mask = EXO_SENSOR_LEFT_HIP_KINEMATICS |
										EXO_SENSOR_RIGHT_HIP_KINEMATICS,
				.required_gait_valid_flags = EXO_GAIT_VALID_NONE,
				.minimum_gait_confidence_01 = 0.0f,
				.fixed_workspace_bytes = 0U,
				.implemented = true,
			},
		.ops = &s_transparent_ops,
	},
	{
		.descriptor =
			{
				.strategy_id = EXO_ASSIST_STRATEGY_PHASE_PROFILE,
				.strategy_version = 1U,
				.supported_mode = EXO_ASSIST_MODE_ACTIVE_PHASE,
				.required_sensor_mask = EXO_SENSOR_LEFT_HIP_KINEMATICS |
										EXO_SENSOR_RIGHT_HIP_KINEMATICS,
				.required_gait_valid_flags = EXO_GAIT_VALID_LEFT_PHASE |
											 EXO_GAIT_VALID_RIGHT_PHASE |
											 EXO_GAIT_VALID_CONFIDENCE,
				.minimum_gait_confidence_01 = 0.40f,
				.fixed_workspace_bytes = sizeof(phase_profile_workspace_t),
				.implemented = true,
			},
		.ops = &s_phase_profile_ops,
	},
	{
		.descriptor =
			{
				.strategy_id = EXO_ASSIST_STRATEGY_AAN_RESIDUAL,
				.strategy_version = 0U,
				.supported_mode = EXO_ASSIST_MODE_ASSIST_AS_NEEDED,
				.required_sensor_mask = EXO_SENSOR_LEFT_HIP_KINEMATICS |
										EXO_SENSOR_RIGHT_HIP_KINEMATICS |
										EXO_SENSOR_INTERACTION_TORQUE,
				.required_gait_valid_flags = EXO_GAIT_VALID_LEFT_PHASE |
											 EXO_GAIT_VALID_RIGHT_PHASE |
											 EXO_GAIT_VALID_CONFIDENCE,
				.minimum_gait_confidence_01 = 0.60f,
				.fixed_workspace_bytes = 64U,
				.implemented = false,
			},
		.ops = NULL,
	},
};

static exo_assist_snapshot_t s_snapshot;
static bool s_initialized;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* 这两个对象只由唯一100 Hz任务访问，不需要跨任务锁。 */
static exo_assist_policy_workspace_t s_policy_workspace;
static uint32_t s_runtime_activation_epoch;

static uint32_t next_nonzero(uint32_t value) {
	++value;
	return value == 0U ? 1U : value;
}

static const exo_assist_policy_entry_t *
find_policy(exo_assist_strategy_id_t strategy_id) {
	for (size_t i = 0; i < sizeof(s_policy_table) / sizeof(s_policy_table[0]);
		 ++i) {
		if (s_policy_table[i].descriptor.strategy_id == strategy_id) {
			return &s_policy_table[i];
		}
	}
	return NULL;
}

bool exo_assist_get_policy_descriptor(
	exo_assist_strategy_id_t strategy_id,
	exo_assist_policy_descriptor_t *descriptor) {
	if (descriptor == NULL) {
		return false;
	}
	const exo_assist_policy_entry_t *entry = find_policy(strategy_id);
	if (entry == NULL) {
		memset(descriptor, 0, sizeof(*descriptor));
		return false;
	}
	*descriptor = entry->descriptor;
	return true;
}

/* 校验结构、模式/策略关系和 profile 存在性；不在这里读取底层控制器。 */
static bool config_shape_is_valid(const exo_assist_config_t *config) {
	if (config == NULL ||
		config->schema_version != EXO_ASSIST_CONFIG_SCHEMA_VERSION ||
		(int)config->mode <= (int)EXO_ASSIST_MODE_NONE ||
		(int)config->mode > (int)EXO_ASSIST_MODE_ANGLE_THRESHOLD ||
		(int)config->requested_execution_state <
			(int)EXO_ASSIST_EXEC_DISABLED ||
		(int)config->requested_execution_state > (int)EXO_ASSIST_EXEC_ACTIVE ||
		config->left_level_percent > 100U ||
		config->right_level_percent > 100U || config->reserved_flags != 0U ||
		config->maximum_assist_torque_mnm >
            (config->strategy_id == EXO_ASSIST_STRATEGY_ANGLE_THRESHOLD
             ? EXO_ASSIST_ANGLE_TORQUE_HARD_MAX_MNM
             : EXO_PHASE_PROFILE_TORQUE_HARD_MAX_MNM)) {
		return false;
	}
	const exo_assist_policy_entry_t *entry = find_policy(config->strategy_id);
	if (entry == NULL || entry->descriptor.supported_mode != config->mode ||
		entry->descriptor.fixed_workspace_bytes >
			EXO_ASSIST_POLICY_WORKSPACE_BYTES ||
		!isfinite(entry->descriptor.minimum_gait_confidence_01) ||
		entry->descriptor.minimum_gait_confidence_01 < 0.0f ||
		entry->descriptor.minimum_gait_confidence_01 > 1.0f) {
		return false;
	}

	/* 只有 PHASE_PROFILE 消费 profile_id；其他策略继续要求保留值 0。 */
	if (config->strategy_id == EXO_ASSIST_STRATEGY_PHASE_PROFILE) {
		exo_phase_profile_info_t info;
		return exo_phase_profile_get_info(config->profile_id, &info);
	}
	return config->profile_id == 0U &&
		(config->strategy_id != EXO_ASSIST_STRATEGY_ANGLE_THRESHOLD ||
		 (config->angle_threshold_cdeg >= EXO_ASSIST_ANGLE_MIN_CDEG &&
		  config->angle_threshold_cdeg <= EXO_ASSIST_ANGLE_MAX_CDEG &&
          exo_swing_pulse_timing_valid(&config->pulse_timing) &&
          config->pulse_timing.peak_mnm<=config->maximum_assist_torque_mnm &&
          exo_swing_pulse_window_valid(&config->pulse_window,config->angle_threshold_cdeg)));
}

static void select_metadata_locked(const exo_assist_policy_entry_t *entry) {
	s_snapshot.strategy_version =
		entry != NULL ? entry->descriptor.strategy_version : 0U;
	s_snapshot.required_sensor_mask =
		entry != NULL ? entry->descriptor.required_sensor_mask : 0U;
	s_snapshot.required_gait_valid_flags =
		entry != NULL ? entry->descriptor.required_gait_valid_flags : 0U;
	s_snapshot.minimum_gait_confidence_01 =
		entry != NULL ? entry->descriptor.minimum_gait_confidence_01 : 0.0f;
	s_snapshot.strategy_ready = entry != NULL &&
								entry->descriptor.implemented &&
								entry->ops != NULL && entry->ops->step != NULL;
}

esp_err_t exo_assist_init(void) {
	portENTER_CRITICAL(&s_lock);
	if (s_initialized) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_OK;
	}
	memset(&s_snapshot, 0, sizeof(s_snapshot));
	memset(&s_policy_workspace, 0, sizeof(s_policy_workspace));
	s_runtime_activation_epoch = 0U;
	s_snapshot.initialized = true;
	s_snapshot.selected_valid = true;
	s_snapshot.selected = (exo_assist_config_t){
		.schema_version = EXO_ASSIST_CONFIG_SCHEMA_VERSION,
		.mode = EXO_ASSIST_MODE_TRANSPARENT,
		.requested_execution_state = EXO_ASSIST_EXEC_DISABLED,
		.strategy_id = EXO_ASSIST_STRATEGY_TRANSPARENT,
		.profile_id = 0U,
		.left_level_percent = 0U,
		.right_level_percent = 0U,
		.reserved_flags = 0U,
		.maximum_assist_torque_mnm = EXO_PHASE_PROFILE_TORQUE_DEFAULT_MNM,
		.angle_threshold_cdeg = EXO_ASSIST_ANGLE_DEFAULT_CDEG,
        .pulse_timing={.schema_version=1,.duration_mode=0,.peak_mnm=500,.fixed_ms=600,
            .ratio_permille=300,.minimum_ms=150,.maximum_ms=1500,
            .period_minimum_ms=400,.period_maximum_ms=3000},
        .pulse_window={.reset_cdeg=800,.window_maximum_cdeg=3000},
	};
	s_snapshot.selected_generation = 1U;
	s_snapshot.activation_epoch = 1U;
	s_snapshot.execution_state = EXO_ASSIST_EXEC_DISABLED;
	s_snapshot.unavailable_reason = EXO_ASSIST_UNAVAILABLE_NOT_ARMED;
	s_snapshot.available_sensor_mask = EXO_SENSOR_CURRENT_HARDWARE_MASK;
	select_metadata_locked(find_policy(s_snapshot.selected.strategy_id));
	s_initialized = true;
	portEXIT_CRITICAL(&s_lock);
	return ESP_OK;
}

esp_err_t exo_assist_set_config(const exo_assist_config_t *config) {
    /* 旧UART/BLE及旧主机夹具未携带扩展字段，只有全零扩展才补安全默认。
     * 任意半填/非零schema0不作为默认接受。 */
    exo_assist_config_t normalized={0};
    if (config) {
        normalized=*config;
        const exo_pulse_timing_t *t=&config->pulse_timing;
        if (t->schema_version==0 && t->duration_mode==0 && t->peak_mnm==0 &&
            t->fixed_ms==0 && t->ratio_permille==0 && t->minimum_ms==0 && t->maximum_ms==0 &&
            t->period_minimum_ms==0 && t->period_maximum_ms==0 &&
            config->pulse_window.reset_cdeg==0 && config->pulse_window.window_maximum_cdeg==0) {
            normalized.pulse_timing=exo_swing_pulse_default_timing(config->maximum_assist_torque_mnm);
            normalized.pulse_window=exo_swing_pulse_default_window(config->angle_threshold_cdeg);
        }
        config=&normalized;
    }
	if (!config_shape_is_valid(config)) {
		portENTER_CRITICAL(&s_lock);
		if (s_initialized) {
			++s_snapshot.rejected_config_count;
			s_snapshot.unavailable_reason =
				EXO_ASSIST_UNAVAILABLE_CONFIG_INVALID;
		}
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_ARG;
	}

	/*
	 * ACTIVE 对显式允许的 PHASE_PROFILE 和 ANGLE_THRESHOLD 开放。底层是否为
	 * DAMPING+CONSTANT 在 START 锁存时由 controller 再做交叉校验。
	 */
	if (config->requested_execution_state == EXO_ASSIST_EXEC_ACTIVE) {
		exo_phase_profile_info_t info;
		const bool active_allowed =
			config->mode == EXO_ASSIST_MODE_ACTIVE_PHASE &&
			config->strategy_id == EXO_ASSIST_STRATEGY_PHASE_PROFILE &&
			exo_phase_profile_get_info(config->profile_id, &info) &&
			info.active_allowed;
		const bool threshold_allowed =
			config->mode == EXO_ASSIST_MODE_ANGLE_THRESHOLD &&
			config->strategy_id == EXO_ASSIST_STRATEGY_ANGLE_THRESHOLD;
		if (!active_allowed && !threshold_allowed) {
			portENTER_CRITICAL(&s_lock);
			if (s_initialized) {
				++s_snapshot.rejected_config_count;
				s_snapshot.unavailable_reason =
					EXO_ASSIST_UNAVAILABLE_ACTIVE_NOT_ALLOWED;
			}
			portEXIT_CRITICAL(&s_lock);
			return ESP_ERR_NOT_SUPPORTED;
		}
	}

	const exo_assist_policy_entry_t *entry = find_policy(config->strategy_id);
	portENTER_CRITICAL(&s_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	if (s_snapshot.config_locked) {
		++s_snapshot.rejected_config_count;
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	s_snapshot.selected = *config;
	s_snapshot.selected_valid = true;
	s_snapshot.selected_generation =
		next_nonzero(s_snapshot.selected_generation);
	s_snapshot.execution_state = EXO_ASSIST_EXEC_DISABLED;
	s_snapshot.required_sensors_ready = false;
	s_snapshot.required_gait_ready = false;
	s_snapshot.suggestion_valid = false;
	s_snapshot.unavailable_reason = EXO_ASSIST_UNAVAILABLE_NOT_ARMED;
	select_metadata_locked(entry);
	portEXIT_CRITICAL(&s_lock);
	return ESP_OK;
}

/* 两个16/4字节纯参数对象原子更新，不能切换执行状态或改变START安全上限。
 * 控制任务下一次复制配置后仅供新事件使用；当前脉冲A/Ta保存在工作区而不读取它们。 */
esp_err_t exo_assist_update_pulse_timing(const exo_pulse_timing_t *timing) {
    if (!exo_swing_pulse_timing_valid(timing)) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s_lock);
    if (!s_initialized || s_snapshot.selected.strategy_id!=EXO_ASSIST_STRATEGY_ANGLE_THRESHOLD ||
        timing->peak_mnm>s_snapshot.selected.maximum_assist_torque_mnm ||
        (s_snapshot.config_locked && (!s_snapshot.active_valid ||
            timing->peak_mnm>s_snapshot.active.maximum_assist_torque_mnm))) {
        portEXIT_CRITICAL(&s_lock); return ESP_ERR_INVALID_STATE;
    }
    s_snapshot.selected.pulse_timing=*timing;
    s_snapshot.selected_generation=next_nonzero(s_snapshot.selected_generation);
    if (s_snapshot.config_locked) s_snapshot.active.pulse_timing=*timing;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}
esp_err_t exo_assist_update_pulse_window(const exo_pulse_window_t *window) {
    portENTER_CRITICAL(&s_lock);
    if (!s_initialized || s_snapshot.selected.strategy_id!=EXO_ASSIST_STRATEGY_ANGLE_THRESHOLD ||
        !exo_swing_pulse_window_valid(window,s_snapshot.selected.angle_threshold_cdeg)) {
        portEXIT_CRITICAL(&s_lock); return ESP_ERR_INVALID_ARG;
    }
    s_snapshot.selected.pulse_window=*window;
    s_snapshot.selected_generation=next_nonzero(s_snapshot.selected_generation);
    if (s_snapshot.config_locked) s_snapshot.active.pulse_window=*window;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

/*
 * 根据一份已经通过形状校验的配置推导 arm 后的有效执行状态。未实现的策略会按
 * 既有语义退化为 DISABLED，而不是把“用户请求 SHADOW”误报成真实可执行状态。
 * 本函数只读静态策略表，可由预检和正式 arm 共用。
 */
static exo_assist_execution_state_t
effective_execution_state(const exo_assist_config_t *config,
						  const exo_assist_policy_entry_t *entry) {
	if (config->requested_execution_state == EXO_ASSIST_EXEC_DISABLED) {
		return EXO_ASSIST_EXEC_DISABLED;
	}
	const bool strategy_ready = entry != NULL &&
								entry->descriptor.implemented &&
								entry->ops != NULL && entry->ops->step != NULL;
	return strategy_ready ? config->requested_execution_state
						  : EXO_ASSIST_EXEC_DISABLED;
}

esp_err_t exo_assist_preview_run_contract(exo_assist_run_contract_t *contract) {
	if (contract == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	memset(contract, 0, sizeof(*contract));

	/* 锁内只复制小对象；策略表查找和配置校验都放到锁外完成。 */
	exo_assist_config_t selected = {0};
	uint32_t selected_generation = 0U;
	bool selected_valid = false;
	portENTER_CRITICAL(&s_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	selected = s_snapshot.selected;
	selected_generation = s_snapshot.selected_generation;
	selected_valid = s_snapshot.selected_valid;
	portEXIT_CRITICAL(&s_lock);

	if (!selected_valid || selected_generation == 0U ||
		!config_shape_is_valid(&selected)) {
		return ESP_ERR_INVALID_STATE;
	}
	const exo_assist_policy_entry_t *entry = find_policy(selected.strategy_id);
	contract->requested_execution_state = selected.requested_execution_state;
	contract->execution_state = effective_execution_state(&selected, entry);
	contract->selected_generation = selected_generation;
	return ESP_OK;
}

esp_err_t exo_assist_arm(void) {
	portENTER_CRITICAL(&s_lock);
	if (!s_initialized || !s_snapshot.selected_valid) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	const exo_assist_policy_entry_t *entry =
		find_policy(s_snapshot.selected.strategy_id);
	if (entry == NULL) {
		s_snapshot.unavailable_reason = EXO_ASSIST_UNAVAILABLE_CONFIG_INVALID;
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	s_snapshot.activation_epoch = next_nonzero(s_snapshot.activation_epoch);
	s_snapshot.active = s_snapshot.selected;
	s_snapshot.active_valid = true;
	s_snapshot.config_locked = true;
	s_snapshot.active_generation = s_snapshot.selected_generation;
	s_snapshot.required_sensors_ready = false;
	s_snapshot.required_gait_ready = false;
	s_snapshot.suggestion_valid = false;
	s_snapshot.real_output_enabled = false;
	memset(&s_snapshot.last_result, 0, sizeof(s_snapshot.last_result));
    memset(&s_snapshot.pulse_left,0,sizeof(s_snapshot.pulse_left));
    memset(&s_snapshot.pulse_right,0,sizeof(s_snapshot.pulse_right));
	select_metadata_locked(entry);

	s_snapshot.execution_state =
		effective_execution_state(&s_snapshot.active, entry);
	if (s_snapshot.active.requested_execution_state ==
		EXO_ASSIST_EXEC_DISABLED) {
		s_snapshot.unavailable_reason = EXO_ASSIST_UNAVAILABLE_NONE;
	} else if (!s_snapshot.strategy_ready) {
		/* requested仍保留SHADOW，effective execution明确显示DISABLED。 */
		s_snapshot.unavailable_reason =
			EXO_ASSIST_UNAVAILABLE_STRATEGY_NOT_IMPLEMENTED;
	} else {
		s_snapshot.unavailable_reason = EXO_ASSIST_UNAVAILABLE_NONE;
	}
	portEXIT_CRITICAL(&s_lock);
	return ESP_OK;
}

esp_err_t exo_assist_disarm(void) {
	portENTER_CRITICAL(&s_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	/* 即使沿用同一selected配置，每次STOP也必须让正在锁外计算的旧结果失效。 */
	s_snapshot.activation_epoch = next_nonzero(s_snapshot.activation_epoch);
	s_snapshot.active_valid = false;
	s_snapshot.config_locked = false;
	s_snapshot.required_sensors_ready = false;
	s_snapshot.required_gait_ready = false;
	s_snapshot.suggestion_valid = false;
	s_snapshot.real_output_enabled = false;
	s_snapshot.execution_state = EXO_ASSIST_EXEC_DISABLED;
	s_snapshot.unavailable_reason = EXO_ASSIST_UNAVAILABLE_NOT_ARMED;
	s_snapshot.valid_sensor_mask = 0U;
	memset(&s_snapshot.active, 0, sizeof(s_snapshot.active));
	memset(&s_snapshot.last_result, 0, sizeof(s_snapshot.last_result));
    memset(&s_snapshot.pulse_left,0,sizeof(s_snapshot.pulse_left));
    memset(&s_snapshot.pulse_right,0,sizeof(s_snapshot.pulse_right));
	/* strategy_version和依赖描述的是selected，STOP后继续保留供手机展示。 */
	select_metadata_locked(find_policy(s_snapshot.selected.strategy_id));
	portEXIT_CRITICAL(&s_lock);
	return ESP_OK;
}

static bool
gait_requirements_ready(const exo_assist_policy_descriptor_t *descriptor,
						const exo_gait_estimate_t *gait) {
	if (descriptor == NULL || gait == NULL ||
		(gait->valid_flags & descriptor->required_gait_valid_flags) !=
			descriptor->required_gait_valid_flags) {
		return false;
	}
	if (descriptor->minimum_gait_confidence_01 <= 0.0f) {
		return true;
	}
	return (gait->valid_flags & EXO_GAIT_VALID_CONFIDENCE) != 0U &&
		   isfinite(gait->confidence_01) &&
		   gait->confidence_01 >= descriptor->minimum_gait_confidence_01;
}

esp_err_t exo_assist_process_tick(const exo_sensor_snapshot_t *sensors,
								  const exo_gait_estimate_t *gait,
								  exo_assist_result_t *result) {
	if (sensors == NULL || gait == NULL || result == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	memset(result, 0, sizeof(*result));

	/* 先在短临界区复制活动配置；策略reset/step全部在锁外执行。 */
	exo_assist_config_t active = {0};
	bool active_valid = false;
	exo_assist_execution_state_t execution_state = EXO_ASSIST_EXEC_DISABLED;
	uint32_t activation_epoch = 0U;
	portENTER_CRITICAL(&s_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	active = s_snapshot.active;
	active_valid = s_snapshot.active_valid;
	execution_state = s_snapshot.execution_state;
	activation_epoch = s_snapshot.activation_epoch;
	portEXIT_CRITICAL(&s_lock);

	exo_assist_result_t local_result = {
		.activation_epoch = activation_epoch,
		.source_sensor_sequence = sensors->sequence,
		.source_gait_sequence = gait->sequence,
		.timestamp_us = sensors->captured_us,
		.execution_state = execution_state,
		.output_kind = EXO_ASSIST_OUTPUT_NONE,
		.valid = false,
	};
	const exo_assist_policy_entry_t *entry =
		active_valid ? find_policy(active.strategy_id) : NULL;
	const bool strategy_ready = entry != NULL &&
								entry->descriptor.implemented &&
								entry->ops != NULL && entry->ops->step != NULL;
	const bool sensors_ready =
		entry != NULL &&
		(sensors->valid_sensor_mask & entry->descriptor.required_sensor_mask) ==
			entry->descriptor.required_sensor_mask;
	const bool gait_ready =
		entry != NULL && gait_requirements_ready(&entry->descriptor, gait);
	exo_assist_unavailable_reason_t reason = EXO_ASSIST_UNAVAILABLE_NOT_ARMED;
	esp_err_t policy_result = ESP_OK;

	/*
	 * 输入无效期间不调用策略，也不能保留上一段有效步态的力矩历史。恢复后的第一
	 * 个有效周期会重新 reset，从 0 N·m 开始 1 s 渐入。
	 */
	if (active_valid &&
		active.requested_execution_state != EXO_ASSIST_EXEC_DISABLED &&
		(!strategy_ready || !sensors_ready || !gait_ready)) {
		memset(&s_policy_workspace, 0, sizeof(s_policy_workspace));
		s_runtime_activation_epoch = 0U;
	}

	if (active_valid) {
		/* DISABLED不需要策略实现、传感器或步态，必须最先判断。 */
		if (active.requested_execution_state == EXO_ASSIST_EXEC_DISABLED) {
			reason = EXO_ASSIST_UNAVAILABLE_NONE;
		} else if (!strategy_ready) {
			reason = EXO_ASSIST_UNAVAILABLE_STRATEGY_NOT_IMPLEMENTED;
		} else if (!sensors_ready) {
			reason = EXO_ASSIST_UNAVAILABLE_REQUIRED_SENSOR_MISSING;
		} else if (!gait_ready) {
			reason = EXO_ASSIST_UNAVAILABLE_REQUIRED_GAIT_INVALID;
		} else if (execution_state == EXO_ASSIST_EXEC_SHADOW ||
				   execution_state == EXO_ASSIST_EXEC_ACTIVE) {
			/* 新START代次第一次进入控制任务时，确定性清零并reset固定工作区。 */
			if (s_runtime_activation_epoch != activation_epoch) {
				memset(&s_policy_workspace, 0, sizeof(s_policy_workspace));
				if (entry->ops->reset != NULL) {
					policy_result = entry->ops->reset(
						&s_policy_workspace,
						entry->descriptor.fixed_workspace_bytes, &active);
				}
				/* reset失败时不提交epoch；下一周期必须重新reset，不能直接step。
				 */
				if (policy_result == ESP_OK) {
					s_runtime_activation_epoch = activation_epoch;
				}
			}
			if (policy_result == ESP_OK &&
				s_runtime_activation_epoch == activation_epoch) {
				policy_result =
					entry->ops->step(&s_policy_workspace,
									 entry->descriptor.fixed_workspace_bytes,
									 &active, sensors, gait, &local_result);
			}
			/*
			 * PHASE_PROFILE 在本轮双侧新屈曲峰到齐前会正常返回 ESP_OK，但
			 * result.valid=false。显式报告等待事件，避免界面把“暂无建议”误读成
			 * 算法故障或已允许输出。原因码本身即可观测，不再占用 assist_flags。
			 */
			if (policy_result != ESP_OK) {
				reason = EXO_ASSIST_UNAVAILABLE_CONFIG_INVALID;
			} else if (active.strategy_id ==
						   EXO_ASSIST_STRATEGY_PHASE_PROFILE &&
					   !local_result.valid) {
				reason = EXO_ASSIST_UNAVAILABLE_WAITING_NEW_GAIT_EVENTS;
			} else {
				reason = EXO_ASSIST_UNAVAILABLE_NONE;
			}
		}
	}
	/* 策略报错时框架必须覆盖策略可能提前写入的valid，防止“错误但仍可用”。 */
	if (policy_result != ESP_OK) {
		memset(&local_result.suggestion, 0, sizeof(local_result.suggestion));
		local_result.output_kind = EXO_ASSIST_OUTPUT_NONE;
		local_result.valid = false;
		memset(&s_policy_workspace, 0, sizeof(s_policy_workspace));
		s_runtime_activation_epoch = 0U;
	}

	/*
	 * STOP或下一次START可能在策略锁外计算期间发生。只有active_valid与activation
	 * epoch都仍一致才发布；否则丢弃旧结果，绝不重新点亮刚被STOP清掉的suggestion。
	 */
	portENTER_CRITICAL(&s_lock);
	if (s_snapshot.active_valid != active_valid ||
		s_snapshot.activation_epoch != activation_epoch) {
		portEXIT_CRITICAL(&s_lock);
		memset(&local_result.suggestion, 0, sizeof(local_result.suggestion));
		local_result.output_kind = EXO_ASSIST_OUTPUT_NONE;
		local_result.valid = false;
		*result = local_result;
		return ESP_OK;
	}
	++s_snapshot.step_count;
	s_snapshot.last_step_us = sensors->captured_us;
	s_snapshot.available_sensor_mask = sensors->available_sensor_mask;
	s_snapshot.valid_sensor_mask = sensors->valid_sensor_mask;
	s_snapshot.required_sensors_ready = active_valid && sensors_ready;
	s_snapshot.required_gait_ready = active_valid && gait_ready;
	s_snapshot.suggestion_valid = local_result.valid;
	s_snapshot.real_output_enabled = false;
	s_snapshot.unavailable_reason = reason;
	s_snapshot.last_result = local_result;
    if (entry && active.strategy_id==EXO_ASSIST_STRATEGY_ANGLE_THRESHOLD &&
        s_runtime_activation_epoch==activation_epoch && policy_result==ESP_OK) {
        const angle_workspace_t *pulse=(const angle_workspace_t *)&s_policy_workspace;
        s_snapshot.pulse_left=pulse->left;
        s_snapshot.pulse_right=pulse->right;
    } else {
        memset(&s_snapshot.pulse_left,0,sizeof(s_snapshot.pulse_left));
        memset(&s_snapshot.pulse_right,0,sizeof(s_snapshot.pulse_right));
    }
	if (local_result.valid) {
		++s_snapshot.valid_suggestion_count;
	}
	portEXIT_CRITICAL(&s_lock);
	*result = local_result;
	return policy_result;
}

void exo_assist_report_output(const exo_assist_result_t *result, bool applied) {
	portENTER_CRITICAL(&s_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_lock);
		return;
	}

	/*
	 * 除了调用方的 applied，还要与框架最近发布结果逐项核对。这样 STOP、下一次
	 * START 或较慢消费者都不能用旧 result 重新点亮真实输出。
	 */
	const bool matches_latest =
		result != NULL && result->valid &&
		result->execution_state == EXO_ASSIST_EXEC_ACTIVE &&
		result->output_kind == EXO_ASSIST_OUTPUT_TORQUE_FF_DELTA &&
		s_snapshot.active_valid &&
		s_snapshot.execution_state == EXO_ASSIST_EXEC_ACTIVE &&
		result->activation_epoch == s_snapshot.activation_epoch &&
		result->activation_epoch == s_snapshot.last_result.activation_epoch &&
		result->source_sensor_sequence ==
			s_snapshot.last_result.source_sensor_sequence &&
		result->source_gait_sequence ==
			s_snapshot.last_result.source_gait_sequence &&
		result->timestamp_us == s_snapshot.last_result.timestamp_us;
	s_snapshot.real_output_enabled = applied && matches_latest;
	if (s_snapshot.real_output_enabled) {
		++s_snapshot.applied_output_count;
		s_snapshot.last_applied_output_us = result->timestamp_us;
	}
	portEXIT_CRITICAL(&s_lock);
}

bool exo_assist_get_snapshot(exo_assist_snapshot_t *snapshot) {
	if (snapshot == NULL) {
		return false;
	}
	portENTER_CRITICAL(&s_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_lock);
		memset(snapshot, 0, sizeof(*snapshot));
		return false;
	}
	*snapshot = s_snapshot;
	portEXIT_CRITICAL(&s_lock);
	return true;
}

const char *exo_assist_mode_name(exo_assist_mode_t mode) {
	switch (mode) {
	case EXO_ASSIST_MODE_TRANSPARENT:
		return "TRANSPARENT";
	case EXO_ASSIST_MODE_ACTIVE_PHASE:
		return "ACTIVE_PHASE";
	case EXO_ASSIST_MODE_ANGLE_THRESHOLD:
		return "ANGLE_THRESHOLD";
	case EXO_ASSIST_MODE_ASSIST_AS_NEEDED:
		return "ASSIST_AS_NEEDED";
	case EXO_ASSIST_MODE_NONE:
	default:
		return "NONE";
	}
}

const char *
exo_assist_execution_state_name(exo_assist_execution_state_t state) {
	switch (state) {
	case EXO_ASSIST_EXEC_DISABLED:
		return "DISABLED";
	case EXO_ASSIST_EXEC_SHADOW:
		return "SHADOW";
	case EXO_ASSIST_EXEC_ACTIVE:
		return "ACTIVE";
	default:
		return "UNKNOWN";
	}
}

const char *
exo_assist_unavailable_reason_name(exo_assist_unavailable_reason_t reason) {
	switch (reason) {
	case EXO_ASSIST_UNAVAILABLE_NONE:
		return "NONE";
	case EXO_ASSIST_UNAVAILABLE_NOT_INITIALIZED:
		return "NOT_INITIALIZED";
	case EXO_ASSIST_UNAVAILABLE_CONFIG_INVALID:
		return "CONFIG_INVALID";
	case EXO_ASSIST_UNAVAILABLE_ACTIVE_NOT_ALLOWED:
		return "ACTIVE_NOT_ALLOWED";
	case EXO_ASSIST_UNAVAILABLE_STRATEGY_NOT_IMPLEMENTED:
		return "STRATEGY_NOT_IMPLEMENTED";
	case EXO_ASSIST_UNAVAILABLE_REQUIRED_SENSOR_MISSING:
		return "REQUIRED_SENSOR_MISSING";
	case EXO_ASSIST_UNAVAILABLE_NOT_ARMED:
		return "NOT_ARMED";
	case EXO_ASSIST_UNAVAILABLE_REQUIRED_GAIT_INVALID:
		return "REQUIRED_GAIT_INVALID";
	case EXO_ASSIST_UNAVAILABLE_WAITING_NEW_GAIT_EVENTS:
		return "WAITING_NEW_GAIT_EVENTS";
	default:
		return "UNKNOWN";
	}
}
