#include "exo_assist.h"

#include <math.h>
#include <string.h>

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

/* 透明策略无历史状态，reset只校验静态接口参数。 */
static esp_err_t transparent_reset(void *workspace, size_t workspace_bytes,
							   const exo_assist_config_t *config) {
	(void)workspace;
	return workspace_bytes == 0U && config != NULL
		   ? ESP_OK
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
	if (workspace_bytes != 0U || config == NULL ||
		sensors == NULL || result == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	result->suggestion.left.position_deg =
		sensors->left.joint_sample.position_deg;
	result->suggestion.right.position_deg =
		sensors->right.joint_sample.position_deg;
	result->valid = true;
	return ESP_OK;
}

static const exo_assist_policy_ops_t s_transparent_ops = {
	.reset = transparent_reset,
	.step = transparent_step,
};

/*
 * 策略表是编译期常量。阶段10之前只有TRANSPARENT带真实ops；相位曲线和AAN条目
 * 只声明未来所需输入，implemented=false确保界面不会把占位ID误认为已有算法。
 */
static const exo_assist_policy_entry_t s_policy_table[] = {
	{
		.descriptor = {
			.strategy_id = EXO_ASSIST_STRATEGY_TRANSPARENT,
			.strategy_version = 1U,
			.supported_mode = EXO_ASSIST_MODE_TRANSPARENT,
			.required_sensor_mask =
				EXO_SENSOR_LEFT_HIP_KINEMATICS |
				EXO_SENSOR_RIGHT_HIP_KINEMATICS,
			.required_gait_valid_flags = EXO_GAIT_VALID_NONE,
			.minimum_gait_confidence_01 = 0.0f,
			.fixed_workspace_bytes = 0U,
			.implemented = true,
		},
		.ops = &s_transparent_ops,
	},
	{
		.descriptor = {
			.strategy_id = EXO_ASSIST_STRATEGY_PHASE_PROFILE,
			.strategy_version = 0U,
			.supported_mode = EXO_ASSIST_MODE_ACTIVE_PHASE,
			.required_sensor_mask =
				EXO_SENSOR_LEFT_HIP_KINEMATICS |
				EXO_SENSOR_RIGHT_HIP_KINEMATICS,
			.required_gait_valid_flags =
				EXO_GAIT_VALID_LEFT_PHASE | EXO_GAIT_VALID_RIGHT_PHASE |
				EXO_GAIT_VALID_CONFIDENCE,
			.minimum_gait_confidence_01 = 0.40f,
			.fixed_workspace_bytes = 32U,
			.implemented = false,
		},
		.ops = NULL,
	},
	{
		.descriptor = {
			.strategy_id = EXO_ASSIST_STRATEGY_AAN_RESIDUAL,
			.strategy_version = 0U,
			.supported_mode = EXO_ASSIST_MODE_ASSIST_AS_NEEDED,
			.required_sensor_mask =
				EXO_SENSOR_LEFT_HIP_KINEMATICS |
				EXO_SENSOR_RIGHT_HIP_KINEMATICS |
				EXO_SENSOR_INTERACTION_TORQUE,
			.required_gait_valid_flags =
				EXO_GAIT_VALID_LEFT_PHASE | EXO_GAIT_VALID_RIGHT_PHASE |
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

static const exo_assist_policy_entry_t *find_policy(
	exo_assist_strategy_id_t strategy_id) {
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

/* 这里只校验结构和模式/策略匹配；ACTIVE由set_config返回明确NOT_SUPPORTED。 */
static bool config_shape_is_valid(const exo_assist_config_t *config) {
	if (config == NULL ||
		config->schema_version != EXO_ASSIST_CONFIG_SCHEMA_VERSION ||
		(int)config->mode <= (int)EXO_ASSIST_MODE_NONE ||
		(int)config->mode > (int)EXO_ASSIST_MODE_ASSIST_AS_NEEDED ||
		(int)config->requested_execution_state <
			(int)EXO_ASSIST_EXEC_DISABLED ||
		(int)config->requested_execution_state > (int)EXO_ASSIST_EXEC_ACTIVE ||
		config->left_level_percent > 100U ||
		config->right_level_percent > 100U || config->reserved_flags != 0U) {
		return false;
	}
	const exo_assist_policy_entry_t *entry = find_policy(config->strategy_id);
	return entry != NULL && entry->descriptor.supported_mode == config->mode &&
		   entry->descriptor.fixed_workspace_bytes <=
			   EXO_ASSIST_POLICY_WORKSPACE_BYTES &&
		   isfinite(entry->descriptor.minimum_gait_confidence_01) &&
		   entry->descriptor.minimum_gait_confidence_01 >= 0.0f &&
		   entry->descriptor.minimum_gait_confidence_01 <= 1.0f;
}

static void select_metadata_locked(const exo_assist_policy_entry_t *entry) {
	s_snapshot.strategy_version = entry != NULL
							  ? entry->descriptor.strategy_version
							  : 0U;
	s_snapshot.required_sensor_mask = entry != NULL
								 ? entry->descriptor.required_sensor_mask
								 : 0U;
	s_snapshot.required_gait_valid_flags =
		entry != NULL ? entry->descriptor.required_gait_valid_flags : 0U;
	s_snapshot.minimum_gait_confidence_01 =
		entry != NULL ? entry->descriptor.minimum_gait_confidence_01 : 0.0f;
	s_snapshot.strategy_ready = entry != NULL && entry->descriptor.implemented &&
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

	/* 阶段9允许协议明确表达ACTIVE，但在配置入口给出可诊断的拒绝结果。 */
	if (config->requested_execution_state == EXO_ASSIST_EXEC_ACTIVE) {
		portENTER_CRITICAL(&s_lock);
		if (s_initialized) {
			++s_snapshot.rejected_config_count;
			s_snapshot.unavailable_reason =
				EXO_ASSIST_UNAVAILABLE_ACTIVE_NOT_ALLOWED;
		}
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_NOT_SUPPORTED;
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

esp_err_t exo_assist_arm(void) {
	portENTER_CRITICAL(&s_lock);
	if (!s_initialized || !s_snapshot.selected_valid) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	const exo_assist_policy_entry_t *entry =
		find_policy(s_snapshot.selected.strategy_id);
	if (entry == NULL) {
		s_snapshot.unavailable_reason =
			EXO_ASSIST_UNAVAILABLE_CONFIG_INVALID;
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	if (s_snapshot.selected.requested_execution_state ==
		EXO_ASSIST_EXEC_ACTIVE) {
		s_snapshot.unavailable_reason =
			EXO_ASSIST_UNAVAILABLE_ACTIVE_NOT_ALLOWED;
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_NOT_SUPPORTED;
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
	select_metadata_locked(entry);

	if (s_snapshot.active.requested_execution_state ==
		EXO_ASSIST_EXEC_DISABLED) {
		s_snapshot.execution_state = EXO_ASSIST_EXEC_DISABLED;
		s_snapshot.unavailable_reason = EXO_ASSIST_UNAVAILABLE_NONE;
	} else if (!s_snapshot.strategy_ready) {
		/* requested仍保留SHADOW，effective execution明确显示DISABLED。 */
		s_snapshot.execution_state = EXO_ASSIST_EXEC_DISABLED;
		s_snapshot.unavailable_reason =
			EXO_ASSIST_UNAVAILABLE_STRATEGY_NOT_IMPLEMENTED;
	} else {
		s_snapshot.execution_state = EXO_ASSIST_EXEC_SHADOW;
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
	/* strategy_version和依赖描述的是selected，STOP后继续保留供手机展示。 */
	select_metadata_locked(find_policy(s_snapshot.selected.strategy_id));
	portEXIT_CRITICAL(&s_lock);
	return ESP_OK;
}

static bool gait_requirements_ready(
	const exo_assist_policy_descriptor_t *descriptor,
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
		.valid = false,
	};
	const exo_assist_policy_entry_t *entry =
		active_valid ? find_policy(active.strategy_id) : NULL;
	const bool strategy_ready =
		entry != NULL && entry->descriptor.implemented && entry->ops != NULL &&
		entry->ops->step != NULL;
	const bool sensors_ready = entry != NULL &&
		(sensors->valid_sensor_mask & entry->descriptor.required_sensor_mask) ==
			entry->descriptor.required_sensor_mask;
	const bool gait_ready =
		entry != NULL && gait_requirements_ready(&entry->descriptor, gait);
	exo_assist_unavailable_reason_t reason =
		EXO_ASSIST_UNAVAILABLE_NOT_ARMED;
	esp_err_t policy_result = ESP_OK;

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
		} else if (execution_state == EXO_ASSIST_EXEC_SHADOW) {
			/* 新START代次第一次进入控制任务时，确定性清零并reset固定工作区。 */
			if (s_runtime_activation_epoch != activation_epoch) {
				memset(&s_policy_workspace, 0, sizeof(s_policy_workspace));
				if (entry->ops->reset != NULL) {
					policy_result = entry->ops->reset(
						&s_policy_workspace,
						entry->descriptor.fixed_workspace_bytes, &active);
				}
				/* reset失败时不提交epoch；下一周期必须重新reset，不能直接step。 */
				if (policy_result == ESP_OK) {
					s_runtime_activation_epoch = activation_epoch;
				}
			}
			if (policy_result == ESP_OK &&
				s_runtime_activation_epoch == activation_epoch) {
				policy_result = entry->ops->step(
					&s_policy_workspace,
					entry->descriptor.fixed_workspace_bytes, &active,
					sensors, gait, &local_result);
			}
			reason = policy_result == ESP_OK
					 ? EXO_ASSIST_UNAVAILABLE_NONE
					 : EXO_ASSIST_UNAVAILABLE_CONFIG_INVALID;
		}
	}
	/* 策略报错时框架必须覆盖策略可能提前写入的valid，防止“错误但仍可用”。 */
	if (policy_result != ESP_OK) {
		memset(&local_result.suggestion, 0, sizeof(local_result.suggestion));
		local_result.valid = false;
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
	if (local_result.valid) {
		++s_snapshot.valid_suggestion_count;
	}
	portEXIT_CRITICAL(&s_lock);
	*result = local_result;
	return policy_result;
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
	case EXO_ASSIST_MODE_ASSIST_AS_NEEDED:
		return "ASSIST_AS_NEEDED";
	case EXO_ASSIST_MODE_NONE:
	default:
		return "NONE";
	}
}

const char *exo_assist_execution_state_name(
	exo_assist_execution_state_t state) {
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

const char *exo_assist_unavailable_reason_name(
	exo_assist_unavailable_reason_t reason) {
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
	default:
		return "UNKNOWN";
	}
}
