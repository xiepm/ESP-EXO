#include "exo_controller.h"

#include <math.h>
#include <string.h>

#include "exo_command_composer.h"
#include "exo_config.h"
#include "exo_joint_command.h"
#include "exo_phase_profile.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rs01_driver.h"
#include "sdkconfig.h"
#include "sine_reference.h"

/*
 * 整个设备只有这一份控制器状态。portMUX 只保护小结构复制和标志，不包围正弦
 * 计算或 CAN 发送；实时路径没有 malloc、日志、队列等待或电机模式切换。
 */
static exo_controller_snapshot_t s_snapshot;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/*
 * STOP 正常只需等待唯一控制任务中已经进入的一个 100 Hz 周期退出。这里仍设置
 * 100 ms 上限：即使驱动或总线异常，也不能让整机 STOP 执行器永久卡在控制器握手。
 * 超时只返回诊断错误，exo_system 仍会继续发送释力帧、失能两台电机并确认 REST。
 */
#define EXO_CONTROLLER_DISARM_WAIT_MS 100U

static bool mode_is_selectable(exo_control_mode_t mode) {
	return mode >= EXO_CONTROL_MODE_HOLD && mode <= EXO_CONTROL_MODE_IMPEDANCE;
}

/* 工程激励默认不进入产品固件；本工作区 sdkconfig 显式打开后才可选择。 */
static bool engineering_gait_stimulus_enabled(void) {
#if defined(CONFIG_EXO_ENGINEERING_GAIT_STIMULUS) &&                           \
	CONFIG_EXO_ENGINEERING_GAIT_STIMULUS
	return true;
#else
	return false;
#endif
}

static bool reference_is_valid(exo_reference_source_t reference) {
	return reference == EXO_REFERENCE_CONSTANT ||
		   reference == EXO_REFERENCE_SINE ||
		   (reference == EXO_REFERENCE_ALTERNATING_SINE_TEST &&
			engineering_gait_stimulus_enabled());
}

static bool reference_is_position_trajectory(exo_reference_source_t reference) {
	return reference == EXO_REFERENCE_SINE ||
		   reference == EXO_REFERENCE_ALTERNATING_SINE_TEST;
}

static bool selection_is_valid(const exo_controller_selection_t *selection) {
	if (selection == NULL || !mode_is_selectable(selection->mode) ||
		!reference_is_valid(selection->reference) ||
		!exo_joint_command_is_valid(&selection->command.left) ||
		!exo_joint_command_is_valid(&selection->command.right)) {
		return false;
	}

	/* 阶段 8 只把旧正弦轨迹定义为 IMPEDANCE 的参考，避免模式语义交叉膨胀。 */
	return !reference_is_position_trajectory(selection->reference) ||
		   selection->mode == EXO_CONTROL_MODE_IMPEDANCE;
}

static bool centers_are_valid(const exo_controller_centers_t *centers) {
	return centers != NULL && isfinite(centers->left_deg) &&
		   isfinite(centers->right_deg);
}

/* 根据模式把一侧静态配置投影为本周期真正要下发的五元命令。 */
static exo_joint_command_t evaluate_one(exo_control_mode_t mode,
										exo_reference_source_t reference,
										const exo_joint_command_t *configured,
										float start_deg, float ramp,
										float sine_offset_deg) {
	exo_joint_command_t output = {
		.position_deg = start_deg,
		.velocity_deg_s = 0.0f,
		.torque_ff_nm = 0.0f,
		.kp = 0.0f,
		.kd = 0.0f,
	};

	switch (mode) {
	case EXO_CONTROL_MODE_HOLD:
		/* HOLD 忽略配置位置，始终保持本次 START 锁存的关节角。 */
		output.kp = configured->kp;
		output.kd = configured->kd;
		break;
	case EXO_CONTROL_MODE_POSITION:
		output.position_deg =
			start_deg + ramp * (configured->position_deg - start_deg);
		output.kp = configured->kp;
		output.kd = configured->kd;
		break;
	case EXO_CONTROL_MODE_VELOCITY:
		/* kp=0 后 position 字段不参与位置误差，仍放 start 以保持可编码。 */
		output.velocity_deg_s = ramp * configured->velocity_deg_s;
		output.kd = configured->kd;
		break;
	case EXO_CONTROL_MODE_TORQUE:
		output.torque_ff_nm = ramp * configured->torque_ff_nm;
		break;
	case EXO_CONTROL_MODE_DAMPING:
		/* 目标速度为 0，仅逐渐增加 Kd，得到相对当前运动方向的阻尼。 */
		output.kd = ramp * configured->kd;
		break;
	case EXO_CONTROL_MODE_IMPEDANCE:
		if (reference_is_position_trajectory(reference)) {
			/*
			 * 阶段 7 的默认 configured.position_deg=0，因此此式严格退化为
			 * center+sine_offset；非零 position 在 SINE 下表示相对 START
			 * 的偏置。
			 */
			output.position_deg =
				start_deg + ramp * configured->position_deg + sine_offset_deg;
		} else {
			output.position_deg =
				start_deg + ramp * (configured->position_deg - start_deg);
		}
		output.velocity_deg_s = ramp * configured->velocity_deg_s;
		output.torque_ff_nm = ramp * configured->torque_ff_nm;
		output.kp = configured->kp;
		output.kd = configured->kd;
		break;
	case EXO_CONTROL_MODE_NONE:
	default:
		/* 调用者已校验；保留零输出使错误枚举不会产生随机构造值。 */
		break;
	}
	return output;
}

bool exo_controller_evaluate(const exo_controller_selection_t *selection,
							 const exo_controller_centers_t *centers,
							 int64_t elapsed_us,
							 exo_joint_command_pair_t *output) {
	if (output == NULL) {
		return false;
	}
	memset(output, 0, sizeof(*output));
	if (!selection_is_valid(selection) || !centers_are_valid(centers)) {
		return false;
	}

	/*
	 * 复用阶段 7 参考生成器取得同一个五次 ramp，避免在两个模块复制时间公式。
	 * CONSTANT 只使用 ramp；两个正弦参考还使用 offset_deg。旧 SINE 左右同相，
	 * 工程 ALTERNATING_SINE_TEST 左右反相，二者不能共享一个隐含的侧别符号。
	 */
	sine_reference_sample_t reference_sample = {0};
	const sine_reference_config_t *reference_config =
		selection->reference == EXO_REFERENCE_ALTERNATING_SINE_TEST
			? &g_exo_alternating_sine_test_config
			: &g_exo_sine_impedance_config.reference;
	if (!sine_reference_evaluate(reference_config, elapsed_us,
								 &reference_sample)) {
		return false;
	}
	const float left_sine_offset =
		reference_is_position_trajectory(selection->reference)
			? reference_sample.offset_deg
			: 0.0f;
	const float right_sine_offset =
		selection->reference == EXO_REFERENCE_ALTERNATING_SINE_TEST
			? -left_sine_offset
			: left_sine_offset;

	exo_joint_command_pair_t generated = {
		.left = evaluate_one(selection->mode, selection->reference,
							 &selection->command.left, centers->left_deg,
							 reference_sample.ramp, left_sine_offset),
		.right = evaluate_one(selection->mode, selection->reference,
							  &selection->command.right, centers->right_deg,
							  reference_sample.ramp, right_sine_offset),
	};
	if (!exo_joint_command_is_valid(&generated.left) ||
		!exo_joint_command_is_valid(&generated.right)) {
		return false;
	}
	*output = generated;
	return true;
}

esp_err_t exo_controller_init(void) {
	portENTER_CRITICAL(&s_lock);
	if (s_snapshot.initialized) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_OK;
	}
	portEXIT_CRITICAL(&s_lock);

	/*
	 * 产品上电默认是“人先运动”的低阻尼底座：没有位置轨迹、速度目标、前馈
	 * 力矩或刚度，仅以 kd=0.1 提供速度阻尼。工程激励必须由上位机显式选择。
	 */
	const exo_controller_selection_t defaults = {
		.mode = EXO_CONTROL_MODE_DAMPING,
		.reference = EXO_REFERENCE_CONSTANT,
		.command =
			{
				.left =
					{
						.position_deg = 0.0f,
						.velocity_deg_s = 0.0f,
						.torque_ff_nm = 0.0f,
						.kp = 0.0f,
						.kd = g_exo_sine_impedance_config.kd,
					},
				.right =
					{
						.position_deg = 0.0f,
						.velocity_deg_s = 0.0f,
						.torque_ff_nm = 0.0f,
						.kp = 0.0f,
						.kd = g_exo_sine_impedance_config.kd,
					},
			},
	};
	if (!selection_is_valid(&defaults) ||
		!sine_reference_config_is_valid(
			&g_exo_sine_impedance_config.reference) ||
		(engineering_gait_stimulus_enabled() &&
		 !sine_reference_config_is_valid(
			 &g_exo_alternating_sine_test_config))) {
		return ESP_ERR_INVALID_ARG;
	}

	portENTER_CRITICAL(&s_lock);
	if (!s_snapshot.initialized) {
		memset(&s_snapshot, 0, sizeof(s_snapshot));
		s_snapshot.initialized = true;
		s_snapshot.selected = defaults;
		s_snapshot.active.mode = EXO_CONTROL_MODE_NONE;
		s_snapshot.active.reference = EXO_REFERENCE_CONSTANT;
		s_snapshot.selected_generation = 1U;
		s_snapshot.last_error = ESP_OK;
	}
	portEXIT_CRITICAL(&s_lock);
	return ESP_OK;
}

esp_err_t exo_controller_set_mode(exo_control_mode_t mode,
								  exo_reference_source_t reference) {
	if (reference == EXO_REFERENCE_ALTERNATING_SINE_TEST &&
		!engineering_gait_stimulus_enabled()) {
		/* 线码可被识别，但产品构建未编入该工程功能。 */
		return ESP_ERR_NOT_SUPPORTED;
	}
	if (!mode_is_selectable(mode) || !reference_is_valid(reference) ||
		(reference_is_position_trajectory(reference) &&
		 mode != EXO_CONTROL_MODE_IMPEDANCE)) {
		return ESP_ERR_INVALID_ARG;
	}

	portENTER_CRITICAL(&s_lock);
	if (!s_snapshot.initialized || s_snapshot.armed || s_snapshot.tick_active) {
		if (s_snapshot.initialized) {
			++s_snapshot.rejected_config_count;
		}
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	if (s_snapshot.selected.mode != mode ||
		s_snapshot.selected.reference != reference) {
		s_snapshot.selected.mode = mode;
		s_snapshot.selected.reference = reference;
		++s_snapshot.selected_generation;
	}
	portEXIT_CRITICAL(&s_lock);
	return ESP_OK;
}

esp_err_t exo_controller_set_joint_command(exo_hip_joint_id_t joint,
										   const exo_joint_command_t *command) {
	if ((unsigned)joint >= (unsigned)EXO_HIP_JOINT_COUNT ||
		!exo_joint_command_is_valid(command)) {
		return ESP_ERR_INVALID_ARG;
	}

	portENTER_CRITICAL(&s_lock);
	if (!s_snapshot.initialized || s_snapshot.armed || s_snapshot.tick_active) {
		if (s_snapshot.initialized) {
			++s_snapshot.rejected_config_count;
		}
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	if (joint == EXO_HIP_JOINT_LEFT) {
		s_snapshot.selected.command.left = *command;
	} else {
		s_snapshot.selected.command.right = *command;
	}
	++s_snapshot.selected_generation;
	portEXIT_CRITICAL(&s_lock);
	return ESP_OK;
}

bool exo_controller_get_snapshot(exo_controller_snapshot_t *snapshot) {
	if (snapshot == NULL) {
		return false;
	}
	portENTER_CRITICAL(&s_lock);
	if (!s_snapshot.initialized) {
		portEXIT_CRITICAL(&s_lock);
		memset(snapshot, 0, sizeof(*snapshot));
		return false;
	}
	*snapshot = s_snapshot;
	portEXIT_CRITICAL(&s_lock);
	return true;
}

/*
 * controller 与高层助力之间的组合合同只有这一份定义。公开预检和正式 arm 均
 * 调用它，避免出现“预检允许、arm 又用另一套规则”的漂移。工程交替激励和
 * ACTIVE 明确返回 NOT_SUPPORTED，便于业务层与普通状态竞争错误区分。
 */
static esp_err_t
validate_selection_run_contract(const exo_controller_selection_t *selection,
								exo_assist_execution_state_t execution_state) {
	if (selection == NULL || execution_state < EXO_ASSIST_EXEC_DISABLED ||
		execution_state > EXO_ASSIST_EXEC_ACTIVE) {
		return ESP_ERR_INVALID_ARG;
	}
	if (selection->reference == EXO_REFERENCE_ALTERNATING_SINE_TEST &&
		!engineering_gait_stimulus_enabled()) {
		return ESP_ERR_NOT_SUPPORTED;
	}
	if (!selection_is_valid(selection)) {
		return ESP_ERR_INVALID_STATE;
	}
	if (execution_state != EXO_ASSIST_EXEC_ACTIVE) {
		return ESP_OK;
	}
	if (selection->reference == EXO_REFERENCE_ALTERNATING_SINE_TEST) {
		/* TEST 只能产生验证激励，绝不能再叠加真实 ACTIVE 助力。 */
		return ESP_ERR_NOT_SUPPORTED;
	}
	return selection->mode == EXO_CONTROL_MODE_DAMPING &&
				   selection->reference == EXO_REFERENCE_CONSTANT
			   ? ESP_OK
			   : ESP_ERR_INVALID_STATE;
}

esp_err_t exo_controller_validate_run_contract(
	const exo_assist_run_contract_t *assist_contract) {
	if (assist_contract == NULL || assist_contract->selected_generation == 0U ||
		assist_contract->requested_execution_state < EXO_ASSIST_EXEC_DISABLED ||
		assist_contract->requested_execution_state > EXO_ASSIST_EXEC_ACTIVE) {
		return ESP_ERR_INVALID_ARG;
	}

	exo_controller_selection_t selected;
	portENTER_CRITICAL(&s_lock);
	if (!s_snapshot.initialized || s_snapshot.armed || s_snapshot.tick_active) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	selected = s_snapshot.selected;
	portEXIT_CRITICAL(&s_lock);
	return validate_selection_run_contract(&selected,
										   assist_contract->execution_state);
}

/*
 * START 前验证本次运行可能到达的端点，而不只验证 t=0 的当前位置。
 * CONSTANT 检查 ramp=1 的最终目标；两种正弦额外检查满幅正峰和负峰。由于
 * 位置映射会叠加每侧真实软件零位，这项检查必须在 hip_joint 归零后执行。
 */
static esp_err_t
validate_run_envelope(const exo_controller_selection_t *selection,
					  const exo_controller_centers_t *centers) {
	exo_joint_command_pair_t endpoint = {
		.left = evaluate_one(selection->mode, selection->reference,
							 &selection->command.left, centers->left_deg, 1.0f,
							 0.0f),
		.right = evaluate_one(selection->mode, selection->reference,
							  &selection->command.right, centers->right_deg,
							  1.0f, 0.0f),
	};
	rs01_motion_command_t mapped[EXO_ACTUATOR_COUNT];
	esp_err_t error = exo_joint_command_map_pair(&endpoint, mapped);
	if (error != ESP_OK ||
		!reference_is_position_trajectory(selection->reference)) {
		return error;
	}

	/* 正负峰分别映射，可捕获任一侧叠加软件零位后的电机绝对位置越界。 */
	const sine_reference_config_t *reference_config =
		selection->reference == EXO_REFERENCE_ALTERNATING_SINE_TEST
			? &g_exo_alternating_sine_test_config
			: &g_exo_sine_impedance_config.reference;
	const float amplitude = reference_config->amplitude_deg;
	for (unsigned peak = 0U; peak < 2U; ++peak) {
		const float offset = peak == 0U ? amplitude : -amplitude;
		const float right_offset =
			selection->reference == EXO_REFERENCE_ALTERNATING_SINE_TEST
				? -offset
				: offset;
		endpoint.left = evaluate_one(selection->mode, selection->reference,
									 &selection->command.left,
									 centers->left_deg, 1.0f, offset);
		endpoint.right = evaluate_one(selection->mode, selection->reference,
									  &selection->command.right,
									  centers->right_deg, 1.0f, right_offset);
		error = exo_joint_command_map_pair(&endpoint, mapped);
		if (error != ESP_OK) {
			return error;
		}
	}
	return ESP_OK;
}

esp_err_t
exo_controller_arm(const exo_controller_centers_t *centers, int64_t start_us,
				   const exo_controller_assist_contract_t *assist_contract) {
	if (!centers_are_valid(centers) || start_us <= 0 ||
		assist_contract == NULL ||
		assist_contract->execution_state < EXO_ASSIST_EXEC_DISABLED ||
		assist_contract->execution_state > EXO_ASSIST_EXEC_ACTIVE ||
        assist_contract->strategy_id < EXO_ASSIST_STRATEGY_NONE ||
        assist_contract->strategy_id > EXO_ASSIST_STRATEGY_ANGLE_THRESHOLD ||
        (assist_contract->execution_state == EXO_ASSIST_EXEC_ACTIVE &&
         assist_contract->strategy_id != EXO_ASSIST_STRATEGY_PHASE_PROFILE &&
         assist_contract->strategy_id != EXO_ASSIST_STRATEGY_ANGLE_THRESHOLD) ||
        assist_contract->maximum_assist_torque_mnm >
            (assist_contract->strategy_id == EXO_ASSIST_STRATEGY_ANGLE_THRESHOLD
             ? EXO_ASSIST_ANGLE_TORQUE_HARD_MAX_MNM : EXO_PHASE_PROFILE_TORQUE_HARD_MAX_MNM) ||
		(assist_contract->execution_state != EXO_ASSIST_EXEC_DISABLED &&
		 assist_contract->activation_epoch == 0U)) {
		return ESP_ERR_INVALID_ARG;
	}

	/* 先原子复制 selected，再在锁外完成纯计算和关节映射预检。 */
	exo_controller_selection_t selected;
	uint32_t generation;
	portENTER_CRITICAL(&s_lock);
	if (!s_snapshot.initialized || s_snapshot.armed || s_snapshot.tick_active) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	selected = s_snapshot.selected;
	generation = s_snapshot.selected_generation;
	portEXIT_CRITICAL(&s_lock);

	const esp_err_t contract_error = validate_selection_run_contract(
		&selected, assist_contract->execution_state);
	if (contract_error != ESP_OK) {
		return contract_error;
	}

	exo_joint_command_pair_t first_output;
	if (!exo_controller_evaluate(&selected, centers, 0, &first_output)) {
		return ESP_ERR_INVALID_ARG;
	}
	/* 映射预检验证本次软件零位和完整 A/B 量程，但不产生 CAN 帧。 */
	rs01_motion_command_t mapped[EXO_ACTUATOR_COUNT];
	const esp_err_t map_error =
		exo_joint_command_map_pair(&first_output, mapped);
	if (map_error != ESP_OK) {
		return map_error;
	}
	const esp_err_t envelope_error = validate_run_envelope(&selected, centers);
	if (envelope_error != ESP_OK) {
		return envelope_error;
	}

	portENTER_CRITICAL(&s_lock);
	if (s_snapshot.armed || s_snapshot.tick_active ||
		s_snapshot.selected_generation != generation) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	s_snapshot.centers = *centers;
	s_snapshot.active = selected;
	s_snapshot.active_generation = generation;
	s_snapshot.start_us = start_us;
	s_snapshot.assist_contract = *assist_contract;
	memset(&s_snapshot.last_tick_result, 0,
		   sizeof(s_snapshot.last_tick_result));
	s_snapshot.last_error = ESP_OK;
	s_snapshot.armed = true;
	portEXIT_CRITICAL(&s_lock);
	return ESP_OK;
}

/*
 * 校验 suggestion 是否确实属于调用者传入的本周期。顺序从最便于诊断的合同错误
 * 到输入绑定错误；任一失败都只会让本周期退回基础 DAMPING，不读取上一周期值。
 */
static exo_controller_assist_reject_reason_t
validate_assist_result(const exo_controller_assist_contract_t *contract,
					   const exo_assist_result_t *assist_result,
					   uint32_t expected_sensor_sequence,
					   uint32_t expected_gait_sequence, int64_t now_us) {
	if (contract->execution_state == EXO_ASSIST_EXEC_DISABLED) {
		return EXO_CONTROLLER_ASSIST_REJECT_NOT_REQUESTED;
	}
	if (assist_result == NULL || !assist_result->valid) {
		return EXO_CONTROLLER_ASSIST_REJECT_RESULT_INVALID;
	}
	if (assist_result->execution_state != contract->execution_state) {
		return EXO_CONTROLLER_ASSIST_REJECT_EXECUTION_MISMATCH;
	}
	if (assist_result->activation_epoch != contract->activation_epoch) {
		return EXO_CONTROLLER_ASSIST_REJECT_EPOCH_MISMATCH;
	}
	if (assist_result->source_sensor_sequence != expected_sensor_sequence) {
		return EXO_CONTROLLER_ASSIST_REJECT_SENSOR_SEQUENCE_MISMATCH;
	}
	if (assist_result->source_gait_sequence != expected_gait_sequence) {
		return EXO_CONTROLLER_ASSIST_REJECT_GAIT_SEQUENCE_MISMATCH;
	}
	if (assist_result->timestamp_us != now_us) {
		return EXO_CONTROLLER_ASSIST_REJECT_TIMESTAMP_MISMATCH;
	}
	if (assist_result->output_kind != EXO_ASSIST_OUTPUT_TORQUE_FF_DELTA) {
		return EXO_CONTROLLER_ASSIST_REJECT_OUTPUT_KIND;
	}
	return contract->execution_state == EXO_ASSIST_EXEC_SHADOW
			   ? EXO_CONTROLLER_ASSIST_REJECT_SHADOW_ONLY
			   : EXO_CONTROLLER_ASSIST_REJECT_NONE;
}

esp_err_t exo_controller_process_tick(
	int64_t now_us, const exo_assist_result_t *assist_result,
	uint32_t expected_sensor_sequence, uint32_t expected_gait_sequence,
	exo_controller_tick_result_t *tick_result) {
	if (tick_result == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	memset(tick_result, 0, sizeof(*tick_result));
	tick_result->send_result = ESP_OK;

	exo_controller_selection_t active;
	exo_controller_centers_t centers;
	exo_controller_assist_contract_t assist_contract;
	int64_t start_us;

	portENTER_CRITICAL(&s_lock);
	if (!s_snapshot.initialized) {
		portEXIT_CRITICAL(&s_lock);
		tick_result->send_result = ESP_ERR_INVALID_STATE;
		return ESP_ERR_INVALID_STATE;
	}
	if (!s_snapshot.armed) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_OK;
	}
	if (s_snapshot.tick_active) {
		portEXIT_CRITICAL(&s_lock);
		tick_result->send_result = ESP_ERR_INVALID_STATE;
		return ESP_ERR_INVALID_STATE;
	}
	s_snapshot.tick_active = true;
	active = s_snapshot.active;
	centers = s_snapshot.centers;
	assist_contract = s_snapshot.assist_contract;
	start_us = s_snapshot.start_us;
	portEXIT_CRITICAL(&s_lock);

	/* controller 只有在整机输出门已打开时才会被调用。 */
	tick_result->output_gate_open = true;
	exo_joint_command_pair_t base_output;
	esp_err_t result = ESP_ERR_INVALID_STATE;
	const bool generated = exo_controller_evaluate(
		&active, &centers, now_us - start_us, &base_output);
	if (generated) {
		tick_result->base_generated = true;
		tick_result->base_command = base_output;
		tick_result->final_command = base_output;

		tick_result->assist_reject_reason = validate_assist_result(
			&assist_contract, assist_result, expected_sensor_sequence,
			expected_gait_sequence, now_us);
		if (assist_result != NULL && assist_result->valid &&
			assist_result->output_kind == EXO_ASSIST_OUTPUT_TORQUE_FF_DELTA &&
			(tick_result->assist_reject_reason ==
				 EXO_CONTROLLER_ASSIST_REJECT_NONE ||
			 tick_result->assist_reject_reason ==
				 EXO_CONTROLLER_ASSIST_REJECT_SHADOW_ONLY)) {
			tick_result->assist_suggested = true;
			tick_result->assist_delta = assist_result->suggestion;
		}

		if (tick_result->assist_reject_reason ==
			EXO_CONTROLLER_ASSIST_REJECT_NONE) {
			exo_joint_command_pair_t composed;
            const float maximum_nm =
                /* 用除法显式换算：15000*0.001f可能略大于15.0f，
                 * 导致正好在新上限的有效合同被最终合成器误拒绝。 */
                (float)assist_contract.maximum_assist_torque_mnm / 1000.0f;
			if (exo_command_composer_add_torque_ff_bounded(
					&base_output, &assist_result->suggestion, maximum_nm, &composed)) {
				tick_result->assist_accepted = true;
				tick_result->assist_suggested = true;
				tick_result->assist_delta = assist_result->suggestion;
				tick_result->final_command = composed;
			} else {
				tick_result->assist_reject_reason =
					EXO_CONTROLLER_ASSIST_REJECT_COMPOSE_INVALID;
			}
		}

		/* 无论是否接纳助力，本周期最终命令对只经过这一处发送一次。 */
		tick_result->send_attempted = true;
		exo_joint_command_send_result_t send_details;
		result = exo_joint_command_try_send_pair_detailed(
			&tick_result->final_command, &send_details);
		tick_result->send_result = result;
		tick_result->actuator_a_send_succeeded =
			send_details.actuator_attempted[EXO_ACTUATOR_A] &&
			send_details.actuator_error[EXO_ACTUATOR_A] == ESP_OK;
		tick_result->actuator_b_send_succeeded =
			send_details.actuator_attempted[EXO_ACTUATOR_B] &&
			send_details.actuator_error[EXO_ACTUATOR_B] == ESP_OK;
		tick_result->send_succeeded = tick_result->actuator_a_send_succeeded &&
									  tick_result->actuator_b_send_succeeded;
		tick_result->send_partially_succeeded =
			tick_result->actuator_a_send_succeeded !=
			tick_result->actuator_b_send_succeeded;
		tick_result->assist_applied = tick_result->assist_accepted &&
									  (tick_result->actuator_a_send_succeeded ||
									   tick_result->actuator_b_send_succeeded);
	} else {
		tick_result->send_result = ESP_ERR_INVALID_STATE;
	}

	/* 所有成功/失败路径都在同一个出口撤销 tick_active，STOP 不会永久等待。 */
	portENTER_CRITICAL(&s_lock);
	if (generated) {
		s_snapshot.last_output = tick_result->final_command;
		++s_snapshot.output_count;
		s_snapshot.last_output_us = now_us;
		if (tick_result->assist_accepted) {
			++s_snapshot.assist_accepted_count;
		}
		if (tick_result->assist_applied) {
			++s_snapshot.assist_applied_count;
		}
		if (assist_contract.execution_state == EXO_ASSIST_EXEC_ACTIVE &&
			!tick_result->assist_accepted) {
			++s_snapshot.assist_rejected_count;
		}
	}
	if (result != ESP_OK) {
		++s_snapshot.output_error_count;
	}
	if (tick_result->send_partially_succeeded) {
		++s_snapshot.partial_send_count;
	}
	s_snapshot.last_error = result;
	s_snapshot.last_tick_result = *tick_result;
	s_snapshot.tick_active = false;
	portEXIT_CRITICAL(&s_lock);
	return result;
}

esp_err_t exo_controller_disarm(void) {
	portENTER_CRITICAL(&s_lock);
	if (!s_snapshot.initialized) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}
	s_snapshot.armed = false;
	portEXIT_CRITICAL(&s_lock);

	/*
	 * 最多只有唯一控制任务中的一个 tick 在锁外。逐 RTOS tick 让出 CPU，并用
	 * 固定次数构成有界等待；不能在 STOP 路径使用 portMAX_DELAY。
	 */
	const TickType_t wait_limit_ticks =
		pdMS_TO_TICKS(EXO_CONTROLLER_DISARM_WAIT_MS) > 0
			? pdMS_TO_TICKS(EXO_CONTROLLER_DISARM_WAIT_MS)
			: 1;
	for (TickType_t waited_ticks = 0; waited_ticks < wait_limit_ticks;
		 ++waited_ticks) {
		portENTER_CRITICAL(&s_lock);
		const bool active = s_snapshot.tick_active;
		portEXIT_CRITICAL(&s_lock);
		if (!active) {
			goto clear_active_selection;
		}
		vTaskDelay(1);
	}

	/*
	 * armed 已经在入口处关闭，所以超时后不会再进入新 tick。保留 active 快照，
	 * 让 UART/BLE 能看出是哪一轮没有及时退出；调用者会继续执行硬件 STOP 清理。
	 */
	return ESP_ERR_TIMEOUT;

clear_active_selection:
	portENTER_CRITICAL(&s_lock);
	s_snapshot.active = (exo_controller_selection_t){
		.mode = EXO_CONTROL_MODE_NONE,
		.reference = EXO_REFERENCE_CONSTANT,
	};
	s_snapshot.active_generation = 0U;
	s_snapshot.start_us = 0;
	memset(&s_snapshot.assist_contract, 0, sizeof(s_snapshot.assist_contract));
	memset(&s_snapshot.last_tick_result, 0,
		   sizeof(s_snapshot.last_tick_result));
	portEXIT_CRITICAL(&s_lock);
	return ESP_OK;
}

const char *exo_control_mode_name(exo_control_mode_t mode) {
	switch (mode) {
	case EXO_CONTROL_MODE_NONE:
		return "NONE";
	case EXO_CONTROL_MODE_HOLD:
		return "HOLD";
	case EXO_CONTROL_MODE_POSITION:
		return "POSITION";
	case EXO_CONTROL_MODE_VELOCITY:
		return "VELOCITY";
	case EXO_CONTROL_MODE_TORQUE:
		return "TORQUE";
	case EXO_CONTROL_MODE_DAMPING:
		return "DAMPING";
	case EXO_CONTROL_MODE_IMPEDANCE:
		return "IMPEDANCE";
	default:
		return "UNKNOWN";
	}
}

const char *exo_reference_source_name(exo_reference_source_t reference) {
	switch (reference) {
	case EXO_REFERENCE_CONSTANT:
		return "CONSTANT";
	case EXO_REFERENCE_SINE:
		return "SINE";
	case EXO_REFERENCE_ALTERNATING_SINE_TEST:
		return "ALTERNATING_SINE_TEST";
	default:
		return "UNKNOWN";
	}
}
