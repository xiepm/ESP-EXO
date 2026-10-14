/*
 * 阶段10.1真实模块主机集成回放。
 *
 * 本测试直接链接固件的sensor snapshot、gait estimator、assist、phase profile、
 * command composer和controller实现。host_fakes只替代RTOS临界区和最终CAN发送；
 * 步态、助力和控制决策不是mock，也没有复制成另一份Python算法。
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "exo_assist.h"
#include "exo_command_composer.h"
#include "exo_config.h"
#include "exo_controller.h"
#include "exo_joint_command.h"
#include "exo_phase_profile.h"
#include "exo_sensor_snapshot.h"
#include "gait_estimator.h"

#define PI_F 3.14159265358979323846f
#define ASSERT_TRUE(condition, message)                                      \
	do {                                                                    \
		if (!(condition)) {                                               \
			fprintf(stderr, "FAIL [%s:%d] %s\n", __func__, __LINE__,  \
					(message));                                      \
			return false;                                               \
		}                                                               \
	} while (0)

const exo_sine_impedance_config_t g_exo_sine_impedance_config = {
	.reference = {.amplitude_deg = 3.0f, .frequency_hz = 0.25f, .ramp_time_s = 2.0f},
	.kp = 1.0f,
	.kd = 0.1f,
};
const sine_reference_config_t g_exo_alternating_sine_test_config = {
	.amplitude_deg = 10.0f,
	.frequency_hz = 0.5f,
	.ramp_time_s = 3.0f,
};

static esp_err_t s_send_error[EXO_ACTUATOR_COUNT] = {ESP_OK, ESP_OK};
static unsigned s_send_count[EXO_ACTUATOR_COUNT];
static exo_joint_command_pair_t s_last_sent;

static bool finite_command(const exo_joint_command_t *command) {
	return command != NULL && isfinite(command->position_deg) &&
		   isfinite(command->velocity_deg_s) &&
		   isfinite(command->torque_ff_nm) && isfinite(command->kp) &&
		   isfinite(command->kd) && fabsf(command->position_deg) <= 720.0f &&
		   fabsf(command->velocity_deg_s) <= 3000.0f &&
		   fabsf(command->torque_ff_nm) <= 17.0f && command->kp >= 0.0f &&
		   command->kp <= 500.0f && command->kd >= 0.0f && command->kd <= 5.0f;
}

bool exo_joint_command_is_valid(const exo_joint_command_t *command) {
	return finite_command(command);
}

esp_err_t exo_joint_command_map_pair(
	const exo_joint_command_pair_t *joint_commands,
	rs01_motion_command_t actuator_commands[EXO_ACTUATOR_COUNT]) {
	if (joint_commands == NULL || actuator_commands == NULL ||
		!finite_command(&joint_commands->left) ||
		!finite_command(&joint_commands->right)) {
		return ESP_ERR_INVALID_ARG;
	}
	memset(actuator_commands, 0,
		   sizeof(*actuator_commands) * EXO_ACTUATOR_COUNT);
	actuator_commands[0].position_rad =
		exo_degrees_to_radians(joint_commands->left.position_deg);
	actuator_commands[1].position_rad =
		exo_degrees_to_radians(joint_commands->right.position_deg);
	return ESP_OK;
}

esp_err_t exo_joint_command_try_send_pair_detailed(
	const exo_joint_command_pair_t *joint_commands,
	exo_joint_command_send_result_t *result) {
	if (result == NULL || joint_commands == NULL ||
		!finite_command(&joint_commands->left) ||
		!finite_command(&joint_commands->right)) {
		return ESP_ERR_INVALID_ARG;
	}
	memset(result, 0, sizeof(*result));
	result->map_error = ESP_OK;
	s_last_sent = *joint_commands;
	for (unsigned i = 0; i < EXO_ACTUATOR_COUNT; ++i) {
		result->actuator_attempted[i] = true;
		result->actuator_error[i] = s_send_error[i];
		++s_send_count[i];
	}
	return s_send_error[0] != ESP_OK ? s_send_error[0] : s_send_error[1];
}

esp_err_t exo_joint_command_try_send_pair(
	const exo_joint_command_pair_t *joint_commands) {
	exo_joint_command_send_result_t result;
	return exo_joint_command_try_send_pair_detailed(joint_commands, &result);
}

static const exo_sensor_snapshot_config_t k_sensor_config = {
	.feedback_freshness_us = 50000U,
	.pair_skew_warning_us = 20000U,
	.available_sensor_mask = EXO_SENSOR_CURRENT_HARDWARE_MASK,
};
static const exo_gait_estimator_config_t k_gait_config = {
	.filter_alpha = 0.25f,
	.trend_velocity_threshold_deg_s = 5.0f,
	.motion_velocity_threshold_deg_s = 4.0f,
	.minimum_excursion_deg = 6.0f,
	.minimum_period_s = 0.5f,
	.maximum_period_s = 3.0f,
	.minimum_event_interval_s = 0.15f,
	.period_ewma_alpha = 0.25f,
	.unstable_period_fraction = 0.35f,
	.bilateral_period_tolerance_fraction = 0.25f,
	.minimum_bilateral_phase_separation = 0.15f,
	.standing_timeout_us = 1500000U,
	.phase_timeout_margin_us = 300000U,
	.lock_period_count = 2U,
};

typedef struct {
	exo_sensor_snapshot_builder_t builder;
	exo_gait_estimator_t estimator;
	exo_sensor_snapshot_t sensors;
	exo_gait_estimate_t gait;
} human_replay_t;

static exo_sensor_joint_snapshot_t make_joint(
	float position_deg, float velocity_deg_s, uint64_t source_us,
	uint32_t feedback_count) {
	return (exo_sensor_joint_snapshot_t){
		.joint_sample = {.position_deg = position_deg,
						 .velocity_deg_s = velocity_deg_s,
						 .torque_nm = 0.0f,
						 .temperature_c = 30.0f},
		.source_timestamp_us = source_us,
		.source_feedback_count = feedback_count,
		.position_epoch = 1U,
		.zero_source_timestamp_us = 1U,
		.zero_source_feedback_count = 1U,
		.zero_source_position_epoch = 1U,
		.snapshot_available = true,
		.mapping_valid = true,
		.direction_valid = true,
		.zero_valid = true,
		.feedback_valid = true,
		.online = true,
		.joint_valid = true,
	};
}

static bool human_replay_init(human_replay_t *replay) {
	memset(replay, 0, sizeof(*replay));
	return exo_sensor_snapshot_builder_init(&replay->builder, &k_sensor_config) &&
		   exo_gait_estimator_init(&replay->estimator, &k_gait_config);
}

static bool human_replay_tick(human_replay_t *replay, uint32_t index) {
	const float time_s = (float)index * 0.01f;
	const float angle = 2.0f * PI_F * time_s;
	const float omega = 2.0f * PI_F;
	const uint64_t source_us = 1000000ULL + (uint64_t)index * 10000ULL;
	exo_sensor_joint_snapshot_t left = make_joint(
		20.0f * sinf(angle), 20.0f * omega * cosf(angle), source_us, index);
	exo_sensor_joint_snapshot_t right = make_joint(
		-20.0f * sinf(angle), -20.0f * omega * cosf(angle), source_us,
		index);
	return exo_sensor_snapshot_build(
			   &replay->builder, (int64_t)source_us + 500, &left, &right,
			   &replay->sensors) &&
		   exo_gait_estimator_update(
			   &replay->estimator, &replay->sensors, &replay->gait);
}

static exo_joint_command_t zero_damping_command(void) {
	return (exo_joint_command_t){.kd = 0.1f};
}

static bool set_product_base(void) {
	const exo_joint_command_t command = zero_damping_command();
	return exo_controller_set_mode(EXO_CONTROL_MODE_DAMPING,
								   EXO_REFERENCE_CONSTANT) == ESP_OK &&
		   exo_controller_set_joint_command(EXO_HIP_JOINT_LEFT, &command) ==
			   ESP_OK &&
		   exo_controller_set_joint_command(EXO_HIP_JOINT_RIGHT, &command) ==
			   ESP_OK;
}

static exo_assist_config_t phase_config(exo_assist_execution_state_t state) {
	return (exo_assist_config_t){
		.schema_version = EXO_ASSIST_CONFIG_SCHEMA_VERSION,
		.mode = EXO_ASSIST_MODE_ACTIVE_PHASE,
		.requested_execution_state = state,
		.strategy_id = EXO_ASSIST_STRATEGY_PHASE_PROFILE,
		.profile_id = 1U,
		.left_level_percent = 100U,
		.right_level_percent = 100U,
		.maximum_assist_torque_mnm = EXO_PHASE_PROFILE_TORQUE_DEFAULT_MNM,
	};
}

static bool arm_run(exo_assist_execution_state_t state, int64_t start_us) {
	const exo_assist_config_t config = phase_config(state);
	ASSERT_TRUE(exo_assist_set_config(&config) == ESP_OK, "assist set config");
	exo_assist_run_contract_t preview;
	ASSERT_TRUE(exo_assist_preview_run_contract(&preview) == ESP_OK,
				"assist preview");
	ASSERT_TRUE(preview.execution_state == state, "preview execution state");
	ASSERT_TRUE(exo_controller_validate_run_contract(&preview) == ESP_OK,
				"controller preflight");
	ASSERT_TRUE(exo_assist_arm() == ESP_OK, "assist arm");
	exo_assist_snapshot_t assist_snapshot;
	ASSERT_TRUE(exo_assist_get_snapshot(&assist_snapshot), "assist snapshot");
	const exo_controller_assist_contract_t contract = {
		.execution_state = assist_snapshot.execution_state,
		.activation_epoch = assist_snapshot.activation_epoch,
		.strategy_id = assist_snapshot.active.strategy_id,
		.maximum_assist_torque_mnm = assist_snapshot.active.maximum_assist_torque_mnm,
	};
	const exo_controller_centers_t centers = {0};
	ASSERT_TRUE(exo_controller_arm(&centers, start_us, &contract) == ESP_OK,
				"controller arm");
	return true;
}

static bool assert_new_event_gate(
	exo_sensor_snapshot_t sensors, exo_gait_estimate_t gait) {
	/* 模拟START前已经LOCKED的历史相位：没有本轮新峰时绝不能直接用于助力。 */
	gait.valid = true;
	gait.valid_flags = EXO_GAIT_VALID_LEFT_PHASE |
				   EXO_GAIT_VALID_RIGHT_PHASE | EXO_GAIT_VALID_CONFIDENCE;
	gait.confidence_01 = 0.9f;
	gait.left_phase_01 = 0.25f;
	gait.right_phase_01 = 0.75f;
	/*
	 * 第一帧故意同时携带两侧峰值，模拟 START 与反馈更新交界处残留的事件位。
	 * 因果门必须只用这帧建立边界，不能直接放行助力。
	 */
	gait.left_event_flags = EXO_GAIT_EVENT_HIP_FLEXION_PEAK;
	gait.right_event_flags = EXO_GAIT_EVENT_HIP_FLEXION_PEAK;
	exo_assist_result_t result;
	ASSERT_TRUE(exo_assist_process_tick(&sensors, &gait, &result) == ESP_OK,
				"old locked phase tick");
	exo_assist_snapshot_t snapshot;
	ASSERT_TRUE(exo_assist_get_snapshot(&snapshot), "assist gate snapshot");
	ASSERT_TRUE(!result.valid && result.output_kind == EXO_ASSIST_OUTPUT_NONE &&
				result.suggestion.left.torque_ff_nm == 0.0f &&
				result.suggestion.right.torque_ff_nm == 0.0f &&
				snapshot.unavailable_reason ==
					EXO_ASSIST_UNAVAILABLE_WAITING_NEW_GAIT_EVENTS,
			"first post-arm tick must not consume pre-arm event flags");

	sensors.sequence += 1U;
	sensors.captured_us += 10000;
	gait.sequence += 1U;
	gait.left_event_flags = EXO_GAIT_EVENT_HIP_FLEXION_PEAK;
	gait.right_event_flags = 0U;
	ASSERT_TRUE(exo_assist_process_tick(&sensors, &gait, &result) == ESP_OK,
				"left new peak tick");
	ASSERT_TRUE(!result.valid, "one-sided event must keep gate closed");
	sensors.sequence += 1U;
	sensors.captured_us += 10000;
	gait.sequence += 1U;
	gait.left_event_flags = 0U;
	gait.right_event_flags = EXO_GAIT_EVENT_HIP_FLEXION_PEAK;
	ASSERT_TRUE(exo_assist_process_tick(&sensors, &gait, &result) == ESP_OK,
				"right new peak tick");
	ASSERT_TRUE(result.valid &&
				fabsf(result.suggestion.left.torque_ff_nm) < 1e-7f &&
				fabsf(result.suggestion.right.torque_ff_nm) < 1e-7f,
			"second side opens gate at zero ramp origin");
	return true;
}

static bool disarm_run(void) {
	ASSERT_TRUE(exo_controller_disarm() == ESP_OK, "controller disarm");
	ASSERT_TRUE(exo_assist_disarm() == ESP_OK, "assist disarm");
	s_send_error[0] = ESP_OK;
	s_send_error[1] = ESP_OK;
	return true;
}

static bool test_product_default_and_alternating_reference(void) {
	exo_controller_snapshot_t snapshot;
	ASSERT_TRUE(exo_controller_get_snapshot(&snapshot), "controller snapshot");
	ASSERT_TRUE(snapshot.selected.mode == EXO_CONTROL_MODE_DAMPING &&
				snapshot.selected.reference == EXO_REFERENCE_CONSTANT,
			"product default mode");
	ASSERT_TRUE(snapshot.selected.command.left.kp == 0.0f &&
				fabsf(snapshot.selected.command.left.kd - 0.1f) < 1e-6f,
			"product default gains");

	ASSERT_TRUE(exo_controller_set_mode(
				EXO_CONTROL_MODE_IMPEDANCE,
				EXO_REFERENCE_ALTERNATING_SINE_TEST) == ESP_OK,
			"select alternating reference");
	exo_joint_command_t command = {.kp = 0.2f, .kd = 0.1f};
	ASSERT_TRUE(exo_controller_set_joint_command(EXO_HIP_JOINT_LEFT, &command) ==
				ESP_OK,
			"left test command");
	ASSERT_TRUE(exo_controller_set_joint_command(EXO_HIP_JOINT_RIGHT, &command) ==
				ESP_OK,
			"right test command");
	ASSERT_TRUE(exo_controller_get_snapshot(&snapshot), "test snapshot");
	exo_joint_command_pair_t output;
	const exo_controller_centers_t centers = {0};
	ASSERT_TRUE(exo_controller_evaluate(
				&snapshot.selected, &centers, 4500000, &output),
			"alternating evaluate");
	ASSERT_TRUE(fabsf(output.left.position_deg - 10.0f) < 0.02f &&
				fabsf(output.right.position_deg + 10.0f) < 0.02f,
			"10 deg anti-phase endpoint");

	const exo_assist_run_contract_t active_preview = {
		.requested_execution_state = EXO_ASSIST_EXEC_ACTIVE,
		.execution_state = EXO_ASSIST_EXEC_ACTIVE,
		.selected_generation = 1U,
	};
	ASSERT_TRUE(exo_controller_validate_run_contract(&active_preview) ==
				ESP_ERR_NOT_SUPPORTED,
			"TEST + ACTIVE preflight must reject");
	const exo_controller_assist_contract_t active_contract = {
		.execution_state = EXO_ASSIST_EXEC_ACTIVE,
		.activation_epoch = 1U,
		.strategy_id = EXO_ASSIST_STRATEGY_PHASE_PROFILE,
		.maximum_assist_torque_mnm = EXO_PHASE_PROFILE_TORQUE_DEFAULT_MNM,
	};
	ASSERT_TRUE(exo_controller_arm(&centers, 1, &active_contract) ==
				ESP_ERR_NOT_SUPPORTED,
			"TEST + ACTIVE arm must reject");

	ASSERT_TRUE(set_product_base(), "restore product base");
	return true;
}

static bool test_human_shadow_chain(void) {
	ASSERT_TRUE(arm_run(EXO_ASSIST_EXEC_SHADOW, 1000000), "arm shadow");
	human_replay_t replay;
	ASSERT_TRUE(human_replay_init(&replay), "human replay init");
	ASSERT_TRUE(human_replay_tick(&replay, 400U), "seed locked-like input");
	ASSERT_TRUE(assert_new_event_gate(replay.sensors, replay.gait),
			"activation event gate");
	unsigned valid_suggestions = 0U;
	for (uint32_t i = 1U; i <= 800U; ++i) {
		ASSERT_TRUE(human_replay_tick(&replay, i), "human replay tick");
		exo_assist_result_t assist;
		ASSERT_TRUE(exo_assist_process_tick(
					&replay.sensors, &replay.gait, &assist) == ESP_OK,
				"assist shadow tick");
		exo_controller_tick_result_t tick;
		ASSERT_TRUE(exo_controller_process_tick(
					replay.sensors.captured_us, &assist,
					replay.sensors.sequence, replay.gait.sequence, &tick) ==
					ESP_OK,
				"controller shadow tick");
		ASSERT_TRUE(!tick.assist_applied &&
					fabsf(tick.final_command.left.torque_ff_nm) < 1e-7f &&
					fabsf(tick.final_command.right.torque_ff_nm) < 1e-7f,
				"SHADOW must not alter final command");
		if (assist.valid &&
			(fabsf(assist.suggestion.left.torque_ff_nm) > 0.001f ||
			 fabsf(assist.suggestion.right.torque_ff_nm) > 0.001f)) {
			++valid_suggestions;
			ASSERT_TRUE(tick.assist_suggested &&
						tick.assist_reject_reason ==
							EXO_CONTROLLER_ASSIST_REJECT_SHADOW_ONLY,
						"SHADOW suggestion diagnostics");
		}
	}
	ASSERT_TRUE(replay.gait.valid && valid_suggestions > 100U,
			"human feedback must lock gait and create SHADOW suggestions");
	return disarm_run();
}

static bool test_human_active_and_rejection_gates(void) {
	ASSERT_TRUE(arm_run(EXO_ASSIST_EXEC_ACTIVE, 1000000), "arm active");
	human_replay_t replay;
	ASSERT_TRUE(human_replay_init(&replay), "human replay init");
	exo_assist_result_t last_valid = {0};
	unsigned applied = 0U;
	for (uint32_t i = 1U; i <= 800U; ++i) {
		ASSERT_TRUE(human_replay_tick(&replay, i), "human replay tick");
		exo_assist_result_t assist;
		ASSERT_TRUE(exo_assist_process_tick(
					&replay.sensors, &replay.gait, &assist) == ESP_OK,
				"assist active tick");
		exo_controller_tick_result_t tick;
		ASSERT_TRUE(exo_controller_process_tick(
					replay.sensors.captured_us, &assist,
					replay.sensors.sequence, replay.gait.sequence, &tick) ==
					ESP_OK,
				"controller active tick");
		exo_assist_report_output(&assist, tick.assist_applied);
		if (tick.assist_applied &&
			(fabsf(tick.final_command.left.torque_ff_nm) > 0.001f ||
			 fabsf(tick.final_command.right.torque_ff_nm) > 0.001f)) {
			++applied;
			last_valid = assist;
		}
	}
	ASSERT_TRUE(replay.gait.valid && applied > 100U,
			"valid human gait must reach ACTIVE command composition");

	/* 无效步态不得复用上一周期建议。 */
	exo_gait_estimate_t invalid_gait = replay.gait;
	invalid_gait.sequence += 1U;
	invalid_gait.valid = false;
	invalid_gait.valid_flags = EXO_GAIT_VALID_NONE;
	exo_sensor_snapshot_t invalid_sensors = replay.sensors;
	invalid_sensors.sequence += 1U;
	invalid_sensors.captured_us += 10000;
	exo_assist_result_t invalid_result;
	ASSERT_TRUE(exo_assist_process_tick(
				&invalid_sensors, &invalid_gait, &invalid_result) == ESP_OK,
			"invalid gait assist tick");
	ASSERT_TRUE(!invalid_result.valid &&
				invalid_result.suggestion.left.torque_ff_nm == 0.0f &&
				invalid_result.suggestion.right.torque_ff_nm == 0.0f,
			"invalid gait must clear suggestion");
	exo_controller_tick_result_t invalid_tick;
	ASSERT_TRUE(exo_controller_process_tick(
				invalid_sensors.captured_us, &invalid_result,
				invalid_sensors.sequence, invalid_gait.sequence,
				&invalid_tick) == ESP_OK,
			"invalid gait controller tick");
	ASSERT_TRUE(!invalid_tick.assist_accepted &&
				invalid_tick.assist_reject_reason ==
					EXO_CONTROLLER_ASSIST_REJECT_RESULT_INVALID,
			"invalid gait must fall back to base command");

	/* 同一非零建议分别注入旧epoch、旧sensor/gait序号和旧时间戳。 */
	const int64_t gate_now = invalid_sensors.captured_us + 10000;
	exo_assist_result_t probe = last_valid;
	probe.source_sensor_sequence = 900U;
	probe.source_gait_sequence = 901U;
	probe.timestamp_us = gate_now;
	exo_controller_tick_result_t gate;
	probe.activation_epoch -= 1U;
	ASSERT_TRUE(exo_controller_process_tick(gate_now, &probe, 900U, 901U, &gate) == ESP_OK &&
				gate.assist_reject_reason == EXO_CONTROLLER_ASSIST_REJECT_EPOCH_MISMATCH,
			"old epoch rejection");
	probe = last_valid;
	probe.activation_epoch += 0U;
	probe.source_sensor_sequence = 899U;
	probe.source_gait_sequence = 901U;
	probe.timestamp_us = gate_now + 10000;
	ASSERT_TRUE(exo_controller_process_tick(gate_now + 10000, &probe, 900U, 901U, &gate) == ESP_OK &&
				gate.assist_reject_reason == EXO_CONTROLLER_ASSIST_REJECT_SENSOR_SEQUENCE_MISMATCH,
			"old sensor sequence rejection");
	probe.source_sensor_sequence = 900U;
	probe.source_gait_sequence = 900U;
	ASSERT_TRUE(exo_controller_process_tick(gate_now + 20000, &probe, 900U, 901U, &gate) == ESP_OK &&
				gate.assist_reject_reason == EXO_CONTROLLER_ASSIST_REJECT_GAIT_SEQUENCE_MISMATCH,
			"old gait sequence rejection");
	probe.source_gait_sequence = 901U;
	probe.timestamp_us = gate_now + 10000;
	ASSERT_TRUE(exo_controller_process_tick(gate_now + 30000, &probe, 900U, 901U, &gate) == ESP_OK &&
				gate.assist_reject_reason == EXO_CONTROLLER_ASSIST_REJECT_TIMESTAMP_MISMATCH,
			"old timestamp rejection");

	/* A失败/B成功必须如实报告部分发送，不能伪装成双侧均未输出。 */
	probe.timestamp_us = gate_now + 40000;
	s_send_error[0] = ESP_ERR_INVALID_STATE;
	s_send_error[1] = ESP_OK;
	ASSERT_TRUE(exo_controller_process_tick(gate_now + 40000, &probe, 900U, 901U, &gate) ==
				ESP_ERR_INVALID_STATE,
			"partial send returns first actuator error");
	ASSERT_TRUE(gate.assist_accepted && gate.assist_applied &&
				gate.send_partially_succeeded &&
				!gate.actuator_a_send_succeeded &&
				gate.actuator_b_send_succeeded,
			"partial A/B send evidence");
	return disarm_run();
}

static bool test_maximum_locked_and_applied(void) {
	exo_assist_config_t config = phase_config(EXO_ASSIST_EXEC_ACTIVE);
	config.maximum_assist_torque_mnm = 200U;
	config.left_level_percent = 10U;
	config.right_level_percent = 50U;
	ASSERT_TRUE(exo_assist_set_config(&config) == ESP_OK, "select 0.2Nm maximum");
	exo_assist_snapshot_t selected;
	ASSERT_TRUE(exo_assist_get_snapshot(&selected), "selected snapshot");
	exo_assist_config_t bad = config;
	bad.maximum_assist_torque_mnm = 501U;
	ASSERT_TRUE(exo_assist_set_config(&bad) == ESP_ERR_INVALID_ARG, "reject above hard limit");
	exo_assist_snapshot_t unchanged;
	ASSERT_TRUE(exo_assist_get_snapshot(&unchanged) &&
		unchanged.selected_generation == selected.selected_generation &&
		unchanged.selected.maximum_assist_torque_mnm == 200U, "invalid write preserves selected configuration");
	exo_assist_run_contract_t contract;
	ASSERT_TRUE(exo_assist_preview_run_contract(&contract) == ESP_OK, "preview");
	ASSERT_TRUE(exo_assist_arm() == ESP_OK, "arm limited policy");
	ASSERT_TRUE(exo_assist_get_snapshot(&unchanged), "active snapshot");
	const exo_controller_assist_contract_t active_contract = {
		.execution_state = unchanged.execution_state,
		.activation_epoch = unchanged.activation_epoch,
		.strategy_id = unchanged.active.strategy_id,
		.maximum_assist_torque_mnm = unchanged.active.maximum_assist_torque_mnm,
	};
	const exo_controller_centers_t centers = {0};
	ASSERT_TRUE(exo_controller_arm(&centers, 1000000, &active_contract) == ESP_OK, "arm limited controller");
	bad = config;
	bad.maximum_assist_torque_mnm = 500U;
	ASSERT_TRUE(exo_assist_set_config(&bad) == ESP_ERR_INVALID_STATE, "running config locked");
	human_replay_t replay;
	ASSERT_TRUE(human_replay_init(&replay), "replay init");
	unsigned nonzero = 0U;
	for (uint32_t i = 1U; i <= 800U; ++i) {
		ASSERT_TRUE(human_replay_tick(&replay, i), "tick");
		exo_assist_result_t assist;
		exo_controller_tick_result_t tick;
		ASSERT_TRUE(exo_assist_process_tick(&replay.sensors, &replay.gait, &assist) == ESP_OK,
			"limited assist tick");
		ASSERT_TRUE(exo_controller_process_tick(replay.sensors.captured_us, &assist,
			replay.sensors.sequence, replay.gait.sequence, &tick) == ESP_OK, "final command tick");
		ASSERT_TRUE(fabsf(tick.final_command.left.torque_ff_nm) <= 0.020001f &&
			fabsf(tick.final_command.right.torque_ff_nm) <= 0.100001f, "actual composed commands respect saved limit and percent");
		if (tick.assist_applied && fabsf(tick.final_command.right.torque_ff_nm) > 0.001f) ++nonzero;
	}
	ASSERT_TRUE(nonzero > 100U, "bounded nonzero applied commands");
	ASSERT_TRUE(exo_assist_get_snapshot(&unchanged) && unchanged.active.maximum_assist_torque_mnm == 200U,
		"active maximum preserved");
	return disarm_run();
}

int main(void) {
	if (exo_assist_init() != ESP_OK || exo_controller_init() != ESP_OK) {
		fprintf(stderr, "FAIL initialization\n");
		return 1;
	}
	struct {
		const char *name;
		bool (*run)(void);
	} tests[] = {
		{"product defaults, anti-phase trajectory, TEST+ACTIVE gate",
		 test_product_default_and_alternating_reference},
		{"human feedback -> gait -> SHADOW -> no applied torque",
		 test_human_shadow_chain},
		{"human ACTIVE, invalid gait, stale contracts, partial send",
		 test_human_active_and_rejection_gates},
		{"maximum torque lock, rejection and final command limits", test_maximum_locked_and_applied},
	};
	unsigned passed = 0U;
	for (unsigned i = 0U; i < sizeof(tests) / sizeof(tests[0]); ++i) {
		if (!tests[i].run()) {
			fprintf(stderr, "[FAIL] %s\n", tests[i].name);
			return 1;
		}
		++passed;
		printf("[PASS] %s\n", tests[i].name);
	}
	printf("[PASS] phase10 assist integration: %u/%u\n", passed,
		   (unsigned)(sizeof(tests) / sizeof(tests[0])));
	return 0;
}
