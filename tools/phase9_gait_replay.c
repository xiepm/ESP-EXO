/*
 * 阶段9主机回放验收
 * =================
 *
 * 本文件直接链接固件中的 exo_sensor_snapshot.c、gait_estimator.c 和
 * exo_ble_protocol.c，不复制一套Python算法。这样测试通过代表ESP32实际编译的
 * 纯C核心通过，而不是“另一个模型看起来正确”。
 *
 * 运行入口见 tools/phase9_replay.ps1。测试不连接电机、不发送CAN，也不写硬件。
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "exo_ble_protocol.h"
#include "exo_sensor_snapshot.h"
#include "gait_estimator.h"

#define PI_F 3.14159265358979323846f
#define TEST_ASSERT(condition, message)                                      \
	do {                                                                   \
		if (!(condition)) {                                              \
			fprintf(stderr, "FAIL [%s:%d] %s\n", __func__, __LINE__, \
					(message));                                  \
			return false;                                              \
		}                                                              \
	} while (0)

typedef struct {
	exo_sensor_snapshot_builder_t builder;
	exo_gait_estimator_t estimator;
	exo_sensor_snapshot_t sensors;
	exo_gait_estimate_t gait;
	uint32_t feedback_count;
} replay_fixture_t;

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

static bool fixture_init(replay_fixture_t *fixture) {
	memset(fixture, 0, sizeof(*fixture));
	return exo_sensor_snapshot_builder_init(&fixture->builder,
										&k_sensor_config) &&
		   exo_gait_estimator_init(&fixture->estimator, &k_gait_config);
}

/* 构造与真实hip_joint快照字段含义一致的一侧输入，角度单位保持deg/deg/s。 */
static exo_sensor_joint_snapshot_t make_joint(float position_deg,
										  float velocity_deg_s,
										  uint64_t source_us,
										  uint32_t feedback_count) {
	return (exo_sensor_joint_snapshot_t){
		.joint_sample = {
			.position_deg = position_deg,
			.velocity_deg_s = velocity_deg_s,
			.torque_nm = 0.25f,
			.temperature_c = 30.0f,
		},
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

static bool replay_tick(replay_fixture_t *fixture, int64_t captured_us,
							exo_sensor_joint_snapshot_t *left,
							exo_sensor_joint_snapshot_t *right) {
	if (!exo_sensor_snapshot_build(&fixture->builder, captured_us, left, right,
								   &fixture->sensors)) {
		return false;
	}
	return exo_gait_estimator_update(&fixture->estimator, &fixture->sensors,
									 &fixture->gait);
}

static void sinusoid(float cycle_phase, float amplitude_deg,
					 float period_s, float *position_deg,
					 float *velocity_deg_s) {
	const float angle = 2.0f * PI_F * cycle_phase;
	*position_deg = amplitude_deg * sinf(angle);
	*velocity_deg_s =
		amplitude_deg * (2.0f * PI_F / period_s) * cosf(angle);
}

static bool estimate_is_finite(const exo_gait_estimate_t *gait) {
	return isfinite(gait->left_phase_01) &&
		   isfinite(gait->right_phase_01) && isfinite(gait->left_period_s) &&
		   isfinite(gait->right_period_s) && isfinite(gait->cadence_spm) &&
		   isfinite(gait->confidence_01);
}

static bool test_standing_never_locks(void) {
	replay_fixture_t f;
	TEST_ASSERT(fixture_init(&f), "fixture init");
	for (uint32_t i = 1U; i <= 300U; ++i) {
		const uint64_t source_us = (uint64_t)i * 10000U;
		exo_sensor_joint_snapshot_t left = make_joint(0.0f, 0.0f, source_us, i);
		exo_sensor_joint_snapshot_t right = left;
		TEST_ASSERT(replay_tick(&f, (int64_t)source_us + 500, &left, &right),
					"standing tick");
		TEST_ASSERT(!f.gait.valid, "standing must not produce valid phase");
		TEST_ASSERT(f.gait.left_event_flags == 0U &&
					f.gait.right_event_flags == 0U,
					"standing must not create events");
		TEST_ASSERT(estimate_is_finite(&f.gait), "standing finite output");
	}
	TEST_ASSERT(f.gait.state == EXO_GAIT_STATE_STANDING,
				"standing timeout must enter STANDING");
	return true;
}

static bool test_duplicate_source_is_not_refiltered(void) {
	replay_fixture_t f;
	TEST_ASSERT(fixture_init(&f), "fixture init");
	exo_sensor_joint_snapshot_t left = make_joint(10.0f, 20.0f, 10000U, 1U);
	exo_sensor_joint_snapshot_t right = left;
	TEST_ASSERT(replay_tick(&f, 10500, &left, &right), "first sample");
	const float filtered_before = f.estimator.left.filtered_position_deg;
	for (int i = 1; i <= 3; ++i) {
		/* 故意改变数值但保持源签名，用来证明算法按签名去重。 */
		left.joint_sample.position_deg = -100.0f;
		left.joint_sample.velocity_deg_s = -100.0f;
		right = left;
		TEST_ASSERT(replay_tick(&f, 10500 + i * 10000, &left, &right),
					"duplicate sample");
	}
	TEST_ASSERT(fabsf(f.estimator.left.filtered_position_deg -
					  filtered_before) < 1.0e-6f,
				"same source must not be filtered repeatedly");
	TEST_ASSERT(f.estimator.left.last_source_feedback_count == 1U,
				"source count must remain the consumed sample");
	return true;
}

static bool test_normal_antiphase_locks(void) {
	replay_fixture_t f;
	TEST_ASSERT(fixture_init(&f), "fixture init");
	bool saw_locked = false;
	unsigned locked_samples = 0U;
	float last_left_phase = 0.0f;
	bool had_left_phase = false;
	for (uint32_t i = 1U; i <= 800U; ++i) {
		const float time_s = (float)i * 0.01f;
		float lp, lv, rp, rv;
		sinusoid(time_s, 20.0f, 1.0f, &lp, &lv);
		sinusoid(time_s + 0.5f, 20.0f, 1.0f, &rp, &rv);
		const uint64_t source_us = (uint64_t)i * 10000U;
		exo_sensor_joint_snapshot_t left = make_joint(lp, lv, source_us, i);
		exo_sensor_joint_snapshot_t right = make_joint(rp, rv, source_us, i);
		TEST_ASSERT(replay_tick(&f, (int64_t)source_us + 500, &left, &right),
					"normal gait tick");
		TEST_ASSERT(estimate_is_finite(&f.gait), "normal gait finite output");
		if ((f.gait.valid_flags & EXO_GAIT_VALID_LEFT_PHASE) != 0U) {
			if (had_left_phase &&
				(f.gait.left_event_flags & EXO_GAIT_EVENT_PHASE_WRAP) == 0U) {
				TEST_ASSERT(f.gait.left_phase_01 + 0.02f >= last_left_phase,
							"phase must not move backward between wraps");
			}
			last_left_phase = f.gait.left_phase_01;
			had_left_phase = true;
		}
		if (f.gait.state == EXO_GAIT_STATE_LOCKED && f.gait.valid) {
			saw_locked = true;
			++locked_samples;
			TEST_ASSERT(f.gait.cadence_spm > 105.0f &&
						f.gait.cadence_spm < 135.0f,
						"1 Hz bilateral gait should be near 120 step/min");
			TEST_ASSERT((f.gait.quality_flags &
						 EXO_GAIT_QUALITY_BILATERAL_INCONSISTENT) == 0U,
						"anti-phase gait should be bilaterally consistent");
		}
	}
	TEST_ASSERT(saw_locked && locked_samples > 100U,
				"normal gait must acquire and remain locked");
	return true;
}

static bool test_same_phase_is_downweighted(void) {
	replay_fixture_t f;
	TEST_ASSERT(fixture_init(&f), "fixture init");
	bool saw_inconsistent = false;
	for (uint32_t i = 1U; i <= 600U; ++i) {
		const float time_s = (float)i * 0.01f;
		float position, velocity;
		sinusoid(time_s, 20.0f, 1.0f, &position, &velocity);
		const uint64_t source_us = (uint64_t)i * 10000U;
		exo_sensor_joint_snapshot_t left =
			make_joint(position, velocity, source_us, i);
		exo_sensor_joint_snapshot_t right = left;
		TEST_ASSERT(replay_tick(&f, (int64_t)source_us + 500, &left, &right),
					"same-phase tick");
		if (f.gait.valid &&
			(f.gait.quality_flags &
			 EXO_GAIT_QUALITY_BILATERAL_INCONSISTENT) != 0U) {
			saw_inconsistent = true;
			TEST_ASSERT(f.gait.confidence_01 < 0.36f,
						"same-phase confidence must be reduced");
		}
	}
	TEST_ASSERT(saw_inconsistent,
				"same-phase bilateral motion must set quality bit14");
	return true;
}

static bool test_period_jump_breaks_stable_vote(void) {
	replay_fixture_t f;
	TEST_ASSERT(fixture_init(&f), "fixture init");
	bool locked_before_jump = false;
	bool saw_unstable_and_reset = false;
	for (uint32_t i = 1U; i <= 850U; ++i) {
		const float time_s = (float)i * 0.01f;
		const float cycle = time_s < 5.0f
						? time_s
						: 5.0f + (time_s - 5.0f) / 0.60f;
		const float period_s = time_s < 5.0f ? 1.0f : 0.60f;
		float lp, lv, rp, rv;
		sinusoid(cycle, 20.0f, period_s, &lp, &lv);
		sinusoid(cycle + 0.5f, 20.0f, period_s, &rp, &rv);
		const uint64_t source_us = (uint64_t)i * 10000U;
		exo_sensor_joint_snapshot_t left = make_joint(lp, lv, source_us, i);
		exo_sensor_joint_snapshot_t right = make_joint(rp, rv, source_us, i);
		TEST_ASSERT(replay_tick(&f, (int64_t)source_us + 500, &left, &right),
					"period jump tick");
		if (time_s < 5.0f && f.gait.valid) {
			locked_before_jump = true;
		}
		if ((f.gait.quality_flags & EXO_GAIT_QUALITY_PERIOD_UNSTABLE) != 0U &&
			(f.estimator.left.accepted_period_count == 0U ||
			 f.estimator.right.accepted_period_count == 0U)) {
			saw_unstable_and_reset = true;
		}
	}
	TEST_ASSERT(locked_before_jump, "baseline must lock before speed change");
	TEST_ASSERT(saw_unstable_and_reset,
				"large period jump must clear at least one stable-period vote");
	return true;
}

static bool test_skew_single_side_and_coordinate_reset(void) {
	replay_fixture_t f;
	TEST_ASSERT(fixture_init(&f), "fixture init");
	bool locked = false;
	for (uint32_t i = 4U; i <= 450U; ++i) {
		const float time_s = (float)i * 0.01f;
		float lp, lv, rp, rv;
		sinusoid(time_s, 20.0f, 1.0f, &lp, &lv);
		sinusoid(time_s + 0.5f, 20.0f, 1.0f, &rp, &rv);
		const uint64_t left_us = (uint64_t)i * 10000U;
		const uint64_t right_us = left_us - 30000U;
		exo_sensor_joint_snapshot_t left = make_joint(lp, lv, left_us, i);
		exo_sensor_joint_snapshot_t right = make_joint(rp, rv, right_us, i);
		TEST_ASSERT(replay_tick(&f, (int64_t)left_us + 500, &left, &right),
					"skew tick");
		TEST_ASSERT((f.sensors.quality_flags &
					 EXO_SENSOR_QUALITY_PAIR_SKEW_HIGH) != 0U,
					"30 ms source skew must be reported");
		if (f.gait.valid) {
			locked = true;
		}
	}
	TEST_ASSERT(locked, "skew case should still acquire before fault injection");

	const uint32_t i = 451U;
	const uint64_t source_us = (uint64_t)i * 10000U;
	exo_sensor_joint_snapshot_t left = make_joint(0.0f, 20.0f, source_us, i);
	exo_sensor_joint_snapshot_t right =
		make_joint(0.0f, -20.0f, source_us, i);
	right.snapshot_available = false;
	TEST_ASSERT(replay_tick(&f, (int64_t)source_us + 500, &left, &right),
				"single-side fault tick");
	TEST_ASSERT(f.gait.state == EXO_GAIT_STATE_DEGRADED && !f.gait.valid &&
				f.gait.valid_flags == EXO_GAIT_VALID_NONE,
				"missing one side must enter DEGRADED without stale phase");

	/* 恢复右侧，同时改变左侧软件零位来源；builder应只推进一次reset_epoch。 */
	const uint32_t epoch_before = f.sensors.reset_epoch;
	right = make_joint(0.0f, -20.0f, source_us + 10000U, i + 1U);
	left = make_joint(0.0f, 20.0f, source_us + 10000U, i + 1U);
	left.zero_source_feedback_count = 99U;
	TEST_ASSERT(replay_tick(&f, (int64_t)source_us + 10500, &left, &right),
				"coordinate reset tick");
	TEST_ASSERT(f.sensors.reset_epoch != epoch_before,
				"zero signature change must advance reset_epoch");
	TEST_ASSERT((f.gait.quality_flags & EXO_GAIT_QUALITY_COORDINATE_RESET) !=
				0U,
				"estimator must report coordinate reset");
	TEST_ASSERT(!f.gait.valid, "coordinate reset must discard learned phase");
	return true;
}

static bool test_missing_source_does_not_create_reset_storm(void) {
	exo_sensor_snapshot_builder_t builder;
	TEST_ASSERT(exo_sensor_snapshot_builder_init(&builder, &k_sensor_config),
				"builder init");
	uint32_t first_epoch = 0U;
	for (int i = 1; i <= 10; ++i) {
		exo_sensor_joint_snapshot_t left = make_joint(0.0f, 0.0f, 0U, 0U);
		exo_sensor_joint_snapshot_t right = left;
		exo_sensor_snapshot_t output;
		TEST_ASSERT(exo_sensor_snapshot_build(&builder, (int64_t)i * 10000,
										 &left, &right, &output),
					"missing source build");
		if (i == 1) {
			first_epoch = output.reset_epoch;
		}
		TEST_ASSERT(output.reset_epoch == first_epoch,
					"missing timestamp must not increment epoch every tick");
	}
	return true;
}

static bool test_protocol_extension_vectors(void) {
	exo_ble_request_t request;
	const uint8_t get_gait[] = {1U, EXO_BLE_OPCODE_GET_GAIT_STATUS, 1U, 0U};
	TEST_ASSERT(exo_ble_protocol_decode_request(get_gait, sizeof(get_gait),
										  &request) == EXO_BLE_RESULT_OK,
				"GET_GAIT vector");

	const uint8_t set_active[] = {
		1U, EXO_BLE_OPCODE_SET_ASSIST_CONFIG, 2U, 8U,
		1U, EXO_BLE_ASSIST_MODE_TRANSPARENT, EXO_BLE_ASSIST_EXEC_ACTIVE,
		EXO_BLE_ASSIST_STRATEGY_TRANSPARENT, 0U, 0U, 0U, 0U,
	};
	TEST_ASSERT(exo_ble_protocol_decode_request(set_active, sizeof(set_active),
										  &request) == EXO_BLE_RESULT_OK &&
				request.assist_config.requested_execution_state ==
					EXO_BLE_ASSIST_EXEC_ACTIVE,
				"ACTIVE is syntactically valid and rejected by business layer");

	uint8_t frame[EXO_BLE_PROTOCOL_MAX_FRAME_SIZE];
	size_t frame_len = 0U;
	const exo_ble_wire_gait_status_t gait = {
		.estimator_id = EXO_BLE_GAIT_ESTIMATOR_HIP_EVENT_TBE,
		.estimator_state = EXO_BLE_GAIT_STATE_ACQUIRING,
		.gait_flags = 0U,
		.phase_origin = EXO_BLE_GAIT_ORIGIN_HIP_FLEXION_PEAK,
		.quality_flags = EXO_GAIT_QUALITY_BILATERAL_INCONSISTENT,
		.left_phase_x10000 = EXO_BLE_GAIT_FIXED_INVALID,
		.right_phase_x10000 = EXO_BLE_GAIT_FIXED_INVALID,
		.cadence_x100 = EXO_BLE_GAIT_FIXED_INVALID,
		.confidence_x10000 = EXO_BLE_GAIT_FIXED_INVALID,
		.estimate_age_ms = 10U,
	};
	TEST_ASSERT(exo_ble_protocol_encode_gait_status(
				1U, &gait, frame, sizeof(frame), &frame_len) &&
				frame_len == EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
				"GAIT 16-byte payload vector");

	const exo_ble_wire_assist_config_t assist = {
		.schema_version = 1U,
		.selected_mode = EXO_BLE_ASSIST_MODE_TRANSPARENT,
		.active_mode = EXO_BLE_ASSIST_MODE_NONE,
		.requested_execution_state = EXO_BLE_ASSIST_EXEC_SHADOW,
		.execution_state = EXO_BLE_ASSIST_EXEC_DISABLED,
		.strategy_id = EXO_BLE_ASSIST_STRATEGY_TRANSPARENT,
		.strategy_version = 1U,
		.profile_id = 0U,
		.left_level_percent = 0U,
		.right_level_percent = 0U,
		.assist_flags = EXO_BLE_ASSIST_FLAG_SELECTED_VALID,
		.unavailable_reason =
			EXO_BLE_ASSIST_UNAVAILABLE_REQUIRED_GAIT_INVALID,
		.config_generation = 1U,
		.required_sensor_mask = 3U,
		.available_sensor_mask = 15U,
	};
	TEST_ASSERT(exo_ble_protocol_encode_assist_config(
				2U, &assist, frame, sizeof(frame), &frame_len) &&
				frame_len == EXO_BLE_PROTOCOL_MAX_FRAME_SIZE &&
				frame[7] == 0x01U,
				"ASSIST requested SHADOW / effective DISABLED vector");

	/*
	 * START 之后，同一个字节的高半字节也必须被真正编码。
	 * 0x11 表示“用户请求 SHADOW，并且当前已经有效执行 SHADOW”。
	 * 如果以后有人把左右移位写反，这个向量会立即失败。
	 */
	exo_ble_wire_assist_config_t assist_running = assist;
	assist_running.active_mode = EXO_BLE_ASSIST_MODE_TRANSPARENT;
	assist_running.execution_state = EXO_BLE_ASSIST_EXEC_SHADOW;
	assist_running.assist_flags |= EXO_BLE_ASSIST_FLAG_ACTIVE_VALID;
	TEST_ASSERT(exo_ble_protocol_encode_assist_config(
				3U, &assist_running, frame, sizeof(frame), &frame_len) &&
				frame_len == EXO_BLE_PROTOCOL_MAX_FRAME_SIZE &&
				frame[7] == 0x11U,
				"ASSIST requested SHADOW / effective SHADOW vector");
	return true;
}

typedef bool (*test_fn_t)(void);

int main(void) {
	const struct {
		const char *name;
		test_fn_t run;
	} tests[] = {
		{"standing_never_locks", test_standing_never_locks},
		{"duplicate_source_is_not_refiltered",
		 test_duplicate_source_is_not_refiltered},
		{"normal_antiphase_locks", test_normal_antiphase_locks},
		{"same_phase_is_downweighted", test_same_phase_is_downweighted},
		{"period_jump_breaks_stable_vote",
		 test_period_jump_breaks_stable_vote},
		{"skew_single_side_and_coordinate_reset",
		 test_skew_single_side_and_coordinate_reset},
		{"missing_source_no_reset_storm",
		 test_missing_source_does_not_create_reset_storm},
		{"protocol_extension_vectors", test_protocol_extension_vectors},
	};

	unsigned passed = 0U;
	for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) {
		printf("RUN  %s\n", tests[i].name);
		if (!tests[i].run()) {
			fprintf(stderr, "RESULT: %u/%zu passed\n", passed,
					sizeof(tests) / sizeof(tests[0]));
			return 1;
		}
		++passed;
		printf("PASS %s\n", tests[i].name);
	}
	printf("RESULT: %u/%zu passed\n", passed,
		   sizeof(tests) / sizeof(tests[0]));
	return 0;
}
