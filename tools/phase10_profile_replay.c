/*
 * 阶段 10 相位曲线与命令合成主机回放
 * ====================================
 *
 * 本程序直接编译固件正在使用的 exo_phase_profile.c 和
 * exo_command_composer.c，不复制第二份算法。它不包含 FreeRTOS、BLE、CAN 或
 * 电机驱动，因此可以在烧录前快速证明曲线插值、左右独立缩放和“只增加前馈
 * 力矩”的合成合同。
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "exo_command_composer.h"
#include "exo_phase_profile.h"

#define TEST_ASSERT(condition, message)                                      \
	do {                                                                   \
		if (!(condition)) {                                              \
			fprintf(stderr, "FAIL [%s:%d] %s\n", __func__, __LINE__, \
					(message));                                  \
			return false;                                              \
		}                                                              \
	} while (0)

static bool nearly_equal(float actual, float expected, float tolerance) {
	return isfinite(actual) && isfinite(expected) &&
		   fabsf(actual - expected) <= tolerance;
}

static bool pair_is_all_zero(const exo_joint_command_pair_t *pair) {
	const exo_joint_command_t *commands[2] = {&pair->left, &pair->right};
	for (size_t i = 0U; i < 2U; ++i) {
		if (commands[i]->position_deg != 0.0f ||
			commands[i]->velocity_deg_s != 0.0f ||
			commands[i]->torque_ff_nm != 0.0f || commands[i]->kp != 0.0f ||
			commands[i]->kd != 0.0f) {
			return false;
		}
	}
	return true;
}

static bool test_profile_metadata(void) {
	exo_phase_profile_info_t info = {0};
	TEST_ASSERT(exo_phase_profile_get_info(
				EXO_PHASE_PROFILE_ID_ENGINEERING_TEST_V1, &info),
			"engineering profile must exist");
	TEST_ASSERT(info.profile_id == EXO_PHASE_PROFILE_ID_ENGINEERING_TEST_V1,
			"profile id");
	TEST_ASSERT(info.profile_version == 1U, "profile version");
	TEST_ASSERT(info.point_count == EXO_PHASE_PROFILE_LUT_POINTS,
			"profile point count");
	TEST_ASSERT(info.required_phase_origin ==
				EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK,
			"phase origin");
	TEST_ASSERT(info.active_allowed, "engineering low-amplitude ACTIVE route");
	TEST_ASSERT(!info.literature_profile,
			"engineering curve must never claim literature provenance");
	TEST_ASSERT(nearly_equal(info.maximum_abs_torque_nm, 0.5f, 1.0e-6f),
			"engineering peak torque");
	TEST_ASSERT(info.name != NULL &&
			strcmp(info.name, "ENGINEERING_TEST_V1") == 0,
			"engineering name");

	memset(&info, 0xA5, sizeof(info));
	TEST_ASSERT(!exo_phase_profile_get_info(0xFFU, &info),
			"unknown profile rejected");
	TEST_ASSERT(info.profile_id == 0U && info.name == NULL,
			"unknown profile clears output");
	TEST_ASSERT(!exo_phase_profile_get_info(0U, NULL), "null metadata output");
	return true;
}

static bool test_zero_and_independent_profiles(void) {
	exo_joint_command_pair_t output;
	TEST_ASSERT(exo_phase_profile_evaluate(
				EXO_PHASE_PROFILE_ID_ZERO_DIAGNOSTIC,
				EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK, 0.123f, 0.876f,
				100U, 37U, &output),
			"zero diagnostic evaluation");
	TEST_ASSERT(pair_is_all_zero(&output), "zero profile must be exactly zero");

	TEST_ASSERT(exo_phase_profile_evaluate(
				EXO_PHASE_PROFILE_ID_ENGINEERING_TEST_V1,
				EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK, 0.25f, 0.75f,
				100U, 50U, &output),
			"independent phase evaluation");
	TEST_ASSERT(nearly_equal(output.left.torque_ff_nm, 0.5f, 1.0e-6f),
			"left quarter-cycle peak");
	TEST_ASSERT(nearly_equal(output.right.torque_ff_nm, -0.25f, 1.0e-6f),
			"right own phase and own level");
	TEST_ASSERT(output.left.position_deg == 0.0f &&
			output.left.velocity_deg_s == 0.0f && output.left.kp == 0.0f &&
			output.left.kd == 0.0f && output.right.position_deg == 0.0f &&
			output.right.velocity_deg_s == 0.0f && output.right.kp == 0.0f &&
			output.right.kd == 0.0f,
			"profile output must contain torque delta only");
	return true;
}

static bool test_interpolation_scaling_and_wrap(void) {
	exo_joint_command_pair_t output;
	/* 1/32 恰好位于 LUT 第0、1点中间：0.5 * 0.382683 * 0.5 N·m。 */
	TEST_ASSERT(exo_phase_profile_evaluate(
				EXO_PHASE_PROFILE_ID_ENGINEERING_TEST_V1,
				EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK, 1.0f / 32.0f,
				0.5f, 100U, 20U, &output),
			"interpolation evaluation");
	TEST_ASSERT(nearly_equal(output.left.torque_ff_nm, 0.09567075f, 1.0e-6f),
			"linear interpolation");
	TEST_ASSERT(nearly_equal(output.right.torque_ff_nm, 0.0f, 1.0e-6f),
			"half-cycle zero and independent level");

	TEST_ASSERT(exo_phase_profile_evaluate(
				EXO_PHASE_PROFILE_ID_ENGINEERING_TEST_V1,
				EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK, 0.999999f, 0.0f,
				100U, 100U, &output),
			"near-wrap phase");
	TEST_ASSERT(fabsf(output.left.torque_ff_nm) < 1.0e-4f,
			"periodic interpolation returns continuously to phase zero");
	TEST_ASSERT(output.left.torque_ff_nm <= 0.0f,
			"near-wrap sign comes from the last LUT interval");
	return true;
}

static bool test_profile_rejects_bad_inputs(void) {
	exo_joint_command_pair_t output;
	memset(&output, 0xA5, sizeof(output));
	TEST_ASSERT(!exo_phase_profile_evaluate(
				0xFFU, EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK, 0.1f,
				0.2f, 10U, 10U, &output),
			"unknown profile");
	TEST_ASSERT(pair_is_all_zero(&output), "failure must clear output");
	TEST_ASSERT(!exo_phase_profile_evaluate(
				EXO_PHASE_PROFILE_ID_ENGINEERING_TEST_V1,
				EXO_GAIT_PHASE_ORIGIN_HIP_EXTENSION_PEAK, 0.1f, 0.2f,
				10U, 10U, &output),
			"wrong origin");
	TEST_ASSERT(!exo_phase_profile_evaluate(
				EXO_PHASE_PROFILE_ID_ENGINEERING_TEST_V1,
				EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK, 1.0f, 0.2f,
				10U, 10U, &output),
			"phase one is outside [0,1)");
	TEST_ASSERT(!exo_phase_profile_evaluate(
				EXO_PHASE_PROFILE_ID_ENGINEERING_TEST_V1,
				EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK, NAN, 0.2f, 10U,
				10U, &output),
			"NaN phase");
	TEST_ASSERT(!exo_phase_profile_evaluate(
				EXO_PHASE_PROFILE_ID_ENGINEERING_TEST_V1,
				EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK, 0.1f, 0.2f,
				101U, 10U, &output),
			"level above 100");
	TEST_ASSERT(!exo_phase_profile_evaluate(
				EXO_PHASE_PROFILE_ID_ENGINEERING_TEST_V1,
				EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK, 0.1f, 0.2f,
				10U, 10U, NULL),
			"null output");
	return true;
}

static bool test_configurable_maximum(void) {
	exo_joint_command_pair_t out;
	TEST_ASSERT(exo_phase_profile_evaluate_with_maximum(1U,
		EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK, 0.25f, 0.75f,
		10U, 100U, 0.2f, &out), "configured maximum");
	TEST_ASSERT(nearly_equal(out.left.torque_ff_nm, 0.02f, 1e-6f) &&
		nearly_equal(out.right.torque_ff_nm, -0.2f, 1e-6f), "Nm times independent percentages");
	for (unsigned i = 0U; i < 1000U; ++i) {
		TEST_ASSERT(exo_phase_profile_evaluate_with_maximum(1U,
			EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK, (float)i / 1000.0f,
			(float)(999U-i) / 1000.0f, 37U, 73U, 0.123f, &out), "phase sweep");
		TEST_ASSERT(fabsf(out.left.torque_ff_nm) <= 0.123f * 0.37f + 1e-6f &&
			fabsf(out.right.torque_ff_nm) <= 0.123f * 0.73f + 1e-6f, "bound both signs throughout curve");
	}
	TEST_ASSERT(exo_phase_profile_evaluate_with_maximum(1U,
		EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK, 0.25f, 0.75f,
		100U, 100U, 0.0f, &out) && pair_is_all_zero(&out), "zero maximum disables delta");
	const float bad[] = {-0.001f, 0.501f, NAN, INFINITY};
	for (unsigned i = 0U; i < sizeof(bad)/sizeof(bad[0]); ++i) {
		memset(&out, 0xA5, sizeof(out));
		TEST_ASSERT(!exo_phase_profile_evaluate_with_maximum(1U,
			EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK, 0.25f, 0.75f,
			100U, 100U, bad[i], &out) && pair_is_all_zero(&out), "bad maximum clears delta");
	}
	return true;
}

static exo_joint_command_pair_t make_base(float left_torque,
						   float right_torque) {
	return (exo_joint_command_pair_t){
		.left = {
			.position_deg = 12.0f,
			.velocity_deg_s = -3.0f,
			.torque_ff_nm = left_torque,
			.kp = 1.25f,
			.kd = 0.4f,
		},
		.right = {
			.position_deg = -9.0f,
			.velocity_deg_s = 2.0f,
			.torque_ff_nm = right_torque,
			.kp = 1.5f,
			.kd = 0.45f,
		},
	};
}

static bool test_command_composer(void) {
	const exo_joint_command_pair_t base = make_base(0.10f, -0.10f);
	exo_joint_command_pair_t delta = {0};
	delta.left.torque_ff_nm = 0.20f;
	delta.right.torque_ff_nm = -0.15f;
	exo_joint_command_pair_t output;
	TEST_ASSERT(exo_command_composer_add_torque_ff(&base, &delta, &output),
			"valid torque-only composition");
	TEST_ASSERT(nearly_equal(output.left.torque_ff_nm, 0.30f, 1.0e-6f) &&
			nearly_equal(output.right.torque_ff_nm, -0.25f, 1.0e-6f),
			"torque increments added");
	TEST_ASSERT(output.left.position_deg == base.left.position_deg &&
			output.left.velocity_deg_s == base.left.velocity_deg_s &&
			output.left.kp == base.left.kp && output.left.kd == base.left.kd &&
			output.right.position_deg == base.right.position_deg &&
			output.right.velocity_deg_s == base.right.velocity_deg_s &&
			output.right.kp == base.right.kp && output.right.kd == base.right.kd,
			"composer must preserve q/dq/kp/kd byte-for-value");

	exo_joint_command_pair_t zero_delta = {0};
	TEST_ASSERT(exo_command_composer_add_torque_ff(
				&base, &zero_delta, &output),
			"ACTIVE zero-percent path");
	TEST_ASSERT(memcmp(&output, &base, sizeof(output)) == 0,
			"zero delta must reproduce base pair exactly");
	return true;
}

static bool test_composer_rejects_wrong_shape(void) {
	exo_joint_command_pair_t base = make_base(0.0f, 0.0f);
	exo_joint_command_pair_t delta = {0};
	exo_joint_command_pair_t output;

	delta.left.position_deg = 1.0f;
	memset(&output, 0xA5, sizeof(output));
	TEST_ASSERT(!exo_command_composer_add_torque_ff(&base, &delta, &output),
			"position delta rejected");
	TEST_ASSERT(pair_is_all_zero(&output), "rejection clears output");

	delta = (exo_joint_command_pair_t){0};
	delta.left.torque_ff_nm = 0.5001f;
	TEST_ASSERT(!exo_command_composer_add_torque_ff(&base, &delta, &output),
			"delta over engineering bound rejected");

	delta = (exo_joint_command_pair_t){0};
	base.left.torque_ff_nm = 0.4f;
	delta.left.torque_ff_nm = 0.2f;
	TEST_ASSERT(!exo_command_composer_add_torque_ff(&base, &delta, &output),
			"composed torque over engineering bound rejected");

	base = make_base(0.0f, 0.0f);
	base.right.kd = NAN;
	delta = (exo_joint_command_pair_t){0};
	TEST_ASSERT(!exo_command_composer_add_torque_ff(&base, &delta, &output),
			"non-finite base rejected");
	TEST_ASSERT(!exo_command_composer_add_torque_ff(NULL, &delta, &output),
			"null base rejected");
	TEST_ASSERT(!exo_command_composer_add_torque_ff(&base, &delta, NULL),
			"null output rejected");
	return true;
}

typedef bool (*test_function_t)(void);

typedef struct {
	const char *name;
	test_function_t function;
} test_case_t;

int main(void) {
	static const test_case_t tests[] = {
		{"profile_metadata", test_profile_metadata},
		{"zero_and_independent_profiles", test_zero_and_independent_profiles},
		{"interpolation_scaling_and_wrap", test_interpolation_scaling_and_wrap},
		{"profile_rejects_bad_inputs", test_profile_rejects_bad_inputs},
		{"configurable_maximum", test_configurable_maximum},
		{"command_composer", test_command_composer},
		{"composer_rejects_wrong_shape", test_composer_rejects_wrong_shape},
	};
	unsigned passed = 0U;
	for (size_t i = 0U; i < sizeof(tests) / sizeof(tests[0]); ++i) {
		printf("RUN  %s\n", tests[i].name);
		if (!tests[i].function()) {
			printf("RESULT: %u/%zu passed\n", passed,
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
