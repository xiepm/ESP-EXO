#include "exo_phase_profile.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * profile 0 是全零诊断曲线。它用于先打通 ACTIVE 路由而不增加力矩，也让旧版
 * 配置中默认 profile_id=0 保持确定含义。
 */
static const float s_zero_left_shape[EXO_PHASE_PROFILE_LUT_POINTS] = {0.0f};
static const float s_zero_right_shape[EXO_PHASE_PROFILE_LUT_POINTS] = {0.0f};

/*
 * ENGINEERING_TEST_V1 是低幅工程验收曲线，不是任何论文曲线，也不代表推荐人体
 * 助力时机。LUT 保存归一化 [-1, 1] 形状，峰值由 metadata 独立版本化；16 个点
 * 构成周期闭合、正负对称、离散均值为零的双向曲线，level=100% 时峰值仅为
 * 0.5 N·m。LEFT/RIGHT 分别保留独立 Flash LUT，当前数值相同，但插值时各自
 * 消费本侧相位，不能用左腿相位替代右腿相位。
 */
static const float
	s_engineering_test_left_shape[EXO_PHASE_PROFILE_LUT_POINTS] = {
	0.000000f,  0.382683f,  0.707107f,  0.923880f,
	1.000000f,  0.923880f,  0.707107f,  0.382683f,
	0.000000f, -0.382683f, -0.707107f, -0.923880f,
	-1.000000f, -0.923880f, -0.707107f, -0.382683f,
};

static const float
	s_engineering_test_right_shape[EXO_PHASE_PROFILE_LUT_POINTS] = {
	0.000000f,  0.382683f,  0.707107f,  0.923880f,
	1.000000f,  0.923880f,  0.707107f,  0.382683f,
	0.000000f, -0.382683f, -0.707107f, -0.923880f,
	-1.000000f, -0.923880f, -0.707107f, -0.382683f,
};

typedef struct {
	exo_phase_profile_info_t info;
	const float *left_shape;
	const float *right_shape;
} exo_phase_profile_entry_t;

/* 所有曲线和索引都为 static const，链接后位于只读 Flash，不占运行期堆。 */
static const exo_phase_profile_entry_t s_profiles[] = {
	{
		.info = {
			.profile_id = EXO_PHASE_PROFILE_ID_ZERO_DIAGNOSTIC,
			.profile_version = 1U,
			.name = "ZERO_DIAGNOSTIC_V1",
			.required_phase_origin =
				EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK,
			.point_count = EXO_PHASE_PROFILE_LUT_POINTS,
			.active_allowed = true,
			.literature_profile = false,
			.maximum_abs_torque_nm = 0.0f,
		},
		.left_shape = s_zero_left_shape,
		.right_shape = s_zero_right_shape,
	},
	{
		.info = {
			.profile_id = EXO_PHASE_PROFILE_ID_ENGINEERING_TEST_V1,
			.profile_version = 1U,
			.name = "ENGINEERING_TEST_V1",
			.required_phase_origin =
				EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK,
			.point_count = EXO_PHASE_PROFILE_LUT_POINTS,
			.active_allowed = true,
			.literature_profile = false,
			.maximum_abs_torque_nm = 0.5f,
		},
		.left_shape = s_engineering_test_left_shape,
		.right_shape = s_engineering_test_right_shape,
	},
};

static const exo_phase_profile_entry_t *find_profile(uint8_t profile_id) {
	for (size_t i = 0U; i < sizeof(s_profiles) / sizeof(s_profiles[0]); ++i) {
		if (s_profiles[i].info.profile_id == profile_id) {
			return &s_profiles[i];
		}
	}
	return NULL;
}

bool exo_phase_profile_get_info(uint8_t profile_id,
					exo_phase_profile_info_t *info) {
	if (info == NULL) {
		return false;
	}
	memset(info, 0, sizeof(*info));
	const exo_phase_profile_entry_t *entry = find_profile(profile_id);
	if (entry == NULL) {
		return false;
	}
	*info = entry->info;
	return true;
}

/*
 * 等间隔周期 LUT 的 O(1) 插值。末点的后一项通过取模回到第 0 项，因此无需在
 * Flash 中重复保存 phase=1 的端点，phase 接近 1 时也会连续回到 phase=0。
 */
static float interpolate_periodic(const float *lut, float phase_01) {
	const float scaled = phase_01 * (float)EXO_PHASE_PROFILE_LUT_POINTS;
	const size_t left_index = (size_t)scaled;
	const size_t right_index =
		(left_index + 1U) % EXO_PHASE_PROFILE_LUT_POINTS;
	const float fraction = scaled - (float)left_index;
	return lut[left_index] + fraction * (lut[right_index] - lut[left_index]);
}

bool exo_phase_profile_evaluate(uint8_t profile_id,
					exo_gait_phase_origin_t phase_origin,
					float left_phase_01, float right_phase_01,
					uint8_t left_level_percent,
					uint8_t right_level_percent,
					exo_joint_command_pair_t *output) {
	return exo_phase_profile_evaluate_with_maximum(
		profile_id, phase_origin, left_phase_01, right_phase_01,
		left_level_percent, right_level_percent,
		(float)EXO_PHASE_PROFILE_TORQUE_DEFAULT_MNM * 0.001f, output);
}

bool exo_phase_profile_evaluate_with_maximum(
	uint8_t profile_id, exo_gait_phase_origin_t phase_origin,
	float left_phase_01, float right_phase_01,
	uint8_t left_level_percent, uint8_t right_level_percent,
	float maximum_assist_torque_nm, exo_joint_command_pair_t *output) {
	if (output == NULL) {
		return false;
	}
	memset(output, 0, sizeof(*output));

	const exo_phase_profile_entry_t *entry = find_profile(profile_id);
	if (entry == NULL || phase_origin != entry->info.required_phase_origin ||
		!isfinite(left_phase_01) || !isfinite(right_phase_01) ||
		left_phase_01 < 0.0f || left_phase_01 >= 1.0f ||
		right_phase_01 < 0.0f || right_phase_01 >= 1.0f ||
		left_level_percent > 100U || right_level_percent > 100U ||
		!isfinite(maximum_assist_torque_nm) || maximum_assist_torque_nm < 0.0f ||
		maximum_assist_torque_nm >
			(float)EXO_PHASE_PROFILE_TORQUE_HARD_MAX_MNM * 0.001f ||
		entry->info.point_count != EXO_PHASE_PROFILE_LUT_POINTS ||
		entry->left_shape == NULL || entry->right_shape == NULL) {
		return false;
	}

	const float left_scale = (float)left_level_percent * 0.01f;
	const float right_scale = (float)right_level_percent * 0.01f;
	/* ZERO_DIAGNOSTIC 的曲线峰值为0，仍必须保持全零输出。 */
	const float peak_nm = fminf(maximum_assist_torque_nm,
							  entry->info.maximum_abs_torque_nm);
	const float left_torque =
		interpolate_periodic(entry->left_shape, left_phase_01) *
		peak_nm * left_scale;
	const float right_torque =
		interpolate_periodic(entry->right_shape, right_phase_01) *
		peak_nm * right_scale;
	if (!isfinite(left_torque) || !isfinite(right_torque) ||
		fabsf(left_torque) > peak_nm || fabsf(right_torque) > peak_nm) {
		return false;
	}

	output->left.torque_ff_nm = left_torque;
	output->right.torque_ff_nm = right_torque;
	return true;
}
