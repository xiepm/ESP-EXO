#include "exo_command_composer.h"

#include <math.h>
#include <string.h>

/* 主机测试可以直接编译本文件，因此这里不依赖驱动或 hip_joint 硬件量程模块。 */

static bool command_is_finite(const exo_joint_command_t *command) {
	return command != NULL && isfinite(command->position_deg) &&
		   isfinite(command->velocity_deg_s) &&
		   isfinite(command->torque_ff_nm) && isfinite(command->kp) &&
		   isfinite(command->kd);
}

/* 判断策略是否真的只提出前馈力矩增量，防止未来误把位置建议接入 ACTIVE。 */
static bool is_pure_torque_delta(const exo_joint_command_t *delta, float limit_nm) {
	return delta != NULL && isfinite(delta->torque_ff_nm) &&
		   delta->position_deg == 0.0f && delta->velocity_deg_s == 0.0f &&
		   delta->kp == 0.0f && delta->kd == 0.0f &&
		   fabsf(delta->torque_ff_nm) <= limit_nm;
}

bool exo_command_composer_add_torque_ff(
	const exo_joint_command_pair_t *base,
	const exo_joint_command_pair_t *torque_delta,
	exo_joint_command_pair_t *output) {
    /* 保留历史低幅入口，避免扩大原相位回放和其他旧调用者的许可范围。 */
    return exo_command_composer_add_torque_ff_bounded(
        base, torque_delta, EXO_COMMAND_COMPOSER_LEGACY_TORQUE_NM, output);
}

bool exo_command_composer_add_torque_ff_bounded(
    const exo_joint_command_pair_t *base,
    const exo_joint_command_pair_t *torque_delta,
    float maximum_abs_torque_nm, exo_joint_command_pair_t *output) {
	if (output == NULL) {
		return false;
	}
	memset(output, 0, sizeof(*output));
	if (base == NULL || torque_delta == NULL ||
        !isfinite(maximum_abs_torque_nm) || maximum_abs_torque_nm < 0.0f ||
        maximum_abs_torque_nm > EXO_COMMAND_COMPOSER_MAX_ABS_ACTIVE_TORQUE_NM ||
		!command_is_finite(&base->left) ||
		!command_is_finite(&base->right) ||
		!is_pure_torque_delta(&torque_delta->left, maximum_abs_torque_nm) ||
		!is_pure_torque_delta(&torque_delta->right, maximum_abs_torque_nm)) {
		return false;
	}

	exo_joint_command_pair_t composed = *base;
	composed.left.torque_ff_nm += torque_delta->left.torque_ff_nm;
	composed.right.torque_ff_nm += torque_delta->right.torque_ff_nm;

	/*
	 * ACTIVE 当前只允许 DAMPING+CONSTANT，所以基础 torque_ff 应为零；这里仍做
	 * 独立的最终限幅检查：只允许本轮锁存的峰值，不能只在网页或策略内限幅。
	 */
	if (!isfinite(composed.left.torque_ff_nm) ||
		!isfinite(composed.right.torque_ff_nm) ||
		fabsf(composed.left.torque_ff_nm) >
			maximum_abs_torque_nm ||
		fabsf(composed.right.torque_ff_nm) >
			maximum_abs_torque_nm ||
		!command_is_finite(&composed.left) ||
		!command_is_finite(&composed.right)) {
		return false;
	}

	*output = composed;
	return true;
}
