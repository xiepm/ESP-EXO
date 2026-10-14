#ifndef EXO_COMMAND_COMPOSER_H
#define EXO_COMMAND_COMPOSER_H

#include <stdbool.h>

#include "exo_types.h"

/*
 * 阶段 10 唯一命令合成器
 * ======================
 *
 * 高层相位曲线只能给阶段 8 的基础五元命令增加 torque_ff_nm，不能覆盖位置、
 * 速度或 KP/KD。合成器是纯函数：不持有上一帧、不发 CAN、不读取全局状态。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 总出口绝对边界为15 N·m；每轮START还会锁存该策略/配置的更小边界。
 * 原相位工程曲线与旧接口仍受0.5 N·m限制。 */
#define EXO_COMMAND_COMPOSER_MAX_ABS_ACTIVE_TORQUE_MNM 15000U
#define EXO_COMMAND_COMPOSER_MAX_ABS_ACTIVE_TORQUE_NM 15.0f
#define EXO_COMMAND_COMPOSER_LEGACY_TORQUE_NM 0.5f

/*
 * torque_delta 的 position/velocity/kp/kd 必须严格为零；两侧前馈力矩必须有限且
 * 不超过低幅边界。成功时 output=base+delta，失败时 output 保持全零。
 */
bool exo_command_composer_add_torque_ff(
	const exo_joint_command_pair_t *base,
	const exo_joint_command_pair_t *torque_delta,
	exo_joint_command_pair_t *output);

/* 新入口：用START锁存的配置峰值检查增量与合成后的最终力矩。
 * maximum_abs_torque_nm须有限且处于0–5 N·m；0只允许全零前馈力矩。
 * 失败清空output，绝不截断并继续发送一个未验证的命令。 */
bool exo_command_composer_add_torque_ff_bounded(
    const exo_joint_command_pair_t *base,
    const exo_joint_command_pair_t *torque_delta,
    float maximum_abs_torque_nm, exo_joint_command_pair_t *output);

#ifdef __cplusplus
}
#endif

#endif /* EXO_COMMAND_COMPOSER_H */
