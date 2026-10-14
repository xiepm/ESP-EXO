#include "exo_joint_command.h"

#include <math.h>
#include <string.h>

#include "hip_joint.h"

/* 闭区间检查；进入前已由调用者排除 NaN/Inf。 */
static bool value_is_in_range(float value, float minimum, float maximum) {
	return value >= minimum && value <= maximum;
}

bool exo_joint_command_is_valid(const exo_joint_command_t *command) {
	if (command == NULL || !isfinite(command->position_deg) ||
		!isfinite(command->velocity_deg_s) ||
		!isfinite(command->torque_ff_nm) || !isfinite(command->kp) ||
		!isfinite(command->kd)) {
		return false;
	}

	const float position_rad = exo_degrees_to_radians(command->position_deg);
	const float velocity_rad_s =
		exo_degrees_to_radians(command->velocity_deg_s);
	if (!isfinite(position_rad) || !isfinite(velocity_rad_s)) {
		return false;
	}

	const rs01_limits_t *limits = rs01_get_limits();
	if (limits == NULL) {
		/* 当前实现总会返回静态量程；保留检查可防止未来驱动改造后误解空指针。 */
		return false;
	}

	/*
	 * direction 可能是 +1 或 -1。这里同时接受“原符号或翻转后符号能够进入量程”
	 * 的值，最终使用哪一个符号由 hip_joint 的真实安装配置决定并再次严格校验。
	 * position 先按 RS01 的相对幅值范围约束，使 UART 能写入的 selected 配置也
	 * 一定能被 BLE 的 mrad 定点格式无损读回；叠加软件零位后的绝对位置在映射和
	 * START 包络预检时还会再次校验。
	 */
	const bool position_possible =
		value_is_in_range(position_rad, limits->position_min_rad,
						  limits->position_max_rad) ||
		value_is_in_range(-position_rad, limits->position_min_rad,
						  limits->position_max_rad);
	const bool velocity_possible =
		value_is_in_range(velocity_rad_s, limits->velocity_min_rad_s,
						  limits->velocity_max_rad_s) ||
		value_is_in_range(-velocity_rad_s, limits->velocity_min_rad_s,
						  limits->velocity_max_rad_s);
	const bool torque_possible =
		value_is_in_range(command->torque_ff_nm, limits->torque_min_nm,
						  limits->torque_max_nm) ||
		value_is_in_range(-command->torque_ff_nm, limits->torque_min_nm,
						  limits->torque_max_nm);

	return position_possible && velocity_possible && torque_possible &&
		   value_is_in_range(command->kp, limits->kp_min, limits->kp_max) &&
		   value_is_in_range(command->kd, limits->kd_min, limits->kd_max);
}

esp_err_t exo_joint_command_map_pair(
	const exo_joint_command_pair_t *joint_commands,
	rs01_motion_command_t actuator_commands[EXO_ACTUATOR_COUNT]) {
	if (actuator_commands == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	/*
	 * API 承诺错误时不发布半套结果，因此一进入函数就清零输出，实际映射先全部
	 * 写在栈上局部变量中。这里没有堆分配，也没有锁或总线操作。
	 */
	memset(actuator_commands, 0,
		   sizeof(*actuator_commands) * EXO_ACTUATOR_COUNT);
	if (joint_commands == NULL ||
		!exo_joint_command_is_valid(&joint_commands->left) ||
		!exo_joint_command_is_valid(&joint_commands->right)) {
		return ESP_ERR_INVALID_ARG;
	}

	exo_actuator_id_t left_actuator = EXO_ACTUATOR_COUNT;
	exo_actuator_id_t right_actuator = EXO_ACTUATOR_COUNT;
	rs01_motion_command_t left_mapped = {0};
	rs01_motion_command_t right_mapped = {0};

	/*
	 * 两侧都只做纯映射。即使左侧失败，仍计算右侧结果，随后统一决定是否发布；
	 * 这不会产生 CAN 副作用，并使一次调用完整检查两侧当前映射状态。
	 */
	const esp_err_t left_error =
		hip_joint_to_actuator_command(EXO_HIP_JOINT_LEFT, &joint_commands->left,
									  &left_actuator, &left_mapped);
	const esp_err_t right_error = hip_joint_to_actuator_command(
		EXO_HIP_JOINT_RIGHT, &joint_commands->right, &right_actuator,
		&right_mapped);
	if (left_error != ESP_OK) {
		return left_error;
	}
	if (right_error != ESP_OK) {
		return right_error;
	}

	/*
	 * 两个髋关节必须映射到两个不同且完整的 A/B 槽位。若配置把双腿都指向同一台
	 * 电机，宁可整对拒绝，也不能让后一条命令覆盖前一条后继续发送。
	 */
	if ((unsigned)left_actuator >= (unsigned)EXO_ACTUATOR_COUNT ||
		(unsigned)right_actuator >= (unsigned)EXO_ACTUATOR_COUNT ||
		left_actuator == right_actuator) {
		return ESP_ERR_INVALID_STATE;
	}

	rs01_motion_command_t mapped[EXO_ACTUATOR_COUNT] = {0};
	mapped[left_actuator] = left_mapped;
	mapped[right_actuator] = right_mapped;

	/*
	 * 当前产品恰好有 A/B 两个槽位；上面的“两个不同且都在范围内”已经证明两个
	 * 元素都被填充。复制发生在最后一步，保证调用者永远看不到半映射结果。
	 */
	memcpy(actuator_commands, mapped, sizeof(mapped));
	return ESP_OK;
}

esp_err_t exo_joint_command_try_send_pair_detailed(
	const exo_joint_command_pair_t *joint_commands,
	exo_joint_command_send_result_t *send_result) {
	if (send_result == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	memset(send_result, 0, sizeof(*send_result));
	send_result->map_error = ESP_OK;
	for (unsigned actuator = 0U; actuator < EXO_ACTUATOR_COUNT; ++actuator) {
		send_result->actuator_error[actuator] = ESP_ERR_INVALID_STATE;
	}

	rs01_motion_command_t commands[EXO_ACTUATOR_COUNT];
	const esp_err_t map_error =
		exo_joint_command_map_pair(joint_commands, commands);
	send_result->map_error = map_error;
	if (map_error != ESP_OK) {
		/* 最重要的不变量：任一侧预检失败时，一帧 CAN 都不能发送。 */
		return map_error;
	}

	/*
	 * 固定 A -> B 发送能让每个 10 ms 周期的总线顺序可重复。A 失败后仍尝试 B，
	 * 避免一次短暂的 A 队列/锁错误无条件饿死 B；返回值仍保留遇到的首个错误。
	 */
	send_result->actuator_attempted[EXO_ACTUATOR_A] = true;
	const esp_err_t actuator_a_error =
		rs01_try_send_motion_command(EXO_ACTUATOR_A, &commands[EXO_ACTUATOR_A]);
	send_result->actuator_error[EXO_ACTUATOR_A] = actuator_a_error;
	send_result->actuator_attempted[EXO_ACTUATOR_B] = true;
	const esp_err_t actuator_b_error =
		rs01_try_send_motion_command(EXO_ACTUATOR_B, &commands[EXO_ACTUATOR_B]);
	send_result->actuator_error[EXO_ACTUATOR_B] = actuator_b_error;
	return actuator_a_error != ESP_OK ? actuator_a_error : actuator_b_error;
}

esp_err_t exo_joint_command_try_send_pair(
	const exo_joint_command_pair_t *joint_commands) {
	exo_joint_command_send_result_t ignored;
	return exo_joint_command_try_send_pair_detailed(joint_commands, &ignored);
}
