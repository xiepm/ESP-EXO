#ifndef HIP_JOINT_H
#define HIP_JOINT_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "exo_types.h"
#include "rs01_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 人体髋关节模块的公开接口。
 *
 * RS01 驱动只知道“执行器 A/B 的电机角”，本模块把它解释成“左/右髋关节角”。
 * 转换包含三件事：查左右映射、减去中立位软件零点、按安装方向乘 +1/-1。
 * 上层控制、状态机和显示应读取本模块，不应重复实现左右腿换算公式。
 */

/*
 * 某一侧髋关节在一个读取时刻的完整只读快照。
 * “快照”表示调用者拿到的是一份一致副本，拿到后底层即使收到新 CAN 帧，也不会
 * 在调用者读到一半时修改这份结构。
 */
typedef struct {
	/* 关节身份和配置映射；LEFT/RIGHT 与执行器 A/B 是两个独立概念。 */
	exo_hip_joint_id_t joint;
	exo_side_t side;
	exo_actuator_id_t actuator;
	exo_direction_t direction;

	/*
	 * 两个采样对应同一个 feedback_timestamp_us。raw_actuator_sample 是
	 * RS01 电机坐标（rad），joint_sample 是人体关节坐标（deg）。
	 */
	exo_motion_sample_t raw_actuator_sample;
	/*
	 * RS01 原始位置每 8π rad 回绕；下面两项是驱动按相邻反馈展开后的连续位置和
	 * 跨界计数。人体关节角使用连续位置，raw_actuator_sample 仍保留线上原值。
	 */
	float continuous_actuator_position_rad;
	int32_t actuator_position_wrap_count;
	uint32_t actuator_position_epoch;
	uint32_t actuator_position_rebase_count;
	exo_joint_sample_t joint_sample;
	/* 人体中立位对应的连续电机输出轴角度；它不是关节当前角度。 */
	float software_zero_motor_deg;
	/* 同一零位在 RS01 -4π～+4π 线上周期坐标中的值，仅用于生成合法命令。 */
	float software_zero_periodic_motor_deg;

	/* 反馈来源和归零来源分开记录，便于确认零位没有被旧反馈覆盖。 */
	/* 时间戳来自 ESP32 单调时钟，单位 us；count 用于识别是否得到过更新。 */
	uint64_t feedback_timestamp_us;
	uint32_t source_feedback_count;
	uint64_t zero_source_timestamp_us;
	uint32_t zero_source_feedback_count;
	uint32_t zero_source_position_epoch;

	/* joint_valid 是映射、方向、零位、反馈和在线状态的最终合取结果。 */
	/*
	 * 分项 valid 便于定位“为什么当前不能控制”：可能是没映射、没标方向、没归零、
	 * 反馈格式无效或电机超时离线，而不是只给一个笼统失败。
	 */
	bool mapping_valid;
	bool direction_valid;
	bool zero_valid;
	bool feedback_valid;
	bool continuous_position_valid;
	bool zero_epoch_matches;
	bool online;
	bool joint_valid;
} hip_joint_snapshot_t;

/*
 * 从 g_exo_hardware_config 初始化左右两个静态髋关节对象。
 * 可重复调用：首次成功后再次调用直接返回 ESP_OK；函数不创建堆对象或后台任务。
 */
esp_err_t hip_joint_init(void);

/*
 * 返回 true 只表示参数有效且快照已复制；控制前仍必须检查 joint_valid，不能把
 * “成功读取结构体”误认为“关节反馈可用于运动”。
 */
bool hip_joint_get_snapshot(exo_hip_joint_id_t joint,
							hip_joint_snapshot_t *snapshot);

/*
 * 把当前有效 RS01 输出轴位置保存为 RAM 软件零位（deg）。本函数不发送 CAN，
 * 也不会修改电机机械零位；设备复位后必须在中立位重新执行。
 */
esp_err_t hip_joint_set_zero_from_current(exo_hip_joint_id_t joint);

/*
 * 只清除指定一侧的 RAM 软件零位，不影响另一侧，也不向电机发送“机械归零”。
 * 清除后该侧 joint_valid 会保持 false，直到重新在中立位采集零点。
 */
esp_err_t hip_joint_clear_zero(exo_hip_joint_id_t joint);

/*
 * 把一侧人体五元命令完整映射为 RS01 电机五元命令：
 *
 *   motor_position_rad = deg_to_rad(zero_motor_deg + direction * joint_deg)
 *   motor_velocity_rad_s = direction * deg_to_rad(joint_velocity_deg_s)
 *   motor_torque_nm = direction * joint_torque_ff_nm
 *   motor_kp = joint_kp
 *   motor_kd = joint_kd
 *
 * q、dq、tau 都是带方向的物理量，所以一起乘安装方向；kp、kd 是协议增益，不能
 * 随右腿安装方向变成负数。本函数会只读一次 RS01 快照以核对在线状态和连续位置
 * epoch，但不发送 CAN。任一输入无效、关节未完成映射/方向/软件归零、零位 epoch
 * 已过期或映射后超出 RS01 量程时，输出会保持安全哨兵/全零并返回错误；位置目标
 * 不会自动跨边界取模，也不会静默限幅。
 */
esp_err_t hip_joint_to_actuator_command(
	exo_hip_joint_id_t joint, const exo_joint_command_t *joint_command,
	exo_actuator_id_t *actuator, rs01_motion_command_t *actuator_command);

/*
 * 把人体关节目标（deg）解析为对应 RS01 目标（rad）：
 * motor_rad = deg_to_rad(zero_motor_deg + direction * joint_deg)。
 * 这是阶段 3～7 调用者继续使用的兼容接口，内部复用完整五元映射（其余四项为
 * 零）。本函数不发送 CAN，但会随完整映射接口核对最新反馈 epoch；缺少映射、
 * 方向、有效软件零点或在线连续反馈时返回 ESP_ERR_INVALID_STATE。
 */
esp_err_t hip_joint_to_actuator_position(exo_hip_joint_id_t joint,
										 float joint_position_deg,
										 exo_actuator_id_t *actuator,
										 float *actuator_position_rad);

#ifdef __cplusplus
}
#endif

#endif /* HIP_JOINT_H */
