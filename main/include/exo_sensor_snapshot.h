#ifndef EXO_SENSOR_SNAPSHOT_H
#define EXO_SENSOR_SNAPSHOT_H

#include <stdbool.h>
#include <stdint.h>

#include "exo_types.h"

/*
 * 阶段 9 统一传感快照
 * ==================
 *
 * 这个模块解决的是“算法在同一个 100 Hz 周期里究竟看到了哪一批数据”。TWAI
 * 接收任务会异步更新左右电机反馈，因此这里的左右数据并不宣称来自硬件同一采样
 * 时刻；它们只是唯一控制任务在同一次唤醒中复制到的一对最新完整反馈。为避免把
 * 这种软件配对误认为硬件同步采样，快照同时保留两侧源时间和 pair_skew_us。
 *
 * 结构体属于固件内部类型，不是 BLE/CAN 线格式，禁止直接 memcpy 到无线帧。
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 传感器能力位既用于每周期有效性，也供助力策略声明依赖。当前真机只有髋关节
 * 编码器运动量和 RS01 的电机估算力矩；足底、IMU、交互力矩、sEMG 均只预留一
 * 个位，不为尚不存在的硬件分配大数组或填入伪数据。
 */
typedef enum {
	EXO_SENSOR_LEFT_HIP_KINEMATICS = 1U << 0,
	EXO_SENSOR_RIGHT_HIP_KINEMATICS = 1U << 1,
	EXO_SENSOR_LEFT_MOTOR_TORQUE_ESTIMATE = 1U << 2,
	EXO_SENSOR_RIGHT_MOTOR_TORQUE_ESTIMATE = 1U << 3,
	EXO_SENSOR_LOWER_LIMB_IMU = 1U << 4,
	EXO_SENSOR_FOOT_CONTACT = 1U << 5,
	EXO_SENSOR_INTERACTION_TORQUE = 1U << 6,
	EXO_SENSOR_SEMG = 1U << 7,
} exo_sensor_mask_t;

/* 当前产品实际焊接并由固件支持的传感器能力。 */
#define EXO_SENSOR_CURRENT_HARDWARE_MASK                                      \
	((uint32_t)(EXO_SENSOR_LEFT_HIP_KINEMATICS |                         \
				EXO_SENSOR_RIGHT_HIP_KINEMATICS |                    \
				EXO_SENSOR_LEFT_MOTOR_TORQUE_ESTIMATE |              \
				EXO_SENSOR_RIGHT_MOTOR_TORQUE_ESTIMATE))

/* 构建快照时发现的质量问题；多个问题可以同时置位。 */
typedef enum {
	EXO_SENSOR_QUALITY_NONE = 0,
	EXO_SENSOR_QUALITY_LEFT_UNAVAILABLE = 1U << 0,
	EXO_SENSOR_QUALITY_RIGHT_UNAVAILABLE = 1U << 1,
	EXO_SENSOR_QUALITY_LEFT_STALE = 1U << 2,
	EXO_SENSOR_QUALITY_RIGHT_STALE = 1U << 3,
	EXO_SENSOR_QUALITY_PAIR_SKEW_HIGH = 1U << 4,
	EXO_SENSOR_QUALITY_TIME_INVALID = 1U << 5,
	EXO_SENSOR_QUALITY_COORDINATE_RESET = 1U << 6,
	EXO_SENSOR_QUALITY_SOURCE_ROLLBACK = 1U << 7,
	EXO_SENSOR_QUALITY_NONFINITE_SAMPLE = 1U << 8,
} exo_sensor_quality_flag_t;

/*
 * 一侧人体髋关节的算法输入。joint_sample.torque_nm 明确表示它是 RS01 对电机
 * 输出的估计值，包含电机、减速器、摩擦和外骨骼动力学影响，不能重命名为人体
 * 主动力矩或人机交互力矩。
 */
typedef struct {
	exo_joint_sample_t joint_sample;
	uint64_t source_timestamp_us;
	uint32_t source_feedback_count;
	uint32_t position_epoch;
	uint64_t zero_source_timestamp_us;
	uint32_t zero_source_feedback_count;
	uint32_t zero_source_position_epoch;
	uint32_t age_us;
	bool snapshot_available;
	bool mapping_valid;
	bool direction_valid;
	bool zero_valid;
	bool feedback_valid;
	bool online;
	bool joint_valid;
} exo_sensor_joint_snapshot_t;

/* 唯一控制周期发布给步态估计器的左右完整输入。 */
typedef struct {
	uint32_t sequence;
	int64_t captured_us;
	/* 坐标系或数据源发生复位时递增；估计器必须清空旧周期历史。 */
	uint32_t reset_epoch;
	uint32_t available_sensor_mask;
	uint32_t valid_sensor_mask;
	uint32_t quality_flags;
	uint32_t pair_skew_us;
	exo_sensor_joint_snapshot_t left;
	exo_sensor_joint_snapshot_t right;
} exo_sensor_snapshot_t;

/* 快照的时效阈值独立于 RS01 的“电机在线”阈值，供步态算法使用。 */
typedef struct {
	uint32_t feedback_freshness_us;
	uint32_t pair_skew_warning_us;
	uint32_t available_sensor_mask;
} exo_sensor_snapshot_config_t;

/*
 * builder 只由唯一 100 Hz 控制任务持有，不需要锁。字段公开只是为了允许静态分配
 * 和主机回放，不代表业务代码应直接修改内部历史。
 */
typedef struct {
	exo_sensor_snapshot_config_t config;
	uint32_t next_sequence;
	uint32_t reset_epoch;
	int64_t last_captured_us;
	bool have_left_signature;
	bool have_right_signature;
	exo_sensor_joint_snapshot_t previous_left;
	exo_sensor_joint_snapshot_t previous_right;
	/* 只在采集时钟首次进入异常状态时推进epoch，避免故障期间每10 ms反复复位。 */
	bool captured_time_invalid_active;
	bool initialized;
} exo_sensor_snapshot_builder_t;

/* 初始化纯内存 builder；不读取硬件、不创建任务，也不动态分配内存。 */
bool exo_sensor_snapshot_builder_init(
	exo_sensor_snapshot_builder_t *builder,
	const exo_sensor_snapshot_config_t *config);

/*
 * 从同一控制周期已经复制好的左右输入构建一个快照。输入指针可以为空，表示该侧
 * 本周期无法取得结构；输出仍会生成连续 sequence，并通过 quality_flags 说明原因。
 */
bool exo_sensor_snapshot_build(exo_sensor_snapshot_builder_t *builder,
				   int64_t captured_us,
				   const exo_sensor_joint_snapshot_t *left,
				   const exo_sensor_joint_snapshot_t *right,
				   exo_sensor_snapshot_t *output);

#ifdef __cplusplus
}
#endif

#endif /* EXO_SENSOR_SNAPSHOT_H */
