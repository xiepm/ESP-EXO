#ifndef EXO_TYPES_H
#define EXO_TYPES_H

#include <stdint.h>

/*
 * 固件内部公共类型（建议新手先读本文件，再读各业务模块）。
 *
 * 这些结构不是 BLE/CAN 线协议，禁止用 memcpy 直接序列化。产品层以“度”描述
 * 人体关节；RS01 驱动层仍以电机手册规定的“弧度”收发。两种单位只能通过下面的
 * 显式转换函数跨越边界，避免把关节角直接误当成电机协议量。
 *
 * 名词关系如下：
 *   CAN 节点号（0x10/0x20） -> 总线上的电机地址；
 *   执行器 A/B              -> 固件内部稳定的数组槽位；
 *   髋关节 LEFT/RIGHT       -> 穿戴者的身体侧别。
 * 三者不是同一种 ID。它们之间的对应关系集中写在 exo_config.c，业务代码不得
 * 依靠数值大小或字母自行猜测。
 */

/* 采用常量乘法而不是在各文件中手写 180/pi，保证所有边界使用同一换算精度。 */
#define EXO_DEGREES_PER_RADIAN 57.2957795130823208768f
#define EXO_RADIANS_PER_DEGREE 0.01745329251994329577f

/* 弧度转度：用于把 RS01 原始反馈提升到人体关节层或显示层。 */
static inline float exo_radians_to_degrees(float radians) {
	return radians * EXO_DEGREES_PER_RADIAN;
}

/* 度转弧度：用于把人体关节目标下沉到 RS01 驱动或 BLE v1 载荷。 */
static inline float exo_degrees_to_radians(float degrees) {
	return degrees * EXO_RADIANS_PER_DEGREE;
}

/* RS01 节点号在线协议中只占一个字节，单独命名可避免与普通计数值混用。 */
typedef uint8_t exo_can_node_id_t;

/* UNKNOWN 用于“尚未配置/尚未识别”，不可当作第三条腿参与控制。 */
typedef enum {
	EXO_SIDE_UNKNOWN = 0,
	EXO_SIDE_LEFT = 1,
	EXO_SIDE_RIGHT = 2,
} exo_side_t;

/*
 * 逻辑执行器槽位。A/B 只决定数组索引和发送顺序，不代表人体的左腿或右腿；
 * A/B 与 LEFT/RIGHT 的物理关系必须从硬件配置读取。
 */
typedef enum {
	EXO_ACTUATOR_A = 0,
	EXO_ACTUATOR_B = 1,
	/* COUNT 既是数组长度，也是“没有有效执行器”的哨兵；不可拿它访问数组。 */
	EXO_ACTUATOR_COUNT = 2,
} exo_actuator_id_t;

/* 人体髋关节对象。LEFT/RIGHT 表示穿戴者侧别，不表示 CAN ID。 */
typedef enum {
	EXO_HIP_JOINT_LEFT = 0,
	EXO_HIP_JOINT_RIGHT = 1,
	/* 与执行器相同，COUNT 只用于循环边界或无效值。 */
	EXO_HIP_JOINT_COUNT = 2,
} exo_hip_joint_id_t;

/*
 * 电机正方向到人体关节正方向的符号。当前人体约定为“髋关节屈曲为正”，因此
 * 同一关节正向在不同安装方向的电机上可能分别对应 +1 或 -1。
 */
typedef enum {
	EXO_DIRECTION_UNKNOWN = 0,
	/* 枚举值刻意取 +1/-1，坐标换算时可直接作为乘法符号使用。 */
	EXO_DIRECTION_POSITIVE = 1,
	EXO_DIRECTION_NEGATIVE = -1,
} exo_direction_t;

/*
 * RS01 执行器层原始采样。位置和速度保留手册单位 rad、rad/s；力矩为 N·m，
 * 温度为 °C。该类型不得直接作为人体关节层或手机显示层的数据类型。
 */
typedef struct {
	/* 电机输出轴的绝对位置；它尚未扣除人体中立位。 */
	float position_rad;
	/* 电机输出轴速度。字段名中的 _rad_s 就是 rad/s，防止单位靠记忆。 */
	float velocity_rad_s;
	float torque_nm;
	float temperature_c;
} exo_motion_sample_t;

/*
 * 人体髋关节层采样。关节位置和速度统一使用 deg、deg/s；力矩正方向也随关节
 * 方向换算，温度不做方向换算。后续五元命令和步态估计应复用这一单位约定。
 */
typedef struct {
	/* 已扣除软件零位并按“屈曲为正”换向后的位置。 */
	float position_deg;
	float velocity_deg_s;
	float torque_nm;
	float temperature_c;
} exo_joint_sample_t;

/*
 * 控制器交给“人体髋关节层”的统一五元命令。
 *
 * 前三个字段使用人体关节坐标：位置/速度的正方向都是髋关节屈曲，前馈力矩也以
 * 同一个正方向为准。kp、kd 是 RS01 Motion 协议中的两个增益字段；当前手册没有
 * 给出足以证明其物理刚度/阻尼单位的依据，因此这里不把它们命名成 N·m/deg 等
 * 物理量。该结构只是固件内部对象，不能直接 memcpy 成 BLE 或 CAN 数据帧。
 */
typedef struct {
	/* 目标人体髋关节角，单位 deg。 */
	float position_deg;
	/* 目标人体髋关节角速度，单位 deg/s。 */
	float velocity_deg_s;
	/* 人体关节正方向上的前馈力矩，单位 N·m。 */
	float torque_ff_nm;
	/* RS01 Motion 位置增益，允许范围最终由 RS01 驱动校验。 */
	float kp;
	/* RS01 Motion 速度增益，允许范围最终由 RS01 驱动校验。 */
	float kd;
} exo_joint_command_t;

/*
 * 一个 100 Hz 控制周期内左右髋关节必须一起生成、一起校验的命令对。
 * 使用具名 left/right，而不是下标 0/1，能让控制算法明确自己描述的是人体侧别；
 * LEFT/RIGHT 到执行器 A/B 的实际映射仍由 hip_joint 层读取硬件配置决定。
 */
typedef struct {
	exo_joint_command_t left;
	exo_joint_command_t right;
} exo_joint_command_pair_t;

/*
 * 下面的静态断言在“编译时”检查枚举是否仍能安全作为数组下标。它们不占运行时
 * CPU，也不会生成周期逻辑；若有人改坏约定，编译会在这里直接失败。
 * C++ 使用 static_assert，C 使用同义的 _Static_assert。
 */
#if defined(__cplusplus)
static_assert(EXO_ACTUATOR_A == 0, "actuator A must index element 0");
static_assert(EXO_ACTUATOR_B == 1, "actuator B must index element 1");
static_assert(EXO_ACTUATOR_COUNT == 2,
			  "phase 1 supports exactly two actuators");
static_assert(EXO_HIP_JOINT_LEFT == 0, "left hip must index element 0");
static_assert(EXO_HIP_JOINT_RIGHT == 1, "right hip must index element 1");
static_assert(EXO_HIP_JOINT_COUNT == 2, "exactly two hip joints are supported");
#else
_Static_assert(EXO_ACTUATOR_A == 0, "actuator A must index element 0");
_Static_assert(EXO_ACTUATOR_B == 1, "actuator B must index element 1");
_Static_assert(EXO_ACTUATOR_COUNT == 2,
			   "phase 1 supports exactly two actuators");
_Static_assert(EXO_HIP_JOINT_LEFT == 0, "left hip must index element 0");
_Static_assert(EXO_HIP_JOINT_RIGHT == 1, "right hip must index element 1");
_Static_assert(EXO_HIP_JOINT_COUNT == 2,
			   "exactly two hip joints are supported");
#endif

#endif /* EXO_TYPES_H */
