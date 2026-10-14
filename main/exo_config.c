#include "exo_config.h"

#include "sdkconfig.h"

/*
 * 产品静态配置总表。
 *
 * 阅读顺序建议：先看“基础常量”，再看“实机左右映射”，最后看文件底部两个全局
 * const 对象。#define/enum 让编译器能执行 _Static_assert；全局对象则给业务模块
 * 提供类型明确的只读访问。
 *
 * 本文件不会初始化硬件、不会发送 CAN、不会创建任务。改动此处必须重新编译烧录。
 */

/* 名称用于 BLE 广播；位率必须与两台 RS01 当前设置一致。 */
#define EXO_CONFIG_BLE_DEVICE_NAME "ESP-EXO"
#define EXO_CONFIG_TWAI_BITRATE_BPS 1000000U
/* 只影响 UART 手工命令默认选中 A，不决定哪一侧先运动。 */
#define EXO_CONFIG_DEFAULT_ACTUATOR EXO_ACTUATOR_A

/*
 * 阶段 7 行为参数也是阶段 8 的默认配置。参考轨迹是“中心位置 + 正弦偏移”：幅值
 * 90 deg、频率 0.5 Hz（每 2 秒一周期），前 3 秒由 0
 * 平滑增加到满幅。人体关节幅值映射到 RS01 时才转换为 pi/2
 * rad，因此物理波形与已经通过实机验收的版本保持一致。 KP/KD 是 RS01 Motion/MIT
 * 形式命令中的默认参数，不是在 ESP32 上另跑一个力矩环。 五元 selected 参数可在
 * IDLE 修改，但正弦轨迹本身仍是编译期只读配置。
 */
#define EXO_CONFIG_IMPEDANCE_KP 0.2f
#define EXO_CONFIG_IMPEDANCE_KD 0.10f
#define EXO_CONFIG_SINE_AMPLITUDE_DEG 90.0f
#define EXO_CONFIG_SINE_FREQUENCY_HZ 0.5f
#define EXO_CONFIG_SINE_RAMP_TIME_S 3.0f

/*
 * 工程交替激励不是产品步态发生器。首轮真机验收只使用单侧 10 deg（峰峰值
 * 20 deg）、0.5 Hz 和 3 s 五次渐入，幅值足以越过当前 6 deg 步态事件门槛，
 * 同时明显低于阶段7原有的 90 deg 同相正弦。以后调整必须升级版本并重新验收。
 */
#define EXO_CONFIG_ALTERNATING_TEST_AMPLITUDE_DEG 10.0f
#define EXO_CONFIG_ALTERNATING_TEST_FREQUENCY_HZ 0.5f
#define EXO_CONFIG_ALTERNATING_TEST_RAMP_TIME_S 3.0f

/*
 * 阶段 9 传感与步态基线参数。
 *
 * 50 ms 新鲜度比 RS01 的 100 ms“在线”阈值更严格：电机仍可被视为在线，但过旧
 * 的样本不会进入相位滤波器。左右源时间相差超过 20 ms 时仍保留独立侧估计，但
 * 降低全局置信度。滤波 alpha=0.25 对应 100 Hz 下约几赫兹的因果低通，不需要
 * 历史数组。周期 0.5～3.0 s 覆盖约 40～240 step/min 的初始部署范围。
 */
#define EXO_CONFIG_SENSOR_FRESHNESS_US 50000U
#define EXO_CONFIG_SENSOR_PAIR_SKEW_WARNING_US 20000U
#define EXO_CONFIG_GAIT_FILTER_ALPHA 0.25f
#define EXO_CONFIG_GAIT_TREND_VELOCITY_DEG_S 5.0f
#define EXO_CONFIG_GAIT_MOTION_VELOCITY_DEG_S 4.0f
#define EXO_CONFIG_GAIT_MINIMUM_EXCURSION_DEG 6.0f
#define EXO_CONFIG_GAIT_MINIMUM_PERIOD_S 0.5f
#define EXO_CONFIG_GAIT_MAXIMUM_PERIOD_S 3.0f
#define EXO_CONFIG_GAIT_MINIMUM_EVENT_INTERVAL_S 0.15f
#define EXO_CONFIG_GAIT_PERIOD_EWMA_ALPHA 0.25f
#define EXO_CONFIG_GAIT_UNSTABLE_PERIOD_FRACTION 0.35f
#define EXO_CONFIG_GAIT_BILATERAL_PERIOD_TOLERANCE_FRACTION 0.25f
#define EXO_CONFIG_GAIT_MINIMUM_BILATERAL_PHASE_SEPARATION 0.15f
#define EXO_CONFIG_GAIT_STANDING_TIMEOUT_US 1500000U
#define EXO_CONFIG_GAIT_PHASE_TIMEOUT_MARGIN_US 300000U
#define EXO_CONFIG_GAIT_LOCK_PERIOD_COUNT 2U

/*
 * 阶段 3 实机标定结果（2026-08-06）：
 *   LEFT  -> 执行器 B / CAN 0x20，髋屈曲时电机原始角增加；
 *   RIGHT -> 执行器 A / CAN 0x10，髋屈曲时电机原始角减小。
 * 人体关节统一约定“屈曲为正”，所以左右安装方向分别为 +1 和 -1。
 * 注意：LEFT/RIGHT 是人体语义，A/B 是软件槽位，0x10/0x20 才是 CAN 地址。
 */
#define EXO_CONFIG_LEFT_ACTUATOR EXO_ACTUATOR_B
#define EXO_CONFIG_RIGHT_ACTUATOR EXO_ACTUATOR_A
#define EXO_CONFIG_LEFT_DIRECTION EXO_DIRECTION_POSITIVE
#define EXO_CONFIG_RIGHT_DIRECTION EXO_DIRECTION_NEGATIVE
#define EXO_CONFIG_LEFT_MAPPING_VALID true
#define EXO_CONFIG_RIGHT_MAPPING_VALID true
#define EXO_CONFIG_LEFT_DIRECTION_VALID true
#define EXO_CONFIG_RIGHT_DIRECTION_VALID true

#if EXO_CONFIG_TWAI_BITRATE_BPS == 1000000U
/* 把容易阅读的 1000000 bit/s 映射成 ESP-IDF 所需的位时序结构。 */
#define EXO_CONFIG_TWAI_TIMING() TWAI_TIMING_CONFIG_1MBITS()
#else
#error "Add an ESP-IDF TWAI timing mapping for the configured bitrate"
#endif

enum {
	/* 32 帧队列用于吸收两个电机主动上报和命令应答短时叠加产生的突发流量。 */
	EXO_CONFIG_TWAI_RX_QUEUE_LENGTH = 32U,
	/* 0xFD 是主站身份；0x10/0x20 是当前两台电机的节点号。 */
	EXO_CONFIG_TWAI_MASTER_ID = 0xFD,
	EXO_CONFIG_ACTUATOR_A_CAN_ID = 0x10,
	EXO_CONFIG_ACTUATOR_B_CAN_ID = 0x20,
	EXO_CONFIG_CONTROL_FREQUENCY_HZ = 100U,
	/* 10 ms 主动上报等于 100 Hz；100 ms 无有效反馈则判为离线。 */
	EXO_CONFIG_RS01_REPORT_INTERVAL_MS = 10U,
	EXO_CONFIG_RS01_FEEDBACK_TIMEOUT_MS = 100U,
	EXO_CONFIG_MAX_MOTOR_CAN_ID = 0x7F,
	EXO_CONFIG_LEGACY_ADV_NAME_MAX_LENGTH = 23U,
};

/*
 * 以下检查全部发生在编译阶段，不消耗 ESP32 运行时间：
 * - GPIO 是否能承担对应输入/输出功能；
 * - 两台电机是否误用了同一个 CAN 地址或同一个执行器；
 * - 左右映射、方向和 valid 标志是否自相矛盾；
 * - 周期、超时以及 BLE 广播名称是否落在实现允许的范围内。
 * 任一条件不满足都会拒绝生成固件，比上电后才发现配置错误更容易定位。
 */
_Static_assert(GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_EXAMPLE_TX_GPIO_NUM),
			   "TWAI TX GPIO must support output");
_Static_assert(GPIO_IS_VALID_GPIO(CONFIG_EXAMPLE_RX_GPIO_NUM),
			   "TWAI RX GPIO must be valid");
_Static_assert(CONFIG_EXAMPLE_TX_GPIO_NUM != CONFIG_EXAMPLE_RX_GPIO_NUM,
			   "TWAI TX and RX GPIOs must differ");
_Static_assert(EXO_CONFIG_ACTUATOR_A_CAN_ID != EXO_CONFIG_ACTUATOR_B_CAN_ID,
			   "actuator CAN IDs must differ");
_Static_assert(EXO_CONFIG_ACTUATOR_A_CAN_ID <= EXO_CONFIG_MAX_MOTOR_CAN_ID,
			   "actuator A CAN ID exceeds the RS01 node-ID range");
_Static_assert(EXO_CONFIG_ACTUATOR_B_CAN_ID <= EXO_CONFIG_MAX_MOTOR_CAN_ID,
			   "actuator B CAN ID exceeds the RS01 node-ID range");
_Static_assert(EXO_CONFIG_DEFAULT_ACTUATOR >= EXO_ACTUATOR_A &&
				   EXO_CONFIG_DEFAULT_ACTUATOR < EXO_ACTUATOR_COUNT,
			   "default actuator must index the actuator array");
_Static_assert(EXO_CONFIG_LEFT_MAPPING_VALID
				   ? EXO_CONFIG_LEFT_ACTUATOR < EXO_ACTUATOR_COUNT
				   : EXO_CONFIG_LEFT_ACTUATOR == EXO_ACTUATOR_COUNT,
			   "left hip mapping and actuator sentinel disagree");
_Static_assert(EXO_CONFIG_RIGHT_MAPPING_VALID
				   ? EXO_CONFIG_RIGHT_ACTUATOR < EXO_ACTUATOR_COUNT
				   : EXO_CONFIG_RIGHT_ACTUATOR == EXO_ACTUATOR_COUNT,
			   "right hip mapping and actuator sentinel disagree");
_Static_assert(!(EXO_CONFIG_LEFT_MAPPING_VALID &&
				 EXO_CONFIG_RIGHT_MAPPING_VALID) ||
				   EXO_CONFIG_LEFT_ACTUATOR != EXO_CONFIG_RIGHT_ACTUATOR,
			   "left and right hips must map to different actuators");
_Static_assert(EXO_CONFIG_LEFT_DIRECTION_VALID
				   ? (EXO_CONFIG_LEFT_DIRECTION == EXO_DIRECTION_POSITIVE ||
					  EXO_CONFIG_LEFT_DIRECTION == EXO_DIRECTION_NEGATIVE)
				   : EXO_CONFIG_LEFT_DIRECTION == EXO_DIRECTION_UNKNOWN,
			   "left hip direction and validity flag disagree");
_Static_assert(EXO_CONFIG_RIGHT_DIRECTION_VALID
				   ? (EXO_CONFIG_RIGHT_DIRECTION == EXO_DIRECTION_POSITIVE ||
					  EXO_CONFIG_RIGHT_DIRECTION == EXO_DIRECTION_NEGATIVE)
				   : EXO_CONFIG_RIGHT_DIRECTION == EXO_DIRECTION_UNKNOWN,
			   "right hip direction and validity flag disagree");
_Static_assert(!EXO_CONFIG_LEFT_DIRECTION_VALID ||
				   EXO_CONFIG_LEFT_MAPPING_VALID,
			   "left hip direction requires a valid actuator mapping");
_Static_assert(!EXO_CONFIG_RIGHT_DIRECTION_VALID ||
				   EXO_CONFIG_RIGHT_MAPPING_VALID,
			   "right hip direction requires a valid actuator mapping");
_Static_assert(EXO_CONFIG_CONTROL_FREQUENCY_HZ > 0U,
			   "control frequency must be positive");
_Static_assert((1000000U % EXO_CONFIG_CONTROL_FREQUENCY_HZ) == 0U,
			   "control frequency must divide one second in microseconds");
_Static_assert(EXO_CONFIG_RS01_REPORT_INTERVAL_MS >= 10U,
			   "RS01 report interval must be at least 10 ms");
_Static_assert(((EXO_CONFIG_RS01_REPORT_INTERVAL_MS - 10U) % 5U) == 0U,
			   "RS01 report interval must use the documented 5 ms steps");
_Static_assert(EXO_CONFIG_RS01_FEEDBACK_TIMEOUT_MS >
				   EXO_CONFIG_RS01_REPORT_INTERVAL_MS,
			   "RS01 feedback timeout must exceed the report interval");
_Static_assert((sizeof(EXO_CONFIG_BLE_DEVICE_NAME) - 1U) <=
				   EXO_CONFIG_LEGACY_ADV_NAME_MAX_LENGTH,
			   "BLE device name does not fit legacy advertising data");

/*
 * 业务模块实际读取的硬件配置实例。指定下标初始化可明确表达 LEFT/RIGHT、A/B，
 * 即使以后调整声明顺序，也不会悄悄交换两侧。
 */
const exo_hardware_config_t g_exo_hardware_config = {
	.twai =
		{
			.tx_gpio = (gpio_num_t)CONFIG_EXAMPLE_TX_GPIO_NUM,
			.rx_gpio = (gpio_num_t)CONFIG_EXAMPLE_RX_GPIO_NUM,
			.bitrate_bps = EXO_CONFIG_TWAI_BITRATE_BPS,
			.timing = EXO_CONFIG_TWAI_TIMING(),
			.rx_queue_length = EXO_CONFIG_TWAI_RX_QUEUE_LENGTH,
			.master_id = EXO_CONFIG_TWAI_MASTER_ID,
		},
	.actuator =
		{
			[EXO_ACTUATOR_A] =
				{
					.can_id = EXO_CONFIG_ACTUATOR_A_CAN_ID,
				},
			[EXO_ACTUATOR_B] =
				{
					.can_id = EXO_CONFIG_ACTUATOR_B_CAN_ID,
				},
		},
	/* 当前实机映射：左髋 -> B/0x20/+1，右髋 -> A/0x10/-1。 */
	.hip_joint =
		{
			[EXO_HIP_JOINT_LEFT] =
				{
					.side = EXO_SIDE_LEFT,
					.actuator = EXO_CONFIG_LEFT_ACTUATOR,
					.direction = EXO_CONFIG_LEFT_DIRECTION,
					/* false 表示这个 0
					   只是占位，上电后仍需在中立位执行软件归零。 */
					.software_zero_deg = 0.0f,
					.mapping_valid = EXO_CONFIG_LEFT_MAPPING_VALID,
					.direction_valid = EXO_CONFIG_LEFT_DIRECTION_VALID,
					.software_zero_valid = false,
				},
			[EXO_HIP_JOINT_RIGHT] =
				{
					.side = EXO_SIDE_RIGHT,
					.actuator = EXO_CONFIG_RIGHT_ACTUATOR,
					.direction = EXO_CONFIG_RIGHT_DIRECTION,
					/* 软件零位只保存在 RAM；重启后必须重新采集。 */
					.software_zero_deg = 0.0f,
					.mapping_valid = EXO_CONFIG_RIGHT_MAPPING_VALID,
					.direction_valid = EXO_CONFIG_RIGHT_DIRECTION_VALID,
					.software_zero_valid = false,
				},
		},
	.default_actuator = EXO_CONFIG_DEFAULT_ACTUATOR,
	.control_frequency_hz = EXO_CONFIG_CONTROL_FREQUENCY_HZ,
	.rs01_report_interval_ms = EXO_CONFIG_RS01_REPORT_INTERVAL_MS,
	.rs01_feedback_timeout_ms = EXO_CONFIG_RS01_FEEDBACK_TIMEOUT_MS,
	.ble_device_name = EXO_CONFIG_BLE_DEVICE_NAME,
};

/* 阶段 8 默认轨迹和增益实例；通用控制器初始化及 SINE 参考计算会读取。 */
const exo_sine_impedance_config_t g_exo_sine_impedance_config = {
	.reference =
		{
			.amplitude_deg = EXO_CONFIG_SINE_AMPLITUDE_DEG,
			.frequency_hz = EXO_CONFIG_SINE_FREQUENCY_HZ,
			.ramp_time_s = EXO_CONFIG_SINE_RAMP_TIME_S,
		},
	.kp = EXO_CONFIG_IMPEDANCE_KP,
	.kd = EXO_CONFIG_IMPEDANCE_KD,
};

/* 仅提供轨迹数值；编译开关和运行合同仍由 controller 强制执行。 */
const sine_reference_config_t g_exo_alternating_sine_test_config = {
	.amplitude_deg = EXO_CONFIG_ALTERNATING_TEST_AMPLITUDE_DEG,
	.frequency_hz = EXO_CONFIG_ALTERNATING_TEST_FREQUENCY_HZ,
	.ramp_time_s = EXO_CONFIG_ALTERNATING_TEST_RAMP_TIME_S,
};

/* 当前能力位只声明已经接入并由代码提供的双髋运动量和电机估算力矩。 */
const exo_sensor_snapshot_config_t g_exo_sensor_snapshot_config = {
	.feedback_freshness_us = EXO_CONFIG_SENSOR_FRESHNESS_US,
	.pair_skew_warning_us = EXO_CONFIG_SENSOR_PAIR_SKEW_WARNING_US,
	.available_sensor_mask = EXO_SENSOR_CURRENT_HARDWARE_MASK,
};

/* HIP_EVENT_TBE 的第一版固定参数；后续调参必须经过回放和真机重新验收。 */
const exo_gait_estimator_config_t g_exo_gait_estimator_config = {
	.filter_alpha = EXO_CONFIG_GAIT_FILTER_ALPHA,
	.trend_velocity_threshold_deg_s = EXO_CONFIG_GAIT_TREND_VELOCITY_DEG_S,
	.motion_velocity_threshold_deg_s = EXO_CONFIG_GAIT_MOTION_VELOCITY_DEG_S,
	.minimum_excursion_deg = EXO_CONFIG_GAIT_MINIMUM_EXCURSION_DEG,
	.minimum_period_s = EXO_CONFIG_GAIT_MINIMUM_PERIOD_S,
	.maximum_period_s = EXO_CONFIG_GAIT_MAXIMUM_PERIOD_S,
	.minimum_event_interval_s = EXO_CONFIG_GAIT_MINIMUM_EVENT_INTERVAL_S,
	.period_ewma_alpha = EXO_CONFIG_GAIT_PERIOD_EWMA_ALPHA,
	.unstable_period_fraction = EXO_CONFIG_GAIT_UNSTABLE_PERIOD_FRACTION,
	.bilateral_period_tolerance_fraction =
		EXO_CONFIG_GAIT_BILATERAL_PERIOD_TOLERANCE_FRACTION,
	.minimum_bilateral_phase_separation =
		EXO_CONFIG_GAIT_MINIMUM_BILATERAL_PHASE_SEPARATION,
	.standing_timeout_us = EXO_CONFIG_GAIT_STANDING_TIMEOUT_US,
	.phase_timeout_margin_us = EXO_CONFIG_GAIT_PHASE_TIMEOUT_MARGIN_US,
	.lock_period_count = EXO_CONFIG_GAIT_LOCK_PERIOD_COUNT,
};
