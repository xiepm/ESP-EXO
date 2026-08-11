#ifndef RS01_DRIVER_H
#define RS01_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"
#include "esp_err.h"
#include "exo_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * RS01 驱动公开接口导读
 * =====================
 *
 * 这个文件只负责“ESP32 如何和两台 RS01 电机通信”，不负责人体关节语义：
 *
 *   - EXO_ACTUATOR_A / B 是固件中的两个逻辑电机槽位，用来索引对象和规定发送
 *     顺序；A/B 不是左腿/右腿，也不能根据 CAN ID 猜人体侧别。LEFT/RIGHT 与
 *     A/B 的安装映射和方向只由 hip_joint / exo_config 层解释。
 *   - 每台电机有自己的节点 ID（当前运行时 CAN ID）；ESP32 还有一个主站 ID。
 *     节点 ID 用来找到具体电机，主站 ID 是电机反馈发回 ESP32 时的目的地址，
 *     两者含义不同，不能互换。
 *   - 驱动同时支持 RS01 私有协议和 MIT 协议。私有协议主要使用 29 位扩展帧，
 *     帧标识符中同时携带目的节点、数据/来源和通信类型；MIT 协议主要使用
 *     11 位标准帧，命令类别由标准帧 ID 或末尾命令字节区分。CANopen 枚举目前
 *     只用于向电机写协议选择值，本驱动没有实现 CANopen 过程数据收发。
 *   - 发送 API 只说明命令是否成功交给 TWAI，不代表电机已经执行。接收任务应把
 *     每个完整 CAN 帧交给 rs01_process_can_frame()，解析结果会汇总进每台电机的
 *     rs01_feedback_snapshot_t，业务层再通过 rs01_get_feedback() 读取一致快照。
 *
 * 单位边界：所有具名运动 API 和原始电机反馈严格使用 RS01 手册单位 rad、
 * rad/s、rad/s²、N·m。人体关节层以及 UART/手机显示使用 deg；进入本驱动前必须
 * 通过 exo_degrees_to_radians() 显式转换。裸寄存器读写完全按手册透传，不推断
 * 寄存器类型或单位。
 *
 * 并发边界：发送路径由递归互斥锁串行化，允许一个组合命令在持锁期间调用其他
 * 发送 API；100 Hz 的 try 接口使用零等待。反馈对象和运行时 CAN ID 由临界区锁
 * 保护，读取者得到的是一次完整复制，不会看到更新一半的结构。
 */

/* 电机反馈上报的运行状态，不等同于 ESP-EXO 的整机状态机。 */
typedef enum {
	RS01_MODE_STATUS_RESET = 0,
	RS01_MODE_STATUS_CALIBRATION = 1,
	RS01_MODE_STATUS_RUNNING = 2,
	RS01_MODE_STATUS_UNKNOWN = 3,
} rs01_mode_status_t;

/* 电机内部控制方式；决定后续运动值是 Motion 帧还是参数寄存器。 */
typedef enum {
	RS01_CONTROL_MODE_MOTION = 0,
	RS01_CONTROL_MODE_POSITION_PP = 1,
	RS01_CONTROL_MODE_VELOCITY = 2,
	RS01_CONTROL_MODE_CURRENT = 3,
	RS01_CONTROL_MODE_POSITION_CSP = 5,
	RS01_CONTROL_MODE_UNKNOWN = 0xFF,
} rs01_control_mode_t;

/* 写入电机的协议选择值；切换后是否需重启，以 RS01 手册和实际固件为准。 */
typedef enum {
	RS01_PROTOCOL_PRIVATE = 0,
	RS01_PROTOCOL_CANOPEN = 1,
	RS01_PROTOCOL_MIT = 2,
} rs01_protocol_t;

typedef struct {
	/* 手册故障位的归一化结果；present 表示至少存在一个故障。 */
	bool present;
	bool warning_present;
	bool uncalibrated;
	bool stall_overload;
	bool magnetic_encoder;
	bool over_temperature;
	bool phase_over_current;
	bool under_voltage;
	bool over_voltage;
	bool driver_chip;
	bool position_initialization;
	bool hardware_identification;
	bool over_temperature_warning;
} rs01_fault_flags_t;

typedef struct {
	/*
	 * 最近一个有效反馈的周期电机采样，单位为 rad、rad/s、N·m、°C。
	 * 注意：RS01 手册规定位置每经过 -4π～+4π 的完整区间就回绕一次，因此
	 * sample.position_rad 适合核对 CAN 原始值，不能直接当作连续多圈位置。
	 */
	exo_motion_sample_t sample;
	/*
	 * 驱动根据相邻 Type-2/Type-24/MIT 反馈重建的连续位置。正向越过 +4π 时
	 * position_wrap_count 加一，反向越过 -4π 时减一；业务层的人体角应使用这里，
	 * 同时保留上面的周期原值用于发送边界和现场诊断。
	 */
	float continuous_position_rad;
	int32_t position_wrap_count;
	/*
	 * epoch 在首次反馈、长时间失联后重建、机械置零时变化。上层软件零位必须保存
	 * 捕获时的 epoch；二者不同就说明连续坐标系已经换代，旧零位不得继续使用。
	 */
	uint32_t position_epoch;
	uint32_t position_rebase_count;
	/* 机械置零只在连续若干反馈接近 0 后完成确认；pending 时连续角不可用于归零。 */
	uint8_t position_reset_confirmation_count;
	bool position_reset_pending;
	bool continuous_position_valid;
	/* mode 来自电机反馈；control_mode 是最近成功请求写入的控制模式。 */
	rs01_mode_status_t mode;
	rs01_fault_flags_t faults;

	/* Type-21/MIT 故障响应的原始位图，便于保留手册尚未归一化的位。 */
	uint32_t fault_raw;
	uint32_t warning_raw;
	/* 私有协议 type-2/type-24 状态字中的六位故障摘要。 */
	uint8_t feedback_fault_raw;

	/* 时间戳单位均为 us；计数器用于判断反馈是否持续更新。 */
	uint64_t timestamp_us;
	uint64_t first_timestamp_us;
	uint64_t last_interval_us;
	uint32_t feedback_count;
	uint32_t type2_feedback_count;
	uint32_t type24_feedback_count;
	uint32_t invalid_frame_count;

	/* valid 表示至少解析过有效反馈；online 在读取快照时按超时动态计算。 */
	bool valid;
	bool online;
	/* requested 只记录命令发送成功，不是电机执行确认。 */
	bool enabled_requested;
	bool reporting_requested;
	rs01_control_mode_t control_mode;
} rs01_feedback_snapshot_t;

typedef struct {
	/* RS01 定点运动帧的编解码量程，角度相关字段均为弧度制。 */
	float position_min_rad;
	float position_max_rad;
	float velocity_min_rad_s;
	float velocity_max_rad_s;
	float torque_min_nm;
	float torque_max_nm;
	float kp_min;
	float kp_max;
	float kd_min;
	float kd_max;
} rs01_limits_t;

/*
 * RS01 Motion 帧的具名电机命令。
 *
 * 这个结构已经处在“执行器/电机坐标”，所以位置和速度必须是 rad、rad/s；它与
 * exo_joint_command_t（人体 LEFT/RIGHT、deg）不是同一种数据。使用具名字段能
 * 避免旧标量接口中 torque、kp、kd 顺序写错。该结构仍然不是 CAN 线格式，驱动
 * 会逐字段校验并按手册量化，禁止直接 memcpy 到 TWAI payload。
 */
typedef struct {
	float position_rad;
	float velocity_rad_s;
	float torque_nm;
	float kp;
	float kd;
} rs01_motion_command_t;

/* 接收解析结果，供 TWAI RX 任务决定是否刷新对应执行器对象或记录异常。 */
typedef enum {
	RS01_RX_EVENT_NONE = 0,
	RS01_RX_EVENT_FEEDBACK,
	RS01_RX_EVENT_FAULT,
	RS01_RX_EVENT_PARAMETER,
	RS01_RX_EVENT_INVALID,
} rs01_rx_event_t;

/* 初始化静态电机对象和发送互斥锁；除纯量程查询外，其他 API 都应在它之后调用。
 */
esp_err_t rs01_driver_init(void);

/* 查询固定编解码量程，以及 A/B 槽位当前对应的运行时节点 ID。 */
const rs01_limits_t *rs01_get_limits(void);
bool rs01_get_can_id(exo_actuator_id_t actuator, exo_can_node_id_t *can_id);
/* 按节点 ID 反查 A/B；这里只识别电机槽位，不产生 LEFT/RIGHT 语义。 */
bool rs01_find_actuator(exo_can_node_id_t can_id, exo_actuator_id_t *actuator);

/*
 * 检查五个字段是否都是有限数，并且处在 RS01 手册定点量程内（含边界）。
 * 驱动不会替调用者静默限幅；返回 false 时命令不得发送。
 */
bool rs01_motion_command_is_valid(const rs01_motion_command_t *command);

/* 私有协议的使能、失能和清故障命令；返回成功只代表 CAN 发送成功。 */
esp_err_t rs01_enable(exo_actuator_id_t actuator);
esp_err_t rs01_disable(exo_actuator_id_t actuator);
esp_err_t rs01_clear_faults(exo_actuator_id_t actuator);

/* 发送一个私有协议 type-1 运动帧，不改变模式或使能；角度参数均为弧度制。 */
esp_err_t rs01_send_motion(exo_actuator_id_t actuator, float position_rad,
						   float velocity_rad_s, float torque_nm, float kp,
						   float kd);

/*
 * 100 Hz 周期路径专用版本：TX 互斥锁和 TWAI 入队都使用零等待。忙/满立即返回，
 * 且不在实时路径打印日志。
 */
esp_err_t rs01_try_send_motion(exo_actuator_id_t actuator, float position_rad,
							   float velocity_rad_s, float torque_nm, float kp,
							   float kd);

/*
 * 上面两个标量接口的具名结构包装。普通版本可按现有驱动策略等待发送资源；try
 * 版本供唯一 100 Hz 控制路径调用，TX 锁和 TWAI 入队均为零等待，忙时立即返回。
 * 两个版本都先完整校验五元命令，不执行静默限幅，也不切换模式或使能电机。
 */
esp_err_t rs01_send_motion_command(exo_actuator_id_t actuator,
								 const rs01_motion_command_t *command);
esp_err_t rs01_try_send_motion_command(exo_actuator_id_t actuator,
									 const rs01_motion_command_t *command);

/* 调试台兼容组合动作：必要时失能，再切 Motion 模式、使能并发送一个运动帧。 */
esp_err_t rs01_move(exo_actuator_id_t actuator, float torque_nm,
					float position_rad, float velocity_rad_s, float kp,
					float kd);

/*
 * 私有协议的具名控制模式。函数会按需要完成模式准备，并把手册规定的参数寄存器
 * 依次写入；位置、速度和加速度参数全部采用弧度制。
 */
esp_err_t rs01_set_control_mode(exo_actuator_id_t actuator,
								rs01_control_mode_t mode);
esp_err_t rs01_speed_control(exo_actuator_id_t actuator, float current_limit_a,
							 float acceleration_rad_s2, float velocity_rad_s);
esp_err_t rs01_position_pp_control(exo_actuator_id_t actuator,
								   float velocity_rad_s,
								   float acceleration_rad_s2,
								   float position_rad);
esp_err_t rs01_position_csp_control(exo_actuator_id_t actuator,
									float velocity_limit_rad_s,
									float position_rad);
esp_err_t rs01_current_control(exo_actuator_id_t actuator, float current_a);

/*
 * 私有协议裸参数访问。index、数值类型、物理单位和可写性由调用者按 RS01 手册
 * 决定；驱动只负责 little-endian 编码，不自动进行 deg/rad 换算。
 */
esp_err_t rs01_read_parameter(exo_actuator_id_t actuator, uint16_t index);
esp_err_t rs01_write_parameter_float(exo_actuator_id_t actuator, uint16_t index,
									 float value);
esp_err_t rs01_write_parameter_u8(exo_actuator_id_t actuator, uint16_t index,
								  uint8_t value);
esp_err_t rs01_write_parameter_u16(exo_actuator_id_t actuator, uint16_t index,
								   uint16_t value);

/*
 * 主动上报分两步：先写上报周期，再打开上报开关。实测产品固件中 10 ms 对应
 * 0x7026=0，之后每增加 5 ms 寄存器值加 1。
 */
esp_err_t rs01_set_report_interval_ms(exo_actuator_id_t actuator,
									  uint32_t interval_ms);
/* 下面两个函数只发送私有协议 type-24 开关，不会改写上报周期。 */
esp_err_t rs01_start_reporting(exo_actuator_id_t actuator);
esp_err_t rs01_stop_reporting(exo_actuator_id_t actuator);

/*
 * 私有协议维护命令。mechanical_zero 修改电机机械零点，不是髋关节 RAM 软件零位；
 * set_can_id 成功后只更新本次运行的槽位映射，静态硬件配置仍需另行同步。
 */
esp_err_t rs01_set_mechanical_zero(exo_actuator_id_t actuator);
esp_err_t rs01_set_can_id(exo_actuator_id_t actuator,
						  exo_can_node_id_t new_can_id);
esp_err_t rs01_save_parameters(exo_actuator_id_t actuator);
esp_err_t rs01_set_baud_rate(exo_actuator_id_t actuator, uint8_t baud_selector);
esp_err_t rs01_set_protocol(exo_actuator_id_t actuator,
							rs01_protocol_t protocol);

/*
 * MIT 标准帧接口。简单终端命令（使能、失能、置零等）使用 0xFF 填充帧和末字节
 * 命令；带一个参数的终端命令把参数放在 data[6]。MIT Motion 使用 16/12 bit
 * 定点拼包，位置/速度模式使用两个 little-endian float。所有角度相关参数仍是
 * rad、rad/s，不因协议名称改变单位。
 */
esp_err_t rs01_mit_enable(exo_actuator_id_t actuator);
esp_err_t rs01_mit_disable(exo_actuator_id_t actuator);
esp_err_t rs01_mit_clear_or_read_fault(exo_actuator_id_t actuator,
									   uint8_t command);
esp_err_t rs01_mit_set_operation_mode(exo_actuator_id_t actuator, uint8_t mode);
esp_err_t rs01_mit_set_can_id(exo_actuator_id_t actuator,
							  exo_can_node_id_t new_can_id);
esp_err_t rs01_mit_set_protocol(exo_actuator_id_t actuator,
								rs01_protocol_t protocol);
esp_err_t rs01_mit_send_motion(exo_actuator_id_t actuator, float position_rad,
							   float velocity_rad_s, float kp, float kd,
							   float torque_nm);
esp_err_t rs01_mit_position_control(exo_actuator_id_t actuator,
									float position_rad, float velocity_rad_s);
esp_err_t rs01_mit_speed_control(exo_actuator_id_t actuator,
								 float velocity_rad_s, float current_limit_a);
esp_err_t rs01_mit_set_zero(exo_actuator_id_t actuator);
esp_err_t rs01_mit_save_parameters(exo_actuator_id_t actuator);
esp_err_t rs01_mit_start_reporting(exo_actuator_id_t actuator);
esp_err_t rs01_mit_stop_reporting(exo_actuator_id_t actuator);
/* MIT 裸参数访问同样按寄存器原生单位透传，不执行产品层单位换算。 */
esp_err_t rs01_mit_read_parameter(exo_actuator_id_t actuator, uint16_t index);
esp_err_t rs01_mit_write_parameter_float(exo_actuator_id_t actuator,
										 uint16_t index, float value);

/* 复制指定 A/B 电机的一致反馈快照，并在复制后根据最新时间戳计算 online。 */
bool rs01_get_feedback(exo_actuator_id_t actuator,
					   rs01_feedback_snapshot_t *snapshot);

/*
 * 接收入口：处理一个完整 TWAI 帧并自动分派到私有扩展帧或 MIT 标准帧解析器。
 * 若帧能归属于已配置电机，updated_actuator 返回 A/B；否则保持 COUNT。该返回值
 * 仍然只是电机槽位，不能据此推断人体左腿或右腿。
 */
rs01_rx_event_t rs01_process_can_frame(const twai_message_t *frame,
									   exo_actuator_id_t *updated_actuator);

#ifdef __cplusplus
}
#endif

#endif /* RS01_DRIVER_H */
