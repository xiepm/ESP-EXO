/*
 * RS01 双电机 CAN 驱动实现导读
 * ===========================
 *
 * 建议按下面顺序阅读本文件：
 *
 *   1. s_motors[]：A/B 两个“逻辑执行器槽位”各自保存当前节点 ID 和反馈快照；
 *   2. transmit_*()：把命令封装成 TWAI 标准帧或扩展帧并串行发送；
 *   3. 私有协议 API：使能、模式、参数、主动上报和维护命令；
 *   4. MIT API：终端命令、定点 Motion 帧和两个 float 的位置/速度帧；
 *   5. rs01_process_can_frame()：接收总入口，校验后把帧分派给对应解析器。
 *
 * 两套协议的直观区别：
 *
 *   - RS01 私有协议使用 29 位扩展 ID：bit[7:0] 是目的地址，bit[23:8] 是随通信
 *     类型变化的数据/来源字段，bit[28:24] 是通信类型。发送给电机时，目的地址
 *     通常是电机节点 ID；电机反馈给 ESP32 时，目的地址必须是主站 ID。
 *   - MIT 协议主要使用 11 位标准 ID。普通 Motion/终端命令以节点 ID 为帧 ID；
 *     位置、速度、参数命令还会在 ID 高位编码操作号；反馈通常发往主站 ID，并在
 *     data[0] 携带电机节点 ID。
 *
 * A/B 只表示本驱动的两个对象槽位，绝不表示左右腿。节点 ID 也只标识总线设备，
 * 不能用 0x10/0x20 等数值猜 LEFT/RIGHT；人体侧别、安装方向和软件零位全部属于
 * hip_joint 层。
 *
 * 所有电机角度边界保持 RS01 手册的 rad、rad/s、rad/s²。本文件不接受人体关节
 * 的 deg，也不会在裸寄存器访问中猜单位。发送使用递归互斥锁；反馈快照、运行时
 * 节点 ID 和 MIT 故障查询状态使用短临界区保护，避免
 * TX/RX/控制任务互相撕裂数据。
 */

#include "rs01_driver.h"
#include "rs01_motor_profile.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "exo_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "rs01";

/* 私有协议通信类型、常用寄存器及本驱动使用的帧/超时常量。 */
enum {
	RS01_PRIVATE_TYPE_CONTROL = 0x01,
	RS01_PRIVATE_TYPE_FEEDBACK = 0x02,
	RS01_PRIVATE_TYPE_ENABLE = 0x03,
	RS01_PRIVATE_TYPE_STOP = 0x04,
	RS01_PRIVATE_TYPE_ZERO = 0x06,
	RS01_PRIVATE_TYPE_SET_CAN_ID = 0x07,
	RS01_PRIVATE_TYPE_PARAMETER_READ = 0x11,
	RS01_PRIVATE_TYPE_PARAMETER_WRITE = 0x12,
	RS01_PRIVATE_TYPE_FAULT = 0x15,
	RS01_PRIVATE_TYPE_SAVE = 0x16,
	RS01_PRIVATE_TYPE_BAUD = 0x17,
	RS01_PRIVATE_TYPE_REPORT = 0x18,
	RS01_PRIVATE_TYPE_PROTOCOL = 0x19,

	RS01_PARAMETER_RUN_MODE = 0x7005,
	RS01_PARAMETER_CURRENT_REFERENCE = 0x7006,
	RS01_PARAMETER_SPEED_REFERENCE = 0x700A,
	RS01_PARAMETER_POSITION_REFERENCE = 0x7016,
	RS01_PARAMETER_CSP_SPEED_LIMIT = 0x7017,
	RS01_PARAMETER_CURRENT_LIMIT = 0x7018,
	RS01_PARAMETER_ACCELERATION = 0x7022,
	RS01_PARAMETER_PP_SPEED = 0x7024,
	RS01_PARAMETER_PP_ACCELERATION = 0x7025,
	RS01_PARAMETER_REPORT_INTERVAL = 0x7026,

	RS01_MAX_CAN_ID = 0x7F,
	RS01_FRAME_DATA_LENGTH = 8,
	RS01_MIT_MANUAL_FAULT_RESPONSE_LENGTH = 8,
	RS01_MIT_LEGACY_FAULT_RESPONSE_LENGTH = 5,
	RS01_MIT_FAULT_RESPONSE_TIMEOUT_MS = 100,
	RS01_TX_TIMEOUT_MS = 10,
	/* 机械置零后要求连续三帧接近 0，避免把 RX 队列中的旧主动上报当成新坐标。 */
	RS01_POSITION_RESET_CONFIRMATION_FRAMES = 3,
};

static const float RS01_POSITION_RESET_CONFIRMATION_RAD = 0.1f;

/* RS01 Motion/MIT 定点编码的手册量程；角度相关量始终是弧度制。 */
static const rs01_limits_t s_limits = {
	.position_min_rad = -12.57f,
	.position_max_rad = 12.57f,
	.velocity_min_rad_s = -EXO_MOTOR_WIRE_SPEED_MAX,
	.velocity_max_rad_s = EXO_MOTOR_WIRE_SPEED_MAX,
	.torque_min_nm = -EXO_MOTOR_WIRE_TORQUE_MAX,
	.torque_max_nm = EXO_MOTOR_WIRE_TORQUE_MAX,
	.kp_min = 0.0f,
	.kp_max = 500.0f,
	.kd_min = 0.0f,
	.kd_max = 5.0f,
};

typedef struct {
	/* actuator 是稳定的 A/B 槽位；can_id 可被改 ID 命令在运行期更新。 */
	exo_actuator_id_t actuator;
	exo_can_node_id_t can_id;
	rs01_feedback_snapshot_t feedback;
	rs01_fault_flags_t summary_faults;
	bool mit_fault_response_pending;
	uint64_t mit_fault_query_timestamp_us;
} rs01_motor_t;

static rs01_motor_t s_motors[EXO_ACTUATOR_COUNT];

/*
 * s_tx_mutex 串行化完整发送序列。使用递归锁是因为 move/模式切换等组合 API
 * 持锁后还会调用底层发送 API。s_feedback_lock 只保护短小的 RAM 状态复制，
 * 临界区内不调用 TWAI、不延时、不打印日志。
 */
static StaticSemaphore_t s_tx_mutex_storage;
static SemaphoreHandle_t s_tx_mutex;
static portMUX_TYPE s_feedback_lock = portMUX_INITIALIZER_UNLOCKED;

/* 在反馈锁下读取 A/B 当前对应的节点 ID，避免改 ID 与收发并发时读到中间状态。 */
static exo_can_node_id_t current_can_id(exo_actuator_id_t actuator) {
	portENTER_CRITICAL(&s_feedback_lock);
	const exo_can_node_id_t can_id = s_motors[actuator].can_id;
	portEXIT_CRITICAL(&s_feedback_lock);
	return can_id;
}

/* 检查新节点 ID 是否会与另一个逻辑槽位冲突。 */
static bool can_id_conflicts(exo_actuator_id_t actuator,
							 exo_can_node_id_t can_id) {
	bool conflict = false;
	portENTER_CRITICAL(&s_feedback_lock);
	for (unsigned i = 0; i < (unsigned)EXO_ACTUATOR_COUNT; ++i) {
		if (i != (unsigned)actuator && s_motors[i].can_id == can_id) {
			conflict = true;
			break;
		}
	}
	portEXIT_CRITICAL(&s_feedback_lock);
	return conflict;
}

/* 仅更新本次运行的 RAM 映射；不会反向修改 g_exo_hardware_config。 */
static void set_current_can_id(exo_actuator_id_t actuator,
							   exo_can_node_id_t can_id) {
	portENTER_CRITICAL(&s_feedback_lock);
	s_motors[actuator].can_id = can_id;
	portEXIT_CRITICAL(&s_feedback_lock);
}

/* 所有公开发送/读取 API 都用此边界拒绝 COUNT 或越界槽位。 */
static bool actuator_is_valid(exo_actuator_id_t actuator) {
	return (unsigned)actuator < (unsigned)EXO_ACTUATOR_COUNT;
}

static const char *actuator_name(exo_actuator_id_t actuator) {
	return actuator == EXO_ACTUATOR_A ? "A" : "B";
}

/*
 * 私有协议 29 位扩展 ID 打包/拆包：
 *   bit[7:0]   destination 目的地址
 *   bit[23:8]  data        类型相关的主站 ID、源节点、状态或定点转矩
 *   bit[28:24] type        私有协议通信类型
 */
static uint32_t make_extended_id(uint8_t destination, uint16_t data,
								 uint8_t type) {
	return (uint32_t)destination | ((uint32_t)data << 8) |
		   (((uint32_t)type & 0x1FU) << 24);
}

static uint8_t extended_id_destination(uint32_t identifier) {
	return (uint8_t)(identifier & 0xFFU);
}

static uint16_t extended_id_data(uint32_t identifier) {
	return (uint16_t)((identifier >> 8) & 0xFFFFU);
}

static uint8_t extended_id_type(uint32_t identifier) {
	return (uint8_t)((identifier >> 24) & 0x1FU);
}

/*
 * 帧内字节序辅助函数。Motion 定点量按大端拼装；参数索引和 32 位参数值按
 * little-endian 编码。显式逐字节处理，避免 CPU 对齐和本机端序影响线格式。
 */
static uint16_t read_u16_be(const uint8_t *data) {
	return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint32_t read_u32_le(const uint8_t *data) {
	return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
		   ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void write_u16_be(uint8_t *data, uint16_t value) {
	data[0] = (uint8_t)(value >> 8);
	data[1] = (uint8_t)value;
}

static void write_u16_le(uint8_t *data, uint16_t value) {
	data[0] = (uint8_t)value;
	data[1] = (uint8_t)(value >> 8);
}

static void write_u32_le(uint8_t *data, uint32_t value) {
	data[0] = (uint8_t)value;
	data[1] = (uint8_t)(value >> 8);
	data[2] = (uint8_t)(value >> 16);
	data[3] = (uint8_t)(value >> 24);
}

/*
 * 把已经通过量程校验的物理量线性映射到指定 bit 数的无符号定点整数。
 * 本函数不做限幅：所有 Motion 入口必须先调用 rs01_motion_command_is_valid()，
 * 越界时直接返回错误，不能把调用者的错误命令悄悄改成边界命令。
 * raw_to_float() 执行逆映射；这里不改变物理单位，rad 输入解码后仍得到 rad。
 */
static uint32_t float_to_raw(float value, float minimum, float maximum,
							 unsigned bits) {
	const float span = maximum - minimum;
	const uint32_t raw_max = (1UL << bits) - 1UL;
	return (uint32_t)((value - minimum) * (float)raw_max / span);
}

static float raw_to_float(uint32_t value, float minimum, float maximum,
						  uint32_t raw_max) {
	return ((float)value / (float)raw_max) * (maximum - minimum) + minimum;
}

/* 统一拒绝 NaN/Inf，防止其进入范围比较、整数转换或 CAN 负载。 */
static bool motion_values_are_finite(float position_rad, float velocity_rad_s,
									 float torque_nm, float kp, float kd) {
	return isfinite(position_rad) && isfinite(velocity_rad_s) &&
		   isfinite(torque_nm) && isfinite(kp) && isfinite(kd);
}

/* 小工具只做闭区间判断；调用前必须先排除 NaN/Inf。 */
static bool value_is_in_range(float value, float minimum, float maximum) {
	return value >= minimum && value <= maximum;
}

bool rs01_motion_command_is_valid(const rs01_motion_command_t *command) {
	/*
	 * 这里刻意采用“拒绝越界”，而不是把越界值裁成边界。静默裁剪会让控制器
	 * 以为电机执行了原命令，实际却执行另一个命令，给回放和参数排查造成歧义。
	 */
	if (command == NULL ||
		!motion_values_are_finite(command->position_rad,
								  command->velocity_rad_s, command->torque_nm,
								  command->kp, command->kd)) {
		return false;
	}

	return value_is_in_range(command->position_rad, s_limits.position_min_rad,
							 s_limits.position_max_rad) &&
		   value_is_in_range(command->velocity_rad_s,
							 s_limits.velocity_min_rad_s,
							 s_limits.velocity_max_rad_s) &&
		   value_is_in_range(command->torque_nm, s_limits.torque_min_nm,
							 s_limits.torque_max_nm) &&
		   value_is_in_range(command->kp, s_limits.kp_min, s_limits.kp_max) &&
		   value_is_in_range(command->kd, s_limits.kd_min, s_limits.kd_max);
}

/* 保留旧标量入口时复用同一份完整量程校验，避免新旧 API 行为不一致。 */
static bool motion_values_are_valid(float position_rad, float velocity_rad_s,
									float torque_nm, float kp, float kd) {
	const rs01_motion_command_t command = {
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.torque_nm = torque_nm,
		.kp = kp,
		.kd = kd,
	};
	return rs01_motion_command_is_valid(&command);
}

/* 普通命令可等待递归 TX 锁；100 Hz 控制路径只能调用零等待版本。 */
static esp_err_t take_tx_lock(void) {
	if (s_tx_mutex == NULL) {
		return ESP_ERR_INVALID_STATE;
	}
	return xSemaphoreTakeRecursive(s_tx_mutex, portMAX_DELAY) == pdTRUE
			   ? ESP_OK
			   : ESP_ERR_TIMEOUT;
}

static esp_err_t try_take_tx_lock(void) {
	if (s_tx_mutex == NULL) {
		return ESP_ERR_INVALID_STATE;
	}
	return xSemaphoreTakeRecursive(s_tx_mutex, 0) == pdTRUE ? ESP_OK
															: ESP_ERR_TIMEOUT;
}

static void give_tx_lock(void) { xSemaphoreGiveRecursive(s_tx_mutex); }

/*
 * 普通发送原语：复制 8 字节负载，按参数选择标准/扩展帧，并最多等待配置的
 * TWAI 发送时间。失败可以打印一次日志，因此不能用于 100 Hz 实时路径。
 */
static esp_err_t transmit_frame(bool extended, uint32_t identifier,
								const uint8_t payload[RS01_FRAME_DATA_LENGTH]) {
	esp_err_t err = take_tx_lock();
	if (err != ESP_OK) {
		return err;
	}

	twai_message_t message = {0};
	message.identifier = identifier;
	message.data_length_code = RS01_FRAME_DATA_LENGTH;
	message.flags = extended ? TWAI_MSG_FLAG_EXTD : TWAI_MSG_FLAG_NONE;
	memcpy(message.data, payload, RS01_FRAME_DATA_LENGTH);

	err = twai_transmit(&message, pdMS_TO_TICKS(RS01_TX_TIMEOUT_MS));
	give_tx_lock();

	if (err != ESP_OK) {
		ESP_LOGW(TAG, "%s TX id=0x%08lX failed: %s",
				 extended ? "extended" : "standard", (unsigned long)identifier,
				 esp_err_to_name(err));
	}
	return err;
}

/*
 * 实时发送原语：TX 锁和 TWAI 入队均为零等待，也不打印日志；忙或队列满直接
 * 返回错误，由控制循环诊断计数处理。
 */
static esp_err_t
try_transmit_frame(bool extended, uint32_t identifier,
				   const uint8_t payload[RS01_FRAME_DATA_LENGTH]) {
	esp_err_t err = try_take_tx_lock();
	if (err != ESP_OK) {
		return err;
	}

	twai_message_t message = {0};
	message.identifier = identifier;
	message.data_length_code = RS01_FRAME_DATA_LENGTH;
	message.flags = extended ? TWAI_MSG_FLAG_EXTD : TWAI_MSG_FLAG_NONE;
	memcpy(message.data, payload, RS01_FRAME_DATA_LENGTH);

	err = twai_transmit(&message, 0);
	give_tx_lock();
	return err;
}

/* 给 100 Hz 控制路径构造私有扩展帧，目的节点取自 A/B 当前运行时映射。 */
static esp_err_t
try_transmit_private(exo_actuator_id_t actuator, uint8_t type,
					 uint16_t data_field,
					 const uint8_t payload[RS01_FRAME_DATA_LENGTH]) {
	if (!actuator_is_valid(actuator)) {
		return ESP_ERR_INVALID_ARG;
	}
	return try_transmit_frame(
		true, make_extended_id(current_can_id(actuator), data_field, type),
		payload);
}

/* 普通私有协议发送包装器：扩展 ID 的目的地址是目标电机节点 ID。 */
static esp_err_t
transmit_private(exo_actuator_id_t actuator, uint8_t type, uint16_t data_field,
				 const uint8_t payload[RS01_FRAME_DATA_LENGTH]) {
	if (!actuator_is_valid(actuator)) {
		return ESP_ERR_INVALID_ARG;
	}
	return transmit_frame(
		true, make_extended_id(current_can_id(actuator), data_field, type),
		payload);
}

/* MIT/兼容命令使用的标准帧发送包装器。identifier 由具体 MIT API 决定。 */
static esp_err_t
transmit_standard(exo_actuator_id_t actuator, uint16_t identifier,
				  const uint8_t payload[RS01_FRAME_DATA_LENGTH]) {
	if (!actuator_is_valid(actuator)) {
		return ESP_ERR_INVALID_ARG;
	}
	return transmit_frame(false, identifier, payload);
}

/*
 * 以下 requested/control_mode 字段记录“成功发出了什么请求”，不是电机已经执行
 * 的确认；真正运行状态仍以随后收到的反馈为准。
 */
static void set_enabled_requested(exo_actuator_id_t actuator, bool requested) {
	portENTER_CRITICAL(&s_feedback_lock);
	s_motors[actuator].feedback.enabled_requested = requested;
	portEXIT_CRITICAL(&s_feedback_lock);
}

static void set_reporting_requested(exo_actuator_id_t actuator,
									bool requested) {
	portENTER_CRITICAL(&s_feedback_lock);
	s_motors[actuator].feedback.reporting_requested = requested;
	portEXIT_CRITICAL(&s_feedback_lock);
}

static void set_control_mode_state(exo_actuator_id_t actuator,
								   rs01_control_mode_t mode) {
	portENTER_CRITICAL(&s_feedback_lock);
	s_motors[actuator].feedback.control_mode = mode;
	portEXIT_CRITICAL(&s_feedback_lock);
}

/* 只给能归属到某个 A/B 的坏帧计数；未知来源无法安全归入任一电机。 */
static void increment_invalid_frame(exo_actuator_id_t actuator) {
	if (!actuator_is_valid(actuator)) {
		return;
	}
	portENTER_CRITICAL(&s_feedback_lock);
	++s_motors[actuator].feedback.invalid_frame_count;
	portEXIT_CRITICAL(&s_feedback_lock);
}

/*
 * MIT 故障响应与普通反馈共用主站 ID，线格式缺少完全独立的类型标志。这里保存
 * 一个短时“正在等待故障响应”窗口，供接收分派器结合帧内容消歧。
 */
static void set_mit_fault_response_pending(exo_actuator_id_t actuator,
										   bool pending) {
	const uint64_t now_us = (uint64_t)esp_timer_get_time();
	portENTER_CRITICAL(&s_feedback_lock);
	s_motors[actuator].mit_fault_response_pending = pending;
	s_motors[actuator].mit_fault_query_timestamp_us = pending ? now_us : 0U;
	portEXIT_CRITICAL(&s_feedback_lock);
}

static bool mit_fault_response_is_pending(exo_actuator_id_t actuator) {
	const uint64_t now_us = (uint64_t)esp_timer_get_time();
	const uint64_t timeout_us =
		(uint64_t)RS01_MIT_FAULT_RESPONSE_TIMEOUT_MS * 1000ULL;
	portENTER_CRITICAL(&s_feedback_lock);
	const uint64_t requested_us =
		s_motors[actuator].mit_fault_query_timestamp_us;
	const bool pending = s_motors[actuator].mit_fault_response_pending &&
						 now_us >= requested_us &&
						 (now_us - requested_us) <= timeout_us;
	if (!pending) {
		s_motors[actuator].mit_fault_response_pending = false;
		s_motors[actuator].mit_fault_query_timestamp_us = 0U;
	}
	portEXIT_CRITICAL(&s_feedback_lock);
	return pending;
}

/* 把私有反馈状态字中的六位摘要解码成统一故障结构。 */
static rs01_fault_flags_t private_summary_faults(uint8_t raw) {
	rs01_fault_flags_t faults = {0};
	faults.present = raw != 0U;
	faults.uncalibrated = (raw & (1U << 5)) != 0U;
	faults.stall_overload = (raw & (1U << 4)) != 0U;
	faults.magnetic_encoder = (raw & (1U << 3)) != 0U;
	faults.over_temperature = (raw & (1U << 2)) != 0U;
	faults.phase_over_current = (raw & (1U << 1)) != 0U;
	faults.under_voltage = (raw & (1U << 0)) != 0U;
	return faults;
}

/* 把 type-21/MIT 的详细 fault/warning 位图解码成相同的统一故障结构。 */
static rs01_fault_flags_t detailed_faults(uint32_t fault, uint32_t warning) {
	rs01_fault_flags_t flags = {0};
	flags.present = fault != 0U;
	flags.warning_present = warning != 0U;
	flags.uncalibrated = (fault & (1UL << 7)) != 0U;
	flags.stall_overload = (fault & (1UL << 14)) != 0U;
	flags.over_temperature = (fault & (1UL << 0)) != 0U;
	flags.phase_over_current =
		(fault & ((1UL << 16) | (1UL << 5) | (1UL << 4))) != 0U;
	flags.under_voltage = (fault & (1UL << 2)) != 0U;
	flags.over_voltage = (fault & (1UL << 3)) != 0U;
	flags.driver_chip = (fault & (1UL << 1)) != 0U;
	flags.position_initialization = (fault & (1UL << 9)) != 0U;
	flags.hardware_identification = (fault & (1UL << 8)) != 0U;
	flags.over_temperature_warning = (warning & 1UL) != 0U;
	return flags;
}

/* 合并摘要与详细故障；任一来源报告的位都会保留。 */
static rs01_fault_flags_t merge_faults(rs01_fault_flags_t first,
									   rs01_fault_flags_t second) {
	rs01_fault_flags_t merged = {
		.present = first.present || second.present,
		.warning_present = first.warning_present || second.warning_present,
		.uncalibrated = first.uncalibrated || second.uncalibrated,
		.stall_overload = first.stall_overload || second.stall_overload,
		.magnetic_encoder = first.magnetic_encoder || second.magnetic_encoder,
		.over_temperature = first.over_temperature || second.over_temperature,
		.phase_over_current =
			first.phase_over_current || second.phase_over_current,
		.under_voltage = first.under_voltage || second.under_voltage,
		.over_voltage = first.over_voltage || second.over_voltage,
		.driver_chip = first.driver_chip || second.driver_chip,
		.position_initialization =
			first.position_initialization || second.position_initialization,
		.hardware_identification =
			first.hardware_identification || second.hardware_identification,
		.over_temperature_warning =
			first.over_temperature_warning || second.over_temperature_warning,
	};
	return merged;
}

/* 未知的两位状态值统一映射为 UNKNOWN，避免把保留值当成运行状态。 */
static rs01_mode_status_t decode_mode(uint8_t raw) {
	return raw <= (uint8_t)RS01_MODE_STATUS_RUNNING ? (rs01_mode_status_t)raw
													: RS01_MODE_STATUS_UNKNOWN;
}

/*
 * RS01 手册把 Type-2 位置定义为 -4π～+4π 的周期值，并明确要求主机据此统计圈数。
 * 这里在发布反馈前比较本帧与上一帧：跨越整个编码边界时，两帧差值的绝对值会
 * 大于半个周期；普通运动在相邻 10 ms 帧之间不会出现这种跳变。
 *
 * sample.position_rad 始终保留线上周期值；continuous_position_rad 才是展开后的
 * 连续值。这样 STOP 等底层接口仍拿合法的协议角，而人体关节层不会看到约 1440°
 * 的假跳变。
 *
 * 调用约束：调用者已经持有 s_feedback_lock，且尚未用本帧覆盖 feedback->sample。
 */
static uint32_t next_position_epoch(uint32_t current_epoch) {
	/* 0 留给“尚未收到过反馈”；极端回绕时重新从 1 开始。 */
	return current_epoch == UINT32_MAX ? 1U : current_epoch + 1U;
}

static void increment_rebase_count(rs01_feedback_snapshot_t *feedback) {
	/* 诊断计数饱和而不回零，避免现场把计数回绕误认为从未重建过。 */
	if (feedback->position_rebase_count != UINT32_MAX) {
		++feedback->position_rebase_count;
	}
}

/* 在当前周期角建立一个新连续坐标世代；旧软件零位会因 epoch 不同而失效。 */
static void rebase_continuous_position_locked(
	rs01_feedback_snapshot_t *feedback, float periodic_position_rad,
	bool count_as_rebase) {
	feedback->position_epoch = next_position_epoch(feedback->position_epoch);
	feedback->position_wrap_count = 0;
	feedback->continuous_position_rad = periodic_position_rad;
	feedback->continuous_position_valid = isfinite(periodic_position_rad);
	if (count_as_rebase) {
		increment_rebase_count(feedback);
	}
}

static void update_continuous_position_locked(
	rs01_feedback_snapshot_t *feedback, float periodic_position_rad,
	uint64_t now_us) {
	const float period_rad = s_limits.position_max_rad -
							 s_limits.position_min_rad;
	const float half_period_rad = period_rad * 0.5f;
	if (!isfinite(periodic_position_rad) || !isfinite(period_rad) ||
		period_rad <= 0.0f) {
		feedback->continuous_position_valid = false;
		return;
	}

	if (feedback->position_reset_pending) {
		/*
		 * twai_transmit() 成功仅代表置零帧入队，不能立即认定电机已经换坐标系。
		 * pending 期间仍发布原始 sample/时间戳，但连续位置保持无效；只有连续三帧
		 * 落在零附近才允许 RAM 软件归零和 START 再次使用该执行器。
		 */
		if (fabsf(periodic_position_rad) <=
			RS01_POSITION_RESET_CONFIRMATION_RAD) {
			if (feedback->position_reset_confirmation_count < UINT8_MAX) {
				++feedback->position_reset_confirmation_count;
			}
		} else {
			feedback->position_reset_confirmation_count = 0U;
		}
		if (feedback->position_reset_confirmation_count <
			RS01_POSITION_RESET_CONFIRMATION_FRAMES) {
			feedback->continuous_position_valid = false;
			return;
		}

		feedback->position_reset_pending = false;
		feedback->position_reset_confirmation_count = 0U;
		feedback->position_wrap_count = 0;
		feedback->continuous_position_rad = periodic_position_rad;
		feedback->continuous_position_valid = true;
		return;
	}

	int32_t wrap_count = feedback->position_wrap_count;
	const bool have_previous =
		feedback->valid && feedback->continuous_position_valid &&
		isfinite(feedback->sample.position_rad);
	if (!have_previous) {
		/* 第一帧只能建立本次启动的局部原点，还无法推断此前已经转过多少圈。 */
		if (feedback->position_epoch == 0U) {
			rebase_continuous_position_locked(feedback,
									  periodic_position_rad, false);
		} else {
			/* 机械置零已先创建新 epoch；本帧只填入该 epoch 的第一个位置。 */
			feedback->position_wrap_count = 0;
			feedback->continuous_position_rad = periodic_position_rad;
			feedback->continuous_position_valid = true;
		}
		return;
	} else {
		const uint64_t timeout_us =
			(uint64_t)g_exo_hardware_config.rs01_feedback_timeout_ms * 1000ULL;
		const uint64_t kinematic_limit_us =
			s_limits.velocity_max_rad_s > 0.0f
				? (uint64_t)(half_period_rad /
							 s_limits.velocity_max_rad_s * 1000000.0f)
				: 0U;
		const bool gap_is_ambiguous =
			timeout_us == 0U || kinematic_limit_us == 0U ||
			now_us < feedback->timestamp_us ||
			(now_us - feedback->timestamp_us) > timeout_us ||
			(now_us - feedback->timestamp_us) >= kinematic_limit_us;
		if (gap_is_ambiguous) {
			/*
			 * 超过在线超时后，按 44 rad/s 上限已经不能保证“最短模差”唯一，
			 * 所以不猜圈数，直接换 epoch 并要求人体层重新捕获软件零位。
			 */
			rebase_continuous_position_locked(feedback, periodic_position_rad,
									  true);
			return;
		}

		const float delta_rad =
			periodic_position_rad - feedback->sample.position_rad;
		if (delta_rad < -half_period_rad) {
			/* 上一帧接近 +4π、本帧回到 -4π：发生一次正向回绕。 */
			if (wrap_count == INT32_MAX) {
				rebase_continuous_position_locked(
					feedback, periodic_position_rad, true);
				return;
			}
			++wrap_count;
		} else if (delta_rad > half_period_rad) {
			/* 上一帧接近 -4π、本帧回到 +4π：发生一次反向回绕。 */
			if (wrap_count == INT32_MIN) {
				rebase_continuous_position_locked(
					feedback, periodic_position_rad, true);
				return;
			}
			--wrap_count;
		} else if (fabsf(delta_rad) == half_period_rad) {
			/* 恰好半周期既可能向前也可能向后，没有唯一展开答案。 */
			rebase_continuous_position_locked(feedback, periodic_position_rad,
									  true);
			return;
		}
	}

	const float continuous_position_rad =
		periodic_position_rad + (float)wrap_count * period_rad;
	if (!isfinite(continuous_position_rad)) {
		feedback->continuous_position_valid = false;
		return;
	}
	feedback->position_wrap_count = wrap_count;
	feedback->continuous_position_rad = continuous_position_rad;
	feedback->continuous_position_valid = true;
}

/*
 * 电机机械置零会改写位置坐标系。命令成功交给 CAN 后先让连续跟踪失效；下一帧
 * 反馈会以新的周期位置重新建立起点，避免把“坐标系突变”误判成正常跨界。
 */
static void reset_continuous_position(exo_actuator_id_t actuator) {
	portENTER_CRITICAL(&s_feedback_lock);
	rs01_feedback_snapshot_t *feedback = &s_motors[actuator].feedback;
	feedback->position_epoch = next_position_epoch(feedback->position_epoch);
	feedback->continuous_position_rad = 0.0f;
	feedback->position_wrap_count = 0;
	feedback->position_reset_confirmation_count = 0U;
	feedback->position_reset_pending = true;
	feedback->continuous_position_valid = false;
	increment_rebase_count(feedback);
	portEXIT_CRITICAL(&s_feedback_lock);
}

/*
 * 在一个临界区中发布完整反馈：采样、状态、故障、时间戳和计数同时更新，读取者
 * 不会看到“新位置配旧时间戳”。online 不在这里固定，由读取快照时按年龄计算。
 */
static void
update_feedback(exo_actuator_id_t actuator, const exo_motion_sample_t *sample,
				rs01_mode_status_t mode, const rs01_fault_flags_t *faults,
				uint8_t feedback_fault_raw, uint8_t private_frame_type) {
	const uint64_t now_us = (uint64_t)esp_timer_get_time();

	portENTER_CRITICAL(&s_feedback_lock);
	rs01_feedback_snapshot_t *feedback = &s_motors[actuator].feedback;
	if (feedback->feedback_count == 0U) {
		feedback->first_timestamp_us = now_us;
		feedback->last_interval_us = 0U;
	} else {
		feedback->last_interval_us = now_us - feedback->timestamp_us;
	}
	/* 必须在覆盖上一帧 sample 之前计算回绕，否则无法判断跨过哪一侧边界。 */
	update_continuous_position_locked(feedback, sample->position_rad, now_us);
	feedback->sample = *sample;
	feedback->mode = mode;
	s_motors[actuator].summary_faults = *faults;
	feedback->faults = merge_faults(
		*faults, detailed_faults(feedback->fault_raw, feedback->warning_raw));
	feedback->feedback_fault_raw = feedback_fault_raw;
	feedback->timestamp_us = now_us;
	++feedback->feedback_count;
	if (private_frame_type == RS01_PRIVATE_TYPE_FEEDBACK) {
		++feedback->type2_feedback_count;
	} else if (private_frame_type == RS01_PRIVATE_TYPE_REPORT) {
		++feedback->type24_feedback_count;
	}
	feedback->valid = true;
	portEXIT_CRITICAL(&s_feedback_lock);
}

/* 更新详细故障原始位图，并与最近一次反馈摘要合并。 */
static void update_fault(exo_actuator_id_t actuator, uint32_t fault,
						 uint32_t warning) {
	const rs01_fault_flags_t flags = detailed_faults(fault, warning);
	portENTER_CRITICAL(&s_feedback_lock);
	rs01_feedback_snapshot_t *feedback = &s_motors[actuator].feedback;
	feedback->fault_raw = fault;
	feedback->warning_raw = warning;
	feedback->faults = merge_faults(s_motors[actuator].summary_faults, flags);
	portEXIT_CRITICAL(&s_feedback_lock);
}

/*
 * 初始化只创建静态递归互斥锁，并用只读硬件配置填充 A/B 电机对象。重复调用幂等；
 * 本函数不安装/启动 TWAI，也不向电机发送任何帧。
 */
esp_err_t rs01_driver_init(void) {
	if (s_tx_mutex != NULL) {
		return ESP_OK;
	}

	SemaphoreHandle_t mutex =
		xSemaphoreCreateRecursiveMutexStatic(&s_tx_mutex_storage);
	if (mutex == NULL) {
		return ESP_ERR_NO_MEM;
	}

	portENTER_CRITICAL(&s_feedback_lock);
	memset(s_motors, 0, sizeof(s_motors));
	for (unsigned i = 0; i < (unsigned)EXO_ACTUATOR_COUNT; ++i) {
		s_motors[i].actuator = (exo_actuator_id_t)i;
		s_motors[i].can_id = g_exo_hardware_config.actuator[i].can_id;
		s_motors[i].feedback.mode = RS01_MODE_STATUS_UNKNOWN;
		s_motors[i].feedback.control_mode = RS01_CONTROL_MODE_UNKNOWN;
	}
	portEXIT_CRITICAL(&s_feedback_lock);

	s_tx_mutex = mutex;
	return ESP_OK;
}

/* 返回驱动内固定量程的只读指针，调用者不得修改。 */
const rs01_limits_t *rs01_get_limits(void) { return &s_limits; }

/* 查询 A/B 当前运行时节点 ID；改 ID 成功后该值会立即变化。 */
bool rs01_get_can_id(exo_actuator_id_t actuator, exo_can_node_id_t *can_id) {
	if (!actuator_is_valid(actuator) || can_id == NULL) {
		return false;
	}
	*can_id = current_can_id(actuator);
	return true;
}

/* 接收解析时按源节点 ID 反查逻辑槽位；结果仍不是人体左右侧。 */
bool rs01_find_actuator(exo_can_node_id_t can_id, exo_actuator_id_t *actuator) {
	if (actuator == NULL) {
		return false;
	}
	bool found = false;
	portENTER_CRITICAL(&s_feedback_lock);
	for (unsigned i = 0; i < (unsigned)EXO_ACTUATOR_COUNT; ++i) {
		if (s_motors[i].can_id == can_id) {
			*actuator = (exo_actuator_id_t)i;
			found = true;
			break;
		}
	}
	portEXIT_CRITICAL(&s_feedback_lock);
	return found;
}

/* 私有协议 type-3 使能：data 字段携带 ESP32 主站 ID，负载全零。 */
esp_err_t rs01_enable(exo_actuator_id_t actuator) {
	uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
	esp_err_t err =
		transmit_private(actuator, RS01_PRIVATE_TYPE_ENABLE,
						 g_exo_hardware_config.twai.master_id, payload);
	if (err == ESP_OK) {
		set_enabled_requested(actuator, true);
	}
	return err;
}

/* 私有协议 type-4 共用停止/清故障帧，payload[0] 决定具体动作。 */
static esp_err_t send_stop(exo_actuator_id_t actuator, bool clear_fault) {
	uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
	payload[0] = clear_fault ? 1U : 0U;
	esp_err_t err =
		transmit_private(actuator, RS01_PRIVATE_TYPE_STOP,
						 g_exo_hardware_config.twai.master_id, payload);
	if (err == ESP_OK && !clear_fault) {
		set_enabled_requested(actuator, false);
	}
	return err;
}

/* 发送普通停止/失能请求，并在发送成功后清除 enabled_requested。 */
esp_err_t rs01_disable(exo_actuator_id_t actuator) {
	return send_stop(actuator, false);
}

/* 使用同一 type-4 帧的 clear_fault 变体；不会把它记成失能请求。 */
esp_err_t rs01_clear_faults(exo_actuator_id_t actuator) {
	return send_stop(actuator, true);
}

/*
 * 私有 Motion 帧编码：位置、速度、KP、KD 各占 payload 中的 16 bit 大端字段，
 * 前馈转矩占扩展 ID 的 16 bit data 字段。所有字段必须先通过完整量程校验，随后
 * 才做线性量化；此处不静默限幅。
 */
static esp_err_t encode_motion_command(exo_actuator_id_t actuator,
									   const rs01_motion_command_t *command,
									   uint8_t payload[RS01_FRAME_DATA_LENGTH],
									   uint16_t *torque_raw) {
	if (!actuator_is_valid(actuator) || payload == NULL || torque_raw == NULL ||
		!rs01_motion_command_is_valid(command)) {
		return ESP_ERR_INVALID_ARG;
	}

	write_u16_be(&payload[0],
				 (uint16_t)float_to_raw(command->position_rad,
										s_limits.position_min_rad,
										s_limits.position_max_rad, 16));
	write_u16_be(&payload[2], (uint16_t)float_to_raw(
								  command->velocity_rad_s,
								  s_limits.velocity_min_rad_s,
								  s_limits.velocity_max_rad_s, 16));
	write_u16_be(&payload[4],
				 (uint16_t)float_to_raw(command->kp, s_limits.kp_min,
										s_limits.kp_max, 16));
	write_u16_be(&payload[6],
				 (uint16_t)float_to_raw(command->kd, s_limits.kd_min,
										s_limits.kd_max, 16));
	*torque_raw = (uint16_t)float_to_raw(
		command->torque_nm, s_limits.torque_min_nm, s_limits.torque_max_nm, 16);
	return ESP_OK;
}

/* 普通具名 Motion 发送：可等待 TX 锁/TWAI 队列，适合启停和调试路径。 */
esp_err_t rs01_send_motion_command(exo_actuator_id_t actuator,
								 const rs01_motion_command_t *command) {
	uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
	uint16_t torque_raw = 0;
	esp_err_t err =
		encode_motion_command(actuator, command, payload, &torque_raw);
	if (err != ESP_OK) {
		return err;
	}

	return transmit_private(actuator, RS01_PRIVATE_TYPE_CONTROL, torque_raw,
							payload);
}

/* 100 Hz 具名发送：编码相同，但发送全过程零等待且不打印日志。 */
esp_err_t rs01_try_send_motion_command(exo_actuator_id_t actuator,
									 const rs01_motion_command_t *command) {
	uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
	uint16_t torque_raw = 0;
	esp_err_t err =
		encode_motion_command(actuator, command, payload, &torque_raw);
	if (err != ESP_OK) {
		return err;
	}

	return try_transmit_private(actuator, RS01_PRIVATE_TYPE_CONTROL, torque_raw,
								payload);
}

/*
 * 阶段 2～7 已使用的标量接口继续保留。它只负责组装具名结构，实际校验、编码和
 * 发送都转交给新接口，避免长期维护两套易产生差异的实现。
 */
esp_err_t rs01_send_motion(exo_actuator_id_t actuator, float position_rad,
						   float velocity_rad_s, float torque_nm, float kp,
						   float kd) {
	const rs01_motion_command_t command = {
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.torque_nm = torque_nm,
		.kp = kp,
		.kd = kd,
	};
	return rs01_send_motion_command(actuator, &command);
}

esp_err_t rs01_try_send_motion(exo_actuator_id_t actuator, float position_rad,
							   float velocity_rad_s, float torque_nm, float kp,
							   float kd) {
	const rs01_motion_command_t command = {
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.torque_nm = torque_nm,
		.kp = kp,
		.kd = kd,
	};
	return rs01_try_send_motion_command(actuator, &command);
}

/*
 * 私有参数写入的公共底层：index 放在 payload[0..1]，4 字节原始值放在
 * payload[4..7]，均按 little-endian。递归锁把写帧和随后的 1 ms 间隔视为一个
 * 串行事务，避免组合参数命令交叉。
 */
static esp_err_t send_parameter(exo_actuator_id_t actuator, uint16_t index,
								const uint8_t value[4]) {
	if (!actuator_is_valid(actuator)) {
		return ESP_ERR_INVALID_ARG;
	}

	esp_err_t err = take_tx_lock();
	if (err != ESP_OK) {
		return err;
	}

	uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
	write_u16_le(&payload[0], index);
	memcpy(&payload[4], value, 4);
	err = transmit_private(actuator, RS01_PRIVATE_TYPE_PARAMETER_WRITE,
						   g_exo_hardware_config.twai.master_id, payload);
	if (err == ESP_OK) {
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	give_tx_lock();
	return err;
}

/* 私有参数读取只编码寄存器索引；响应由 process_private_parameter() 打印。 */
esp_err_t rs01_read_parameter(exo_actuator_id_t actuator, uint16_t index) {
	uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
	write_u16_le(&payload[0], index);
	return transmit_private(actuator, RS01_PRIVATE_TYPE_PARAMETER_READ,
							g_exo_hardware_config.twai.master_id, payload);
}

/*
 * 三个裸写函数只选择 32-bit float、8-bit 或 16-bit 的线表示，不理解寄存器含义。
 * 对角度寄存器，调用者必须按手册自行提供 rad 等原生单位。
 */
esp_err_t rs01_write_parameter_float(exo_actuator_id_t actuator, uint16_t index,
									 float value) {
	if (!isfinite(value)) {
		return ESP_ERR_INVALID_ARG;
	}
	uint32_t raw;
	memcpy(&raw, &value, sizeof(raw));
	uint8_t encoded[4];
	write_u32_le(encoded, raw);
	return send_parameter(actuator, index, encoded);
}

/* 8-bit 参数占 value 区首字节，其余三字节补零。 */
esp_err_t rs01_write_parameter_u8(exo_actuator_id_t actuator, uint16_t index,
								  uint8_t value) {
	const uint8_t encoded[4] = {value, 0, 0, 0};
	esp_err_t err = send_parameter(actuator, index, encoded);
	if (err == ESP_OK && index == RS01_PARAMETER_RUN_MODE) {
		rs01_control_mode_t mode = RS01_CONTROL_MODE_UNKNOWN;
		if (value == (uint8_t)RS01_CONTROL_MODE_MOTION ||
			value == (uint8_t)RS01_CONTROL_MODE_POSITION_PP ||
			value == (uint8_t)RS01_CONTROL_MODE_VELOCITY ||
			value == (uint8_t)RS01_CONTROL_MODE_CURRENT ||
			value == (uint8_t)RS01_CONTROL_MODE_POSITION_CSP) {
			mode = (rs01_control_mode_t)value;
		}
		set_control_mode_state(actuator, mode);
	}
	return err;
}

/* 16-bit 参数按 little-endian 放在 value 区前两字节，其余补零。 */
esp_err_t rs01_write_parameter_u16(exo_actuator_id_t actuator, uint16_t index,
								   uint16_t value) {
	uint8_t encoded[4] = {0};
	write_u16_le(encoded, value);
	return send_parameter(actuator, index, encoded);
}

/* 只允许当前驱动已经实现并由 RS01 手册定义的五种控制模式。 */
static bool control_mode_is_valid(rs01_control_mode_t mode) {
	return mode == RS01_CONTROL_MODE_MOTION ||
		   mode == RS01_CONTROL_MODE_POSITION_PP ||
		   mode == RS01_CONTROL_MODE_VELOCITY ||
		   mode == RS01_CONTROL_MODE_CURRENT ||
		   mode == RS01_CONTROL_MODE_POSITION_CSP;
}

/* 写 0x7005 切换私有协议运行模式，并同步本地“最近请求模式”。 */
esp_err_t rs01_set_control_mode(exo_actuator_id_t actuator,
								rs01_control_mode_t mode) {
	if (!control_mode_is_valid(mode)) {
		return ESP_ERR_INVALID_ARG;
	}
	return rs01_write_parameter_u8(actuator, RS01_PARAMETER_RUN_MODE,
								   (uint8_t)mode);
}

/*
 * 组合控制 API 的准备步骤：若最新反馈表明电机正在运行，则先失能；随后写运行
 * 模式并重新使能。调用者通常已持有递归 TX 锁，因此整组帧不会被其他发送插入。
 */
static esp_err_t prepare_control_mode(exo_actuator_id_t actuator,
									  rs01_control_mode_t mode) {
	rs01_feedback_snapshot_t feedback;
	if (rs01_get_feedback(actuator, &feedback) && feedback.valid &&
		feedback.mode == RS01_MODE_STATUS_RUNNING) {
		esp_err_t err = rs01_disable(actuator);
		if (err != ESP_OK) {
			return err;
		}
	}

	esp_err_t err = rs01_set_control_mode(actuator, mode);
	if (err != ESP_OK) {
		return err;
	}
	return rs01_enable(actuator);
}

/* 调试兼容入口：把模式准备和一帧私有 Motion 命令组成同一串行事务。 */
esp_err_t rs01_move(exo_actuator_id_t actuator, float torque_nm,
					float position_rad, float velocity_rad_s, float kp,
					float kd) {
	if (!motion_values_are_valid(position_rad, velocity_rad_s, torque_nm, kp,
								 kd)) {
		return ESP_ERR_INVALID_ARG;
	}
	esp_err_t err = take_tx_lock();
	if (err != ESP_OK) {
		return err;
	}
	err = prepare_control_mode(actuator, RS01_CONTROL_MODE_MOTION);
	if (err == ESP_OK) {
		err = rs01_send_motion(actuator, position_rad, velocity_rad_s,
							   torque_nm, kp, kd);
	}
	give_tx_lock();
	return err;
}

/* 速度模式组合命令：依次写电流限制、加速度 rad/s² 和目标速度 rad/s。 */
esp_err_t rs01_speed_control(exo_actuator_id_t actuator, float current_limit_a,
							 float acceleration_rad_s2, float velocity_rad_s) {
	if (!isfinite(current_limit_a) || !isfinite(acceleration_rad_s2) ||
		!isfinite(velocity_rad_s)) {
		return ESP_ERR_INVALID_ARG;
	}
	esp_err_t err = take_tx_lock();
	if (err != ESP_OK) {
		return err;
	}
	err = prepare_control_mode(actuator, RS01_CONTROL_MODE_VELOCITY);
	if (err == ESP_OK) {
		err = rs01_write_parameter_float(actuator, RS01_PARAMETER_CURRENT_LIMIT,
										 current_limit_a);
	}
	if (err == ESP_OK) {
		err = rs01_write_parameter_float(actuator, RS01_PARAMETER_ACCELERATION,
										 acceleration_rad_s2);
	}
	if (err == ESP_OK) {
		err = rs01_write_parameter_float(
			actuator, RS01_PARAMETER_SPEED_REFERENCE, velocity_rad_s);
	}
	give_tx_lock();
	return err;
}

/* PP 位置模式组合命令：依次写速度、加速度和绝对电机位置，角度量均为弧度制。 */
esp_err_t rs01_position_pp_control(exo_actuator_id_t actuator,
								   float velocity_rad_s,
								   float acceleration_rad_s2,
								   float position_rad) {
	if (!isfinite(velocity_rad_s) || !isfinite(acceleration_rad_s2) ||
		!isfinite(position_rad)) {
		return ESP_ERR_INVALID_ARG;
	}
	esp_err_t err = take_tx_lock();
	if (err != ESP_OK) {
		return err;
	}
	err = prepare_control_mode(actuator, RS01_CONTROL_MODE_POSITION_PP);
	if (err == ESP_OK) {
		err = rs01_write_parameter_float(actuator, RS01_PARAMETER_PP_SPEED,
										 velocity_rad_s);
	}
	if (err == ESP_OK) {
		err = rs01_write_parameter_float(
			actuator, RS01_PARAMETER_PP_ACCELERATION, acceleration_rad_s2);
	}
	if (err == ESP_OK) {
		err = rs01_write_parameter_float(
			actuator, RS01_PARAMETER_POSITION_REFERENCE, position_rad);
	}
	give_tx_lock();
	return err;
}

/* CSP 位置模式组合命令：先写速度限制，再写绝对电机位置。 */
esp_err_t rs01_position_csp_control(exo_actuator_id_t actuator,
									float velocity_limit_rad_s,
									float position_rad) {
	if (!isfinite(velocity_limit_rad_s) || !isfinite(position_rad)) {
		return ESP_ERR_INVALID_ARG;
	}
	esp_err_t err = take_tx_lock();
	if (err != ESP_OK) {
		return err;
	}
	err = prepare_control_mode(actuator, RS01_CONTROL_MODE_POSITION_CSP);
	if (err == ESP_OK) {
		err = rs01_write_parameter_float(
			actuator, RS01_PARAMETER_CSP_SPEED_LIMIT, velocity_limit_rad_s);
	}
	if (err == ESP_OK) {
		err = rs01_write_parameter_float(
			actuator, RS01_PARAMETER_POSITION_REFERENCE, position_rad);
	}
	give_tx_lock();
	return err;
}

/* 电流模式组合命令：准备 CURRENT 模式后写入电流参考值。 */
esp_err_t rs01_current_control(exo_actuator_id_t actuator, float current_a) {
	if (!isfinite(current_a)) {
		return ESP_ERR_INVALID_ARG;
	}
	esp_err_t err = take_tx_lock();
	if (err != ESP_OK) {
		return err;
	}
	err = prepare_control_mode(actuator, RS01_CONTROL_MODE_CURRENT);
	if (err == ESP_OK) {
		err = rs01_write_parameter_float(
			actuator, RS01_PARAMETER_CURRENT_REFERENCE, current_a);
	}
	give_tx_lock();
	return err;
}

/*
 * 把产品实际测得的毫秒周期换算成 0x7026 值。10 ms 对应 0，之后按 5 ms 步进；
 * 本函数只配置周期，不会自动打开主动上报。
 */
esp_err_t rs01_set_report_interval_ms(exo_actuator_id_t actuator,
									  uint32_t interval_ms) {
	if (interval_ms < 10U || ((interval_ms - 10U) % 5U) != 0U) {
		return ESP_ERR_INVALID_ARG;
	}
	/* 当前两台产品电机已实测：手册易被理解为 1=10 ms，但实际 1 约为 15 ms。 */
	const uint32_t encoded = (interval_ms - 10U) / 5U;
	if (encoded > UINT16_MAX) {
		return ESP_ERR_INVALID_ARG;
	}
	return rs01_write_parameter_u16(actuator, RS01_PARAMETER_REPORT_INTERVAL,
									(uint16_t)encoded);
}

/* 私有维护命令共用 01..06、命令值和末尾 0x08 的 8 字节负载模板。 */
static void fill_private_maintenance_payload(uint8_t payload[8],
											 uint8_t command) {
	for (unsigned i = 0; i < 6U; ++i) {
		payload[i] = (uint8_t)(i + 1U);
	}
	payload[6] = command;
	payload[7] = 0x08;
}

/* 缺少电机ACK时，普通CAN帧可能由控制器不断重传。恢复请求使用SS标志：
 * 一次入队只尝试一次总线发送，不会形成无人应答的后台重传风暴。
 * 该有限原语与实时运动出口分开；不清BUS_OFF、不使能电机。 */
esp_err_t rs01_request_feedback_once(exo_actuator_id_t actuator,
                                    uint32_t interval_ms) {
    if (!actuator_is_valid(actuator) || interval_ms < 10U ||
        (interval_ms - 10U) % 5U != 0U ||
        (interval_ms - 10U) / 5U > UINT16_MAX) return ESP_ERR_INVALID_ARG;
    esp_err_t err = try_take_tx_lock();
    if (err != ESP_OK) return err;
    uint8_t payload[8] = {0};
    write_u16_le(payload, RS01_PARAMETER_REPORT_INTERVAL);
    write_u16_le(&payload[4], (uint16_t)((interval_ms - 10U) / 5U));
    for (unsigned step = 0; step < 2U; ++step) {
        twai_status_info_t can;
        if (twai_get_status_info(&can) != ESP_OK || can.state != TWAI_STATE_RUNNING ||
            can.tx_error_counter >= 128U || can.rx_error_counter >= 128U) {
            err = ESP_ERR_INVALID_STATE;
            break;
        }
        if (step == 1U) fill_private_maintenance_payload(payload, 1U);
        twai_message_t frame = {0};
        frame.flags = TWAI_MSG_FLAG_EXTD | TWAI_MSG_FLAG_SS;
        frame.identifier = make_extended_id(current_can_id(actuator),
            g_exo_hardware_config.twai.master_id,
            step == 0U ? RS01_PRIVATE_TYPE_PARAMETER_WRITE : RS01_PRIVATE_TYPE_REPORT);
        frame.data_length_code = 8U;
        memcpy(frame.data, payload, sizeof(payload));
        err = twai_transmit(&frame, 0);
        if (err != ESP_OK) break;
    }
    if (err == ESP_OK) set_reporting_requested(actuator, true);
    give_tx_lock();
    return err;
}

/*
 * 发送私有协议 type-24 主动上报开关。它只改变开关，不写 0x7026；发送成功后
 * reporting_requested=true 也只是本地请求记录，是否真正上报要看反馈计数和年龄。
 */
static esp_err_t set_private_reporting(exo_actuator_id_t actuator,
									   bool enabled) {
	uint8_t payload[RS01_FRAME_DATA_LENGTH];
	/*
	 * 手册表格只列出 01..06、F_CMD，漏写第八字节；阶段 0/1 实机基线已验证末尾
	 * 0x08 可用。在新的电机固件或明确应答证明其他格式前保持这一已验证线格式。
	 */
	fill_private_maintenance_payload(payload, enabled ? 1U : 0U);
	esp_err_t err =
		transmit_private(actuator, RS01_PRIVATE_TYPE_REPORT,
						 g_exo_hardware_config.twai.master_id, payload);
	if (err == ESP_OK) {
		set_reporting_requested(actuator, enabled);
	}
	return err;
}

/* 打开私有协议主动上报；上报周期应事先单独配置。 */
esp_err_t rs01_start_reporting(exo_actuator_id_t actuator) {
	return set_private_reporting(actuator, true);
}

/* 关闭私有协议主动上报，不改变已经写入的上报周期。 */
esp_err_t rs01_stop_reporting(exo_actuator_id_t actuator) {
	return set_private_reporting(actuator, false);
}

/*
 * 私有机械置零：先失能，再发送 type-6。它修改电机侧机械零点，与 hip_joint 的
 * RAM 软件零位完全不同。
 */
esp_err_t rs01_set_mechanical_zero(exo_actuator_id_t actuator) {
	esp_err_t err = take_tx_lock();
	if (err != ESP_OK) {
		return err;
	}
	err = rs01_disable(actuator);
	if (err == ESP_OK) {
		uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
		payload[0] = 1U;
		err = transmit_private(actuator, RS01_PRIVATE_TYPE_ZERO,
							   g_exo_hardware_config.twai.master_id, payload);
	}
	give_tx_lock();
	if (err == ESP_OK) {
		reset_continuous_position(actuator);
	}
	return err;
}

/*
 * 私有改节点 ID：持 TX 锁检查 A/B 不冲突、先失能、发送 type-7，成功后更新 RAM
 * 映射。该更新不会修改编译期硬件配置，设备重启后的期望 ID 仍由配置决定。
 */
esp_err_t rs01_set_can_id(exo_actuator_id_t actuator,
						  exo_can_node_id_t new_can_id) {
	if (!actuator_is_valid(actuator) || new_can_id > RS01_MAX_CAN_ID) {
		return ESP_ERR_INVALID_ARG;
	}
	esp_err_t err = take_tx_lock();
	if (err != ESP_OK) {
		return err;
	}
	if (can_id_conflicts(actuator, new_can_id)) {
		give_tx_lock();
		return ESP_ERR_INVALID_ARG;
	}
	err = rs01_disable(actuator);
	if (err == ESP_OK) {
		uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
		const uint16_t data_field =
			((uint16_t)new_can_id << 8) | g_exo_hardware_config.twai.master_id;
		err = transmit_private(actuator, RS01_PRIVATE_TYPE_SET_CAN_ID,
							   data_field, payload);
	}
	if (err == ESP_OK) {
		set_current_can_id(actuator, new_can_id);
	}
	give_tx_lock();
	return err;
}

/* 私有 type-22 保存参数命令，负载按手册使用 01..08。 */
esp_err_t rs01_save_parameters(exo_actuator_id_t actuator) {
	uint8_t payload[RS01_FRAME_DATA_LENGTH];
	for (unsigned i = 0; i < RS01_FRAME_DATA_LENGTH; ++i) {
		payload[i] = (uint8_t)(i + 1U);
	}
	return transmit_private(actuator, RS01_PRIVATE_TYPE_SAVE,
							g_exo_hardware_config.twai.master_id, payload);
}

/* 私有 type-23 设置电机 CAN 波特率选择值；ESP32 自身 TWAI 位率不会随之改变。 */
esp_err_t rs01_set_baud_rate(exo_actuator_id_t actuator,
							 uint8_t baud_selector) {
	if (baud_selector < 1U || baud_selector > 4U) {
		return ESP_ERR_INVALID_ARG;
	}
	uint8_t payload[RS01_FRAME_DATA_LENGTH];
	fill_private_maintenance_payload(payload, baud_selector);
	return transmit_private(actuator, RS01_PRIVATE_TYPE_BAUD,
							g_exo_hardware_config.twai.master_id, payload);
}

/* 私有 type-25 请求切换电机协议；是否立即生效由电机固件决定。 */
esp_err_t rs01_set_protocol(exo_actuator_id_t actuator,
							rs01_protocol_t protocol) {
	if ((unsigned)protocol > (unsigned)RS01_PROTOCOL_MIT) {
		return ESP_ERR_INVALID_ARG;
	}
	uint8_t payload[RS01_FRAME_DATA_LENGTH];
	fill_private_maintenance_payload(payload, (uint8_t)protocol);
	return transmit_private(actuator, RS01_PRIVATE_TYPE_PROTOCOL,
							g_exo_hardware_config.twai.master_id, payload);
}

/* MIT 终端命令模板：前 7 字节填 0xFF，最后一字节放使能/失能/置零等命令码。 */
static esp_err_t send_mit_terminal_command(exo_actuator_id_t actuator,
										   uint8_t terminal) {
	uint8_t payload[RS01_FRAME_DATA_LENGTH];
	memset(payload, 0xFF, sizeof(payload));
	payload[7] = terminal;
	return transmit_standard(actuator, current_can_id(actuator), payload);
}

/* MIT 带一个参数的终端模板：前 6 字节 0xFF，data[6] 是值，data[7] 是命令码。 */
static esp_err_t send_mit_command_value(exo_actuator_id_t actuator,
										uint8_t value, uint8_t terminal) {
	uint8_t payload[RS01_FRAME_DATA_LENGTH];
	memset(payload, 0xFF, 6);
	payload[6] = value;
	payload[7] = terminal;
	return transmit_standard(actuator, current_can_id(actuator), payload);
}

/* MIT 使能/失能：标准帧 ID 为当前节点 ID，末字节分别是 0xFC/0xFD。 */
esp_err_t rs01_mit_enable(exo_actuator_id_t actuator) {
	if (!actuator_is_valid(actuator)) {
		return ESP_ERR_INVALID_ARG;
	}
	esp_err_t err = send_mit_terminal_command(actuator, 0xFC);
	if (err == ESP_OK) {
		set_enabled_requested(actuator, true);
	}
	return err;
}

/* MIT 失能：发送成功后只更新本地 enabled_requested=false。 */
esp_err_t rs01_mit_disable(exo_actuator_id_t actuator) {
	if (!actuator_is_valid(actuator)) {
		return ESP_ERR_INVALID_ARG;
	}
	esp_err_t err = send_mit_terminal_command(actuator, 0xFD);
	if (err == ESP_OK) {
		set_enabled_requested(actuator, false);
	}
	return err;
}

/*
 * MIT 故障命令：0xFF 表示清除，其余值表示查询。查询发送成功后开启短暂等待窗口，
 * 帮助接收端区分同一主站 ID 下的故障响应与普通反馈。
 */
esp_err_t rs01_mit_clear_or_read_fault(exo_actuator_id_t actuator,
									   uint8_t command) {
	if (!actuator_is_valid(actuator)) {
		return ESP_ERR_INVALID_ARG;
	}
	esp_err_t err = send_mit_command_value(actuator, command, 0xFB);
	if (err == ESP_OK) {
		/* 0xFF 清故障后返回普通反馈；其他值按查询处理。 */
		set_mit_fault_response_pending(actuator, command != 0xFFU);
	}
	return err;
}

/* MIT 模式值 0/1/2 分别映射到 Motion、位置 CSP、速度控制。 */
esp_err_t rs01_mit_set_operation_mode(exo_actuator_id_t actuator,
									  uint8_t mode) {
	if (!actuator_is_valid(actuator) || mode > 2U) {
		return ESP_ERR_INVALID_ARG;
	}
	esp_err_t err = send_mit_command_value(actuator, mode, 0xFC);
	if (err == ESP_OK) {
		const rs01_control_mode_t mapped[] = {
			RS01_CONTROL_MODE_MOTION,
			RS01_CONTROL_MODE_POSITION_CSP,
			RS01_CONTROL_MODE_VELOCITY,
		};
		set_control_mode_state(actuator, mapped[mode]);
	}
	return err;
}

/* MIT 改节点 ID：检查 A/B 冲突，发送成功后更新本次运行的 RAM 映射。 */
esp_err_t rs01_mit_set_can_id(exo_actuator_id_t actuator,
							  exo_can_node_id_t new_can_id) {
	if (!actuator_is_valid(actuator) || new_can_id > RS01_MAX_CAN_ID) {
		return ESP_ERR_INVALID_ARG;
	}
	esp_err_t err = take_tx_lock();
	if (err != ESP_OK) {
		return err;
	}
	if (can_id_conflicts(actuator, new_can_id)) {
		give_tx_lock();
		return ESP_ERR_INVALID_ARG;
	}
	err = send_mit_command_value(actuator, new_can_id, 0xFA);
	if (err == ESP_OK) {
		set_current_can_id(actuator, new_can_id);
	}
	give_tx_lock();
	return err;
}

/* MIT 协议切换命令；这里的末字节 0xFD 是命令码，不是 ESP32 主站 ID。 */
esp_err_t rs01_mit_set_protocol(exo_actuator_id_t actuator,
								rs01_protocol_t protocol) {
	if (!actuator_is_valid(actuator) ||
		(unsigned)protocol > (unsigned)RS01_PROTOCOL_MIT) {
		return ESP_ERR_INVALID_ARG;
	}
	/* 0xFD 是 MIT 命令字节，不是 g_exo_hardware_config 中配置的主站 ID。 */
	return send_mit_command_value(actuator, (uint8_t)protocol, 0xFD);
}

/*
 * MIT Motion 定点帧：position 使用 16 bit，velocity/KP/KD/torque 各使用 12
 * bit，按手册跨字节紧密拼接。输入必须完整落在 s_limits 内，不静默限幅；位置
 * 和速度单位仍为 rad、rad/s。
 */
esp_err_t rs01_mit_send_motion(exo_actuator_id_t actuator, float position_rad,
							   float velocity_rad_s, float kp, float kd,
							   float torque_nm) {
	if (!actuator_is_valid(actuator) ||
		!motion_values_are_valid(position_rad, velocity_rad_s, torque_nm, kp,
								 kd)) {
		return ESP_ERR_INVALID_ARG;
	}

	const uint16_t position = (uint16_t)float_to_raw(
		position_rad, s_limits.position_min_rad, s_limits.position_max_rad, 16);
	const uint16_t velocity =
		(uint16_t)float_to_raw(velocity_rad_s, s_limits.velocity_min_rad_s,
							   s_limits.velocity_max_rad_s, 12);
	const uint16_t kp_raw =
		(uint16_t)float_to_raw(kp, s_limits.kp_min, s_limits.kp_max, 12);
	const uint16_t kd_raw =
		(uint16_t)float_to_raw(kd, s_limits.kd_min, s_limits.kd_max, 12);
	const uint16_t torque = (uint16_t)float_to_raw(
		torque_nm, s_limits.torque_min_nm, s_limits.torque_max_nm, 12);

	uint8_t payload[RS01_FRAME_DATA_LENGTH];
	payload[0] = (uint8_t)(position >> 8);
	payload[1] = (uint8_t)position;
	payload[2] = (uint8_t)(velocity >> 4);
	payload[3] = (uint8_t)((velocity << 4) | (kp_raw >> 8));
	payload[4] = (uint8_t)kp_raw;
	payload[5] = (uint8_t)(kd_raw >> 4);
	payload[6] = (uint8_t)((kd_raw << 4) | (torque >> 8));
	payload[7] = (uint8_t)torque;
	return transmit_standard(actuator, current_can_id(actuator), payload);
}

/*
 * MIT 位置/速度专用帧的通用编码器：两个 IEEE-754 float32 分别按 little-endian
 * 放入前后 4 字节。函数不解释单位，具名调用者负责传入手册规定的量。
 */
static esp_err_t send_two_floats(exo_actuator_id_t actuator,
								 uint16_t identifier, float first,
								 float second) {
	if (!actuator_is_valid(actuator) || !isfinite(first) || !isfinite(second)) {
		return ESP_ERR_INVALID_ARG;
	}
	uint32_t first_raw;
	uint32_t second_raw;
	memcpy(&first_raw, &first, sizeof(first_raw));
	memcpy(&second_raw, &second, sizeof(second_raw));
	uint8_t payload[RS01_FRAME_DATA_LENGTH];
	write_u32_le(&payload[0], first_raw);
	write_u32_le(&payload[4], second_raw);
	return transmit_standard(actuator, identifier, payload);
}

/* MIT 位置模式：标准帧 ID 高位操作号为 1，负载为 position_rad、velocity_rad_s。
 */
esp_err_t rs01_mit_position_control(exo_actuator_id_t actuator,
									float position_rad, float velocity_rad_s) {
	if (!actuator_is_valid(actuator)) {
		return ESP_ERR_INVALID_ARG;
	}
	const uint16_t identifier =
		(uint16_t)((1U << 8) | current_can_id(actuator));
	return send_two_floats(actuator, identifier, position_rad, velocity_rad_s);
}

/* MIT 速度模式：标准帧 ID 高位操作号为 2，负载为速度和电流限制。 */
esp_err_t rs01_mit_speed_control(exo_actuator_id_t actuator,
								 float velocity_rad_s, float current_limit_a) {
	if (!actuator_is_valid(actuator)) {
		return ESP_ERR_INVALID_ARG;
	}
	const uint16_t identifier =
		(uint16_t)((2U << 8) | current_can_id(actuator));
	return send_two_floats(actuator, identifier, velocity_rad_s,
						   current_limit_a);
}

/* MIT 机械置零和保存参数仍是电机侧维护动作，不涉及髋关节 RAM 软件零位。 */
esp_err_t rs01_mit_set_zero(exo_actuator_id_t actuator) {
	if (!actuator_is_valid(actuator)) {
		return ESP_ERR_INVALID_ARG;
	}
	const esp_err_t err = send_mit_terminal_command(actuator, 0xFE);
	if (err == ESP_OK) {
		reset_continuous_position(actuator);
	}
	return err;
}

/* MIT 保存参数使用末字节 0xF8 的简单终端命令。 */
esp_err_t rs01_mit_save_parameters(exo_actuator_id_t actuator) {
	if (!actuator_is_valid(actuator)) {
		return ESP_ERR_INVALID_ARG;
	}
	return send_mit_terminal_command(actuator, 0xF8);
}

/* MIT 主动上报开关；发送成功仅更新 reporting_requested 请求记录。 */
static esp_err_t set_mit_reporting(exo_actuator_id_t actuator, bool enabled) {
	if (!actuator_is_valid(actuator)) {
		return ESP_ERR_INVALID_ARG;
	}
	esp_err_t err = send_mit_command_value(actuator, enabled ? 1U : 0U, 0xF9);
	if (err == ESP_OK) {
		set_reporting_requested(actuator, enabled);
	}
	return err;
}

/* 打开 MIT 主动上报。 */
esp_err_t rs01_mit_start_reporting(exo_actuator_id_t actuator) {
	return set_mit_reporting(actuator, true);
}

/* 关闭 MIT 主动上报。 */
esp_err_t rs01_mit_stop_reporting(exo_actuator_id_t actuator) {
	return set_mit_reporting(actuator, false);
}

/* MIT 参数读：操作号 3 写入标准帧 ID 高位，payload[0..1] 保存寄存器索引。 */
esp_err_t rs01_mit_read_parameter(exo_actuator_id_t actuator, uint16_t index) {
	if (!actuator_is_valid(actuator)) {
		return ESP_ERR_INVALID_ARG;
	}
	uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
	write_u16_le(&payload[0], index);
	const uint16_t identifier =
		(uint16_t)((3U << 8) | current_can_id(actuator));
	return transmit_standard(actuator, identifier, payload);
}

/* MIT 参数写：操作号 4；索引和 float32 均按 little-endian 原样编码。 */
esp_err_t rs01_mit_write_parameter_float(exo_actuator_id_t actuator,
										 uint16_t index, float value) {
	if (!actuator_is_valid(actuator) || !isfinite(value)) {
		return ESP_ERR_INVALID_ARG;
	}
	uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
	write_u16_le(&payload[0], index);
	uint32_t raw;
	memcpy(&raw, &value, sizeof(raw));
	write_u32_le(&payload[4], raw);
	const uint16_t identifier =
		(uint16_t)((4U << 8) | current_can_id(actuator));
	return transmit_standard(actuator, identifier, payload);
}

/*
 * 先在临界区复制完整反馈，再在锁外用当前时间计算 online。valid 表示曾收到有效
 * 帧；online 进一步要求最近反馈年龄不超过配置超时，两者不能混为一谈。
 */
bool rs01_get_feedback(exo_actuator_id_t actuator,
					   rs01_feedback_snapshot_t *snapshot) {
	if (!actuator_is_valid(actuator) || snapshot == NULL) {
		return false;
	}

	portENTER_CRITICAL(&s_feedback_lock);
	*snapshot = s_motors[actuator].feedback;
	portEXIT_CRITICAL(&s_feedback_lock);

	const uint64_t now_us = (uint64_t)esp_timer_get_time();
	const uint64_t timeout_us =
		(uint64_t)g_exo_hardware_config.rs01_feedback_timeout_ms * 1000ULL;
	snapshot->online = snapshot->valid && timeout_us > 0U &&
					   now_us >= snapshot->timestamp_us &&
					   (now_us - snapshot->timestamp_us) <= timeout_us;
	return true;
}

/* 已知来源的坏帧累加到对应 A/B；未知来源只返回 INVALID，不猜测归属。 */
static rs01_rx_event_t invalid_event(exo_actuator_id_t actuator,
									 exo_actuator_id_t *updated_actuator) {
	if (actuator_is_valid(actuator)) {
		increment_invalid_frame(actuator);
		if (updated_actuator != NULL) {
			*updated_actuator = actuator;
		}
	}
	return RS01_RX_EVENT_INVALID;
}

/*
 * 解析私有 type-2/type-24 反馈：扩展 ID 的低 8 位目的地址必须是主站 ID，data
 * 字段低 8 位是源电机节点、高 8 位是状态；payload
 * 解码为位置、速度、转矩和温度。
 */
static rs01_rx_event_t
process_private_feedback(const twai_message_t *frame,
						 exo_actuator_id_t *updated_actuator) {
	const uint16_t data_field = extended_id_data(frame->identifier);
	exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
	const bool source_known =
		rs01_find_actuator((uint8_t)data_field, &actuator);

	if (frame->data_length_code != RS01_FRAME_DATA_LENGTH ||
		extended_id_destination(frame->identifier) !=
			g_exo_hardware_config.twai.master_id ||
		!source_known) {
		return invalid_event(actuator, updated_actuator);
	}

	const uint8_t status = (uint8_t)(data_field >> 8);
	const uint8_t feedback_fault_raw = status & 0x3FU;
	const rs01_fault_flags_t faults =
		private_summary_faults(feedback_fault_raw);
	exo_motion_sample_t sample = {
		.position_rad = raw_to_float(read_u16_be(&frame->data[0]),
									 s_limits.position_min_rad,
									 s_limits.position_max_rad, UINT16_MAX),
		.velocity_rad_s = raw_to_float(read_u16_be(&frame->data[2]),
									   s_limits.velocity_min_rad_s,
									   s_limits.velocity_max_rad_s, UINT16_MAX),
		.torque_nm =
			raw_to_float(read_u16_be(&frame->data[4]), s_limits.torque_min_nm,
						 s_limits.torque_max_nm, UINT16_MAX),
		.temperature_c = (float)read_u16_be(&frame->data[6]) / 10.0f,
	};
	update_feedback(actuator, &sample, decode_mode(status >> 6), &faults,
					feedback_fault_raw, extended_id_type(frame->identifier));
	if (updated_actuator != NULL) {
		*updated_actuator = actuator;
	}
	return RS01_RX_EVENT_FEEDBACK;
}

/* 解析私有 type-21 故障响应，并保存原始 fault/warning 位图及统一故障标志。 */
static rs01_rx_event_t
process_private_fault(const twai_message_t *frame,
					  exo_actuator_id_t *updated_actuator) {
	const uint16_t data_field = extended_id_data(frame->identifier);
	exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
	const bool source_known =
		rs01_find_actuator((uint8_t)data_field, &actuator);
	if (frame->data_length_code != RS01_FRAME_DATA_LENGTH ||
		extended_id_destination(frame->identifier) !=
			g_exo_hardware_config.twai.master_id ||
		!source_known) {
		return invalid_event(actuator, updated_actuator);
	}

	/*
	 * type-21 手册给出两个
	 * uint32，但没有明确字节序；当前沿用参数数据的小端约定。
	 * 在通过实机故障注入/抓帧确认前，详细故障位解释仍应视为待验证。
	 */
	const uint32_t fault = read_u32_le(&frame->data[0]);
	const uint32_t warning = read_u32_le(&frame->data[4]);
	update_fault(actuator, fault, warning);
	if (updated_actuator != NULL) {
		*updated_actuator = actuator;
	}
	ESP_LOGW(TAG, "actuator %s fault=0x%08lX warning=0x%08lX",
			 actuator_name(actuator), (unsigned long)fault,
			 (unsigned long)warning);
	return RS01_RX_EVENT_FAULT;
}

/*
 * 解析私有参数读/写响应：校验目的主站、源节点和状态后，同时以 raw、uint、float
 * 三种形式打印，避免驱动擅自猜测某个寄存器的数据类型和单位。
 */
static rs01_rx_event_t
process_private_parameter(const twai_message_t *frame,
						  exo_actuator_id_t *updated_actuator) {
	const uint16_t data_field = extended_id_data(frame->identifier);
	exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
	const bool source_known =
		rs01_find_actuator((uint8_t)data_field, &actuator);
	const uint8_t response_status = (uint8_t)(data_field >> 8);
	if (frame->data_length_code != RS01_FRAME_DATA_LENGTH ||
		extended_id_destination(frame->identifier) !=
			g_exo_hardware_config.twai.master_id ||
		!source_known || response_status > 1U) {
		return invalid_event(actuator, updated_actuator);
	}

	const uint16_t index =
		(uint16_t)(frame->data[0] | ((uint16_t)frame->data[1] << 8));
	const uint32_t raw = read_u32_le(&frame->data[4]);
	float value;
	memcpy(&value, &raw, sizeof(value));
	ESP_LOGI(TAG,
			 "actuator %s parameter index=0x%04X status=%u "
			 "raw=0x%08lX uint=%lu float=%g",
			 actuator_name(actuator), index, response_status,
			 (unsigned long)raw, (unsigned long)raw, (double)value);
	if (updated_actuator != NULL) {
		*updated_actuator = actuator;
	}
	return RS01_RX_EVENT_PARAMETER;
}

/* 根据扩展 ID 的 type 字段分派私有反馈、故障和参数响应；无关类型返回 NONE。 */
static rs01_rx_event_t
process_private_frame(const twai_message_t *frame,
					  exo_actuator_id_t *updated_actuator) {
	switch (extended_id_type(frame->identifier)) {
	case RS01_PRIVATE_TYPE_FEEDBACK:
	case RS01_PRIVATE_TYPE_REPORT:
		return process_private_feedback(frame, updated_actuator);
	case RS01_PRIVATE_TYPE_FAULT:
		return process_private_fault(frame, updated_actuator);
	case RS01_PRIVATE_TYPE_PARAMETER_READ:
	case RS01_PRIVATE_TYPE_PARAMETER_WRITE:
		return process_private_parameter(frame, updated_actuator);
	default:
		return RS01_RX_EVENT_NONE;
	}
}

/*
 * 解析 MIT 普通反馈：data[0] 是源节点，位置 16 bit，速度/转矩/温度 12 bit；
 * 定点值按 s_limits 还原，输出仍是原始电机 rad、rad/s、N·m、°C。
 */
static rs01_rx_event_t
process_mit_feedback(const twai_message_t *frame,
					 exo_actuator_id_t *updated_actuator) {
	exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
	const bool source_known = frame->data_length_code > 0U &&
							  rs01_find_actuator(frame->data[0], &actuator);
	if (frame->data_length_code != RS01_FRAME_DATA_LENGTH || !source_known) {
		return invalid_event(actuator, updated_actuator);
	}

	const uint16_t position_raw = read_u16_be(&frame->data[1]);
	const uint16_t velocity_raw =
		(uint16_t)(((uint16_t)frame->data[3] << 4) | (frame->data[4] >> 4));
	const uint16_t torque_raw =
		(uint16_t)((((uint16_t)frame->data[4] & 0x0FU) << 8) | frame->data[5]);
	const uint16_t temperature_raw =
		(uint16_t)((((uint16_t)frame->data[6] & 0x0FU) << 8) | frame->data[7]);
	const uint8_t status = frame->data[6];

	exo_motion_sample_t sample = {
		.position_rad = raw_to_float(position_raw, s_limits.position_min_rad,
									 s_limits.position_max_rad, UINT16_MAX),
		.velocity_rad_s =
			raw_to_float(velocity_raw, s_limits.velocity_min_rad_s,
						 s_limits.velocity_max_rad_s, 0x0FFFU),
		.torque_nm = raw_to_float(torque_raw, s_limits.torque_min_nm,
								  s_limits.torque_max_nm, 0x0FFFU),
		.temperature_c = (float)temperature_raw / 10.0f,
	};
	rs01_fault_flags_t faults = {0};
	faults.present = (status & (1U << 5)) != 0U;
	faults.warning_present = (status & (1U << 4)) != 0U;
	update_feedback(actuator, &sample, decode_mode(status >> 6), &faults, 0U,
					0U);
	if (updated_actuator != NULL) {
		*updated_actuator = actuator;
	}
	return RS01_RX_EVENT_FEEDBACK;
}

/* 解析 MIT 8 字节手册故障响应及旧固件观察到的 5 字节兼容响应。 */
static rs01_rx_event_t process_mit_fault(const twai_message_t *frame,
										 exo_actuator_id_t *updated_actuator) {
	exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
	const bool source_known = frame->data_length_code > 0U &&
							  rs01_find_actuator(frame->data[0], &actuator);
	if ((frame->data_length_code != RS01_MIT_MANUAL_FAULT_RESPONSE_LENGTH &&
		 frame->data_length_code != RS01_MIT_LEGACY_FAULT_RESPONSE_LENGTH) ||
		!source_known) {
		return invalid_event(actuator, updated_actuator);
	}
	const uint32_t fault = read_u32_le(&frame->data[1]);
	update_fault(actuator, fault, 0U);
	set_mit_fault_response_pending(actuator, false);
	if (updated_actuator != NULL) {
		*updated_actuator = actuator;
	}
	ESP_LOGW(TAG, "actuator %s MIT fault=0x%08lX", actuator_name(actuator),
			 (unsigned long)fault);
	return RS01_RX_EVENT_FAULT;
}

/* 解析 MIT 操作号 3/4 的参数响应；同样保留 raw/uint/float 三种观察方式。 */
static rs01_rx_event_t
process_mit_parameter(const twai_message_t *frame, exo_actuator_id_t actuator,
					  uint8_t operation, exo_actuator_id_t *updated_actuator) {
	if (frame->data_length_code != RS01_FRAME_DATA_LENGTH) {
		return invalid_event(actuator, updated_actuator);
	}
	const uint16_t index =
		(uint16_t)(frame->data[0] | ((uint16_t)frame->data[1] << 8));
	const uint32_t raw = read_u32_le(&frame->data[4]);
	float value;
	memcpy(&value, &raw, sizeof(value));
	ESP_LOGI(TAG,
			 "actuator %s MIT parameter %s index=0x%04X "
			 "raw=0x%08lX uint=%lu float=%g",
			 actuator_name(actuator), operation == 3U ? "read" : "write", index,
			 (unsigned long)raw, (unsigned long)raw, (double)value);
	if (updated_actuator != NULL) {
		*updated_actuator = actuator;
	}
	return RS01_RX_EVENT_PARAMETER;
}

/*
 * MIT 标准帧总分派：帧 ID 等于主站 ID 时处理反馈/故障；ID 高位操作号为 3/4 时
 * 处理参数响应。故障查询窗口和三个保留零字节共同用于避免误吞主动反馈。
 */
static rs01_rx_event_t
process_standard_frame(const twai_message_t *frame,
					   exo_actuator_id_t *updated_actuator) {
	if (frame->identifier == g_exo_hardware_config.twai.master_id) {
		if (frame->data_length_code == RS01_FRAME_DATA_LENGTH) {
			exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
			if (rs01_find_actuator(frame->data[0], &actuator) &&
				mit_fault_response_is_pending(actuator) &&
				frame->data[5] == 0U && frame->data[6] == 0U &&
				frame->data[7] == 0U) {
				/*
				 * 手册故障响应与普通反馈共用主站 ID
				 * 和源节点字节。只有在查询窗口
				 * 有效且三个保留字节为零时才判为故障，避免误吞穿插的主动反馈。
				 */
				return process_mit_fault(frame, updated_actuator);
			}
			return process_mit_feedback(frame, updated_actuator);
		}
		if (frame->data_length_code == RS01_MIT_LEGACY_FAULT_RESPONSE_LENGTH) {
			/* 保留旧驱动实测过的 5 字节故障响应兼容路径。 */
			return process_mit_fault(frame, updated_actuator);
		}
		exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
		if (frame->data_length_code > 0U) {
			(void)rs01_find_actuator(frame->data[0], &actuator);
		}
		return invalid_event(actuator, updated_actuator);
	}

	const uint8_t operation = (uint8_t)((frame->identifier >> 8) & 0x07U);
	if (operation == 3U || operation == 4U) {
		exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
		if (!rs01_find_actuator((uint8_t)frame->identifier, &actuator)) {
			return RS01_RX_EVENT_INVALID;
		}
		return process_mit_parameter(frame, actuator, operation,
									 updated_actuator);
	}
	return RS01_RX_EVENT_NONE;
}

/*
 * 唯一公开接收入口。先拒绝 RTR、超长帧，再依据 EXTD 标志选择私有或 MIT 解析器。
 * updated_actuator 进入时先置 COUNT；只有能可靠识别来源时才写回 A/B，绝不按帧
 * 到达顺序或节点 ID 数值猜测左右腿。
 */
rs01_rx_event_t rs01_process_can_frame(const twai_message_t *frame,
									   exo_actuator_id_t *updated_actuator) {
	if (updated_actuator != NULL) {
		*updated_actuator = EXO_ACTUATOR_COUNT;
	}
	if (frame == NULL) {
		return RS01_RX_EVENT_INVALID;
	}

	if ((frame->flags & TWAI_MSG_FLAG_RTR) != 0U ||
		frame->data_length_code > RS01_FRAME_DATA_LENGTH) {
		exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
		if ((frame->flags & TWAI_MSG_FLAG_EXTD) != 0U) {
			(void)rs01_find_actuator(
				(uint8_t)extended_id_data(frame->identifier), &actuator);
		} else if (frame->identifier == g_exo_hardware_config.twai.master_id &&
				   frame->data_length_code > 0U) {
			(void)rs01_find_actuator(frame->data[0], &actuator);
		} else {
			(void)rs01_find_actuator((uint8_t)frame->identifier, &actuator);
		}
		return invalid_event(actuator, updated_actuator);
	}

	if ((frame->flags & TWAI_MSG_FLAG_EXTD) != 0U) {
		return process_private_frame(frame, updated_actuator);
	}
	return process_standard_frame(frame, updated_actuator);
}
