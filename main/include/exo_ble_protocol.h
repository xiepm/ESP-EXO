#ifndef EXO_BLE_PROTOCOL_H
#define EXO_BLE_PROTOCOL_H

/*
 * ESP-EXO 产品 BLE v1 的纯线协议定义。
 *
 * 本文件只回答“手机和 ESP32 之间的字节是什么”，不包含扫描、连接、GATT、
 * FreeRTOS、CAN 或电机控制。手机把一帧写入 Command characteristic；固件处理后，
 * 从 Response characteristic 发出一帧 Notify。Notify 是设备主动推送，手机必须先
 * 完成订阅，否则 Command Write 会被传输层拒绝。
 *
 * 一次 GATT Write 必须正好携带一帧，不能分片拼接。请求和响应用 request_id
 * 关联： 同一连接会话内，相同 request_id
 * 与相同完整请求只返回缓存响应而不重复执行； 相同 request_id
 * 搭配不同请求返回冲突。断开并重连后进入新会话，旧缓存不参与
 * 新会话判断。该会话规则由 exo_ble.c 实现，request_id 本身只是 1-byte 线字段。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 产品 BLE v1 线格式（每帧由一次完整 GATT 操作承载）：
 *
 *   byte 0  protocol version
 *   byte 1  opcode
 *   byte 2  request id (1..255 for phone requests)
 *   byte 3  payload length
 *   byte 4  payload
 *
 * 每个 v1 帧都能放入 MTU 协商前可用的 20-byte ATT 载荷。此文件只定义线格式，
 * 禁止直接 memcpy 固件结构体，以免受对齐、端序或编译器布局影响。
 */
#define EXO_BLE_PROTOCOL_VERSION 1U
#define EXO_BLE_PROTOCOL_HEADER_SIZE 4U
#define EXO_BLE_PROTOCOL_MAX_PAYLOAD_SIZE 16U
#define EXO_BLE_PROTOCOL_MAX_FRAME_SIZE 20U
#define EXO_BLE_PROTOCOL_RESPONSE_BIT 0x80U

/*
 * 手机可发送的操作码。GET_* 只读；START/STOP 向整机状态机提交异步请求；
 * CAPTURE_ZERO/CLEAR_ZERO 只处理指定人体侧的 RAM 软件零位，不改电机机械零位。
 */
typedef enum {
	EXO_BLE_OPCODE_GET_INFO = 0x01,
	EXO_BLE_OPCODE_GET_SYSTEM_STATUS = 0x02,
	EXO_BLE_OPCODE_GET_JOINT_STATUS = 0x03,
	EXO_BLE_OPCODE_GET_CONTROL_CONFIG = 0x04,
	EXO_BLE_OPCODE_GET_JOINT_COMMAND = 0x05,
	EXO_BLE_OPCODE_GET_GAIT_STATUS = 0x06,
	EXO_BLE_OPCODE_GET_ASSIST_CONFIG = 0x07,
	EXO_BLE_OPCODE_GET_TELEMETRY_STATUS = 0x08,
	/* 只读：实际安装映射、连续电机角和 RAM 零位，不发送 CAN。 */
	EXO_BLE_OPCODE_GET_CALIBRATION_STATUS = 0x09,
	EXO_BLE_OPCODE_GET_ASSIST_TORQUE_LIMIT = 0x0A,
	EXO_BLE_OPCODE_GET_ANGLE_ASSIST_CONFIG = 0x0B,
	EXO_BLE_OPCODE_START = 0x10,
	EXO_BLE_OPCODE_STOP = 0x11,
	EXO_BLE_OPCODE_CAPTURE_ZERO = 0x12,
	EXO_BLE_OPCODE_CLEAR_ZERO = 0x13,
	EXO_BLE_OPCODE_SET_CONTROL_CONFIG = 0x14,
	EXO_BLE_OPCODE_SET_JOINT_COMMAND = 0x15,
	EXO_BLE_OPCODE_SET_ASSIST_CONFIG = 0x16,
	EXO_BLE_OPCODE_START_TELEMETRY = 0x17,
	EXO_BLE_OPCODE_STOP_TELEMETRY = 0x18,
	/* 原8字节助力配置 + 最大附加力矩u16 mN·m，一次原子写入。 */
	EXO_BLE_OPCODE_SET_ASSIST_CONFIG_WITH_LIMIT = 0x19,
	EXO_BLE_OPCODE_SET_ANGLE_ASSIST_CONFIG = 0x1D,
	/* side/action/token：人体手动前抬腿示教方向，不发送CAN运动命令。 */
	EXO_BLE_OPCODE_TEACH_DIRECTION = 0x1A,
	EXO_BLE_OPCODE_GET_CAN_STATUS = 0x1B,
	EXO_BLE_OPCODE_RESTORE_FEEDBACK = 0x1C,
    EXO_BLE_OPCODE_GET_SAVED_ZERO = 0x1E, /* side，候选和沿用预览，只读。 */
    EXO_BLE_OPCODE_SAVED_ZERO_ACTION = 0x1F, /* action/confirmed，双侧原子操作；0.10.11新增action=3采集双零位。 */
    EXO_BLE_OPCODE_GET_PULSE_TIMING = 0x20,
    EXO_BLE_OPCODE_SET_PULSE_TIMING = 0x21,
    EXO_BLE_OPCODE_GET_PULSE_WINDOW = 0x22,
    EXO_BLE_OPCODE_SET_PULSE_WINDOW = 0x23,
    EXO_BLE_OPCODE_GET_PULSE_STATUS = 0x24,
} exo_ble_opcode_t;

/* 响应操作码 = 请求操作码按位或 0x80，手机据此识别响应类型。 */
typedef enum {
	EXO_BLE_RESPONSE_SAVED_ZERO = 0x1E | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_INFO =
		EXO_BLE_OPCODE_GET_INFO | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_SYSTEM_STATUS =
		EXO_BLE_OPCODE_GET_SYSTEM_STATUS | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_JOINT_STATUS =
		EXO_BLE_OPCODE_GET_JOINT_STATUS | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_CONTROL_CONFIG =
		EXO_BLE_OPCODE_GET_CONTROL_CONFIG | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_JOINT_COMMAND =
		EXO_BLE_OPCODE_GET_JOINT_COMMAND | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_GAIT_STATUS =
		EXO_BLE_OPCODE_GET_GAIT_STATUS | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_ASSIST_CONFIG =
		EXO_BLE_OPCODE_GET_ASSIST_CONFIG | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_TELEMETRY_STATUS =
		EXO_BLE_OPCODE_GET_TELEMETRY_STATUS | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_CALIBRATION_STATUS =
		EXO_BLE_OPCODE_GET_CALIBRATION_STATUS | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_ASSIST_TORQUE_LIMIT =
		EXO_BLE_OPCODE_GET_ASSIST_TORQUE_LIMIT | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_START_ACK =
		EXO_BLE_OPCODE_START | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_STOP_ACK =
		EXO_BLE_OPCODE_STOP | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_CAPTURE_ZERO_ACK =
		EXO_BLE_OPCODE_CAPTURE_ZERO | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_CLEAR_ZERO_ACK =
		EXO_BLE_OPCODE_CLEAR_ZERO | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_SET_CONTROL_CONFIG_ACK =
		EXO_BLE_OPCODE_SET_CONTROL_CONFIG | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_SET_JOINT_COMMAND_ACK =
		EXO_BLE_OPCODE_SET_JOINT_COMMAND | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_SET_ASSIST_CONFIG_ACK =
		EXO_BLE_OPCODE_SET_ASSIST_CONFIG | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_START_TELEMETRY_ACK =
		EXO_BLE_OPCODE_START_TELEMETRY | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_STOP_TELEMETRY_ACK =
		EXO_BLE_OPCODE_STOP_TELEMETRY | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_SET_ASSIST_CONFIG_WITH_LIMIT_ACK =
		EXO_BLE_OPCODE_SET_ASSIST_CONFIG_WITH_LIMIT | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_TEACH_DIRECTION_ACK =
		EXO_BLE_OPCODE_TEACH_DIRECTION | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_CAN_STATUS = EXO_BLE_OPCODE_GET_CAN_STATUS | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_RESTORE_FEEDBACK_ACK = EXO_BLE_OPCODE_RESTORE_FEEDBACK | EXO_BLE_PROTOCOL_RESPONSE_BIT,
} exo_ble_response_opcode_t;

/* 稳定的协议结果码，不是 esp_err_t，也不能把内部错误值直接发给手机。 */
typedef enum {
	EXO_BLE_RESULT_OK = 0x00,
	EXO_BLE_RESULT_ACCEPTED = 0x01,
	EXO_BLE_RESULT_ALREADY_SATISFIED = 0x02,
	EXO_BLE_RESULT_ALREADY_PENDING = 0x03,

	EXO_BLE_RESULT_REJECTED_NOT_READY = 0x10,
	EXO_BLE_RESULT_REJECTED_BUSY = 0x11,
	EXO_BLE_RESULT_REJECTED_ERROR = 0x12,
	EXO_BLE_RESULT_INTERNAL_ERROR = 0x13,

	EXO_BLE_RESULT_UNSUPPORTED_VERSION = 0x20,
	EXO_BLE_RESULT_INVALID_LENGTH = 0x21,
	EXO_BLE_RESULT_UNKNOWN_OPCODE = 0x22,
	EXO_BLE_RESULT_INVALID_REQUEST_ID = 0x23,
	EXO_BLE_RESULT_INVALID_PAYLOAD = 0x24,
	EXO_BLE_RESULT_REQUEST_ID_CONFLICT = 0x25,

	/* Used only in status snapshots before any request has been received. */
	EXO_BLE_RESULT_NONE = 0x7F,
} exo_ble_wire_result_t;

/* 固定的整机状态线码；不要把内部 C enum 值直接发送给手机。 */
typedef enum {
	EXO_BLE_STATE_INIT = 0,
	EXO_BLE_STATE_IDLE = 1,
	EXO_BLE_STATE_STARTING = 2,
	EXO_BLE_STATE_RUNNING = 3,
	EXO_BLE_STATE_STOPPING = 4,
	EXO_BLE_STATE_ERROR = 5,
	EXO_BLE_STATE_UNKNOWN = 0xFF,
} exo_ble_wire_state_t;

/* START/STOP 异步执行器当前步骤，以及最近失败步骤使用的稳定线码。 */
typedef enum {
	EXO_BLE_STEP_NONE = 0,
	EXO_BLE_STEP_CHECK_PRECONDITIONS = 1,
	EXO_BLE_STEP_DISABLE_MOTORS = 2,
	EXO_BLE_STEP_SET_MOTION_MODE = 3,
	EXO_BLE_STEP_ENABLE_MOTORS = 4,
	EXO_BLE_STEP_SEND_ZERO_OUTPUT = 5,
	EXO_BLE_STEP_WAIT_FRESH_FEEDBACK = 6,
	EXO_BLE_STEP_ARM_CONTROLLER = 7,
	EXO_BLE_STEP_DISARM_CONTROLLER = 8,
	EXO_BLE_STEP_RELEASE_OUTPUT = 9,
	EXO_BLE_STEP_VERIFY_DISABLED = 10,
	EXO_BLE_STEP_UNKNOWN = 0xFF,
} exo_ble_wire_step_t;

/* START 不能执行时的稳定原因码，供手机显示具体前置条件。 */
typedef enum {
	EXO_BLE_BLOCK_NONE = 0,
	EXO_BLE_BLOCK_INITIALIZING = 1,
	EXO_BLE_BLOCK_SYSTEM_NOT_IDLE = 2,
	EXO_BLE_BLOCK_OPERATION_BUSY = 3,
	EXO_BLE_BLOCK_CONTROL_TASK_NOT_READY = 4,
	EXO_BLE_BLOCK_TWAI_NOT_RUNNING = 5,
	EXO_BLE_BLOCK_LEFT_MAPPING_INVALID = 6,
	EXO_BLE_BLOCK_RIGHT_MAPPING_INVALID = 7,
	EXO_BLE_BLOCK_LEFT_DIRECTION_INVALID = 8,
	EXO_BLE_BLOCK_RIGHT_DIRECTION_INVALID = 9,
	EXO_BLE_BLOCK_LEFT_ZERO_INVALID = 10,
	EXO_BLE_BLOCK_RIGHT_ZERO_INVALID = 11,
	EXO_BLE_BLOCK_LEFT_FEEDBACK_INVALID = 12,
	EXO_BLE_BLOCK_RIGHT_FEEDBACK_INVALID = 13,
	EXO_BLE_BLOCK_LEFT_OFFLINE = 14,
	EXO_BLE_BLOCK_RIGHT_OFFLINE = 15,
	EXO_BLE_BLOCK_LEFT_MOTOR_FAULT = 16,
	EXO_BLE_BLOCK_RIGHT_MOTOR_FAULT = 17,
	EXO_BLE_BLOCK_SYSTEM_ERROR = 18,
	EXO_BLE_BLOCK_RUN_CONTRACT_INVALID = 19,
	EXO_BLE_BLOCK_UNKNOWN = 0xFF,
} exo_ble_wire_block_reason_t;

/* 人体侧别线码固定为 LEFT=0、RIGHT=1，与执行器 A/B 和 CAN ID 无关。 */
typedef enum {
	EXO_BLE_SIDE_LEFT = 0,
	EXO_BLE_SIDE_RIGHT = 1,
	EXO_BLE_SIDE_UNKNOWN = 0xFF,
} exo_ble_wire_side_t;

/* 人体关节正方向相对电机正方向的线码。 */
typedef enum {
	EXO_BLE_DIRECTION_UNKNOWN = 0,
	EXO_BLE_DIRECTION_POSITIVE = 1,
	EXO_BLE_DIRECTION_NEGATIVE = 2,
} exo_ble_wire_direction_t;

/*
 * 阶段 8 的产品控制模式线码。NONE 只用于 GET_CONTROL_CONFIG 回报“当前没有
 * active 运行配置”；手机不得把 NONE 写入 selected_mode。
 */
typedef enum {
	EXO_BLE_CONTROL_MODE_NONE = 0,
	EXO_BLE_CONTROL_MODE_HOLD = 1,
	EXO_BLE_CONTROL_MODE_POSITION = 2,
	EXO_BLE_CONTROL_MODE_VELOCITY = 3,
	EXO_BLE_CONTROL_MODE_TORQUE = 4,
	EXO_BLE_CONTROL_MODE_DAMPING = 5,
	EXO_BLE_CONTROL_MODE_IMPEDANCE = 6,
} exo_ble_wire_control_mode_t;

/*
 * 参考源与控制模式分离。线码 2 永久保留给隔离的工程交替激励；设备只有声明
 * ENGINEERING_GAIT_STIMULUS 能力后才会在业务层接受它。
 */
typedef enum {
	EXO_BLE_REFERENCE_CONSTANT = 0,
	EXO_BLE_REFERENCE_SINE = 1,
	EXO_BLE_REFERENCE_ALTERNATING_SINE_TEST = 2,
} exo_ble_wire_reference_t;

/* GET_CONTROL_CONFIG 的第 5 字节用于区分正常结构化响应与 4-byte 通用 ACK。 */
enum {
	EXO_BLE_CONTROL_CONFIG_FLAG_SELECTED_VALID = 1U << 0,
	EXO_BLE_CONTROL_CONFIG_FLAG_ACTIVE_VALID = 1U << 1,
	EXO_BLE_CONTROL_CONFIG_FLAGS_MASK = 0x03U,
};

/* SYSTEM_STATUS/ACK 中 system_flags 的逐位定义。 */
enum {
	EXO_BLE_SYSTEM_FLAG_INITIALIZED = 1U << 0,
	EXO_BLE_SYSTEM_FLAG_READY_TO_START = 1U << 1,
	EXO_BLE_SYSTEM_FLAG_RUN_REQUESTED = 1U << 2,
	EXO_BLE_SYSTEM_FLAG_OUTPUT_ALLOWED = 1U << 3,
	EXO_BLE_SYSTEM_FLAG_MAINTENANCE_ACTIVE = 1U << 4,
	EXO_BLE_SYSTEM_FLAG_EXECUTOR_ALIVE = 1U << 5,
	EXO_BLE_SYSTEM_FLAGS_MASK = 0x3FU,
};

/* JOINT_STATUS 中 joint_flags 的逐位定义。 */
enum {
	EXO_BLE_JOINT_FLAG_MAPPING_VALID = 1U << 0,
	EXO_BLE_JOINT_FLAG_DIRECTION_VALID = 1U << 1,
	EXO_BLE_JOINT_FLAG_ZERO_VALID = 1U << 2,
	EXO_BLE_JOINT_FLAG_FEEDBACK_VALID = 1U << 3,
	EXO_BLE_JOINT_FLAG_ONLINE = 1U << 4,
	EXO_BLE_JOINT_FLAG_JOINT_VALID = 1U << 5,
	EXO_BLE_JOINT_FLAGS_MASK = 0x3FU,
};

/* GET_INFO 返回的能力位；手机只能使用设备声明支持的功能。 */
enum {
	EXO_BLE_CAPABILITY_SYSTEM_CONTROL = 1U << 0,
	EXO_BLE_CAPABILITY_JOINT_CALIBRATION = 1U << 1,
	EXO_BLE_CAPABILITY_ON_DEMAND_STATUS = 1U << 2,
	EXO_BLE_CAPABILITY_CONTROL_CONFIGURATION = 1U << 3,
	EXO_BLE_CAPABILITY_GAIT_STATUS = 1U << 4,
	EXO_BLE_CAPABILITY_ASSIST_CONFIGURATION = 1U << 5,
	EXO_BLE_CAPABILITY_TELEMETRY_STREAM = 1U << 6,
	/* 仅 CONFIG_EXO_ENGINEERING_GAIT_STIMULUS=y 的构建会声明 bit 7。 */
	EXO_BLE_CAPABILITY_ENGINEERING_GAIT_STIMULUS = 1U << 7,
	EXO_BLE_CAPABILITY_CALIBRATION_STATUS = 1U << 8,
	EXO_BLE_CAPABILITY_ASSIST_TORQUE_LIMIT = 1U << 9,
	EXO_BLE_CAPABILITY_DIRECTION_TEACHING = 1U << 10,
	EXO_BLE_CAPABILITY_FEEDBACK_RECOVERY = 1U << 11,
	EXO_BLE_CAPABILITY_ANGLE_THRESHOLD_ASSIST = 1U << 12,
    EXO_BLE_CAPABILITY_ANGLE_ASSIST_5NM = 1U << 13,
    EXO_BLE_CAPABILITY_SAVED_WEAR_ZERO = 1U << 14,
    EXO_BLE_CAPABILITY_SWING_EVENT_PULSE = 1U << 15,
	EXO_BLE_CAPABILITIES_MASK = 0xFFFFU,
};

/*
 * 遥测通道线码与 exo_telemetry.h 中的内部掩码保持数值一致，但单独定义，
 * 以免纯字节协议层依赖 FreeRTOS 遥测队列实现。每个样本 Notify 只能携带
 * 一个 single-bit channel；同时请求多通道时会产生多帧共用 sample_sequence。
 */
enum {
	EXO_BLE_TELEMETRY_CHANNEL_JOINT_POSITION = 1U << 0,
	EXO_BLE_TELEMETRY_CHANNEL_JOINT_VELOCITY = 1U << 1,
	EXO_BLE_TELEMETRY_CHANNEL_MOTOR_TORQUE = 1U << 2,
	EXO_BLE_TELEMETRY_CHANNEL_GAIT_PHASE = 1U << 3,
	EXO_BLE_TELEMETRY_CHANNEL_ASSIST_SUGGESTION = 1U << 4,
	EXO_BLE_TELEMETRY_CHANNEL_GAIT_CONFIDENCE = 1U << 5,
	EXO_BLE_TELEMETRY_CHANNEL_APPLIED_TORQUE = 1U << 6,
	EXO_BLE_TELEMETRY_CHANNELS_MASK = 0x007FU,
};

#define EXO_BLE_TELEMETRY_SCHEMA_VERSION 1U
#define EXO_BLE_TELEMETRY_FRAME_TYPE_SAMPLE 1U
#define EXO_BLE_TELEMETRY_FRAME_SIZE 20U
#define EXO_BLE_TELEMETRY_SIGNED_INVALID ((uint16_t)0x8000U)
#define EXO_BLE_TELEMETRY_UNSIGNED_INVALID UINT16_MAX

enum {
	EXO_BLE_TELEMETRY_FLAG_LEFT_VALID = 1U << 0,
	EXO_BLE_TELEMETRY_FLAG_RIGHT_VALID = 1U << 1,
	EXO_BLE_TELEMETRY_FLAG_DROPPED = 1U << 2,
	EXO_BLE_TELEMETRY_FLAG_SYSTEM_STATE_SHIFT = 3,
	EXO_BLE_TELEMETRY_FLAG_SYSTEM_STATE_MASK = 0x38U,
	EXO_BLE_TELEMETRY_FLAG_OUTPUT_ALLOWED = 1U << 6,
	EXO_BLE_TELEMETRY_FLAG_REAL_OUTPUT_ENABLED = 1U << 7,
};

/* 阶段9步态状态线码与内部枚举分开定义，后续内部重排不会破坏手机兼容。 */
typedef enum {
	EXO_BLE_GAIT_ESTIMATOR_NONE = 0,
	EXO_BLE_GAIT_ESTIMATOR_HIP_EVENT_TBE = 1,
} exo_ble_wire_gait_estimator_t;

typedef enum {
	EXO_BLE_GAIT_STATE_RESET = 0,
	EXO_BLE_GAIT_STATE_STANDING = 1,
	EXO_BLE_GAIT_STATE_ACQUIRING = 2,
	EXO_BLE_GAIT_STATE_LOCKED = 3,
	EXO_BLE_GAIT_STATE_DEGRADED = 4,
} exo_ble_wire_gait_state_t;

typedef enum {
	EXO_BLE_GAIT_ORIGIN_UNKNOWN = 0,
	EXO_BLE_GAIT_ORIGIN_HIP_FLEXION_PEAK = 1,
	EXO_BLE_GAIT_ORIGIN_HIP_EXTENSION_PEAK = 2,
} exo_ble_wire_gait_origin_t;

enum {
	EXO_BLE_GAIT_FLAG_OVERALL_VALID = 1U << 0,
	EXO_BLE_GAIT_FLAG_LEFT_PHASE_VALID = 1U << 1,
	EXO_BLE_GAIT_FLAG_RIGHT_PHASE_VALID = 1U << 2,
	EXO_BLE_GAIT_FLAG_MOTION_DETECTED = 1U << 3,
	EXO_BLE_GAIT_FLAG_LEFT_PHASE_WRAP = 1U << 4,
	EXO_BLE_GAIT_FLAG_RIGHT_PHASE_WRAP = 1U << 5,
	EXO_BLE_GAIT_FLAGS_MASK = 0x3FU,
};

#define EXO_BLE_GAIT_FIXED_INVALID UINT16_MAX
#define EXO_BLE_GAIT_PHASE_X10000_MAX 9999U
#define EXO_BLE_GAIT_CADENCE_X100_MAX 30000U
#define EXO_BLE_GAIT_CONFIDENCE_X10000_MAX 10000U
#define EXO_BLE_GAIT_QUALITY_FLAGS_MASK 0x7FFFU

/* 高层助力模式、执行状态和策略线码；它们不是阶段8底层控制模式。 */
typedef enum {
	EXO_BLE_ASSIST_MODE_NONE = 0,
	EXO_BLE_ASSIST_MODE_TRANSPARENT = 1,
	EXO_BLE_ASSIST_MODE_ACTIVE_PHASE = 2,
	EXO_BLE_ASSIST_MODE_ASSIST_AS_NEEDED = 3,
	EXO_BLE_ASSIST_MODE_ANGLE_THRESHOLD = 4,
} exo_ble_wire_assist_mode_t;

typedef enum {
	EXO_BLE_ASSIST_EXEC_DISABLED = 0,
	EXO_BLE_ASSIST_EXEC_SHADOW = 1,
	EXO_BLE_ASSIST_EXEC_ACTIVE = 2,
} exo_ble_wire_assist_execution_t;

typedef enum {
	EXO_BLE_ASSIST_STRATEGY_NONE = 0,
	EXO_BLE_ASSIST_STRATEGY_TRANSPARENT = 1,
	EXO_BLE_ASSIST_STRATEGY_PHASE_PROFILE = 2,
	EXO_BLE_ASSIST_STRATEGY_AAN_RESIDUAL = 3,
	EXO_BLE_ASSIST_STRATEGY_ANGLE_THRESHOLD = 4,
} exo_ble_wire_assist_strategy_t;

typedef enum {
	EXO_BLE_ASSIST_UNAVAILABLE_NONE = 0,
	EXO_BLE_ASSIST_UNAVAILABLE_NOT_INITIALIZED = 1,
	EXO_BLE_ASSIST_UNAVAILABLE_CONFIG_INVALID = 2,
	EXO_BLE_ASSIST_UNAVAILABLE_ACTIVE_NOT_ALLOWED = 3,
	EXO_BLE_ASSIST_UNAVAILABLE_STRATEGY_NOT_IMPLEMENTED = 4,
	EXO_BLE_ASSIST_UNAVAILABLE_REQUIRED_SENSOR_MISSING = 5,
	EXO_BLE_ASSIST_UNAVAILABLE_NOT_ARMED = 6,
	EXO_BLE_ASSIST_UNAVAILABLE_REQUIRED_GAIT_INVALID = 7,
	EXO_BLE_ASSIST_UNAVAILABLE_WAITING_NEW_GAIT_EVENTS = 8,
} exo_ble_wire_assist_unavailable_t;

enum {
	EXO_BLE_ASSIST_FLAG_SELECTED_VALID = 1U << 0,
	EXO_BLE_ASSIST_FLAG_ACTIVE_VALID = 1U << 1,
	EXO_BLE_ASSIST_FLAG_CONFIG_LOCKED = 1U << 2,
	EXO_BLE_ASSIST_FLAG_STRATEGY_READY = 1U << 3,
	EXO_BLE_ASSIST_FLAG_SENSORS_READY = 1U << 4,
	EXO_BLE_ASSIST_FLAG_SUGGESTION_VALID = 1U << 5,
	EXO_BLE_ASSIST_FLAG_REAL_OUTPUT_ENABLED = 1U << 6,
	EXO_BLE_ASSIST_FLAGS_MASK = 0x7FU,
};

/*
 * 五元命令在线上使用整数定点数，以便单侧命令连同 4-byte 帧头仍小于 20 bytes。
 * 范围采用 RS01 Motion 协议公开边界；产品层可以在业务处理时继续施加更窄限制。
 */
#define EXO_BLE_JOINT_POSITION_MRAD_MIN (-12570)
#define EXO_BLE_JOINT_POSITION_MRAD_MAX 12570
#define EXO_BLE_JOINT_VELOCITY_MRAD_S_MIN (-44000)
#define EXO_BLE_JOINT_VELOCITY_MRAD_S_MAX 44000
#define EXO_BLE_JOINT_TORQUE_MNM_MIN (-17000)
#define EXO_BLE_JOINT_TORQUE_MNM_MAX 17000
#define EXO_BLE_JOINT_KP_X100_MAX 50000U
#define EXO_BLE_JOINT_KD_X1000_MAX 5000U

/* GET/SET_CONTROL_CONFIG 使用的纯线协议 DTO。 */
typedef struct {
	exo_ble_wire_control_mode_t selected_mode;
	exo_ble_wire_control_mode_t active_mode;
	exo_ble_wire_reference_t selected_reference;
	exo_ble_wire_reference_t active_reference;
	uint8_t config_flags;
} exo_ble_wire_control_config_t;

/* GET/SET_JOINT_COMMAND 使用的 15-byte payload 对应 DTO。 */
typedef struct {
	exo_ble_wire_side_t side;
	int32_t position_mrad;
	int32_t velocity_mrad_s;
	int16_t torque_mnm;
	uint16_t kp_x100;
	uint16_t kd_x1000;
} exo_ble_wire_joint_command_t;

/* GET_GAIT_STATUS固定16-byte payload中的纯线DTO。 */
typedef struct {
	exo_ble_wire_gait_estimator_t estimator_id;
	exo_ble_wire_gait_state_t estimator_state;
	uint8_t gait_flags;
	exo_ble_wire_gait_origin_t phase_origin;
	uint16_t quality_flags;
	uint16_t left_phase_x10000;
	uint16_t right_phase_x10000;
	uint16_t cadence_x100;
	uint16_t confidence_x10000;
	uint16_t estimate_age_ms;
} exo_ble_wire_gait_status_t;

/* GET_ASSIST_CONFIG固定16-byte payload中的纯线DTO。 */
typedef struct {
	uint8_t schema_version;
	exo_ble_wire_assist_mode_t selected_mode;
	exo_ble_wire_assist_mode_t active_mode;
	/* 线上共用1 byte：低4位requested，高4位effective。 */
	exo_ble_wire_assist_execution_t requested_execution_state;
	exo_ble_wire_assist_execution_t execution_state;
	exo_ble_wire_assist_strategy_t strategy_id;
	uint8_t strategy_version;
	uint8_t profile_id;
	uint8_t left_level_percent;
	uint8_t right_level_percent;
	uint8_t assist_flags;
	exo_ble_wire_assist_unavailable_t unavailable_reason;
	uint8_t config_generation;
	uint16_t required_sensor_mask;
	uint16_t available_sensor_mask;
} exo_ble_wire_assist_config_t;

/* SET_ASSIST_CONFIG使用8-byte payload；v1 reserved_flags 必须为0。 */
typedef struct {
	uint8_t schema_version;
	exo_ble_wire_assist_mode_t selected_mode;
	exo_ble_wire_assist_execution_t requested_execution_state;
	exo_ble_wire_assist_strategy_t strategy_id;
	uint8_t profile_id;
	uint8_t left_level_percent;
	uint8_t right_level_percent;
	uint8_t reserved_flags;
} exo_ble_wire_assist_request_t;

/* GET_ASSIST_TORQUE_LIMIT: schema=1、flags(选择/活动有效)、三个u16LE，8字节。
 * 上限是附加助力的绝对值，左右各自按等级百分比缩放，不含基础阻尼。
 */
#define EXO_BLE_ASSIST_TORQUE_HARD_MAX_MNM 500U
#define EXO_BLE_ANGLE_TORQUE_HARD_MAX_MNM 15000U
#define EXO_BLE_PULSE_FIXED_MAX_MS 3000U
#define EXO_BLE_PULSE_SCALED_MAX_MS 2000U
#define EXO_BLE_ANGLE_MIN_CDEG 100U
#define EXO_BLE_ANGLE_MAX_CDEG 6000U
#define EXO_BLE_ANGLE_HYSTERESIS_CDEG 100U
#define EXO_BLE_ANGLE_RAMP_MS 200U
/* 0x1D: 原8字节配置 + u16力矩mN·m + u16人体角0.01度，原子更新。
 * 0x0B读回12字节: schema, flags, selected/active阈值, 回差, 渐入ms,
 * generation低8位, reserved=0。旧帧和旧opcode保持原长度。
 */
typedef struct {
    uint8_t flags, generation;
    uint16_t selected_cdeg, active_cdeg;
    bool pulse_model; /* schema2：旧回差/渐入字段为0，改读脉冲配置。 */
} exo_ble_wire_angle_assist_t;
bool exo_ble_protocol_encode_angle_assist(uint8_t request_id,
    const exo_ble_wire_angle_assist_t *config, uint8_t *frame,
    size_t capacity, size_t *length);
typedef struct {
	uint8_t flags;
	uint16_t selected_maximum_mnm;
	uint16_t active_maximum_mnm;
	uint16_t hard_maximum_mnm;
} exo_ble_wire_assist_torque_limit_t;

/* START_TELEMETRY 的 3-byte payload；rate 仅允许 5/10/20 Hz，mask 必须非空。 */
typedef struct {
	uint8_t rate_hz;
	uint16_t channel_mask;
} exo_ble_wire_telemetry_start_t;

/* GET_TELEMETRY_STATUS 的 16-byte payload DTO；计数器在每次成功 START 时清零。
 */
typedef struct {
	bool streaming;
	uint8_t rate_hz;
	uint16_t channel_mask;
	uint16_t stream_sequence;
	uint32_t notify_success_count;
	uint32_t dropped_sample_count;
	uint8_t queued_samples;
	uint8_t queue_capacity;
} exo_ble_wire_telemetry_status_t;

/*
 * 独立 Telemetry characteristic 上的固定 20-byte 样本帧 DTO。left/right_value
 * 只是 16-bit 位模式：有符号通道按 i16 解释，相位/置信度按 u16 解释。
 */
typedef struct {
	uint8_t channel;
	uint8_t flags;
	uint16_t stream_sequence;
	uint32_t sample_sequence;
	uint32_t timestamp_ms;
	uint16_t activation_epoch_lsb;
	uint16_t left_value;
	uint16_t right_value;
} exo_ble_wire_telemetry_sample_t;

/* 16-byte参数载荷：schema/mode，7个u16；显式字段编码，不传内部结构体。 */
typedef struct {
    uint8_t schema_version, duration_mode;
    uint16_t peak_mnm, fixed_ms, ratio_permille, minimum_ms, maximum_ms;
    uint16_t period_minimum_ms, period_maximum_ms;
} exo_ble_wire_pulse_timing_t;
typedef struct { int16_t reset_cdeg; uint16_t maximum_cdeg; } exo_ble_wire_pulse_window_t;
typedef struct {
    uint8_t side, state, flags, exit_reason;
    uint16_t event_count, pulse_count, duration_ms, period_ms, peak_mnm, progress_x10000;
} exo_ble_wire_pulse_status_t;
bool exo_ble_protocol_encode_pulse_timing(uint8_t request_id,
    const exo_ble_wire_pulse_timing_t *timing,uint8_t *frame,size_t capacity,size_t *length);
bool exo_ble_protocol_encode_pulse_window(uint8_t request_id,
    const exo_ble_wire_pulse_window_t *window,uint8_t *frame,size_t capacity,size_t *length);
bool exo_ble_protocol_encode_pulse_status(uint8_t request_id,
    const exo_ble_wire_pulse_status_t *status,uint8_t *frame,size_t capacity,size_t *length);

/*
 * 解码后的请求 DTO。简单请求只读取 side；阶段 8 SET 请求分别填入
 * control_config 或 joint_command。它不是线上的 C 结构体布局，协议实现会从
 * frame 中逐字节填充，业务层必须根据 opcode 只读取对应成员。
 */
typedef struct {
	uint8_t version;
	uint8_t opcode;
	uint8_t request_id;
	uint8_t payload_len;
	exo_ble_wire_side_t side;
	exo_ble_wire_control_config_t control_config;
	uint8_t direction_teach_action;
	uint8_t direction_teach_token;
    uint8_t saved_zero_action;
	exo_ble_wire_joint_command_t joint_command;
	exo_ble_wire_assist_request_t assist_config;
	uint16_t maximum_assist_torque_mnm;
	uint16_t angle_threshold_cdeg;
	exo_ble_wire_pulse_timing_t pulse_timing;
	exo_ble_wire_pulse_window_t pulse_window;
	exo_ble_wire_telemetry_start_t telemetry_start;
} exo_ble_request_t;

/* 16字节payload：side/actuator/CAN/direction/flags/reason/reserved u16、
 * 候选周期电机零位i32毫度、当前沿用人体角预览i32毫度；缺失值INT32_MIN。
 * flags: stored/binding_matches/preview_valid/reuse_ready，预览不写RAM。 */
typedef struct {
    exo_ble_wire_side_t side;
    uint8_t actuator, can_id;
    exo_ble_wire_direction_t direction;
    uint8_t flags, reason;
    int32_t saved_zero_mdeg, preview_joint_mdeg;
} exo_ble_saved_zero_status_t;
bool exo_ble_protocol_encode_saved_zero(uint8_t request_id,
    const exo_ble_saved_zero_status_t *status, uint8_t *frame,
    size_t frame_capacity, size_t *frame_len);

/*
 * ACK payload（4 bytes）：result、state、system_flags、detail。
 * START 返回 ACCEPTED 只表示异步请求已接收，手机仍需轮询 SYSTEM_STATUS，
 * 直到 RUNNING、IDLE 或 ERROR。
 */
typedef struct {
	exo_ble_wire_result_t result;
	exo_ble_wire_state_t state;
	uint8_t system_flags;
	uint8_t detail;
} exo_ble_ack_t;

/*
 * INFO payload（12 bytes）：协议最小/最大版本、固件 major/minor/patch、
 * capabilities u16 LE、最大帧长和 boot_id u32 LE。boot_id 每次设备启动变化，
 * 手机可用它判断连接期间设备是否重启。
 */
typedef struct {
	uint8_t protocol_min_version;
	uint8_t protocol_max_version;
	uint8_t firmware_major;
	uint8_t firmware_minor;
	uint8_t firmware_patch;
	uint16_t capabilities;
	uint8_t max_frame_size;
	uint32_t boot_id;
} exo_ble_info_t;

/*
 * SYSTEM_STATUS payload（10 bytes）：state、active_step、system_flags、
 * start_block_reason、last_request_result、last_failed_step 和
 * transition_count u32 LE。
 */
typedef struct {
	exo_ble_wire_state_t state;
	exo_ble_wire_step_t active_step;
	uint8_t system_flags;
	exo_ble_wire_block_reason_t start_block_reason;
	exo_ble_wire_result_t last_request_result;
	exo_ble_wire_step_t last_failed_step;
	uint32_t transition_count;
} exo_ble_system_status_t;

/*
 * JOINT_STATUS payload（13 bytes）：side、flags、direction、i32 LE 位置 mrad、
 * i32 LE 速度 mrad/s、u16 LE 反馈年龄 ms。
 *
 * 注意：mrad 只是已发布 v1 的定点传输单位；产品对象和手机显示统一用 deg。
 * 手机显示边界必须换算：
 *   position_deg   = position_mrad   * 180 / (pi * 1000)
 *   velocity_deg_s = velocity_mrad_s * 180 / (pi * 1000)
 * 固件不能在版本仍为 v1 时把字段静默改成毫度，否则旧客户端会误解约 57.3 倍。
 */
typedef struct {
	exo_ble_wire_side_t side;
	uint8_t joint_flags;
	exo_ble_wire_direction_t direction;
	int32_t position_mrad;
	int32_t velocity_mrad_s;
	uint16_t feedback_age_ms;
} exo_ble_joint_status_t;

/*
 * 出厂核对/穿戴标定只读快照，payload 16 B（整帧仍为20 B）：
 * side, actuator(A=0/B=1/unknown=255), CAN ID, direction, joint_flags,
 * reserved=0, i32 LE 连续电机角mrad, i32 LE 软件零位mrad, u16 LE反馈年龄ms。
 * 无有效数值使用INT32_MIN，不能用0冒充真实测量。零位是电机坐标偏置，
 * 人体关节角仍由JOINT_STATUS提供；两者不得混用。没有运行期改映射/机械置零接口。
 */
typedef struct {
	exo_ble_wire_side_t side;
	uint8_t actuator;
	uint8_t can_id;
	exo_ble_wire_direction_t direction;
	uint8_t joint_flags;
	int32_t raw_position_mrad;
	int32_t software_zero_mrad;
	uint16_t feedback_age_ms;
} exo_ble_calibration_status_t;

/* CAN诊断固定16字节：state(0停止/1运行/2BUS_OFF/3恢复/4不可读),
 * flags(bit0总线可用于请求/bit1待机允许恢复), TEC/REC u16, bus_errors u32,
 * TX失败/RX丢帧饱和u16, A/B flags(曾有效/在线/请求上报/电机故障/预算耗尽)。
 * 错误总数从启动累计，不能仅凭历史总数判断此刻故障。 */
typedef struct {
    uint8_t state, flags;
    uint16_t tx_error_counter, rx_error_counter;
    uint32_t bus_error_count;
    uint16_t tx_failed_count, rx_missed_count;
    uint8_t motor_flags[2];
} exo_ble_can_status_t;
bool exo_ble_protocol_encode_can_status(uint8_t request_id,
    const exo_ble_can_status_t *status, uint8_t *frame, size_t capacity, size_t *length);

bool exo_ble_protocol_encode_calibration_status(
	uint8_t request_id, const exo_ble_calibration_status_t *status,
	uint8_t *frame, size_t frame_capacity, size_t *frame_len);

/*
 * 解码并验证一帧完整请求。函数先清空 request；即使帧错误，也尽量保留已经收到的
 * header 字节，使服务层能用 opcode/request_id 返回可关联的错误 ACK。
 * 只有返回 EXO_BLE_RESULT_OK 时，request 才可以交给业务分发。
 */
exo_ble_wire_result_t
exo_ble_protocol_decode_request(const uint8_t *frame, size_t frame_len,
								exo_ble_request_t *request);

/*
 * 以下响应编码器都显式逐字节写入 frame，不依赖 C 结构体对齐或主机端序。
 * 参数、线码或容量不合法时返回 false，并把可用的 frame_len 清零。
 */
bool exo_ble_protocol_encode_ack(uint8_t request_opcode, uint8_t request_id,
								 const exo_ble_ack_t *ack, uint8_t *frame,
								 size_t frame_capacity, size_t *frame_len);
bool exo_ble_protocol_encode_info(uint8_t request_id,
								  const exo_ble_info_t *info, uint8_t *frame,
								  size_t frame_capacity, size_t *frame_len);
bool exo_ble_protocol_encode_system_status(
	uint8_t request_id, const exo_ble_system_status_t *status, uint8_t *frame,
	size_t frame_capacity, size_t *frame_len);
bool exo_ble_protocol_encode_joint_status(uint8_t request_id,
										  const exo_ble_joint_status_t *status,
										  uint8_t *frame, size_t frame_capacity,
										  size_t *frame_len);
bool exo_ble_protocol_encode_control_config(
	uint8_t request_id, const exo_ble_wire_control_config_t *config,
	uint8_t *frame, size_t frame_capacity, size_t *frame_len);
bool exo_ble_protocol_encode_joint_command(
	uint8_t request_id, const exo_ble_wire_joint_command_t *command,
	uint8_t *frame, size_t frame_capacity, size_t *frame_len);
bool exo_ble_protocol_encode_gait_status(
	uint8_t request_id, const exo_ble_wire_gait_status_t *status,
	uint8_t *frame, size_t frame_capacity, size_t *frame_len);
bool exo_ble_protocol_encode_assist_config(
	uint8_t request_id, const exo_ble_wire_assist_config_t *config,
	uint8_t *frame, size_t frame_capacity, size_t *frame_len);
bool exo_ble_protocol_encode_assist_torque_limit(
	uint8_t request_id, const exo_ble_wire_assist_torque_limit_t *limit,
	uint8_t *frame, size_t frame_capacity, size_t *frame_len);
bool exo_ble_protocol_encode_telemetry_status(
	uint8_t request_id, const exo_ble_wire_telemetry_status_t *status,
	uint8_t *frame, size_t frame_capacity, size_t *frame_len);
bool exo_ble_protocol_encode_telemetry_sample(
	const exo_ble_wire_telemetry_sample_t *sample, uint8_t *frame,
	size_t frame_capacity, size_t *frame_len);

#ifdef __cplusplus
}
#endif

#endif /* EXO_BLE_PROTOCOL_H */
