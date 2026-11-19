#include "exo_ble_protocol.h"
#include <string.h>
#include <math.h>

/*
 * 产品 BLE v1 的纯字节编解码器。
 *
 * 请求方向：Command characteristic 收到的完整字节数组
 *           -> exo_ble_protocol_decode_request()
 *           -> 经过验证的 exo_ble_request_t
 *
 * 响应方向：服务层准备好的 ACK/INFO/STATUS DTO
 *           -> 对应 exo_ble_protocol_encode_*()
 *           -> Response characteristic Notify 的完整字节数组
 *
 * 本文件不调用 NimBLE、FreeRTOS、状态机或关节对象，也不发送 Notify。它只按
 * v1 合同检查线码、长度和容量，并显式按 little-endian 写入，避免直接序列化
 * C 结构体造成端序、padding 或编译器布局差异。
 */

/* 帧头固定占 4 bytes；下面的 offset 和各响应 payload 大小是唯一布局依据。 */
enum {
	FRAME_VERSION_OFFSET = 0,
	FRAME_OPCODE_OFFSET = 1,
	FRAME_REQUEST_ID_OFFSET = 2,
	FRAME_PAYLOAD_LENGTH_OFFSET = 3,
	FRAME_PAYLOAD_OFFSET = EXO_BLE_PROTOCOL_HEADER_SIZE,

	ACK_PAYLOAD_SIZE = 4,
	INFO_PAYLOAD_SIZE = 12,
	SYSTEM_STATUS_PAYLOAD_SIZE = 10,
	JOINT_STATUS_PAYLOAD_SIZE = 13,
	CALIBRATION_STATUS_PAYLOAD_SIZE = 16,
	CONTROL_CONFIG_PAYLOAD_SIZE = 5,
	JOINT_COMMAND_PAYLOAD_SIZE = 15,
	GAIT_STATUS_PAYLOAD_SIZE = 16,
	ASSIST_CONFIG_PAYLOAD_SIZE = 16,
	ASSIST_REQUEST_PAYLOAD_SIZE = 8,
	TELEMETRY_STATUS_PAYLOAD_SIZE = 16,
	TELEMETRY_START_PAYLOAD_SIZE = 3,
};

/* 编译期保证每一种响应都能放入 MTU=23 时可用的 20-byte ATT 载荷。 */
_Static_assert(EXO_BLE_PROTOCOL_HEADER_SIZE +
					   EXO_BLE_PROTOCOL_MAX_PAYLOAD_SIZE ==
				   EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
			   "BLE protocol frame constants are inconsistent");
_Static_assert(EXO_BLE_PROTOCOL_HEADER_SIZE + INFO_PAYLOAD_SIZE <=
				   EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
			   "INFO response exceeds one default ATT payload");
_Static_assert(EXO_BLE_PROTOCOL_HEADER_SIZE + SYSTEM_STATUS_PAYLOAD_SIZE <=
				   EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
			   "SYSTEM_STATUS response exceeds one default ATT payload");
_Static_assert(EXO_BLE_PROTOCOL_HEADER_SIZE + JOINT_STATUS_PAYLOAD_SIZE <=
				   EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
			   "JOINT_STATUS response exceeds one default ATT payload");
_Static_assert(EXO_BLE_PROTOCOL_HEADER_SIZE + CONTROL_CONFIG_PAYLOAD_SIZE <=
				   EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
			   "CONTROL_CONFIG response exceeds one default ATT payload");
_Static_assert(EXO_BLE_PROTOCOL_HEADER_SIZE + JOINT_COMMAND_PAYLOAD_SIZE <=
				   EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
			   "JOINT_COMMAND response exceeds one default ATT payload");
_Static_assert(EXO_BLE_PROTOCOL_HEADER_SIZE + GAIT_STATUS_PAYLOAD_SIZE <=
				   EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
			   "GAIT_STATUS response exceeds one default ATT payload");
_Static_assert(EXO_BLE_PROTOCOL_HEADER_SIZE + ASSIST_CONFIG_PAYLOAD_SIZE <=
				   EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
			   "ASSIST_CONFIG response exceeds one default ATT payload");
_Static_assert(EXO_BLE_PROTOCOL_HEADER_SIZE + TELEMETRY_STATUS_PAYLOAD_SIZE ==
				   EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
			   "TELEMETRY_STATUS must fill one default ATT payload");
_Static_assert(EXO_BLE_TELEMETRY_FRAME_SIZE <= EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
			   "telemetry sample exceeds one default ATT payload");

/* 把无符号 16-bit 值逐字节写成低字节在前的 little-endian。 */
static void write_u16_le(uint8_t *destination, uint16_t value) {
	destination[0] = (uint8_t)value;
	destination[1] = (uint8_t)(value >> 8);
}

/* 把无符号 32-bit 值逐字节写成 little-endian。 */
static void write_u32_le(uint8_t *destination, uint32_t value) {
	destination[0] = (uint8_t)value;
	destination[1] = (uint8_t)(value >> 8);
	destination[2] = (uint8_t)(value >> 16);
	destination[3] = (uint8_t)(value >> 24);
}

/* 有符号数保持相同二进制位模式，再复用 u32 little-endian 写入。 */
static void write_i32_le(uint8_t *destination, int32_t value) {
	write_u32_le(destination, (uint32_t)value);
}

/* i16 同样保留补码位模式，再按 little-endian 写入。 */
static void write_i16_le(uint8_t *destination, int16_t value) {
	write_u16_le(destination, (uint16_t)value);
}

/* 请求解码不依赖 CPU 端序，所有多字节字段都从字节流显式拼装。 */
static uint16_t read_u16_le(const uint8_t *source) {
	return (uint16_t)source[0] | ((uint16_t)source[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *source) {
	return (uint32_t)source[0] | ((uint32_t)source[1] << 8) |
		   ((uint32_t)source[2] << 16) | ((uint32_t)source[3] << 24);
}

static int16_t read_i16_le(const uint8_t *source) {
	return (int16_t)read_u16_le(source);
}

static int32_t read_i32_le(const uint8_t *source) {
	return (int32_t)read_u32_le(source);
}

/*
 * 给定请求 opcode，返回 v1 规定的精确 payload 长度：旧请求仍保持原来的
 * 0/1 字节；阶段 8 的控制配置为 2 字节、单侧五元命令为 15 字节。未知
 * opcode 返回 false，不能进入业务层。
 */
static bool request_payload_size(uint8_t opcode, uint8_t *payload_size) {
	if (payload_size == NULL) {
		return false;
	}

	switch (opcode) {
	case EXO_BLE_OPCODE_GET_INFO:
    case EXO_BLE_OPCODE_PREPARE_ADAPTIVE_TRIAL:
    case EXO_BLE_OPCODE_CHECK_ADAPTIVE_TRIAL:
	case EXO_BLE_OPCODE_GET_SYSTEM_STATUS:
	case EXO_BLE_OPCODE_GET_CONTROL_CONFIG:
	case EXO_BLE_OPCODE_GET_GAIT_STATUS:
	case EXO_BLE_OPCODE_GET_ASSIST_CONFIG:
	case EXO_BLE_OPCODE_GET_ASSIST_TORQUE_LIMIT:
	case EXO_BLE_OPCODE_GET_ANGLE_ASSIST_CONFIG:
    case EXO_BLE_OPCODE_GET_PULSE_TIMING:
    case EXO_BLE_OPCODE_GET_PULSE_WINDOW:
	case EXO_BLE_OPCODE_GET_CAN_STATUS:
	case EXO_BLE_OPCODE_RESTORE_FEEDBACK:
	case EXO_BLE_OPCODE_GET_TELEMETRY_STATUS:
	case EXO_BLE_OPCODE_START:
	case EXO_BLE_OPCODE_STOP:
	case EXO_BLE_OPCODE_STOP_TELEMETRY:
		*payload_size = 0U;
		return true;
	case EXO_BLE_OPCODE_GET_JOINT_STATUS:
    case EXO_BLE_OPCODE_GET_BOARD_SENSOR:
    case EXO_BLE_OPCODE_GET_DIRECTION_PREVIEW:
    case EXO_BLE_OPCODE_GET_AFO_STATUS:
	case EXO_BLE_OPCODE_GET_CALIBRATION_STATUS:
    case EXO_BLE_OPCODE_GET_SAVED_ZERO:
    case EXO_BLE_OPCODE_GET_PULSE_STATUS:
	case EXO_BLE_OPCODE_GET_JOINT_COMMAND:
	case EXO_BLE_OPCODE_CAPTURE_ZERO:
	case EXO_BLE_OPCODE_CLEAR_ZERO:
		*payload_size = 1U;
		return true;
	case EXO_BLE_OPCODE_SET_CONTROL_CONFIG:
    case EXO_BLE_OPCODE_SAVED_ZERO_ACTION:
		*payload_size = 2U;
		return true;
	case EXO_BLE_OPCODE_TEACH_DIRECTION:
		*payload_size = 3U;
		return true;
	case EXO_BLE_OPCODE_SET_JOINT_COMMAND:
		*payload_size = JOINT_COMMAND_PAYLOAD_SIZE;
		return true;
	case EXO_BLE_OPCODE_SET_ASSIST_CONFIG:
		*payload_size = ASSIST_REQUEST_PAYLOAD_SIZE;
		return true;
	case EXO_BLE_OPCODE_SET_ANGLE_ASSIST_CONFIG:
		*payload_size = ASSIST_REQUEST_PAYLOAD_SIZE + 4U;
		return true;
    case EXO_BLE_OPCODE_SET_PULSE_TIMING:
        *payload_size=16U; return true;
    case EXO_BLE_OPCODE_SET_PULSE_WINDOW:
        *payload_size=4U; return true;
	case EXO_BLE_OPCODE_SET_ASSIST_CONFIG_WITH_LIMIT:
		*payload_size = ASSIST_REQUEST_PAYLOAD_SIZE + 2U;
		return true;
	case EXO_BLE_OPCODE_START_TELEMETRY:
		*payload_size = TELEMETRY_START_PAYLOAD_SIZE;
		return true;
	default:
		return false;
	}
}

/* result 可以出现在 SYSTEM_STATUS；NONE 仅表示“尚无历史请求”。 */
static bool result_is_valid(exo_ble_wire_result_t result) {
	switch (result) {
	case EXO_BLE_RESULT_OK:
	case EXO_BLE_RESULT_ACCEPTED:
	case EXO_BLE_RESULT_ALREADY_SATISFIED:
	case EXO_BLE_RESULT_ALREADY_PENDING:
	case EXO_BLE_RESULT_REJECTED_NOT_READY:
	case EXO_BLE_RESULT_REJECTED_BUSY:
	case EXO_BLE_RESULT_REJECTED_ERROR:
	case EXO_BLE_RESULT_INTERNAL_ERROR:
	case EXO_BLE_RESULT_UNSUPPORTED_VERSION:
	case EXO_BLE_RESULT_INVALID_LENGTH:
	case EXO_BLE_RESULT_UNKNOWN_OPCODE:
	case EXO_BLE_RESULT_INVALID_REQUEST_ID:
	case EXO_BLE_RESULT_INVALID_PAYLOAD:
	case EXO_BLE_RESULT_REQUEST_ID_CONFLICT:
	case EXO_BLE_RESULT_NONE:
		return true;
	default:
		return false;
	}
}

/* ACK 必须描述一次实际结果，因此不允许使用仅供快照使用的 NONE。 */
static bool ack_result_is_valid(exo_ble_wire_result_t result) {
	return result != EXO_BLE_RESULT_NONE && result_is_valid(result);
}

/* 以下小函数把允许的稳定线码限制在协议公开范围，拒绝内部或越界值。 */
static bool state_is_valid(exo_ble_wire_state_t state) {
	return (state >= EXO_BLE_STATE_INIT && state <= EXO_BLE_STATE_ERROR) ||
		   state == EXO_BLE_STATE_UNKNOWN;
}

static bool step_is_valid(exo_ble_wire_step_t step) {
	return (step >= EXO_BLE_STEP_NONE &&
			step <= EXO_BLE_STEP_VERIFY_DISABLED) ||
		   step == EXO_BLE_STEP_UNKNOWN;
}

static bool block_reason_is_valid(exo_ble_wire_block_reason_t reason) {
	return (reason >= EXO_BLE_BLOCK_NONE &&
			reason <= EXO_BLE_BLOCK_RUN_CONTRACT_INVALID) ||
		   reason == EXO_BLE_BLOCK_UNKNOWN;
}

static bool side_is_valid(exo_ble_wire_side_t side) {
	return side == EXO_BLE_SIDE_LEFT || side == EXO_BLE_SIDE_RIGHT;
}

static bool direction_is_valid(exo_ble_wire_direction_t direction) {
	return direction == EXO_BLE_DIRECTION_UNKNOWN ||
		   direction == EXO_BLE_DIRECTION_POSITIVE ||
		   direction == EXO_BLE_DIRECTION_NEGATIVE;
}

static bool selected_mode_is_valid(exo_ble_wire_control_mode_t mode) {
	return mode >= EXO_BLE_CONTROL_MODE_HOLD &&
		   mode <= EXO_BLE_CONTROL_MODE_IMPEDANCE;
}

static bool active_mode_is_valid(exo_ble_wire_control_mode_t mode) {
	return mode >= EXO_BLE_CONTROL_MODE_NONE &&
		   mode <= EXO_BLE_CONTROL_MODE_IMPEDANCE;
}

static bool reference_is_valid(exo_ble_wire_reference_t reference) {
	return reference == EXO_BLE_REFERENCE_CONSTANT ||
		   reference == EXO_BLE_REFERENCE_SINE ||
		   reference == EXO_BLE_REFERENCE_ALTERNATING_SINE_TEST || reference == EXO_BLE_REFERENCE_BENCH_PEAK;
}

static bool assist_mode_is_selectable(exo_ble_wire_assist_mode_t mode) {
	return mode >= EXO_BLE_ASSIST_MODE_TRANSPARENT &&
		   mode <= EXO_BLE_ASSIST_MODE_ADAPTIVE_GAUSSIAN;
}

static bool assist_mode_is_snapshot_value(exo_ble_wire_assist_mode_t mode) {
	return mode >= EXO_BLE_ASSIST_MODE_NONE &&
		   mode <= EXO_BLE_ASSIST_MODE_ADAPTIVE_GAUSSIAN;
}

static bool assist_execution_is_valid(exo_ble_wire_assist_execution_t state) {
	return state >= EXO_BLE_ASSIST_EXEC_DISABLED &&
		   state <= EXO_BLE_ASSIST_EXEC_ACTIVE;
}

static bool assist_strategy_is_valid(exo_ble_wire_assist_strategy_t strategy) {
	return strategy >= EXO_BLE_ASSIST_STRATEGY_NONE &&
		   strategy <= EXO_BLE_ASSIST_STRATEGY_ADAPTIVE_GAUSSIAN;
}

static bool
assist_mode_matches_strategy(exo_ble_wire_assist_mode_t mode,
							 exo_ble_wire_assist_strategy_t strategy) {
	return (mode == EXO_BLE_ASSIST_MODE_TRANSPARENT &&
			strategy == EXO_BLE_ASSIST_STRATEGY_TRANSPARENT) ||
		   (mode == EXO_BLE_ASSIST_MODE_ACTIVE_PHASE &&
			strategy == EXO_BLE_ASSIST_STRATEGY_PHASE_PROFILE) ||
		   (mode == EXO_BLE_ASSIST_MODE_ASSIST_AS_NEEDED &&
			strategy == EXO_BLE_ASSIST_STRATEGY_AAN_RESIDUAL) ||
		   (mode == EXO_BLE_ASSIST_MODE_ANGLE_THRESHOLD &&
		    strategy == EXO_BLE_ASSIST_STRATEGY_ANGLE_THRESHOLD) ||
        (mode==EXO_BLE_ASSIST_MODE_ADAPTIVE_GAUSSIAN && strategy==EXO_BLE_ASSIST_STRATEGY_ADAPTIVE_GAUSSIAN);
}

/*
 * 线协议先实施 RS01 Motion 的公开绝对边界；人体侧更窄的产品限制由业务层检查。
 * 这样格式层不会把越界数悄悄截断后交给电机驱动。
 */
static bool
joint_command_is_valid(const exo_ble_wire_joint_command_t *command) {
	return command != NULL && side_is_valid(command->side) &&
		   command->position_mrad >= EXO_BLE_JOINT_POSITION_MRAD_MIN &&
		   command->position_mrad <= EXO_BLE_JOINT_POSITION_MRAD_MAX &&
		   command->velocity_mrad_s >= EXO_BLE_JOINT_VELOCITY_MRAD_S_MIN &&
		   command->velocity_mrad_s <= EXO_BLE_JOINT_VELOCITY_MRAD_S_MAX &&
		   command->torque_mnm >= EXO_BLE_JOINT_TORQUE_MNM_MIN &&
		   command->torque_mnm <= EXO_BLE_JOINT_TORQUE_MNM_MAX &&
		   command->kp_x100 <= EXO_BLE_JOINT_KP_X100_MAX &&
		   command->kd_x1000 <= EXO_BLE_JOINT_KD_X1000_MAX;
}

/*
 * 为所有响应建立相同的 4-byte 帧头并预先检查输出容量。
 * response opcode 的最高位必须已经置 1；INFO/STATUS 等结构化响应要求
 * request_id 非 0，但 ACK 需要在错误请求可关联信息不足时允许 request_id=0。
 * 任一检查失败都保持 frame_len=0，调用者不会发送半成品 Notify。
 */
static bool begin_response(uint8_t response_opcode, uint8_t request_id,
						   uint8_t payload_size, bool require_request_id,
						   uint8_t *frame, size_t frame_capacity,
						   size_t *frame_len) {
	if (frame_len != NULL) {
		*frame_len = 0U;
	}
	if (frame == NULL || frame_len == NULL ||
		(response_opcode & EXO_BLE_PROTOCOL_RESPONSE_BIT) == 0U ||
		(require_request_id && request_id == 0U) ||
		payload_size > EXO_BLE_PROTOCOL_MAX_PAYLOAD_SIZE) {
		return false;
	}

	const size_t required_size =
		EXO_BLE_PROTOCOL_HEADER_SIZE + (size_t)payload_size;
	if (frame_capacity < required_size) {
		return false;
	}

	frame[FRAME_VERSION_OFFSET] = EXO_BLE_PROTOCOL_VERSION;
	frame[FRAME_OPCODE_OFFSET] = response_opcode;
	frame[FRAME_REQUEST_ID_OFFSET] = request_id;
	frame[FRAME_PAYLOAD_LENGTH_OFFSET] = payload_size;
	*frame_len = required_size;
	return true;
}

/*
 * 按固定顺序解码一帧手机请求：
 *   1. 清空 DTO，并尽量保存已经存在的 header 字节；
 *   2. 验证总长、声明 payload_len 和 20-byte 上限；
 *   3. 验证协议版本和 request_id；
 *   4. 验证 opcode 对应的精确 payload 长度；
 *   5. 对带 side 的请求验证 LEFT/RIGHT 线码。
 *
 * 先保留 header 再报错很重要：服务层可用原 opcode/request_id 返回相关联的
 * INVALID_LENGTH、UNSUPPORTED_VERSION 等 ACK，而不是让手机无从判断响应属于谁。
 */
exo_ble_wire_result_t
exo_ble_protocol_decode_request(const uint8_t *frame, size_t frame_len,
								exo_ble_request_t *request) {
	if (request == NULL) {
		return EXO_BLE_RESULT_INTERNAL_ERROR;
	}

	*request = (exo_ble_request_t){
		.side = EXO_BLE_SIDE_UNKNOWN,
		.control_config =
			{
				.selected_mode = EXO_BLE_CONTROL_MODE_NONE,
				.active_mode = EXO_BLE_CONTROL_MODE_NONE,
				.selected_reference = EXO_BLE_REFERENCE_CONSTANT,
				.active_reference = EXO_BLE_REFERENCE_CONSTANT,
			},
		.joint_command =
			{
				.side = EXO_BLE_SIDE_UNKNOWN,
			},
		.assist_config =
			{
				.selected_mode = EXO_BLE_ASSIST_MODE_NONE,
				.requested_execution_state = EXO_BLE_ASSIST_EXEC_DISABLED,
				.strategy_id = EXO_BLE_ASSIST_STRATEGY_NONE,
			},
		.telemetry_start =
			{
				.rate_hz = 20U,
			},
	};
	if (frame == NULL) {
		return EXO_BLE_RESULT_INVALID_LENGTH;
	}

	/* 错误帧也尽量保留已收到的 header，供服务层生成可关联 ACK。 */
	if (frame_len > FRAME_VERSION_OFFSET) {
		request->version = frame[FRAME_VERSION_OFFSET];
	}
	if (frame_len > FRAME_OPCODE_OFFSET) {
		request->opcode = frame[FRAME_OPCODE_OFFSET];
	}
	if (frame_len > FRAME_REQUEST_ID_OFFSET) {
		request->request_id = frame[FRAME_REQUEST_ID_OFFSET];
	}
	if (frame_len > FRAME_PAYLOAD_LENGTH_OFFSET) {
		request->payload_len = frame[FRAME_PAYLOAD_LENGTH_OFFSET];
	}

	if (frame_len < EXO_BLE_PROTOCOL_HEADER_SIZE ||
		frame_len > EXO_BLE_PROTOCOL_MAX_FRAME_SIZE ||
		request->payload_len > EXO_BLE_PROTOCOL_MAX_PAYLOAD_SIZE ||
		frame_len != EXO_BLE_PROTOCOL_HEADER_SIZE + request->payload_len) {
		return EXO_BLE_RESULT_INVALID_LENGTH;
	}
	if (request->version != EXO_BLE_PROTOCOL_VERSION) {
		return EXO_BLE_RESULT_UNSUPPORTED_VERSION;
	}
	if (request->request_id == 0U) {
		return EXO_BLE_RESULT_INVALID_REQUEST_ID;
	}

	uint8_t expected_payload_size;
	if (!request_payload_size(request->opcode, &expected_payload_size)) {
		return EXO_BLE_RESULT_UNKNOWN_OPCODE;
	}
	if (request->payload_len != expected_payload_size) {
		return EXO_BLE_RESULT_INVALID_PAYLOAD;
	}

    if (request->opcode == EXO_BLE_OPCODE_GET_BOARD_SENSOR) {
        if (frame[4] > 3U) return EXO_BLE_RESULT_INVALID_PAYLOAD;
        request->board_sensor_page = frame[4];
        return EXO_BLE_RESULT_OK;
    }
	if (request->opcode == EXO_BLE_OPCODE_GET_JOINT_STATUS ||
        request->opcode == EXO_BLE_OPCODE_GET_DIRECTION_PREVIEW ||
        request->opcode == EXO_BLE_OPCODE_GET_AFO_STATUS ||
		request->opcode == EXO_BLE_OPCODE_GET_CALIBRATION_STATUS ||
        request->opcode == EXO_BLE_OPCODE_GET_SAVED_ZERO ||
        request->opcode == EXO_BLE_OPCODE_GET_PULSE_STATUS ||
		request->opcode == EXO_BLE_OPCODE_GET_JOINT_COMMAND ||
		request->opcode == EXO_BLE_OPCODE_CAPTURE_ZERO ||
		request->opcode == EXO_BLE_OPCODE_CLEAR_ZERO) {
		const uint8_t side = frame[FRAME_PAYLOAD_OFFSET];
		if (side != EXO_BLE_SIDE_LEFT && side != EXO_BLE_SIDE_RIGHT) {
			return EXO_BLE_RESULT_INVALID_PAYLOAD;
		}
		request->side = (exo_ble_wire_side_t)side;
		return EXO_BLE_RESULT_OK;
	}

    if (request->opcode == EXO_BLE_OPCODE_SET_PULSE_TIMING) {
        const exo_ble_wire_pulse_timing_t t={
            .schema_version=frame[4],.duration_mode=frame[5],
            .peak_mnm=read_u16_le(&frame[6]),.fixed_ms=read_u16_le(&frame[8]),
            .ratio_permille=read_u16_le(&frame[10]),.minimum_ms=read_u16_le(&frame[12]),
            .maximum_ms=read_u16_le(&frame[14]),.period_minimum_ms=read_u16_le(&frame[16]),
            .period_maximum_ms=read_u16_le(&frame[18])};
        if(t.schema_version!=1 || t.duration_mode>1 || t.peak_mnm>EXO_BLE_ANGLE_TORQUE_HARD_MAX_MNM ||
            t.fixed_ms<100 || t.fixed_ms>EXO_BLE_PULSE_FIXED_MAX_MS || t.ratio_permille<100 || t.ratio_permille>600 ||
            t.minimum_ms<100 || t.maximum_ms>EXO_BLE_PULSE_SCALED_MAX_MS || t.minimum_ms>t.maximum_ms ||
            t.period_minimum_ms<300 || t.period_maximum_ms>5000 ||
            t.period_minimum_ms>=t.period_maximum_ms) return EXO_BLE_RESULT_INVALID_PAYLOAD;
        request->pulse_timing=t;
        return EXO_BLE_RESULT_OK;
    }
    if(request->opcode==EXO_BLE_OPCODE_SET_PULSE_WINDOW) {
        const exo_ble_wire_pulse_window_t w={.reset_cdeg=read_i16_le(&frame[4]),
            .maximum_cdeg=read_u16_le(&frame[6])};
        if(w.reset_cdeg< -3000 || w.reset_cdeg>5900 || w.maximum_cdeg<300 ||
            w.maximum_cdeg>9000 || (int)w.maximum_cdeg-(int)w.reset_cdeg<300)
            return EXO_BLE_RESULT_INVALID_PAYLOAD;
        request->pulse_window=w;
        return EXO_BLE_RESULT_OK;
    }
    if (request->opcode == EXO_BLE_OPCODE_SAVED_ZERO_ACTION) {
        if (frame[4] > 4U || frame[5] != 1U) return EXO_BLE_RESULT_INVALID_PAYLOAD;
        request->saved_zero_action = frame[4];
        return EXO_BLE_RESULT_OK;
    }
	if (request->opcode == EXO_BLE_OPCODE_TEACH_DIRECTION) {
		const uint8_t *payload = &frame[FRAME_PAYLOAD_OFFSET];
		if ((payload[0] != EXO_BLE_SIDE_LEFT && payload[0] != EXO_BLE_SIDE_RIGHT) ||
			payload[1] > 2U || payload[2] == 0U) return EXO_BLE_RESULT_INVALID_PAYLOAD;
		request->side = (exo_ble_wire_side_t)payload[0];
		request->direction_teach_action = payload[1];
		request->direction_teach_token = payload[2];
		return EXO_BLE_RESULT_OK;
	}

	if (request->opcode == EXO_BLE_OPCODE_SET_CONTROL_CONFIG) {
		const exo_ble_wire_control_mode_t selected_mode =
			(exo_ble_wire_control_mode_t)frame[FRAME_PAYLOAD_OFFSET + 0U];
		const exo_ble_wire_reference_t selected_reference =
			(exo_ble_wire_reference_t)frame[FRAME_PAYLOAD_OFFSET + 1U];
		if (!selected_mode_is_valid(selected_mode) ||
			!reference_is_valid(selected_reference)) {
			return EXO_BLE_RESULT_INVALID_PAYLOAD;
		}
		request->control_config.selected_mode = selected_mode;
		request->control_config.selected_reference = selected_reference;
		request->control_config.config_flags =
			EXO_BLE_CONTROL_CONFIG_FLAG_SELECTED_VALID;
		return EXO_BLE_RESULT_OK;
	}

	if (request->opcode == EXO_BLE_OPCODE_SET_JOINT_COMMAND) {
		exo_ble_wire_joint_command_t command = {
			.side = (exo_ble_wire_side_t)frame[FRAME_PAYLOAD_OFFSET + 0U],
			.position_mrad = read_i32_le(&frame[FRAME_PAYLOAD_OFFSET + 1U]),
			.velocity_mrad_s = read_i32_le(&frame[FRAME_PAYLOAD_OFFSET + 5U]),
			.torque_mnm = read_i16_le(&frame[FRAME_PAYLOAD_OFFSET + 9U]),
			.kp_x100 = read_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 11U]),
			.kd_x1000 = read_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 13U]),
		};
		if (!joint_command_is_valid(&command)) {
			return EXO_BLE_RESULT_INVALID_PAYLOAD;
		}
		request->side = command.side;
		request->joint_command = command;
	}

	if (request->opcode == EXO_BLE_OPCODE_SET_ASSIST_CONFIG ||
		request->opcode == EXO_BLE_OPCODE_SET_ASSIST_CONFIG_WITH_LIMIT ||
		request->opcode == EXO_BLE_OPCODE_SET_ANGLE_ASSIST_CONFIG) {
		const exo_ble_wire_assist_request_t config = {
			.schema_version = frame[FRAME_PAYLOAD_OFFSET + 0U],
			.selected_mode =
				(exo_ble_wire_assist_mode_t)frame[FRAME_PAYLOAD_OFFSET + 1U],
			.requested_execution_state = (exo_ble_wire_assist_execution_t)
				frame[FRAME_PAYLOAD_OFFSET + 2U],
			.strategy_id = (exo_ble_wire_assist_strategy_t)
				frame[FRAME_PAYLOAD_OFFSET + 3U],
			.profile_id = frame[FRAME_PAYLOAD_OFFSET + 4U],
			.left_level_percent = frame[FRAME_PAYLOAD_OFFSET + 5U],
			.right_level_percent = frame[FRAME_PAYLOAD_OFFSET + 6U],
			.reserved_flags = frame[FRAME_PAYLOAD_OFFSET + 7U],
		};
		/*
		 * ACTIVE 是可识别的线码；是否允许由业务层按策略/profile元数据判断。
		 * 这样手机能区分“字段损坏”和“组合合法但当前固件没有执行许可”。
		 */
		if (config.schema_version != 1U ||
			!assist_mode_is_selectable(config.selected_mode) ||
			config.requested_execution_state > EXO_BLE_ASSIST_EXEC_ACTIVE ||
			!assist_strategy_is_valid(config.strategy_id) ||
			!assist_mode_matches_strategy(config.selected_mode,
										  config.strategy_id) ||
			config.left_level_percent > 100U ||
			config.right_level_percent > 100U || config.reserved_flags != 0U) {
			return EXO_BLE_RESULT_INVALID_PAYLOAD;
		}
		if ((config.strategy_id == EXO_BLE_ASSIST_STRATEGY_ANGLE_THRESHOLD) !=
            (request->opcode == EXO_BLE_OPCODE_SET_ANGLE_ASSIST_CONFIG) ||
            (config.strategy_id == EXO_BLE_ASSIST_STRATEGY_ANGLE_THRESHOLD &&
             config.profile_id != 0U)) return EXO_BLE_RESULT_INVALID_PAYLOAD;
        request->assist_config = config;
        if (request->opcode == EXO_BLE_OPCODE_SET_ANGLE_ASSIST_CONFIG) {
            request->angle_threshold_cdeg = read_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 10U]);
            if (request->angle_threshold_cdeg < EXO_BLE_ANGLE_MIN_CDEG || request->angle_threshold_cdeg > EXO_BLE_ANGLE_MAX_CDEG)
                return EXO_BLE_RESULT_INVALID_PAYLOAD;
        }
		if (request->opcode == EXO_BLE_OPCODE_SET_ASSIST_CONFIG_WITH_LIMIT ||
		request->opcode == EXO_BLE_OPCODE_SET_ANGLE_ASSIST_CONFIG) {
			request->maximum_assist_torque_mnm =
				read_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 8U]);
			if (request->maximum_assist_torque_mnm >
                (request->opcode == EXO_BLE_OPCODE_SET_ANGLE_ASSIST_CONFIG ||
                 config.strategy_id == EXO_BLE_ASSIST_STRATEGY_ADAPTIVE_GAUSSIAN
                 ? EXO_BLE_ANGLE_TORQUE_HARD_MAX_MNM
                 : EXO_BLE_ASSIST_TORQUE_HARD_MAX_MNM)) {
				return EXO_BLE_RESULT_INVALID_PAYLOAD;
			}
		}
	}

	if (request->opcode == EXO_BLE_OPCODE_START_TELEMETRY) {
		const uint8_t rate_hz = frame[FRAME_PAYLOAD_OFFSET + 0U];
		const uint16_t channel_mask =
			read_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 1U]);
		if ((rate_hz != 5U && rate_hz != 10U && rate_hz != 20U) ||
			channel_mask == 0U ||
			(channel_mask & ~EXO_BLE_TELEMETRY_CHANNELS_MASK) != 0U) {
			return EXO_BLE_RESULT_INVALID_PAYLOAD;
		}
		request->telemetry_start.rate_hz = rate_hz;
		request->telemetry_start.channel_mask = channel_mask;
	}

	return EXO_BLE_RESULT_OK;
}

/*
 * 编码通用 ACK：先验证 result/state/flags，再把请求 opcode 的响应位设为 1。
 * payload 固定为 result、state、system_flags、detail 四个单字节字段。
 */
bool exo_ble_protocol_encode_ack(uint8_t request_opcode, uint8_t request_id,
								 const exo_ble_ack_t *ack, uint8_t *frame,
								 size_t frame_capacity, size_t *frame_len) {
	if (frame_len != NULL) {
		*frame_len = 0U;
	}
	if (ack == NULL || request_opcode >= EXO_BLE_PROTOCOL_RESPONSE_BIT ||
		!ack_result_is_valid(ack->result) || !state_is_valid(ack->state) ||
		(ack->system_flags & ~EXO_BLE_SYSTEM_FLAGS_MASK) != 0U) {
		return false;
	}

	const uint8_t response_opcode =
		request_opcode | EXO_BLE_PROTOCOL_RESPONSE_BIT;
	if (!begin_response(response_opcode, request_id, ACK_PAYLOAD_SIZE, false,
						frame, frame_capacity, frame_len)) {
		return false;
	}

	frame[FRAME_PAYLOAD_OFFSET + 0U] = (uint8_t)ack->result;
	frame[FRAME_PAYLOAD_OFFSET + 1U] = (uint8_t)ack->state;
	frame[FRAME_PAYLOAD_OFFSET + 2U] = ack->system_flags;
	frame[FRAME_PAYLOAD_OFFSET + 3U] = ack->detail;
	return true;
}

/*
 * 编码 GET_INFO 响应：协议范围必须包含当前 v1，能力位只能使用公开 mask，
 * max_frame_size 必须与本实现的 20 bytes 一致。多字节能力和 boot_id 使用 LE。
 */
bool exo_ble_protocol_encode_info(uint8_t request_id,
								  const exo_ble_info_t *info, uint8_t *frame,
								  size_t frame_capacity, size_t *frame_len) {
	if (frame_len != NULL) {
		*frame_len = 0U;
	}
	if (info == NULL || info->protocol_min_version == 0U ||
		info->protocol_min_version > EXO_BLE_PROTOCOL_VERSION ||
		info->protocol_max_version < EXO_BLE_PROTOCOL_VERSION ||
		info->max_frame_size != EXO_BLE_PROTOCOL_MAX_FRAME_SIZE ||
		(info->capabilities & ~EXO_BLE_CAPABILITIES_MASK) != 0U) {
		return false;
	}
	if (!begin_response(EXO_BLE_RESPONSE_INFO, request_id, INFO_PAYLOAD_SIZE,
						true, frame, frame_capacity, frame_len)) {
		return false;
	}

	frame[FRAME_PAYLOAD_OFFSET + 0U] = info->protocol_min_version;
	frame[FRAME_PAYLOAD_OFFSET + 1U] = info->protocol_max_version;
	frame[FRAME_PAYLOAD_OFFSET + 2U] = info->firmware_major;
	frame[FRAME_PAYLOAD_OFFSET + 3U] = info->firmware_minor;
	frame[FRAME_PAYLOAD_OFFSET + 4U] = info->firmware_patch;
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 5U], info->capabilities);
	frame[FRAME_PAYLOAD_OFFSET + 7U] = info->max_frame_size;
	write_u32_le(&frame[FRAME_PAYLOAD_OFFSET + 8U], info->boot_id);
	return true;
}

/*
 * 编码 GET_SYSTEM_STATUS 响应：逐一验证状态、步骤、flags、阻塞原因和最近结果，
 * 再按固定顺序写入 10-byte payload。transition_count 使用 u32 little-endian。
 */
bool exo_ble_protocol_encode_system_status(
	uint8_t request_id, const exo_ble_system_status_t *status, uint8_t *frame,
	size_t frame_capacity, size_t *frame_len) {
	if (frame_len != NULL) {
		*frame_len = 0U;
	}
	if (status == NULL || !state_is_valid(status->state) ||
		!step_is_valid(status->active_step) ||
		(status->system_flags & ~EXO_BLE_SYSTEM_FLAGS_MASK) != 0U ||
		!block_reason_is_valid(status->start_block_reason) ||
		!result_is_valid(status->last_request_result) ||
		!step_is_valid(status->last_failed_step)) {
		return false;
	}
	if (!begin_response(EXO_BLE_RESPONSE_SYSTEM_STATUS, request_id,
						SYSTEM_STATUS_PAYLOAD_SIZE, true, frame, frame_capacity,
						frame_len)) {
		return false;
	}

	frame[FRAME_PAYLOAD_OFFSET + 0U] = (uint8_t)status->state;
	frame[FRAME_PAYLOAD_OFFSET + 1U] = (uint8_t)status->active_step;
	frame[FRAME_PAYLOAD_OFFSET + 2U] = status->system_flags;
	frame[FRAME_PAYLOAD_OFFSET + 3U] = (uint8_t)status->start_block_reason;
	frame[FRAME_PAYLOAD_OFFSET + 4U] = (uint8_t)status->last_request_result;
	frame[FRAME_PAYLOAD_OFFSET + 5U] = (uint8_t)status->last_failed_step;
	write_u32_le(&frame[FRAME_PAYLOAD_OFFSET + 6U], status->transition_count);
	return true;
}

/*
 * 编码 GET_JOINT_STATUS 响应：side/direction/flags 都是稳定线码；位置和速度
 * 已由 exo_ble.c 转成 v1 规定的 mrad、mrad/s，这里只按 i32 LE 写入。
 * 手机解析后再换算为 deg、deg/s 显示；本函数不参与物理单位转换。
 */
_Static_assert(EXO_BLE_PROTOCOL_HEADER_SIZE + CALIBRATION_STATUS_PAYLOAD_SIZE ==
	EXO_BLE_PROTOCOL_MAX_FRAME_SIZE, "calibration must fit default ATT payload");

bool exo_ble_protocol_encode_saved_zero(uint8_t request_id,
    const exo_ble_saved_zero_status_t *status, uint8_t *frame,
    size_t frame_capacity, size_t *frame_len) {
    if (frame_len) *frame_len = 0U;
    if (!status || !side_is_valid(status->side) || !direction_is_valid(status->direction) ||
        (status->actuator > 1U && status->actuator != 0xFFU) ||
        (status->can_id > 0x7FU && status->can_id != 0xFFU) ||
        (status->flags & ~15U) || status->reason > 13U ||
        ((status->flags & 2U) && (!(status->flags & 1U) || status->actuator > 1U ||
            status->can_id > 0x7FU || status->direction == EXO_BLE_DIRECTION_UNKNOWN)) ||
        (((status->flags & 1U) != 0U) != (status->saved_zero_mdeg != INT32_MIN)) ||
        (((status->flags & 4U) != 0U) != (status->preview_joint_mdeg != INT32_MIN)) ||
        ((status->flags & 1U) && (status->saved_zero_mdeg < -720000 || status->saved_zero_mdeg > 720000)) ||
        ((status->flags & 4U) && ((status->flags & 3U) != 3U ||
            status->preview_joint_mdeg < -720000 || status->preview_joint_mdeg > 720000)) ||
        ((status->reason == 0U) != ((status->flags & 8U) != 0U)) ||
        ((status->flags & 8U) && (status->flags != 15U ||
            status->preview_joint_mdeg < -5000 || status->preview_joint_mdeg > 5000))) return false;
    if (!begin_response(EXO_BLE_RESPONSE_SAVED_ZERO, request_id, 16U, true,
        frame, frame_capacity, frame_len)) return false;
    frame[4] = (uint8_t)status->side;
    frame[5] = status->actuator;
    frame[6] = status->can_id;
    frame[7] = (uint8_t)status->direction;
    frame[8] = status->flags;
    frame[9] = status->reason;
    write_u16_le(&frame[10], 0U);
    write_i32_le(&frame[12], status->saved_zero_mdeg);
    write_i32_le(&frame[16], status->preview_joint_mdeg);
    return true;
}

bool exo_ble_protocol_encode_calibration_status(
	uint8_t request_id, const exo_ble_calibration_status_t *status,
	uint8_t *frame, size_t frame_capacity, size_t *frame_len) {
	if (frame_len != NULL) *frame_len = 0U;
	if (status == NULL || !side_is_valid(status->side) ||
		!direction_is_valid(status->direction) ||
		(status->joint_flags & ~EXO_BLE_JOINT_FLAGS_MASK) != 0U ||
		(status->actuator > 1U && status->actuator != 0xFFU) ||
		(status->can_id > 0x7FU && status->can_id != 0xFFU)) return false;
	if (!begin_response(EXO_BLE_RESPONSE_CALIBRATION_STATUS, request_id,
		CALIBRATION_STATUS_PAYLOAD_SIZE, true, frame, frame_capacity, frame_len))
		return false;
	frame[4] = (uint8_t)status->side;
	frame[5] = status->actuator;
	frame[6] = status->can_id;
	frame[7] = (uint8_t)status->direction;
	frame[8] = status->joint_flags;
	frame[9] = 0U;
	write_i32_le(&frame[10], status->raw_position_mrad);
	write_i32_le(&frame[14], status->software_zero_mrad);
	write_u16_le(&frame[18], status->feedback_age_ms);
	return true;
}

bool exo_ble_protocol_encode_joint_status(uint8_t request_id,
										  const exo_ble_joint_status_t *status,
										  uint8_t *frame, size_t frame_capacity,
										  size_t *frame_len) {
	if (frame_len != NULL) {
		*frame_len = 0U;
	}
	if (status == NULL || !side_is_valid(status->side) ||
		(status->joint_flags & ~EXO_BLE_JOINT_FLAGS_MASK) != 0U ||
		!direction_is_valid(status->direction)) {
		return false;
	}
	if (!begin_response(EXO_BLE_RESPONSE_JOINT_STATUS, request_id,
						JOINT_STATUS_PAYLOAD_SIZE, true, frame, frame_capacity,
						frame_len)) {
		return false;
	}

	frame[FRAME_PAYLOAD_OFFSET + 0U] = (uint8_t)status->side;
	frame[FRAME_PAYLOAD_OFFSET + 1U] = status->joint_flags;
	frame[FRAME_PAYLOAD_OFFSET + 2U] = (uint8_t)status->direction;
	write_i32_le(&frame[FRAME_PAYLOAD_OFFSET + 3U], status->position_mrad);
	write_i32_le(&frame[FRAME_PAYLOAD_OFFSET + 7U], status->velocity_mrad_s);
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 11U], status->feedback_age_ms);
	return true;
}

/*
 * 编码阶段 8 的模式/参考源快照。正常响应使用 5-byte payload；最后一个
 * config_flags 不仅表达 selected/active 是否有效，也让客户端能与 4-byte ACK
 * 无歧义地区分。无效槽位必须使用 NONE，不能残留一个看似可用的旧模式。
 */
bool exo_ble_protocol_encode_control_config(
	uint8_t request_id, const exo_ble_wire_control_config_t *config,
	uint8_t *frame, size_t frame_capacity, size_t *frame_len) {
	if (frame_len != NULL) {
		*frame_len = 0U;
	}
	if (config == NULL ||
		(config->config_flags & ~EXO_BLE_CONTROL_CONFIG_FLAGS_MASK) != 0U ||
		!reference_is_valid(config->selected_reference) ||
		!reference_is_valid(config->active_reference)) {
		return false;
	}

	const bool selected_valid =
		(config->config_flags & EXO_BLE_CONTROL_CONFIG_FLAG_SELECTED_VALID) !=
		0U;
	const bool active_valid =
		(config->config_flags & EXO_BLE_CONTROL_CONFIG_FLAG_ACTIVE_VALID) != 0U;
	if ((selected_valid && !selected_mode_is_valid(config->selected_mode)) ||
		(!selected_valid &&
		 config->selected_mode != EXO_BLE_CONTROL_MODE_NONE) ||
		(active_valid && !selected_mode_is_valid(config->active_mode)) ||
		(!active_valid && config->active_mode != EXO_BLE_CONTROL_MODE_NONE) ||
		(active_valid && !selected_valid) ||
		!active_mode_is_valid(config->active_mode)) {
		return false;
	}
	if (!begin_response(EXO_BLE_RESPONSE_CONTROL_CONFIG, request_id,
						CONTROL_CONFIG_PAYLOAD_SIZE, true, frame,
						frame_capacity, frame_len)) {
		return false;
	}

	frame[FRAME_PAYLOAD_OFFSET + 0U] = (uint8_t)config->selected_mode;
	frame[FRAME_PAYLOAD_OFFSET + 1U] = (uint8_t)config->active_mode;
	frame[FRAME_PAYLOAD_OFFSET + 2U] = (uint8_t)config->selected_reference;
	frame[FRAME_PAYLOAD_OFFSET + 3U] = (uint8_t)config->active_reference;
	frame[FRAME_PAYLOAD_OFFSET + 4U] = config->config_flags;
	return true;
}

/*
 * 编码单侧五元命令。字段按协议固定顺序逐项写入，既不 memcpy C 结构体，也
 * 不把越界数饱和到边界。业务层只有在参数本来就合法时才能得到可发送帧。
 */
bool exo_ble_protocol_encode_joint_command(
	uint8_t request_id, const exo_ble_wire_joint_command_t *command,
	uint8_t *frame, size_t frame_capacity, size_t *frame_len) {
	if (frame_len != NULL) {
		*frame_len = 0U;
	}
	if (!joint_command_is_valid(command)) {
		return false;
	}
	if (!begin_response(EXO_BLE_RESPONSE_JOINT_COMMAND, request_id,
						JOINT_COMMAND_PAYLOAD_SIZE, true, frame, frame_capacity,
						frame_len)) {
		return false;
	}

	frame[FRAME_PAYLOAD_OFFSET + 0U] = (uint8_t)command->side;
	write_i32_le(&frame[FRAME_PAYLOAD_OFFSET + 1U], command->position_mrad);
	write_i32_le(&frame[FRAME_PAYLOAD_OFFSET + 5U], command->velocity_mrad_s);
	write_i16_le(&frame[FRAME_PAYLOAD_OFFSET + 9U], command->torque_mnm);
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 11U], command->kp_x100);
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 13U], command->kd_x1000);
	return true;
}

/*
 * 编码阶段9步态状态。相位、步频、置信度和年龄均使用0xFFFF表示无效，手机不能
 * 把无效0误显示成“0%相位”或“0 step/min”。允许左右单侧有效，但overall_valid
 * 只有在两侧都有效时才能置位。
 */
bool exo_ble_protocol_encode_gait_status(
	uint8_t request_id, const exo_ble_wire_gait_status_t *status,
	uint8_t *frame, size_t frame_capacity, size_t *frame_len) {
	if (frame_len != NULL) {
		*frame_len = 0U;
	}
	if (status == NULL ||
		status->estimator_id > EXO_BLE_GAIT_ESTIMATOR_HIP_EVENT_TBE ||
		status->estimator_state > EXO_BLE_GAIT_STATE_DEGRADED ||
		(status->gait_flags & ~EXO_BLE_GAIT_FLAGS_MASK) != 0U ||
		status->phase_origin > EXO_BLE_GAIT_ORIGIN_HIP_EXTENSION_PEAK ||
		(status->quality_flags & ~EXO_BLE_GAIT_QUALITY_FLAGS_MASK) != 0U) {
		return false;
	}
	const bool left_valid =
		(status->gait_flags & EXO_BLE_GAIT_FLAG_LEFT_PHASE_VALID) != 0U;
	const bool right_valid =
		(status->gait_flags & EXO_BLE_GAIT_FLAG_RIGHT_PHASE_VALID) != 0U;
	const bool overall_valid =
		(status->gait_flags & EXO_BLE_GAIT_FLAG_OVERALL_VALID) != 0U;
	const bool any_phase_valid = left_valid || right_valid;
	if ((overall_valid && (!left_valid || !right_valid)) ||
		(left_valid
			 ? status->left_phase_x10000 > EXO_BLE_GAIT_PHASE_X10000_MAX
			 : status->left_phase_x10000 != EXO_BLE_GAIT_FIXED_INVALID) ||
		(right_valid
			 ? status->right_phase_x10000 > EXO_BLE_GAIT_PHASE_X10000_MAX
			 : status->right_phase_x10000 != EXO_BLE_GAIT_FIXED_INVALID) ||
		(any_phase_valid
			 ? status->cadence_x100 > EXO_BLE_GAIT_CADENCE_X100_MAX
			 : status->cadence_x100 != EXO_BLE_GAIT_FIXED_INVALID) ||
		(any_phase_valid
			 ? status->confidence_x10000 > EXO_BLE_GAIT_CONFIDENCE_X10000_MAX
			 : status->confidence_x10000 != EXO_BLE_GAIT_FIXED_INVALID)) {
		return false;
	}
	if (!begin_response(EXO_BLE_RESPONSE_GAIT_STATUS, request_id,
						GAIT_STATUS_PAYLOAD_SIZE, true, frame, frame_capacity,
						frame_len)) {
		return false;
	}
	frame[FRAME_PAYLOAD_OFFSET + 0U] = (uint8_t)status->estimator_id;
	frame[FRAME_PAYLOAD_OFFSET + 1U] = (uint8_t)status->estimator_state;
	frame[FRAME_PAYLOAD_OFFSET + 2U] = status->gait_flags;
	frame[FRAME_PAYLOAD_OFFSET + 3U] = (uint8_t)status->phase_origin;
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 4U], status->quality_flags);
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 6U], status->left_phase_x10000);
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 8U], status->right_phase_x10000);
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 10U], status->cadence_x100);
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 12U], status->confidence_x10000);
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 14U], status->estimate_age_ms);
	return true;
}

/* 编码高层助力配置/状态；16-byte payload明确与4-byte通用ACK区分。 */
bool exo_ble_protocol_encode_assist_config(
	uint8_t request_id, const exo_ble_wire_assist_config_t *config,
	uint8_t *frame, size_t frame_capacity, size_t *frame_len) {
	if (frame_len != NULL) {
		*frame_len = 0U;
	}
	if (config == NULL || config->schema_version != 1U ||
		!assist_mode_is_snapshot_value(config->selected_mode) ||
		!assist_mode_is_snapshot_value(config->active_mode) ||
		!assist_execution_is_valid(config->requested_execution_state) ||
		!assist_execution_is_valid(config->execution_state) ||
		!assist_strategy_is_valid(config->strategy_id) ||
		config->left_level_percent > 100U ||
		config->right_level_percent > 100U ||
		(config->assist_flags & ~EXO_BLE_ASSIST_FLAGS_MASK) != 0U ||
		config->unavailable_reason >
			EXO_BLE_ASSIST_UNAVAILABLE_AFO_INPUT_FAULT) {
		return false;
	}
	const bool selected_valid =
		(config->assist_flags & EXO_BLE_ASSIST_FLAG_SELECTED_VALID) != 0U;
	const bool active_valid =
		(config->assist_flags & EXO_BLE_ASSIST_FLAG_ACTIVE_VALID) != 0U;
	const bool real_output =
		(config->assist_flags & EXO_BLE_ASSIST_FLAG_REAL_OUTPUT_ENABLED) != 0U;
	if ((selected_valid && (!assist_mode_is_selectable(config->selected_mode) ||
							!assist_mode_matches_strategy(
								config->selected_mode, config->strategy_id))) ||
		(!selected_valid &&
		 config->selected_mode != EXO_BLE_ASSIST_MODE_NONE) ||
		(active_valid && !assist_mode_is_selectable(config->active_mode)) ||
		(!active_valid && config->active_mode != EXO_BLE_ASSIST_MODE_NONE) ||
		(active_valid && !selected_valid) ||
		(!active_valid &&
		 config->execution_state != EXO_BLE_ASSIST_EXEC_DISABLED) ||
		(!selected_valid &&
		 config->strategy_id != EXO_BLE_ASSIST_STRATEGY_NONE) ||
		(!selected_valid &&
		 config->requested_execution_state != EXO_BLE_ASSIST_EXEC_DISABLED) ||
		(real_output &&
		 config->execution_state != EXO_BLE_ASSIST_EXEC_ACTIVE)) {
		return false;
	}
	if (!begin_response(EXO_BLE_RESPONSE_ASSIST_CONFIG, request_id,
						ASSIST_CONFIG_PAYLOAD_SIZE, true, frame, frame_capacity,
						frame_len)) {
		return false;
	}
	frame[FRAME_PAYLOAD_OFFSET + 0U] = config->schema_version;
	frame[FRAME_PAYLOAD_OFFSET + 1U] = (uint8_t)config->selected_mode;
	frame[FRAME_PAYLOAD_OFFSET + 2U] = (uint8_t)config->active_mode;
	/* 一个字节的低/高半字节分别保存selected请求和当前有效状态。 */
	frame[FRAME_PAYLOAD_OFFSET + 3U] =
		(uint8_t)config->requested_execution_state |
		(uint8_t)((uint8_t)config->execution_state << 4U);
	frame[FRAME_PAYLOAD_OFFSET + 4U] = (uint8_t)config->strategy_id;
	frame[FRAME_PAYLOAD_OFFSET + 5U] = config->strategy_version;
	frame[FRAME_PAYLOAD_OFFSET + 6U] = config->profile_id;
	frame[FRAME_PAYLOAD_OFFSET + 7U] = config->left_level_percent;
	frame[FRAME_PAYLOAD_OFFSET + 8U] = config->right_level_percent;
	frame[FRAME_PAYLOAD_OFFSET + 9U] = config->assist_flags;
	frame[FRAME_PAYLOAD_OFFSET + 10U] = (uint8_t)config->unavailable_reason;
	frame[FRAME_PAYLOAD_OFFSET + 11U] = config->config_generation;
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 12U],
				 config->required_sensor_mask);
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 14U],
				 config->available_sensor_mask);
	return true;
}

/* 编码最大附加力矩快照；显式区分保存值、本轮锁存值和固件硬上限。 */
bool exo_ble_protocol_encode_assist_torque_limit(
	uint8_t request_id, const exo_ble_wire_assist_torque_limit_t *limit,
	uint8_t *frame, size_t frame_capacity, size_t *frame_len) {
	if (frame_len != NULL) *frame_len = 0U;
	if (limit == NULL || (limit->flags & ~3U) != 0U ||
		(limit->hard_maximum_mnm != EXO_BLE_ASSIST_TORQUE_HARD_MAX_MNM &&
         limit->hard_maximum_mnm != EXO_BLE_ANGLE_TORQUE_HARD_MAX_MNM) ||
		limit->selected_maximum_mnm > limit->hard_maximum_mnm ||
		limit->active_maximum_mnm > limit->hard_maximum_mnm ||
		((limit->flags & 1U) == 0U && limit->selected_maximum_mnm != 0U) ||
		((limit->flags & 2U) == 0U && limit->active_maximum_mnm != 0U) ||
		((limit->flags & 2U) != 0U && (limit->flags & 1U) == 0U)) return false;
	if (!begin_response(EXO_BLE_RESPONSE_ASSIST_TORQUE_LIMIT, request_id,
						8U, true, frame, frame_capacity, frame_len)) return false;
	frame[4] = 1U;
	frame[5] = limit->flags;
	write_u16_le(&frame[6], limit->selected_maximum_mnm);
	write_u16_le(&frame[8], limit->active_maximum_mnm);
	write_u16_le(&frame[10], limit->hard_maximum_mnm);
	return true;
}

/* 经由Response返回遥测流状态，不与无request_id的独立流样本混淆。 */
bool exo_ble_protocol_encode_telemetry_status(
	uint8_t request_id, const exo_ble_wire_telemetry_status_t *status,
	uint8_t *frame, size_t frame_capacity, size_t *frame_len) {
	if (frame_len != NULL) {
		*frame_len = 0U;
	}
	if (status == NULL ||
		(status->rate_hz != 5U && status->rate_hz != 10U &&
		 status->rate_hz != 20U) ||
		status->channel_mask == 0U ||
		(status->channel_mask & ~EXO_BLE_TELEMETRY_CHANNELS_MASK) != 0U ||
		status->queued_samples > status->queue_capacity) {
		return false;
	}
	if (!begin_response(EXO_BLE_RESPONSE_TELEMETRY_STATUS, request_id,
						TELEMETRY_STATUS_PAYLOAD_SIZE, true, frame,
						frame_capacity, frame_len)) {
		return false;
	}
	frame[FRAME_PAYLOAD_OFFSET + 0U] = status->streaming ? 1U : 0U;
	frame[FRAME_PAYLOAD_OFFSET + 1U] = status->rate_hz;
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 2U], status->channel_mask);
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 4U], status->stream_sequence);
	write_u32_le(&frame[FRAME_PAYLOAD_OFFSET + 6U],
				 status->notify_success_count);
	write_u32_le(&frame[FRAME_PAYLOAD_OFFSET + 10U],
				 status->dropped_sample_count);
	frame[FRAME_PAYLOAD_OFFSET + 14U] = status->queued_samples;
	frame[FRAME_PAYLOAD_OFFSET + 15U] = status->queue_capacity;
	return true;
}

static bool telemetry_channel_is_unsigned(uint8_t channel) {
	return channel == EXO_BLE_TELEMETRY_CHANNEL_GAIT_PHASE ||
		   channel == EXO_BLE_TELEMETRY_CHANNEL_GAIT_CONFIDENCE;
}

/*
 * 编码独立 Telemetry Notify，固定为 20 bytes：
 *   0 schema, 1 type, 2 channel, 3 flags,
 *   4..5 stream_seq, 6..9 sample_seq, 10..13 timestamp_ms,
 *   14..15 activation_epoch low16, 16..17 left, 18..19 right。
 */
bool exo_ble_protocol_encode_telemetry_sample(
	const exo_ble_wire_telemetry_sample_t *sample, uint8_t *frame,
	size_t frame_capacity, size_t *frame_len) {
	if (frame_len != NULL) {
		*frame_len = 0U;
	}
	if (sample == NULL || frame == NULL || frame_len == NULL ||
		frame_capacity < EXO_BLE_TELEMETRY_FRAME_SIZE ||
		sample->stream_sequence == 0U || sample->sample_sequence == 0U ||
		sample->channel == 0U ||
		(sample->channel & ~EXO_BLE_TELEMETRY_CHANNELS_MASK) != 0U ||
		(sample->channel & (sample->channel - 1U)) != 0U) {
		return false;
	}
	const bool left_valid =
		(sample->flags & EXO_BLE_TELEMETRY_FLAG_LEFT_VALID) != 0U;
	const bool right_valid =
		(sample->flags & EXO_BLE_TELEMETRY_FLAG_RIGHT_VALID) != 0U;
	const uint16_t invalid = telemetry_channel_is_unsigned(sample->channel)
								 ? EXO_BLE_TELEMETRY_UNSIGNED_INVALID
								 : EXO_BLE_TELEMETRY_SIGNED_INVALID;
	if ((left_valid && sample->left_value == invalid) ||
		(!left_valid && sample->left_value != invalid) ||
		(right_valid && sample->right_value == invalid) ||
		(!right_valid && sample->right_value != invalid)) {
		return false;
	}

	frame[0] = EXO_BLE_TELEMETRY_SCHEMA_VERSION;
	frame[1] = EXO_BLE_TELEMETRY_FRAME_TYPE_SAMPLE;
	frame[2] = sample->channel;
	frame[3] = sample->flags;
	write_u16_le(&frame[4], sample->stream_sequence);
	write_u32_le(&frame[6], sample->sample_sequence);
	write_u32_le(&frame[10], sample->timestamp_ms);
	write_u16_le(&frame[14], sample->activation_epoch_lsb);
	write_u16_le(&frame[16], sample->left_value);
	write_u16_le(&frame[18], sample->right_value);
	*frame_len = EXO_BLE_TELEMETRY_FRAME_SIZE;
	return true;
}

/* 逐字节编码，MTU=23也只占一个20字节响应。计数饱和由业务层完成。 */
bool exo_ble_protocol_encode_afo_status(uint8_t id,const exo_ble_afo_status_t *s,
    uint8_t *frame,size_t capacity,size_t *length) {
    if(length)*length=0;
    if(!s||s->side>1||(s->faults&~127U)||(s->first_faults&~127U)||
       (s->flags&~7U)||(s->lock_reasons&~511U)||
       !begin_response(EXO_BLE_OPCODE_GET_AFO_STATUS|0x80,id,16,true,frame,capacity,length))return false;
    frame[4]=s->side;frame[5]=s->faults;frame[6]=s->first_faults;frame[7]=s->flags;
    write_u16_le(frame+8,s->lock_reasons);write_u32_le(frame+10,s->first_timestamp_ms);
    write_i16_le(frame+14,s->first_q_cdeg);write_i16_le(frame+16,s->first_v_ddeg_s);
    frame[18]=s->first_source_interval_ms;frame[19]=s->first_age_ms;return true;
}
bool exo_ble_protocol_encode_direction_preview(uint8_t request_id, uint8_t side,
    uint8_t token, uint8_t result, uint8_t actuator, int32_t baseline_mrad,
    int32_t current_mrad, uint16_t remaining_ms, uint16_t age_ms,
    uint8_t *frame, size_t capacity, size_t *length) {
    if (length) *length=0;
    if (side>1 || result>13 || (actuator>1 && actuator!=255) || remaining_ms>60000 ||
        (result==0 && (!token || actuator>1 || !remaining_ms || age_ms>50 ||
         baseline_mrad==INT32_MIN || current_mrad==INT32_MIN))) return false;
    if (!begin_response(EXO_BLE_OPCODE_GET_DIRECTION_PREVIEW|EXO_BLE_PROTOCOL_RESPONSE_BIT,
        request_id,16,true,frame,capacity,length)) return false;
    frame[4]=side;frame[5]=token;frame[6]=result;frame[7]=actuator;
    write_u32_le(frame+8,(uint32_t)baseline_mrad);
    write_u32_le(frame+12,(uint32_t)current_mrad);
    write_u16_le(frame+16,remaining_ms);write_u16_le(frame+18,age_ms);
    return true;
}

bool exo_ble_protocol_encode_board_sensor(uint8_t request_id, uint8_t page,
    uint8_t flags, uint16_t age_ms, const float values[3],
    uint8_t *frame, size_t capacity, size_t *length) {
    if (length) *length = 0;
    if (!values || page > 3 || (flags & ~15U) ||
        ((flags & 1U) && (age_ms >= 250 || !(flags & 8U)))) return false;
    for (unsigned i=0; i<3; ++i) if (!isfinite(values[i])) return false;
    if (!begin_response(EXO_BLE_OPCODE_GET_BOARD_SENSOR | EXO_BLE_PROTOCOL_RESPONSE_BIT,
        request_id, 16U, true, frame, capacity, length)) return false;
    frame[4] = page; frame[5] = flags; write_u16_le(frame+6, age_ms);
    for (unsigned i=0; i<3; ++i) {
        uint32_t bits; memcpy(&bits, values+i, sizeof(bits));
        write_u32_le(frame+8+4*i, bits);
    }
    return true;
}

bool exo_ble_protocol_encode_can_status(uint8_t request_id,
    const exo_ble_can_status_t *status, uint8_t *frame, size_t capacity, size_t *length) {
    if (length != NULL) *length = 0U;
    if (status == NULL || status->state > 4U || (status->flags & ~3U) != 0U ||
        (status->motor_flags[0] & ~31U) != 0U || (status->motor_flags[1] & ~31U) != 0U)
        return false;
    if (!begin_response(EXO_BLE_RESPONSE_CAN_STATUS, request_id, 16U, true,
        frame, capacity, length)) return false;
    frame[4] = status->state;
    frame[5] = status->flags;
    write_u16_le(&frame[6], status->tx_error_counter);
    write_u16_le(&frame[8], status->rx_error_counter);
    write_u32_le(&frame[10], status->bus_error_count);
    write_u16_le(&frame[14], status->tx_failed_count);
    write_u16_le(&frame[16], status->rx_missed_count);
    frame[18] = status->motor_flags[0];
    frame[19] = status->motor_flags[1];
    return true;
}

/* 阈值读回携带配置代数，网页可核对它与0x07来自同一份保存值。 */
bool exo_ble_protocol_encode_angle_assist(uint8_t request_id,
    const exo_ble_wire_angle_assist_t *config, uint8_t *frame,
    size_t capacity, size_t *length) {
    if (length) *length = 0U;
    if (!config || (config->flags & ~3U) != 0U ||
        config->selected_cdeg > EXO_BLE_ANGLE_MAX_CDEG || config->active_cdeg > EXO_BLE_ANGLE_MAX_CDEG ||
        (!(config->flags & 1U) && config->selected_cdeg != 0U) ||
        (!(config->flags & 2U) && config->active_cdeg != 0U) ||
        ((config->flags & 2U) && !(config->flags & 1U))) return false;
    if (!begin_response(EXO_BLE_OPCODE_GET_ANGLE_ASSIST_CONFIG |
        EXO_BLE_PROTOCOL_RESPONSE_BIT, request_id, 12U, true,
        frame, capacity, length)) return false;
    frame[4] = config->pulse_model ? 2U : 1U; frame[5] = config->flags;
    write_u16_le(&frame[6], config->selected_cdeg);
    write_u16_le(&frame[8], config->active_cdeg);
    write_u16_le(&frame[10], config->pulse_model ? 0U : EXO_BLE_ANGLE_HYSTERESIS_CDEG);
    write_u16_le(&frame[12], config->pulse_model ? 0U : EXO_BLE_ANGLE_RAMP_MS);
    frame[14] = config->generation; frame[15] = 0U;
    return true;
}

bool exo_ble_protocol_encode_pulse_timing(uint8_t id,
    const exo_ble_wire_pulse_timing_t *t,uint8_t *frame,size_t cap,size_t *len) {
    /* 复用同一纯线解码校验，避免请求与响应的参数边界漂移。 */
    uint8_t check[20]={1,EXO_BLE_OPCODE_SET_PULSE_TIMING,1,16};
    if(len) *len=0;
    if(!t) return false;
    check[4]=t->schema_version;check[5]=t->duration_mode;
    write_u16_le(&check[6],t->peak_mnm);write_u16_le(&check[8],t->fixed_ms);
    write_u16_le(&check[10],t->ratio_permille);write_u16_le(&check[12],t->minimum_ms);
    write_u16_le(&check[14],t->maximum_ms);write_u16_le(&check[16],t->period_minimum_ms);
    write_u16_le(&check[18],t->period_maximum_ms);
    exo_ble_request_t parsed;
    if(exo_ble_protocol_decode_request(check,sizeof(check),&parsed)!=EXO_BLE_RESULT_OK ||
        !begin_response(EXO_BLE_OPCODE_GET_PULSE_TIMING|0x80,id,16,true,frame,cap,len)) return false;
    memcpy(&frame[4],&check[4],16);
    return true;
}
bool exo_ble_protocol_encode_pulse_window(uint8_t id,
    const exo_ble_wire_pulse_window_t *w,uint8_t *frame,size_t cap,size_t *len) {
    if(len) *len=0;
    if(!w || w->reset_cdeg< -3000 || w->reset_cdeg>5900 ||
        w->maximum_cdeg<300 || w->maximum_cdeg>9000 ||
        (int)w->maximum_cdeg-(int)w->reset_cdeg<300 ||
        !begin_response(EXO_BLE_OPCODE_GET_PULSE_WINDOW|0x80,id,8,true,frame,cap,len)) return false;
    frame[4]=1;frame[5]=0;
    write_i16_le(&frame[6],w->reset_cdeg);write_u16_le(&frame[8],w->maximum_cdeg);
    write_u16_le(&frame[10],500); /* 5 deg/s，只确认事件，不按速度持续给力。 */
    return true;
}
bool exo_ble_protocol_encode_pulse_status(uint8_t id,
    const exo_ble_wire_pulse_status_t *s,uint8_t *frame,size_t cap,size_t *len) {
    if(len) *len=0;
    if(!s || s->side>1 || s->state>4 || (s->flags&~7U) || s->exit_reason>7 ||
        s->duration_ms>EXO_BLE_PULSE_FIXED_MAX_MS || s->period_ms>5000 ||
        s->peak_mnm>EXO_BLE_ANGLE_TORQUE_HARD_MAX_MNM || s->progress_x10000>10000 ||
        !begin_response(EXO_BLE_OPCODE_GET_PULSE_STATUS|0x80,id,16,true,frame,cap,len)) return false;
    frame[4]=s->side;frame[5]=s->state;frame[6]=s->flags;frame[7]=s->exit_reason;
    write_u16_le(&frame[8],s->event_count);write_u16_le(&frame[10],s->pulse_count);
    write_u16_le(&frame[12],s->duration_ms);write_u16_le(&frame[14],s->period_ms);
    write_u16_le(&frame[16],s->peak_mnm);write_u16_le(&frame[18],s->progress_x10000);
    return true;
}
