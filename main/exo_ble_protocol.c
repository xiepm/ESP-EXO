#include "exo_ble_protocol.h"

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
	CONTROL_CONFIG_PAYLOAD_SIZE = 5,
	JOINT_COMMAND_PAYLOAD_SIZE = 15,
	GAIT_STATUS_PAYLOAD_SIZE = 16,
	ASSIST_CONFIG_PAYLOAD_SIZE = 16,
	ASSIST_REQUEST_PAYLOAD_SIZE = 8,
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
	case EXO_BLE_OPCODE_GET_SYSTEM_STATUS:
	case EXO_BLE_OPCODE_GET_CONTROL_CONFIG:
	case EXO_BLE_OPCODE_GET_GAIT_STATUS:
	case EXO_BLE_OPCODE_GET_ASSIST_CONFIG:
	case EXO_BLE_OPCODE_START:
	case EXO_BLE_OPCODE_STOP:
		*payload_size = 0U;
		return true;
	case EXO_BLE_OPCODE_GET_JOINT_STATUS:
	case EXO_BLE_OPCODE_GET_JOINT_COMMAND:
	case EXO_BLE_OPCODE_CAPTURE_ZERO:
	case EXO_BLE_OPCODE_CLEAR_ZERO:
		*payload_size = 1U;
		return true;
	case EXO_BLE_OPCODE_SET_CONTROL_CONFIG:
		*payload_size = 2U;
		return true;
	case EXO_BLE_OPCODE_SET_JOINT_COMMAND:
		*payload_size = JOINT_COMMAND_PAYLOAD_SIZE;
		return true;
	case EXO_BLE_OPCODE_SET_ASSIST_CONFIG:
		*payload_size = ASSIST_REQUEST_PAYLOAD_SIZE;
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
			reason <= EXO_BLE_BLOCK_SYSTEM_ERROR) ||
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
		   reference == EXO_BLE_REFERENCE_SINE;
}

static bool assist_mode_is_selectable(exo_ble_wire_assist_mode_t mode) {
	return mode >= EXO_BLE_ASSIST_MODE_TRANSPARENT &&
		   mode <= EXO_BLE_ASSIST_MODE_ASSIST_AS_NEEDED;
}

static bool assist_mode_is_snapshot_value(exo_ble_wire_assist_mode_t mode) {
	return mode >= EXO_BLE_ASSIST_MODE_NONE &&
		   mode <= EXO_BLE_ASSIST_MODE_ASSIST_AS_NEEDED;
}

static bool assist_execution_is_valid(exo_ble_wire_assist_execution_t state) {
	return state >= EXO_BLE_ASSIST_EXEC_DISABLED &&
		   state <= EXO_BLE_ASSIST_EXEC_ACTIVE;
}

static bool assist_strategy_is_valid(exo_ble_wire_assist_strategy_t strategy) {
	return strategy >= EXO_BLE_ASSIST_STRATEGY_NONE &&
		   strategy <= EXO_BLE_ASSIST_STRATEGY_AAN_RESIDUAL;
}

static bool assist_mode_matches_strategy(exo_ble_wire_assist_mode_t mode,
						 exo_ble_wire_assist_strategy_t strategy) {
	return (mode == EXO_BLE_ASSIST_MODE_TRANSPARENT &&
			strategy == EXO_BLE_ASSIST_STRATEGY_TRANSPARENT) ||
		   (mode == EXO_BLE_ASSIST_MODE_ACTIVE_PHASE &&
			strategy == EXO_BLE_ASSIST_STRATEGY_PHASE_PROFILE) ||
		   (mode == EXO_BLE_ASSIST_MODE_ASSIST_AS_NEEDED &&
			strategy == EXO_BLE_ASSIST_STRATEGY_AAN_RESIDUAL);
}

/*
 * 线协议先实施 RS01 Motion 的公开绝对边界；人体侧更窄的产品限制由业务层检查。
 * 这样格式层不会把越界数悄悄截断后交给电机驱动。
 */
static bool joint_command_is_valid(
	const exo_ble_wire_joint_command_t *command) {
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
		.control_config = {
			.selected_mode = EXO_BLE_CONTROL_MODE_NONE,
			.active_mode = EXO_BLE_CONTROL_MODE_NONE,
			.selected_reference = EXO_BLE_REFERENCE_CONSTANT,
			.active_reference = EXO_BLE_REFERENCE_CONSTANT,
		},
		.joint_command = {
			.side = EXO_BLE_SIDE_UNKNOWN,
		},
		.assist_config = {
			.selected_mode = EXO_BLE_ASSIST_MODE_NONE,
			.requested_execution_state = EXO_BLE_ASSIST_EXEC_DISABLED,
			.strategy_id = EXO_BLE_ASSIST_STRATEGY_NONE,
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

	if (request->opcode == EXO_BLE_OPCODE_GET_JOINT_STATUS ||
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
			.side =
				(exo_ble_wire_side_t)frame[FRAME_PAYLOAD_OFFSET + 0U],
			.position_mrad =
				read_i32_le(&frame[FRAME_PAYLOAD_OFFSET + 1U]),
			.velocity_mrad_s =
				read_i32_le(&frame[FRAME_PAYLOAD_OFFSET + 5U]),
			.torque_mnm =
				read_i16_le(&frame[FRAME_PAYLOAD_OFFSET + 9U]),
			.kp_x100 = read_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 11U]),
			.kd_x1000 = read_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 13U]),
		};
		if (!joint_command_is_valid(&command)) {
			return EXO_BLE_RESULT_INVALID_PAYLOAD;
		}
		request->side = command.side;
		request->joint_command = command;
	}

	if (request->opcode == EXO_BLE_OPCODE_SET_ASSIST_CONFIG) {
		const exo_ble_wire_assist_request_t config = {
			.schema_version = frame[FRAME_PAYLOAD_OFFSET + 0U],
			.selected_mode = (exo_ble_wire_assist_mode_t)
				frame[FRAME_PAYLOAD_OFFSET + 1U],
			.requested_execution_state =
				(exo_ble_wire_assist_execution_t)
					frame[FRAME_PAYLOAD_OFFSET + 2U],
			.strategy_id = (exo_ble_wire_assist_strategy_t)
				frame[FRAME_PAYLOAD_OFFSET + 3U],
			.profile_id = frame[FRAME_PAYLOAD_OFFSET + 4U],
			.left_level_percent = frame[FRAME_PAYLOAD_OFFSET + 5U],
			.right_level_percent = frame[FRAME_PAYLOAD_OFFSET + 6U],
			.reserved_flags = frame[FRAME_PAYLOAD_OFFSET + 7U],
		};
		/*
		 * ACTIVE是可识别的线码，但业务层会返回ACTIVE_NOT_ALLOWED；这样手机能区分
		 * “字段损坏”和“阶段9固件有意禁止真实助力”。
		 */
		if (config.schema_version != 1U ||
			!assist_mode_is_selectable(config.selected_mode) ||
			config.requested_execution_state > EXO_BLE_ASSIST_EXEC_ACTIVE ||
			!assist_strategy_is_valid(config.strategy_id) ||
			!assist_mode_matches_strategy(config.selected_mode,
								  config.strategy_id) ||
			config.left_level_percent > 100U ||
			config.right_level_percent > 100U ||
			config.reserved_flags != 0U) {
			return EXO_BLE_RESULT_INVALID_PAYLOAD;
		}
		request->assist_config = config;
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
		(!selected_valid && config->selected_mode != EXO_BLE_CONTROL_MODE_NONE) ||
		(active_valid && !selected_mode_is_valid(config->active_mode)) ||
		(!active_valid && config->active_mode != EXO_BLE_CONTROL_MODE_NONE) ||
		(active_valid && !selected_valid) || !active_mode_is_valid(config->active_mode)) {
		return false;
	}
	if (!begin_response(EXO_BLE_RESPONSE_CONTROL_CONFIG, request_id,
						CONTROL_CONFIG_PAYLOAD_SIZE, true, frame, frame_capacity,
						frame_len)) {
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
			 ? status->confidence_x10000 >
				   EXO_BLE_GAIT_CONFIDENCE_X10000_MAX
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
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 6U],
				 status->left_phase_x10000);
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 8U],
				 status->right_phase_x10000);
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 10U], status->cadence_x100);
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 12U],
				 status->confidence_x10000);
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
			EXO_BLE_ASSIST_UNAVAILABLE_REQUIRED_GAIT_INVALID) {
		return false;
	}
	const bool selected_valid =
		(config->assist_flags & EXO_BLE_ASSIST_FLAG_SELECTED_VALID) != 0U;
	const bool active_valid =
		(config->assist_flags & EXO_BLE_ASSIST_FLAG_ACTIVE_VALID) != 0U;
	const bool real_output =
		(config->assist_flags & EXO_BLE_ASSIST_FLAG_REAL_OUTPUT_ENABLED) != 0U;
	if ((selected_valid &&
		 (!assist_mode_is_selectable(config->selected_mode) ||
		  !assist_mode_matches_strategy(config->selected_mode,
								config->strategy_id))) ||
		(!selected_valid && config->selected_mode != EXO_BLE_ASSIST_MODE_NONE) ||
		(active_valid && !assist_mode_is_selectable(config->active_mode)) ||
		(!active_valid && config->active_mode != EXO_BLE_ASSIST_MODE_NONE) ||
		(active_valid && !selected_valid) ||
		(!active_valid &&
		 config->execution_state != EXO_BLE_ASSIST_EXEC_DISABLED) ||
		(!selected_valid && config->strategy_id != EXO_BLE_ASSIST_STRATEGY_NONE) ||
		(!selected_valid &&
		 config->requested_execution_state !=
			 EXO_BLE_ASSIST_EXEC_DISABLED) ||
		(real_output && config->execution_state != EXO_BLE_ASSIST_EXEC_ACTIVE)) {
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
	frame[FRAME_PAYLOAD_OFFSET + 10U] =
		(uint8_t)config->unavailable_reason;
	frame[FRAME_PAYLOAD_OFFSET + 11U] = config->config_generation;
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 12U],
				 config->required_sensor_mask);
	write_u16_le(&frame[FRAME_PAYLOAD_OFFSET + 14U],
				 config->available_sensor_mask);
	return true;
}
