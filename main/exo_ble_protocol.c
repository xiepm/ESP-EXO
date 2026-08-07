#include "exo_ble_protocol.h"

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
};

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

static void write_u16_le(uint8_t *destination, uint16_t value) {
	destination[0] = (uint8_t)value;
	destination[1] = (uint8_t)(value >> 8);
}

static void write_u32_le(uint8_t *destination, uint32_t value) {
	destination[0] = (uint8_t)value;
	destination[1] = (uint8_t)(value >> 8);
	destination[2] = (uint8_t)(value >> 16);
	destination[3] = (uint8_t)(value >> 24);
}

static void write_i32_le(uint8_t *destination, int32_t value) {
	write_u32_le(destination, (uint32_t)value);
}

static bool request_payload_size(uint8_t opcode, uint8_t *payload_size) {
	if (payload_size == NULL) {
		return false;
	}

	switch (opcode) {
	case EXO_BLE_OPCODE_GET_INFO:
	case EXO_BLE_OPCODE_GET_SYSTEM_STATUS:
	case EXO_BLE_OPCODE_START:
	case EXO_BLE_OPCODE_STOP:
		*payload_size = 0U;
		return true;
	case EXO_BLE_OPCODE_GET_JOINT_STATUS:
	case EXO_BLE_OPCODE_CAPTURE_ZERO:
	case EXO_BLE_OPCODE_CLEAR_ZERO:
		*payload_size = 1U;
		return true;
	default:
		return false;
	}
}

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

static bool ack_result_is_valid(exo_ble_wire_result_t result) {
	return result != EXO_BLE_RESULT_NONE && result_is_valid(result);
}

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

exo_ble_wire_result_t
exo_ble_protocol_decode_request(const uint8_t *frame, size_t frame_len,
								exo_ble_request_t *request) {
	if (request == NULL) {
		return EXO_BLE_RESULT_INTERNAL_ERROR;
	}

	*request = (exo_ble_request_t){
		.side = EXO_BLE_SIDE_UNKNOWN,
	};
	if (frame == NULL) {
		return EXO_BLE_RESULT_INVALID_LENGTH;
	}

	/* Preserve every available header byte so the service can report a
	 * correlated protocol error whenever the malformed frame permits it. */
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

	if (expected_payload_size == 1U) {
		const uint8_t side = frame[FRAME_PAYLOAD_OFFSET];
		if (side != EXO_BLE_SIDE_LEFT && side != EXO_BLE_SIDE_RIGHT) {
			return EXO_BLE_RESULT_INVALID_PAYLOAD;
		}
		request->side = (exo_ble_wire_side_t)side;
	}

	return EXO_BLE_RESULT_OK;
}

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
