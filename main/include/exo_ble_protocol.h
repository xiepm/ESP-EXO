#ifndef EXO_BLE_PROTOCOL_H
#define EXO_BLE_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Product BLE protocol v1 wire format:
 *
 *   byte 0  protocol version
 *   byte 1  opcode
 *   byte 2  request id (1..255 for phone requests)
 *   byte 3  payload length
 *   byte 4  payload
 *
 * A complete frame is carried by one GATT operation. Every v1 frame fits in
 * the 20-byte ATT payload available before MTU negotiation.
 */
#define EXO_BLE_PROTOCOL_VERSION 1U
#define EXO_BLE_PROTOCOL_HEADER_SIZE 4U
#define EXO_BLE_PROTOCOL_MAX_PAYLOAD_SIZE 16U
#define EXO_BLE_PROTOCOL_MAX_FRAME_SIZE 20U
#define EXO_BLE_PROTOCOL_RESPONSE_BIT 0x80U

typedef enum {
	EXO_BLE_OPCODE_GET_INFO = 0x01,
	EXO_BLE_OPCODE_GET_SYSTEM_STATUS = 0x02,
	EXO_BLE_OPCODE_GET_JOINT_STATUS = 0x03,
	EXO_BLE_OPCODE_START = 0x10,
	EXO_BLE_OPCODE_STOP = 0x11,
	EXO_BLE_OPCODE_CAPTURE_ZERO = 0x12,
	EXO_BLE_OPCODE_CLEAR_ZERO = 0x13,
} exo_ble_opcode_t;

typedef enum {
	EXO_BLE_RESPONSE_INFO =
		EXO_BLE_OPCODE_GET_INFO | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_SYSTEM_STATUS =
		EXO_BLE_OPCODE_GET_SYSTEM_STATUS | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_JOINT_STATUS =
		EXO_BLE_OPCODE_GET_JOINT_STATUS | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_START_ACK =
		EXO_BLE_OPCODE_START | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_STOP_ACK =
		EXO_BLE_OPCODE_STOP | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_CAPTURE_ZERO_ACK =
		EXO_BLE_OPCODE_CAPTURE_ZERO | EXO_BLE_PROTOCOL_RESPONSE_BIT,
	EXO_BLE_RESPONSE_CLEAR_ZERO_ACK =
		EXO_BLE_OPCODE_CLEAR_ZERO | EXO_BLE_PROTOCOL_RESPONSE_BIT,
} exo_ble_response_opcode_t;

/* Stable protocol result codes. These are not esp_err_t values. */
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

typedef enum {
	EXO_BLE_STATE_INIT = 0,
	EXO_BLE_STATE_IDLE = 1,
	EXO_BLE_STATE_STARTING = 2,
	EXO_BLE_STATE_RUNNING = 3,
	EXO_BLE_STATE_STOPPING = 4,
	EXO_BLE_STATE_ERROR = 5,
	EXO_BLE_STATE_UNKNOWN = 0xFF,
} exo_ble_wire_state_t;

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
	EXO_BLE_BLOCK_UNKNOWN = 0xFF,
} exo_ble_wire_block_reason_t;

typedef enum {
	EXO_BLE_SIDE_LEFT = 0,
	EXO_BLE_SIDE_RIGHT = 1,
	EXO_BLE_SIDE_UNKNOWN = 0xFF,
} exo_ble_wire_side_t;

typedef enum {
	EXO_BLE_DIRECTION_UNKNOWN = 0,
	EXO_BLE_DIRECTION_POSITIVE = 1,
	EXO_BLE_DIRECTION_NEGATIVE = 2,
} exo_ble_wire_direction_t;

enum {
	EXO_BLE_SYSTEM_FLAG_INITIALIZED = 1U << 0,
	EXO_BLE_SYSTEM_FLAG_READY_TO_START = 1U << 1,
	EXO_BLE_SYSTEM_FLAG_RUN_REQUESTED = 1U << 2,
	EXO_BLE_SYSTEM_FLAG_OUTPUT_ALLOWED = 1U << 3,
	EXO_BLE_SYSTEM_FLAG_MAINTENANCE_ACTIVE = 1U << 4,
	EXO_BLE_SYSTEM_FLAG_EXECUTOR_ALIVE = 1U << 5,
	EXO_BLE_SYSTEM_FLAGS_MASK = 0x3FU,
};

enum {
	EXO_BLE_JOINT_FLAG_MAPPING_VALID = 1U << 0,
	EXO_BLE_JOINT_FLAG_DIRECTION_VALID = 1U << 1,
	EXO_BLE_JOINT_FLAG_ZERO_VALID = 1U << 2,
	EXO_BLE_JOINT_FLAG_FEEDBACK_VALID = 1U << 3,
	EXO_BLE_JOINT_FLAG_ONLINE = 1U << 4,
	EXO_BLE_JOINT_FLAG_JOINT_VALID = 1U << 5,
	EXO_BLE_JOINT_FLAGS_MASK = 0x3FU,
};

enum {
	EXO_BLE_CAPABILITY_SYSTEM_CONTROL = 1U << 0,
	EXO_BLE_CAPABILITY_JOINT_CALIBRATION = 1U << 1,
	EXO_BLE_CAPABILITY_ON_DEMAND_STATUS = 1U << 2,
	EXO_BLE_CAPABILITIES_MASK = 0x0007U,
};

typedef struct {
	uint8_t version;
	uint8_t opcode;
	uint8_t request_id;
	uint8_t payload_len;
	exo_ble_wire_side_t side;
} exo_ble_request_t;

/* ACK payload (4 bytes): result, state, system_flags, detail. */
typedef struct {
	exo_ble_wire_result_t result;
	exo_ble_wire_state_t state;
	uint8_t system_flags;
	uint8_t detail;
} exo_ble_ack_t;

/*
 * INFO payload (12 bytes): protocol min/max, firmware major/minor/patch,
 * capabilities u16 LE, maximum frame size, boot id u32 LE.
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
 * SYSTEM_STATUS payload (10 bytes): state, active step, flags, start block,
 * last result, last failed step, transition count u32 LE.
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
 * JOINT_STATUS payload (13 bytes): side, flags, direction, position i32 LE in
 * mrad, velocity i32 LE in mrad/s, feedback age u16 LE in ms.
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
 * Decodes and validates one complete request frame. The output is cleared
 * first and contains any header bytes that were present even when validation
 * fails. EXO_BLE_RESULT_OK means the request is ready for dispatch.
 */
exo_ble_wire_result_t
exo_ble_protocol_decode_request(const uint8_t *frame, size_t frame_len,
								exo_ble_request_t *request);

/* Response encoders return false for invalid arguments or insufficient space.
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

#ifdef __cplusplus
}
#endif

#endif /* EXO_BLE_PROTOCOL_H */
