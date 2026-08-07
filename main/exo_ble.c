#include "exo_ble.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "exo_ble_protocol.h"
#include "exo_config.h"
#include "exo_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hip_joint.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define EXO_BLE_QUEUE_DEPTH 8U
#define EXO_BLE_CACHE_DEPTH 8U
#define EXO_BLE_TASK_STACK_BYTES 4096U
#define EXO_BLE_TASK_PRIORITY 5U
#define EXO_BLE_TASK_CORE 0
#define EXO_BLE_DEFAULT_ATT_MTU 23U
#define EXO_BLE_NOTIFY_RETRY_COUNT 3U
#define EXO_BLE_NOTIFY_RETRY_DELAY_MS 10U

#define EXO_FIRMWARE_VERSION_MAJOR 0U
#define EXO_FIRMWARE_VERSION_MINOR 7U
#define EXO_FIRMWARE_VERSION_PATCH 0U

static const char *TAG = "exo_ble";

/* UUIDs are stored in Bluetooth little-endian byte order. */
static const ble_uuid128_t s_product_service_uuid =
	BLE_UUID128_INIT(0x01, 0x00, 0x64, 0x8B, 0x1C, 0x3A, 0x7C, 0x9E, 0x92, 0x4D,
					 0x5A, 0x6B, 0x01, 0x00, 0xA1, 0xF7);
static const ble_uuid128_t s_command_uuid =
	BLE_UUID128_INIT(0x01, 0x00, 0x64, 0x8B, 0x1C, 0x3A, 0x7C, 0x9E, 0x92, 0x4D,
					 0x5A, 0x6B, 0x02, 0x00, 0xA1, 0xF7);
static const ble_uuid128_t s_response_uuid =
	BLE_UUID128_INIT(0x01, 0x00, 0x64, 0x8B, 0x1C, 0x3A, 0x7C, 0x9E, 0x92, 0x4D,
					 0x5A, 0x6B, 0x03, 0x00, 0xA1, 0xF7);

typedef struct {
	uint32_t session_generation;
	uint16_t connection_handle;
	uint8_t length;
	uint8_t bytes[EXO_BLE_PROTOCOL_MAX_FRAME_SIZE];
} exo_ble_rx_item_t;

typedef struct {
	bool valid;
	uint32_t session_generation;
	uint8_t request_id;
	uint8_t request_length;
	uint8_t request[EXO_BLE_PROTOCOL_MAX_FRAME_SIZE];
	uint8_t response_length;
	uint8_t response[EXO_BLE_PROTOCOL_MAX_FRAME_SIZE];
} exo_ble_cache_entry_t;

static StaticQueue_t s_queue_buffer;
static uint8_t s_queue_storage[EXO_BLE_QUEUE_DEPTH * sizeof(exo_ble_rx_item_t)];
static QueueHandle_t s_command_queue;

static StaticTask_t s_worker_task_buffer;
static StackType_t
	s_worker_task_stack[EXO_BLE_TASK_STACK_BYTES / sizeof(StackType_t)];
static TaskHandle_t s_worker_task;

static exo_ble_cache_entry_t s_cache[EXO_BLE_CACHE_DEPTH];
static size_t s_next_cache_entry;

static uint16_t s_command_value_handle;
static uint16_t s_response_value_handle;
static uint8_t s_own_addr_type = BLE_OWN_ADDR_PUBLIC;
static exo_ble_snapshot_t s_snapshot;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

_Static_assert(EXO_BLE_PROTOCOL_MAX_FRAME_SIZE <= 20U,
			   "phase 6 frames must fit the default ATT payload");
_Static_assert((EXO_BLE_TASK_STACK_BYTES % sizeof(StackType_t)) == 0U,
			   "BLE worker stack size must align to StackType_t");

static exo_ble_wire_state_t wire_state(exo_system_state_t state) {
	switch (state) {
	case EXO_SYSTEM_STATE_INIT:
		return EXO_BLE_STATE_INIT;
	case EXO_SYSTEM_STATE_IDLE:
		return EXO_BLE_STATE_IDLE;
	case EXO_SYSTEM_STATE_STARTING:
		return EXO_BLE_STATE_STARTING;
	case EXO_SYSTEM_STATE_RUNNING:
		return EXO_BLE_STATE_RUNNING;
	case EXO_SYSTEM_STATE_STOPPING:
		return EXO_BLE_STATE_STOPPING;
	case EXO_SYSTEM_STATE_ERROR:
		return EXO_BLE_STATE_ERROR;
	default:
		return EXO_BLE_STATE_UNKNOWN;
	}
}

static exo_ble_wire_step_t wire_step(exo_system_step_t step) {
	switch (step) {
	case EXO_SYSTEM_STEP_NONE:
		return EXO_BLE_STEP_NONE;
	case EXO_SYSTEM_STEP_CHECK_PRECONDITIONS:
		return EXO_BLE_STEP_CHECK_PRECONDITIONS;
	case EXO_SYSTEM_STEP_DISABLE_MOTORS:
		return EXO_BLE_STEP_DISABLE_MOTORS;
	case EXO_SYSTEM_STEP_SET_MOTION_MODE:
		return EXO_BLE_STEP_SET_MOTION_MODE;
	case EXO_SYSTEM_STEP_ENABLE_MOTORS:
		return EXO_BLE_STEP_ENABLE_MOTORS;
	case EXO_SYSTEM_STEP_SEND_ZERO_OUTPUT:
		return EXO_BLE_STEP_SEND_ZERO_OUTPUT;
	case EXO_SYSTEM_STEP_WAIT_FRESH_FEEDBACK:
		return EXO_BLE_STEP_WAIT_FRESH_FEEDBACK;
	case EXO_SYSTEM_STEP_ARM_CONTROLLER:
		return EXO_BLE_STEP_ARM_CONTROLLER;
	case EXO_SYSTEM_STEP_DISARM_CONTROLLER:
		return EXO_BLE_STEP_DISARM_CONTROLLER;
	case EXO_SYSTEM_STEP_RELEASE_OUTPUT:
		return EXO_BLE_STEP_RELEASE_OUTPUT;
	case EXO_SYSTEM_STEP_VERIFY_DISABLED:
		return EXO_BLE_STEP_VERIFY_DISABLED;
	default:
		return EXO_BLE_STEP_UNKNOWN;
	}
}

static exo_ble_wire_block_reason_t
wire_block_reason(exo_start_block_reason_t reason) {
	switch (reason) {
	case EXO_START_BLOCK_NONE:
		return EXO_BLE_BLOCK_NONE;
	case EXO_START_BLOCK_INITIALIZING:
		return EXO_BLE_BLOCK_INITIALIZING;
	case EXO_START_BLOCK_SYSTEM_NOT_IDLE:
		return EXO_BLE_BLOCK_SYSTEM_NOT_IDLE;
	case EXO_START_BLOCK_OPERATION_BUSY:
		return EXO_BLE_BLOCK_OPERATION_BUSY;
	case EXO_START_BLOCK_CONTROL_TASK_NOT_READY:
		return EXO_BLE_BLOCK_CONTROL_TASK_NOT_READY;
	case EXO_START_BLOCK_TWAI_NOT_RUNNING:
		return EXO_BLE_BLOCK_TWAI_NOT_RUNNING;
	case EXO_START_BLOCK_LEFT_MAPPING_INVALID:
		return EXO_BLE_BLOCK_LEFT_MAPPING_INVALID;
	case EXO_START_BLOCK_RIGHT_MAPPING_INVALID:
		return EXO_BLE_BLOCK_RIGHT_MAPPING_INVALID;
	case EXO_START_BLOCK_LEFT_DIRECTION_INVALID:
		return EXO_BLE_BLOCK_LEFT_DIRECTION_INVALID;
	case EXO_START_BLOCK_RIGHT_DIRECTION_INVALID:
		return EXO_BLE_BLOCK_RIGHT_DIRECTION_INVALID;
	case EXO_START_BLOCK_LEFT_ZERO_INVALID:
		return EXO_BLE_BLOCK_LEFT_ZERO_INVALID;
	case EXO_START_BLOCK_RIGHT_ZERO_INVALID:
		return EXO_BLE_BLOCK_RIGHT_ZERO_INVALID;
	case EXO_START_BLOCK_LEFT_FEEDBACK_INVALID:
		return EXO_BLE_BLOCK_LEFT_FEEDBACK_INVALID;
	case EXO_START_BLOCK_RIGHT_FEEDBACK_INVALID:
		return EXO_BLE_BLOCK_RIGHT_FEEDBACK_INVALID;
	case EXO_START_BLOCK_LEFT_OFFLINE:
		return EXO_BLE_BLOCK_LEFT_OFFLINE;
	case EXO_START_BLOCK_RIGHT_OFFLINE:
		return EXO_BLE_BLOCK_RIGHT_OFFLINE;
	case EXO_START_BLOCK_LEFT_MOTOR_FAULT:
		return EXO_BLE_BLOCK_LEFT_MOTOR_FAULT;
	case EXO_START_BLOCK_RIGHT_MOTOR_FAULT:
		return EXO_BLE_BLOCK_RIGHT_MOTOR_FAULT;
	case EXO_START_BLOCK_SYSTEM_ERROR:
		return EXO_BLE_BLOCK_SYSTEM_ERROR;
	default:
		return EXO_BLE_BLOCK_UNKNOWN;
	}
}

static exo_ble_wire_result_t wire_request_result(exo_request_result_t result) {
	switch (result) {
	case EXO_REQUEST_NONE:
		return EXO_BLE_RESULT_NONE;
	case EXO_REQUEST_ACCEPTED:
		return EXO_BLE_RESULT_ACCEPTED;
	case EXO_REQUEST_ALREADY_SATISFIED:
		return EXO_BLE_RESULT_ALREADY_SATISFIED;
	case EXO_REQUEST_ALREADY_PENDING:
		return EXO_BLE_RESULT_ALREADY_PENDING;
	case EXO_REQUEST_REJECTED_NOT_READY:
		return EXO_BLE_RESULT_REJECTED_NOT_READY;
	case EXO_REQUEST_REJECTED_BUSY:
		return EXO_BLE_RESULT_REJECTED_BUSY;
	case EXO_REQUEST_REJECTED_ERROR:
		return EXO_BLE_RESULT_REJECTED_ERROR;
	case EXO_REQUEST_INTERNAL_ERROR:
	default:
		return EXO_BLE_RESULT_INTERNAL_ERROR;
	}
}

static uint8_t system_flags(const exo_system_snapshot_t *system) {
	uint8_t flags = 0U;
	if (system->initialized) {
		flags |= EXO_BLE_SYSTEM_FLAG_INITIALIZED;
	}
	if (system->ready_to_start) {
		flags |= EXO_BLE_SYSTEM_FLAG_READY_TO_START;
	}
	if (system->run_requested) {
		flags |= EXO_BLE_SYSTEM_FLAG_RUN_REQUESTED;
	}
	if (system->output_allowed) {
		flags |= EXO_BLE_SYSTEM_FLAG_OUTPUT_ALLOWED;
	}
	if (system->maintenance_active) {
		flags |= EXO_BLE_SYSTEM_FLAG_MAINTENANCE_ACTIVE;
	}
	if (system->executor_task_alive) {
		flags |= EXO_BLE_SYSTEM_FLAG_EXECUTOR_ALIVE;
	}
	return flags;
}

static bool get_system_snapshot(exo_system_snapshot_t *system) {
	if (system == NULL || !exo_system_get_snapshot(system)) {
		if (system != NULL) {
			memset(system, 0, sizeof(*system));
			system->state = EXO_SYSTEM_STATE_INIT;
		}
		return false;
	}
	return true;
}

static exo_ble_ack_t make_ack(exo_ble_wire_result_t result, uint8_t detail) {
	exo_system_snapshot_t system;
	const bool valid = get_system_snapshot(&system);
	return (exo_ble_ack_t){
		.result = result,
		.state = valid ? wire_state(system.state) : EXO_BLE_STATE_UNKNOWN,
		.system_flags = valid ? system_flags(&system) : 0U,
		.detail = detail,
	};
}

static bool encode_ack(uint8_t opcode, uint8_t request_id,
					   exo_ble_wire_result_t result, uint8_t detail,
					   uint8_t *response, size_t *response_len) {
	const exo_ble_ack_t ack = make_ack(result, detail);
	const uint8_t safe_opcode =
		opcode & (uint8_t)~EXO_BLE_PROTOCOL_RESPONSE_BIT;
	return exo_ble_protocol_encode_ack(safe_opcode, request_id, &ack, response,
									   EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
									   response_len);
}

static exo_hip_joint_id_t joint_from_wire_side(exo_ble_wire_side_t side) {
	return side == EXO_BLE_SIDE_LEFT ? EXO_HIP_JOINT_LEFT : EXO_HIP_JOINT_RIGHT;
}

static exo_ble_wire_direction_t wire_direction(exo_direction_t direction) {
	if (direction == EXO_DIRECTION_POSITIVE) {
		return EXO_BLE_DIRECTION_POSITIVE;
	}
	if (direction == EXO_DIRECTION_NEGATIVE) {
		return EXO_BLE_DIRECTION_NEGATIVE;
	}
	return EXO_BLE_DIRECTION_UNKNOWN;
}

static int32_t scaled_i32(float value, float scale) {
	if (!isfinite(value) || !isfinite(scale)) {
		return 0;
	}
	const double scaled = (double)value * (double)scale;
	if (scaled >= (double)INT32_MAX) {
		return INT32_MAX;
	}
	if (scaled <= (double)INT32_MIN) {
		return INT32_MIN;
	}
	return (int32_t)llround(scaled);
}

static uint16_t feedback_age_ms(uint64_t timestamp_us) {
	if (timestamp_us == 0U) {
		return UINT16_MAX;
	}
	const int64_t now_us = esp_timer_get_time();
	if (now_us <= 0 || (uint64_t)now_us <= timestamp_us) {
		return 0U;
	}
	const uint64_t age_ms = ((uint64_t)now_us - timestamp_us) / 1000U;
	return age_ms > UINT16_MAX ? UINT16_MAX : (uint16_t)age_ms;
}

static bool encode_info_response(uint8_t request_id, uint8_t *response,
								 size_t *response_len) {
	portENTER_CRITICAL(&s_lock);
	const uint32_t boot_id = s_snapshot.boot_id;
	portEXIT_CRITICAL(&s_lock);

	const exo_ble_info_t info = {
		.protocol_min_version = EXO_BLE_PROTOCOL_VERSION,
		.protocol_max_version = EXO_BLE_PROTOCOL_VERSION,
		.firmware_major = EXO_FIRMWARE_VERSION_MAJOR,
		.firmware_minor = EXO_FIRMWARE_VERSION_MINOR,
		.firmware_patch = EXO_FIRMWARE_VERSION_PATCH,
		.capabilities = EXO_BLE_CAPABILITY_SYSTEM_CONTROL |
						EXO_BLE_CAPABILITY_JOINT_CALIBRATION |
						EXO_BLE_CAPABILITY_ON_DEMAND_STATUS,
		.max_frame_size = EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
		.boot_id = boot_id,
	};
	return exo_ble_protocol_encode_info(request_id, &info, response,
										EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
										response_len);
}

static bool encode_system_response(uint8_t request_id, uint8_t *response,
								   size_t *response_len) {
	exo_system_snapshot_t system;
	if (!get_system_snapshot(&system)) {
		return encode_ack(EXO_BLE_OPCODE_GET_SYSTEM_STATUS, request_id,
						  EXO_BLE_RESULT_INTERNAL_ERROR, 0U, response,
						  response_len);
	}
	const exo_ble_system_status_t status = {
		.state = wire_state(system.state),
		.active_step = wire_step(system.active_step),
		.system_flags = system_flags(&system),
		.start_block_reason = wire_block_reason(system.start_block_reason),
		.last_request_result = wire_request_result(system.last_request_result),
		.last_failed_step = wire_step(system.last_failed_step),
		.transition_count = system.transition_count,
	};
	return exo_ble_protocol_encode_system_status(
		request_id, &status, response, EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
		response_len);
}

static bool encode_joint_response(const exo_ble_request_t *request,
								  uint8_t *response, size_t *response_len) {
	hip_joint_snapshot_t joint;
	if (!hip_joint_get_snapshot(joint_from_wire_side(request->side), &joint)) {
		return encode_ack(request->opcode, request->request_id,
						  EXO_BLE_RESULT_INTERNAL_ERROR, 0U, response,
						  response_len);
	}

	uint8_t flags = 0U;
	if (joint.mapping_valid) {
		flags |= EXO_BLE_JOINT_FLAG_MAPPING_VALID;
	}
	if (joint.direction_valid) {
		flags |= EXO_BLE_JOINT_FLAG_DIRECTION_VALID;
	}
	if (joint.zero_valid) {
		flags |= EXO_BLE_JOINT_FLAG_ZERO_VALID;
	}
	if (joint.feedback_valid) {
		flags |= EXO_BLE_JOINT_FLAG_FEEDBACK_VALID;
	}
	if (joint.online) {
		flags |= EXO_BLE_JOINT_FLAG_ONLINE;
	}
	if (joint.joint_valid) {
		flags |= EXO_BLE_JOINT_FLAG_JOINT_VALID;
	}

	const exo_ble_joint_status_t status = {
		.side = request->side,
		.joint_flags = flags,
		.direction = wire_direction(joint.direction),
		.position_mrad =
			joint.joint_valid
				? scaled_i32(joint.joint_sample.position_rad, 1000.0f)
				: 0,
		.velocity_mrad_s =
			joint.joint_valid
				? scaled_i32(joint.joint_sample.velocity_rad_s, 1000.0f)
				: 0,
		.feedback_age_ms = feedback_age_ms(joint.feedback_timestamp_us),
	};
	return exo_ble_protocol_encode_joint_status(
		request->request_id, &status, response, EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
		response_len);
}

static exo_ble_wire_result_t
execute_zero_command(const exo_ble_request_t *request, bool capture) {
	if (!exo_system_try_begin_maintenance()) {
		return EXO_BLE_RESULT_REJECTED_BUSY;
	}

	const exo_hip_joint_id_t joint = joint_from_wire_side(request->side);
	const esp_err_t error = capture ? hip_joint_set_zero_from_current(joint)
									: hip_joint_clear_zero(joint);
	exo_system_end_maintenance();

	if (error == ESP_OK) {
		return EXO_BLE_RESULT_OK;
	}
	if (error == ESP_ERR_INVALID_STATE) {
		return EXO_BLE_RESULT_REJECTED_NOT_READY;
	}
	return EXO_BLE_RESULT_INTERNAL_ERROR;
}

static bool dispatch_request(const exo_ble_request_t *request,
							 uint8_t *response, size_t *response_len) {
	switch (request->opcode) {
	case EXO_BLE_OPCODE_GET_INFO:
		return encode_info_response(request->request_id, response,
									response_len);
	case EXO_BLE_OPCODE_GET_SYSTEM_STATUS:
		return encode_system_response(request->request_id, response,
									  response_len);
	case EXO_BLE_OPCODE_GET_JOINT_STATUS:
		return encode_joint_response(request, response, response_len);
	case EXO_BLE_OPCODE_START: {
		const exo_request_result_t result = exo_system_request_start();
		uint8_t detail = (uint8_t)EXO_BLE_BLOCK_NONE;
		if (result == EXO_REQUEST_REJECTED_NOT_READY ||
			result == EXO_REQUEST_REJECTED_BUSY ||
			result == EXO_REQUEST_REJECTED_ERROR ||
			result == EXO_REQUEST_INTERNAL_ERROR) {
			exo_system_snapshot_t system;
			if (get_system_snapshot(&system)) {
				detail = (uint8_t)wire_block_reason(system.start_block_reason);
			}
		}
		return encode_ack(request->opcode, request->request_id,
						  wire_request_result(result), detail, response,
						  response_len);
	}
	case EXO_BLE_OPCODE_STOP:
		return encode_ack(request->opcode, request->request_id,
						  wire_request_result(exo_system_request_stop()), 0U,
						  response, response_len);
	case EXO_BLE_OPCODE_CAPTURE_ZERO:
	case EXO_BLE_OPCODE_CLEAR_ZERO: {
		const bool capture = request->opcode == EXO_BLE_OPCODE_CAPTURE_ZERO;
		return encode_ack(request->opcode, request->request_id,
						  execute_zero_command(request, capture), 0U, response,
						  response_len);
	}
	default:
		return encode_ack(request->opcode, request->request_id,
						  EXO_BLE_RESULT_UNKNOWN_OPCODE, 0U, response,
						  response_len);
	}
}

static bool session_is_current(uint32_t generation, uint16_t connection,
							   bool require_subscription) {
	portENTER_CRITICAL(&s_lock);
	const bool current =
		s_snapshot.connected && s_snapshot.session_generation == generation &&
		s_snapshot.connection_handle == connection &&
		(!require_subscription || s_snapshot.response_subscribed);
	portEXIT_CRITICAL(&s_lock);
	return current;
}

static void record_notify_result(bool success) {
	portENTER_CRITICAL(&s_lock);
	if (success) {
		++s_snapshot.notify_success_count;
	} else {
		++s_snapshot.notify_failure_count;
	}
	portEXIT_CRITICAL(&s_lock);
}

static bool notify_response(uint32_t generation, uint16_t connection,
							const uint8_t *response, uint8_t response_len) {
	if (response == NULL || response_len == 0U ||
		response_len > EXO_BLE_PROTOCOL_MAX_FRAME_SIZE) {
		record_notify_result(false);
		return false;
	}

	int rc = BLE_HS_ENOMEM;
	for (unsigned attempt = 0U; attempt < EXO_BLE_NOTIFY_RETRY_COUNT;
		 ++attempt) {
		if (!session_is_current(generation, connection, true)) {
			record_notify_result(false);
			return false;
		}
		struct os_mbuf *packet = ble_hs_mbuf_from_flat(response, response_len);
		if (packet == NULL) {
			rc = BLE_HS_ENOMEM;
		} else {
			rc = ble_gatts_notify_custom(connection, s_response_value_handle,
										 packet);
		}
		if (rc == 0) {
			record_notify_result(true);
			return true;
		}
		if (attempt + 1U < EXO_BLE_NOTIFY_RETRY_COUNT) {
			vTaskDelay(pdMS_TO_TICKS(EXO_BLE_NOTIFY_RETRY_DELAY_MS));
		}
	}

	record_notify_result(false);
	ESP_LOGW(TAG, "response notify failed: %d", rc);
	return false;
}

static exo_ble_cache_entry_t *find_cache_entry(uint32_t generation,
											   uint8_t request_id) {
	for (size_t i = 0U; i < EXO_BLE_CACHE_DEPTH; ++i) {
		if (s_cache[i].valid && s_cache[i].session_generation == generation &&
			s_cache[i].request_id == request_id) {
			return &s_cache[i];
		}
	}
	return NULL;
}

static void cache_response(const exo_ble_rx_item_t *item, uint8_t request_id,
						   const uint8_t *response, uint8_t response_len) {
	exo_ble_cache_entry_t *entry = &s_cache[s_next_cache_entry];
	s_next_cache_entry = (s_next_cache_entry + 1U) % EXO_BLE_CACHE_DEPTH;
	*entry = (exo_ble_cache_entry_t){
		.valid = true,
		.session_generation = item->session_generation,
		.request_id = request_id,
		.request_length = item->length,
		.response_length = response_len,
	};
	memcpy(entry->request, item->bytes, item->length);
	memcpy(entry->response, response, response_len);
}

static bool cache_request_matches(const exo_ble_cache_entry_t *entry,
								  const exo_ble_rx_item_t *item) {
	return entry->request_length == item->length &&
		   memcmp(entry->request, item->bytes, item->length) == 0;
}

static void worker_task(void *arg) {
	(void)arg;
	portENTER_CRITICAL(&s_lock);
	s_snapshot.worker_task_alive = true;
	portEXIT_CRITICAL(&s_lock);

	for (;;) {
		exo_ble_rx_item_t item;
		if (xQueueReceive(s_command_queue, &item, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		portENTER_CRITICAL(&s_lock);
		++s_snapshot.processed_count;
		s_snapshot.worker_stack_free_min_bytes =
			(uint32_t)uxTaskGetStackHighWaterMark(NULL);
		portEXIT_CRITICAL(&s_lock);

		if (!session_is_current(item.session_generation, item.connection_handle,
								true)) {
			portENTER_CRITICAL(&s_lock);
			++s_snapshot.stale_drop_count;
			portEXIT_CRITICAL(&s_lock);
			continue;
		}

		exo_ble_request_t request;
		const exo_ble_wire_result_t decode_result =
			exo_ble_protocol_decode_request(item.bytes, item.length, &request);
		uint8_t response[EXO_BLE_PROTOCOL_MAX_FRAME_SIZE] = {0};
		size_t response_size = 0U;

		if (decode_result != EXO_BLE_RESULT_OK) {
			portENTER_CRITICAL(&s_lock);
			++s_snapshot.malformed_count;
			portEXIT_CRITICAL(&s_lock);
			if (encode_ack(request.opcode, request.request_id, decode_result,
						   0U, response, &response_size)) {
				(void)notify_response(item.session_generation,
									  item.connection_handle, response,
									  (uint8_t)response_size);
			}
			continue;
		}

		exo_ble_cache_entry_t *cached =
			find_cache_entry(item.session_generation, request.request_id);
		if (cached != NULL) {
			if (cache_request_matches(cached, &item)) {
				portENTER_CRITICAL(&s_lock);
				++s_snapshot.duplicate_count;
				portEXIT_CRITICAL(&s_lock);
				(void)notify_response(item.session_generation,
									  item.connection_handle, cached->response,
									  cached->response_length);
			} else {
				portENTER_CRITICAL(&s_lock);
				++s_snapshot.request_id_conflict_count;
				portEXIT_CRITICAL(&s_lock);
				if (encode_ack(request.opcode, request.request_id,
							   EXO_BLE_RESULT_REQUEST_ID_CONFLICT, 0U, response,
							   &response_size)) {
					(void)notify_response(item.session_generation,
										  item.connection_handle, response,
										  (uint8_t)response_size);
				}
			}
			continue;
		}

		if (!dispatch_request(&request, response, &response_size) ||
			response_size == 0U ||
			response_size > EXO_BLE_PROTOCOL_MAX_FRAME_SIZE) {
			response_size = 0U;
			(void)encode_ack(request.opcode, request.request_id,
							 EXO_BLE_RESULT_INTERNAL_ERROR, 0U, response,
							 &response_size);
		}

		if (response_size > 0U) {
			cache_response(&item, request.request_id, response,
						   (uint8_t)response_size);
			(void)notify_response(item.session_generation,
								  item.connection_handle, response,
								  (uint8_t)response_size);
		}
	}
}

static int command_access(uint16_t connection_handle, uint16_t attr_handle,
						  struct ble_gatt_access_ctxt *context, void *arg) {
	(void)attr_handle;
	(void)arg;
	if (context == NULL || context->op != BLE_GATT_ACCESS_OP_WRITE_CHR ||
		context->om == NULL) {
		return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
	}

	const uint16_t total_length = OS_MBUF_PKTLEN(context->om);
	portENTER_CRITICAL(&s_lock);
	++s_snapshot.rx_write_count;
	const bool accepted = s_snapshot.connected &&
						  s_snapshot.response_subscribed &&
						  connection_handle == s_snapshot.connection_handle;
	const uint32_t generation = s_snapshot.session_generation;
	portEXIT_CRITICAL(&s_lock);

	if (!accepted) {
		return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
	}
	if (total_length > EXO_BLE_PROTOCOL_MAX_FRAME_SIZE) {
		portENTER_CRITICAL(&s_lock);
		++s_snapshot.malformed_count;
		portEXIT_CRITICAL(&s_lock);
		return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
	}

	exo_ble_rx_item_t item = {
		.session_generation = generation,
		.connection_handle = connection_handle,
		.length = (uint8_t)total_length,
	};
	uint16_t copied_length = 0U;
	const int rc = ble_hs_mbuf_to_flat(context->om, item.bytes,
									   sizeof(item.bytes), &copied_length);
	if (rc != 0 || copied_length != total_length) {
		return BLE_ATT_ERR_UNLIKELY;
	}
	if (xQueueSend(s_command_queue, &item, 0) != pdTRUE) {
		portENTER_CRITICAL(&s_lock);
		++s_snapshot.queue_full_count;
		portEXIT_CRITICAL(&s_lock);
		return BLE_ATT_ERR_INSUFFICIENT_RES;
	}

	portENTER_CRITICAL(&s_lock);
	++s_snapshot.queued_count;
	portEXIT_CRITICAL(&s_lock);
	return 0;
}

static int response_access(uint16_t connection_handle, uint16_t attr_handle,
						   struct ble_gatt_access_ctxt *context, void *arg) {
	(void)connection_handle;
	(void)attr_handle;
	(void)context;
	(void)arg;
	return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
}

static const struct ble_gatt_chr_def s_product_characteristics[] = {
	{
		.uuid = &s_command_uuid.u,
		.access_cb = command_access,
		.flags = BLE_GATT_CHR_F_WRITE,
		.val_handle = &s_command_value_handle,
	},
	{
		.uuid = &s_response_uuid.u,
		.access_cb = response_access,
		.flags = BLE_GATT_CHR_F_NOTIFY,
		.val_handle = &s_response_value_handle,
	},
	{0},
};

static const struct ble_gatt_svc_def s_product_services[] = {
	{
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &s_product_service_uuid.u,
		.characteristics = s_product_characteristics,
	},
	{0},
};

static int register_gatt_services(void) {
	ble_svc_gap_init();
	ble_svc_gatt_init();
	int rc = ble_gatts_count_cfg(s_product_services);
	if (rc == 0) {
		rc = ble_gatts_add_svcs(s_product_services);
	}
	return rc;
}

static void start_advertising(void);

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
	(void)arg;
	switch (event->type) {
	case BLE_GAP_EVENT_CONNECT:
		if (event->connect.status == 0) {
			portENTER_CRITICAL(&s_lock);
			++s_snapshot.session_generation;
			s_snapshot.connected = true;
			s_snapshot.response_subscribed = false;
			s_snapshot.connection_handle = event->connect.conn_handle;
			s_snapshot.negotiated_mtu = EXO_BLE_DEFAULT_ATT_MTU;
			portEXIT_CRITICAL(&s_lock);
			ESP_LOGI(TAG, "product client connected");
		} else {
			ESP_LOGW(TAG, "connection failed: %d", event->connect.status);
			start_advertising();
		}
		break;
	case BLE_GAP_EVENT_DISCONNECT:
		ESP_LOGI(TAG, "product client disconnected: %d",
				 event->disconnect.reason);
		portENTER_CRITICAL(&s_lock);
		if (s_snapshot.connection_handle ==
			event->disconnect.conn.conn_handle) {
			++s_snapshot.session_generation;
			s_snapshot.connected = false;
			s_snapshot.response_subscribed = false;
			s_snapshot.connection_handle = BLE_HS_CONN_HANDLE_NONE;
			s_snapshot.negotiated_mtu = EXO_BLE_DEFAULT_ATT_MTU;
		}
		portEXIT_CRITICAL(&s_lock);
		start_advertising();
		break;
	case BLE_GAP_EVENT_SUBSCRIBE:
		if (event->subscribe.attr_handle == s_response_value_handle) {
			portENTER_CRITICAL(&s_lock);
			if (s_snapshot.connected &&
				s_snapshot.connection_handle == event->subscribe.conn_handle) {
				s_snapshot.response_subscribed =
					event->subscribe.cur_notify > 0;
			}
			portEXIT_CRITICAL(&s_lock);
		}
		break;
	case BLE_GAP_EVENT_MTU:
		portENTER_CRITICAL(&s_lock);
		if (s_snapshot.connected &&
			s_snapshot.connection_handle == event->mtu.conn_handle) {
			s_snapshot.negotiated_mtu = event->mtu.value;
		}
		portEXIT_CRITICAL(&s_lock);
		break;
	case BLE_GAP_EVENT_NOTIFY_TX:
		if (event->notify_tx.attr_handle == s_response_value_handle &&
			event->notify_tx.status != 0) {
			portENTER_CRITICAL(&s_lock);
			++s_snapshot.notify_tx_failure_count;
			portEXIT_CRITICAL(&s_lock);
		}
		break;
	case BLE_GAP_EVENT_ADV_COMPLETE:
		start_advertising();
		break;
	default:
		break;
	}
	return 0;
}

static void start_advertising(void) {
	struct ble_hs_adv_fields advertising = {0};
	advertising.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
	advertising.uuids128 = (ble_uuid128_t *)&s_product_service_uuid;
	advertising.num_uuids128 = 1;
	advertising.uuids128_is_complete = 1;
	int rc = ble_gap_adv_set_fields(&advertising);
	if (rc != 0) {
		ESP_LOGE(TAG, "failed to set advertising fields: %d", rc);
		return;
	}

	struct ble_hs_adv_fields scan_response = {0};
	scan_response.name = (uint8_t *)g_exo_hardware_config.ble_device_name;
	scan_response.name_len =
		(uint8_t)strlen(g_exo_hardware_config.ble_device_name);
	scan_response.name_is_complete = 1;
	rc = ble_gap_adv_rsp_set_fields(&scan_response);
	if (rc != 0) {
		ESP_LOGE(TAG, "failed to set scan response: %d", rc);
		return;
	}

	struct ble_gap_adv_params parameters = {0};
	parameters.conn_mode = BLE_GAP_CONN_MODE_UND;
	parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;
	rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &parameters,
						   gap_event_handler, NULL);
	if (rc != 0) {
		ESP_LOGE(TAG, "failed to start advertising: %d", rc);
	}
}

static void on_reset(int reason) {
	ESP_LOGW(TAG, "NimBLE host reset: %d", reason);
	portENTER_CRITICAL(&s_lock);
	++s_snapshot.session_generation;
	s_snapshot.host_synced = false;
	s_snapshot.connected = false;
	s_snapshot.response_subscribed = false;
	s_snapshot.connection_handle = BLE_HS_CONN_HANDLE_NONE;
	s_snapshot.negotiated_mtu = EXO_BLE_DEFAULT_ATT_MTU;
	portEXIT_CRITICAL(&s_lock);
}

static void on_sync(void) {
	int rc = ble_hs_util_ensure_addr(0);
	if (rc == 0) {
		rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
	}
	if (rc != 0) {
		ESP_LOGE(TAG, "failed to infer BLE address: %d", rc);
		return;
	}
	portENTER_CRITICAL(&s_lock);
	s_snapshot.host_synced = true;
	portEXIT_CRITICAL(&s_lock);
	start_advertising();
}

static void host_task(void *arg) {
	(void)arg;
	nimble_port_run();
	nimble_port_freertos_deinit();
}

static esp_err_t initialize_nvs(void) {
	esp_err_t error = nvs_flash_init();
	if (error == ESP_ERR_NVS_NO_FREE_PAGES ||
		error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		error = nvs_flash_erase();
		if (error == ESP_OK) {
			error = nvs_flash_init();
		}
	}
	return error;
}

esp_err_t exo_ble_init(void) {
	portENTER_CRITICAL(&s_lock);
	const bool already_initialized = s_snapshot.initialized;
	portEXIT_CRITICAL(&s_lock);
	if (already_initialized) {
		return ESP_OK;
	}

	esp_err_t error = initialize_nvs();
	if (error != ESP_OK) {
		return error;
	}
	error = nimble_port_init();
	if (error != ESP_OK) {
		return error;
	}

	ble_hs_cfg.reset_cb = on_reset;
	ble_hs_cfg.sync_cb = on_sync;
	const int gatt_error = register_gatt_services();
	if (gatt_error != 0) {
		ESP_LOGE(TAG, "GATT registration failed: %d", gatt_error);
		return ESP_FAIL;
	}
	const int name_error =
		ble_svc_gap_device_name_set(g_exo_hardware_config.ble_device_name);
	if (name_error != 0) {
		ESP_LOGE(TAG, "device name failed: %d", name_error);
		return ESP_FAIL;
	}

	s_command_queue =
		xQueueCreateStatic(EXO_BLE_QUEUE_DEPTH, sizeof(exo_ble_rx_item_t),
						   s_queue_storage, &s_queue_buffer);
	if (s_command_queue == NULL) {
		return ESP_ERR_NO_MEM;
	}
	s_worker_task = xTaskCreateStaticPinnedToCore(
		worker_task, "exo_ble_cmd", EXO_BLE_TASK_STACK_BYTES, NULL,
		EXO_BLE_TASK_PRIORITY, s_worker_task_stack, &s_worker_task_buffer,
		EXO_BLE_TASK_CORE);
	if (s_worker_task == NULL) {
		return ESP_ERR_NO_MEM;
	}

	portENTER_CRITICAL(&s_lock);
	s_snapshot.initialized = true;
	s_snapshot.boot_id = esp_random();
	if (s_snapshot.boot_id == 0U) {
		s_snapshot.boot_id = 1U;
	}
	s_snapshot.connection_handle = BLE_HS_CONN_HANDLE_NONE;
	s_snapshot.negotiated_mtu = EXO_BLE_DEFAULT_ATT_MTU;
	s_snapshot.worker_task_identity = (uintptr_t)s_worker_task;
	portEXIT_CRITICAL(&s_lock);

	nimble_port_freertos_init(host_task);
	ESP_LOGI(TAG, "ESP-EXO product BLE initialized");
	return ESP_OK;
}

bool exo_ble_get_snapshot(exo_ble_snapshot_t *snapshot) {
	if (snapshot == NULL) {
		return false;
	}
	portENTER_CRITICAL(&s_lock);
	*snapshot = s_snapshot;
	portEXIT_CRITICAL(&s_lock);
	if (s_worker_task != NULL) {
		snapshot->worker_stack_free_min_bytes =
			(uint32_t)uxTaskGetStackHighWaterMark(s_worker_task);
	}
	return snapshot->initialized;
}
