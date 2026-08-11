/*
 * ESP-EXO 产品 BLE 传输与业务适配层。
 *
 * 对 BLE 零基础读者，完整数据流如下：
 *
 *   手机扫描 ESP-EXO，连接 Product Service
 *     -> 订阅 Response characteristic 的 Notify
 *     -> 向 Command characteristic Write 一帧（最多 20 bytes）
 *     -> command_access() 只检查会话/长度、复制并零等待入静态队列
 *     -> worker_task() 检查 session generation、解码、去重并分发业务
 *     -> exo_system / hip_joint 返回快照或接收异步请求
 *     -> exo_ble_protocol.c 编成稳定 v1 字节
 *     -> 先缓存响应，再由 Response characteristic Notify 给手机
 *
 * GATT（Generic Attribute Profile）是 BLE 的数据接口表：Service 是一组功能，
 * Characteristic 是其中一个可读写/通知的数据端点。本服务只有 Command Write 和
 * Response Notify 两个产品端点。Notify 不是普通读取；手机必须先订阅，ESP32 才能
 * 主动推送响应。
 *
 * BLE 会话不拥有外骨骼生命周期：连接不会 START，断开不会 STOP。连接变化只更新
 * session generation，使旧队列项和旧缓存失效；已经交给 exo_system 的启停请求仍
 * 按整机状态机执行。本文件也不创建周期运动源，唯一 100 Hz 控制仍在控制任务中。
 */

#include "exo_ble.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "exo_assist.h"
#include "exo_ble_protocol.h"
#include "exo_config.h"
#include "exo_control_loop.h"
#include "exo_controller.h"
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
#define EXO_FIRMWARE_VERSION_MINOR 9U
#define EXO_FIRMWARE_VERSION_PATCH 0U

static const char *TAG = "exo_ble";

/*
 * GATT UUID 在 NimBLE 宏中按 Bluetooth little-endian 字节顺序存放：
 *   Product Service：手机发现 ESP-EXO 产品接口的功能分组；
 *   Command：手机使用 Write 写入完整 v1 请求帧；
 *   Response：手机订阅 Notify，设备从这里主动推送对应响应帧。
 */
static const ble_uuid128_t s_product_service_uuid =
	BLE_UUID128_INIT(0x01, 0x00, 0x64, 0x8B, 0x1C, 0x3A, 0x7C, 0x9E, 0x92, 0x4D,
					 0x5A, 0x6B, 0x01, 0x00, 0xA1, 0xF7);
static const ble_uuid128_t s_command_uuid =
	BLE_UUID128_INIT(0x01, 0x00, 0x64, 0x8B, 0x1C, 0x3A, 0x7C, 0x9E, 0x92, 0x4D,
					 0x5A, 0x6B, 0x02, 0x00, 0xA1, 0xF7);
static const ble_uuid128_t s_response_uuid =
	BLE_UUID128_INIT(0x01, 0x00, 0x64, 0x8B, 0x1C, 0x3A, 0x7C, 0x9E, 0x92, 0x4D,
					 0x5A, 0x6B, 0x03, 0x00, 0xA1, 0xF7);

/*
 * GATT 回调复制到静态队列的最小工作项。generation + connection_handle 把命令
 * 绑定到接收它的连接会话，避免断开后才被 worker 取出的旧命令误作用于新连接。
 */
typedef struct {
	uint32_t session_generation;
	uint16_t connection_handle;
	uint8_t length;
	uint8_t bytes[EXO_BLE_PROTOCOL_MAX_FRAME_SIZE];
} exo_ble_rx_item_t;

/*
 * 最近 8 个请求的会话内幂等缓存。键由 session_generation + request_id 构成，
 * 同时保留完整请求以区分“合法重试”和“同 ID 不同内容”的客户端错误。
 */
typedef struct {
	bool valid;
	uint32_t session_generation;
	uint8_t request_id;
	uint8_t request_length;
	uint8_t request[EXO_BLE_PROTOCOL_MAX_FRAME_SIZE];
	uint8_t response_length;
	uint8_t response[EXO_BLE_PROTOCOL_MAX_FRAME_SIZE];
} exo_ble_cache_entry_t;

/* Command Write 回调与 BLE worker 之间的固定容量、静态分配队列。 */
static StaticQueue_t s_queue_buffer;
static uint8_t s_queue_storage[EXO_BLE_QUEUE_DEPTH * sizeof(exo_ble_rx_item_t)];
static QueueHandle_t s_command_queue;

/* 唯一 BLE 命令 worker 的静态任务控制块和栈。 */
static StaticTask_t s_worker_task_buffer;
static StackType_t
	s_worker_task_stack[EXO_BLE_TASK_STACK_BYTES / sizeof(StackType_t)];
static TaskHandle_t s_worker_task;

/* 环形覆盖的幂等缓存；generation 变化后旧条目无需立即清零也不会命中。 */
static exo_ble_cache_entry_t s_cache[EXO_BLE_CACHE_DEPTH];
static size_t s_next_cache_entry;

static uint16_t s_command_value_handle;
static uint16_t s_response_value_handle;
static uint8_t s_own_addr_type = BLE_OWN_ADDR_PUBLIC;

/* 连接状态与诊断计数由 GAP、GATT 回调和 worker 共享，访问时使用 s_lock。 */
static exo_ble_snapshot_t s_snapshot;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

_Static_assert(EXO_BLE_PROTOCOL_MAX_FRAME_SIZE <= 20U,
			   "phase 6 frames must fit the default ATT payload");
_Static_assert((EXO_BLE_TASK_STACK_BYTES % sizeof(StackType_t)) == 0U,
			   "BLE worker stack size must align to StackType_t");

/*
 * 以下 wire_*() 适配器把内部 enum 显式翻译成稳定线码。不能直接强转内部值，
 * 否则以后重排内部 enum 会在无意中破坏手机协议。
 */
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

/* 把整机快照中的 bool 状态压缩成 SYSTEM_STATUS/ACK 共用的 1-byte flags。 */
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

/*
 * 取得一份整机快照；失败时填入可安全编码的 INIT 默认值。返回 false 仍允许
 * 上层生成 INTERNAL_ERROR ACK，而不会读取未初始化内存。
 */
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

/* ACK 总是附带“生成 ACK 此刻”的整机状态和 flags，便于手机立即刷新界面。 */
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

/*
 * 业务层的 ACK 统一出口。错误请求可能携带 response-bit opcode，因此先清最高位，
 * 再交给纯协议层重新生成规范的响应 opcode。
 */
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

/* 线协议 LEFT/RIGHT 转为人体髋关节对象；它们不是执行器 A/B。 */
static exo_hip_joint_id_t joint_from_wire_side(exo_ble_wire_side_t side) {
	return side == EXO_BLE_SIDE_LEFT ? EXO_HIP_JOINT_LEFT : EXO_HIP_JOINT_RIGHT;
}

/* 将内部关节方向显式映射为手机可长期依赖的稳定线码。 */
static exo_ble_wire_direction_t wire_direction(exo_direction_t direction) {
	if (direction == EXO_DIRECTION_POSITIVE) {
		return EXO_BLE_DIRECTION_POSITIVE;
	}
	if (direction == EXO_DIRECTION_NEGATIVE) {
		return EXO_BLE_DIRECTION_NEGATIVE;
	}
	return EXO_BLE_DIRECTION_UNKNOWN;
}

/* 产品模式与参考源显式翻译成稳定线码，避免依赖两个 enum 恰好同值。 */
static exo_ble_wire_control_mode_t wire_control_mode(exo_control_mode_t mode) {
	switch (mode) {
	case EXO_CONTROL_MODE_NONE:
		return EXO_BLE_CONTROL_MODE_NONE;
	case EXO_CONTROL_MODE_HOLD:
		return EXO_BLE_CONTROL_MODE_HOLD;
	case EXO_CONTROL_MODE_POSITION:
		return EXO_BLE_CONTROL_MODE_POSITION;
	case EXO_CONTROL_MODE_VELOCITY:
		return EXO_BLE_CONTROL_MODE_VELOCITY;
	case EXO_CONTROL_MODE_TORQUE:
		return EXO_BLE_CONTROL_MODE_TORQUE;
	case EXO_CONTROL_MODE_DAMPING:
		return EXO_BLE_CONTROL_MODE_DAMPING;
	case EXO_CONTROL_MODE_IMPEDANCE:
		return EXO_BLE_CONTROL_MODE_IMPEDANCE;
	default:
		return EXO_BLE_CONTROL_MODE_NONE;
	}
}

static bool control_mode_from_wire(exo_ble_wire_control_mode_t wire,
								   exo_control_mode_t *mode) {
	if (mode == NULL) {
		return false;
	}
	switch (wire) {
	case EXO_BLE_CONTROL_MODE_HOLD:
		*mode = EXO_CONTROL_MODE_HOLD;
		return true;
	case EXO_BLE_CONTROL_MODE_POSITION:
		*mode = EXO_CONTROL_MODE_POSITION;
		return true;
	case EXO_BLE_CONTROL_MODE_VELOCITY:
		*mode = EXO_CONTROL_MODE_VELOCITY;
		return true;
	case EXO_BLE_CONTROL_MODE_TORQUE:
		*mode = EXO_CONTROL_MODE_TORQUE;
		return true;
	case EXO_BLE_CONTROL_MODE_DAMPING:
		*mode = EXO_CONTROL_MODE_DAMPING;
		return true;
	case EXO_BLE_CONTROL_MODE_IMPEDANCE:
		*mode = EXO_CONTROL_MODE_IMPEDANCE;
		return true;
	case EXO_BLE_CONTROL_MODE_NONE:
	default:
		return false;
	}
}

static exo_ble_wire_reference_t
wire_reference(exo_reference_source_t reference) {
	return reference == EXO_REFERENCE_SINE ? EXO_BLE_REFERENCE_SINE
										 : EXO_BLE_REFERENCE_CONSTANT;
}

static bool reference_from_wire(exo_ble_wire_reference_t wire,
								exo_reference_source_t *reference) {
	if (reference == NULL) {
		return false;
	}
	if (wire == EXO_BLE_REFERENCE_CONSTANT) {
		*reference = EXO_REFERENCE_CONSTANT;
		return true;
	}
	if (wire == EXO_BLE_REFERENCE_SINE) {
		*reference = EXO_REFERENCE_SINE;
		return true;
	}
	return false;
}

/*
 * 步态与高层助力也使用“内部枚举 <-> 稳定线码”的显式适配。虽然当前数值恰好
 * 相同，也不能依赖强制类型转换；未来算法模块调整枚举顺序时，BLE v1 仍须兼容。
 */
static exo_ble_wire_gait_estimator_t
wire_gait_estimator(exo_gait_estimator_id_t estimator) {
	return estimator == EXO_GAIT_ESTIMATOR_HIP_EVENT_TBE
			   ? EXO_BLE_GAIT_ESTIMATOR_HIP_EVENT_TBE
			   : EXO_BLE_GAIT_ESTIMATOR_NONE;
}

static exo_ble_wire_gait_state_t
wire_gait_state(exo_gait_estimator_state_t state) {
	switch (state) {
	case EXO_GAIT_STATE_STANDING:
		return EXO_BLE_GAIT_STATE_STANDING;
	case EXO_GAIT_STATE_ACQUIRING:
		return EXO_BLE_GAIT_STATE_ACQUIRING;
	case EXO_GAIT_STATE_LOCKED:
		return EXO_BLE_GAIT_STATE_LOCKED;
	case EXO_GAIT_STATE_DEGRADED:
		return EXO_BLE_GAIT_STATE_DEGRADED;
	case EXO_GAIT_STATE_RESET:
	default:
		return EXO_BLE_GAIT_STATE_RESET;
	}
}

static exo_ble_wire_gait_origin_t
wire_gait_origin(exo_gait_phase_origin_t origin) {
	if (origin == EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK) {
		return EXO_BLE_GAIT_ORIGIN_HIP_FLEXION_PEAK;
	}
	if (origin == EXO_GAIT_PHASE_ORIGIN_HIP_EXTENSION_PEAK) {
		return EXO_BLE_GAIT_ORIGIN_HIP_EXTENSION_PEAK;
	}
	return EXO_BLE_GAIT_ORIGIN_UNKNOWN;
}

static exo_ble_wire_assist_mode_t wire_assist_mode(exo_assist_mode_t mode) {
	switch (mode) {
	case EXO_ASSIST_MODE_TRANSPARENT:
		return EXO_BLE_ASSIST_MODE_TRANSPARENT;
	case EXO_ASSIST_MODE_ACTIVE_PHASE:
		return EXO_BLE_ASSIST_MODE_ACTIVE_PHASE;
	case EXO_ASSIST_MODE_ASSIST_AS_NEEDED:
		return EXO_BLE_ASSIST_MODE_ASSIST_AS_NEEDED;
	case EXO_ASSIST_MODE_NONE:
	default:
		return EXO_BLE_ASSIST_MODE_NONE;
	}
}

static bool assist_mode_from_wire(exo_ble_wire_assist_mode_t wire,
							  exo_assist_mode_t *mode) {
	if (mode == NULL) {
		return false;
	}
	switch (wire) {
	case EXO_BLE_ASSIST_MODE_TRANSPARENT:
		*mode = EXO_ASSIST_MODE_TRANSPARENT;
		return true;
	case EXO_BLE_ASSIST_MODE_ACTIVE_PHASE:
		*mode = EXO_ASSIST_MODE_ACTIVE_PHASE;
		return true;
	case EXO_BLE_ASSIST_MODE_ASSIST_AS_NEEDED:
		*mode = EXO_ASSIST_MODE_ASSIST_AS_NEEDED;
		return true;
	case EXO_BLE_ASSIST_MODE_NONE:
	default:
		return false;
	}
}

static exo_ble_wire_assist_execution_t
wire_assist_execution(exo_assist_execution_state_t state) {
	if (state == EXO_ASSIST_EXEC_SHADOW) {
		return EXO_BLE_ASSIST_EXEC_SHADOW;
	}
	if (state == EXO_ASSIST_EXEC_ACTIVE) {
		return EXO_BLE_ASSIST_EXEC_ACTIVE;
	}
	return EXO_BLE_ASSIST_EXEC_DISABLED;
}

static bool assist_execution_from_wire(exo_ble_wire_assist_execution_t wire,
								   exo_assist_execution_state_t *state) {
	if (state == NULL) {
		return false;
	}
	if (wire == EXO_BLE_ASSIST_EXEC_DISABLED) {
		*state = EXO_ASSIST_EXEC_DISABLED;
		return true;
	}
	if (wire == EXO_BLE_ASSIST_EXEC_SHADOW) {
		*state = EXO_ASSIST_EXEC_SHADOW;
		return true;
	}
	if (wire == EXO_BLE_ASSIST_EXEC_ACTIVE) {
		*state = EXO_ASSIST_EXEC_ACTIVE;
		return true;
	}
	return false;
}

static exo_ble_wire_assist_strategy_t
wire_assist_strategy(exo_assist_strategy_id_t strategy) {
	switch (strategy) {
	case EXO_ASSIST_STRATEGY_TRANSPARENT:
		return EXO_BLE_ASSIST_STRATEGY_TRANSPARENT;
	case EXO_ASSIST_STRATEGY_PHASE_PROFILE:
		return EXO_BLE_ASSIST_STRATEGY_PHASE_PROFILE;
	case EXO_ASSIST_STRATEGY_AAN_RESIDUAL:
		return EXO_BLE_ASSIST_STRATEGY_AAN_RESIDUAL;
	case EXO_ASSIST_STRATEGY_NONE:
	default:
		return EXO_BLE_ASSIST_STRATEGY_NONE;
	}
}

static bool assist_strategy_from_wire(exo_ble_wire_assist_strategy_t wire,
								  exo_assist_strategy_id_t *strategy) {
	if (strategy == NULL) {
		return false;
	}
	switch (wire) {
	case EXO_BLE_ASSIST_STRATEGY_TRANSPARENT:
		*strategy = EXO_ASSIST_STRATEGY_TRANSPARENT;
		return true;
	case EXO_BLE_ASSIST_STRATEGY_PHASE_PROFILE:
		*strategy = EXO_ASSIST_STRATEGY_PHASE_PROFILE;
		return true;
	case EXO_BLE_ASSIST_STRATEGY_AAN_RESIDUAL:
		*strategy = EXO_ASSIST_STRATEGY_AAN_RESIDUAL;
		return true;
	case EXO_BLE_ASSIST_STRATEGY_NONE:
	default:
		return false;
	}
}

static exo_ble_wire_assist_unavailable_t wire_assist_unavailable(
	exo_assist_unavailable_reason_t reason) {
	switch (reason) {
	case EXO_ASSIST_UNAVAILABLE_NOT_INITIALIZED:
		return EXO_BLE_ASSIST_UNAVAILABLE_NOT_INITIALIZED;
	case EXO_ASSIST_UNAVAILABLE_CONFIG_INVALID:
		return EXO_BLE_ASSIST_UNAVAILABLE_CONFIG_INVALID;
	case EXO_ASSIST_UNAVAILABLE_ACTIVE_NOT_ALLOWED:
		return EXO_BLE_ASSIST_UNAVAILABLE_ACTIVE_NOT_ALLOWED;
	case EXO_ASSIST_UNAVAILABLE_STRATEGY_NOT_IMPLEMENTED:
		return EXO_BLE_ASSIST_UNAVAILABLE_STRATEGY_NOT_IMPLEMENTED;
	case EXO_ASSIST_UNAVAILABLE_REQUIRED_SENSOR_MISSING:
		return EXO_BLE_ASSIST_UNAVAILABLE_REQUIRED_SENSOR_MISSING;
	case EXO_ASSIST_UNAVAILABLE_NOT_ARMED:
		return EXO_BLE_ASSIST_UNAVAILABLE_NOT_ARMED;
	case EXO_ASSIST_UNAVAILABLE_REQUIRED_GAIT_INVALID:
		return EXO_BLE_ASSIST_UNAVAILABLE_REQUIRED_GAIT_INVALID;
	case EXO_ASSIST_UNAVAILABLE_NONE:
	default:
		return EXO_BLE_ASSIST_UNAVAILABLE_NONE;
	}
}

/*
 * 浮点量转 i32 定点量：先用 double 做乘法和四舍五入，再在 int32 边界饱和。
 * 非有限输入编码为 0；有效性由相应 flags 告知手机，不能只凭数值 0 判断有效。
 */
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

static int16_t scaled_i16(float value, float scale) {
	if (!isfinite(value) || !isfinite(scale)) {
		return 0;
	}
	const double scaled = (double)value * (double)scale;
	if (scaled >= (double)INT16_MAX) {
		return INT16_MAX;
	}
	if (scaled <= (double)INT16_MIN) {
		return INT16_MIN;
	}
	return (int16_t)llround(scaled);
}

static uint16_t scaled_u16(float value, float scale) {
	if (!isfinite(value) || !isfinite(scale) || value <= 0.0f) {
		return 0U;
	}
	const double scaled = (double)value * (double)scale;
	return scaled >= (double)UINT16_MAX ? UINT16_MAX
										 : (uint16_t)llround(scaled);
}

/* 将反馈时间戳换算为 u16 ms；无时间戳或超过范围时使用协议规定的饱和值。 */
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

/* 组装 GET_INFO 的协议/固件/能力信息；boot_id 用于识别设备是否重启。 */
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
						EXO_BLE_CAPABILITY_ON_DEMAND_STATUS |
						EXO_BLE_CAPABILITY_CONTROL_CONFIGURATION |
						EXO_BLE_CAPABILITY_GAIT_STATUS |
						EXO_BLE_CAPABILITY_ASSIST_CONFIGURATION,
		.max_frame_size = EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
		.boot_id = boot_id,
	};
	return exo_ble_protocol_encode_info(request_id, &info, response,
										EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
										response_len);
}

/* 读取 exo_system 的自洽快照，并编码 GET_SYSTEM_STATUS 响应。 */
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

/*
 * 读取指定 LEFT/RIGHT 髋关节快照，生成 flags 并跨越显示/线协议单位边界。
 * 产品关节对象使用 deg、deg/s；已发布 BLE v1 必须继续传 mrad、mrad/s，
 * 所以在这里显式 deg -> rad -> mrad。手机收到 i32 后再换算为 deg 显示。
 * joint_valid=false 时数值字段发送 0，手机必须同时检查 joint_flags。
 */
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
		.position_mrad = joint.joint_valid
							 ? scaled_i32(exo_degrees_to_radians(
											  joint.joint_sample.position_deg),
										  1000.0f)
							 : 0,
		.velocity_mrad_s =
			joint.joint_valid
				? scaled_i32(
					  exo_degrees_to_radians(joint.joint_sample.velocity_deg_s),
					  1000.0f)
				: 0,
		.feedback_age_ms = feedback_age_ms(joint.feedback_timestamp_us),
	};
	return exo_ble_protocol_encode_joint_status(
		request->request_id, &status, response, EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
		response_len);
}

/* 把控制器内部五元对象编码为协议定点 DTO；网页边界再换算成度制显示。 */
static exo_ble_wire_joint_command_t
wire_joint_command(exo_ble_wire_side_t side,
				   const exo_joint_command_t *command) {
	return (exo_ble_wire_joint_command_t){
		.side = side,
		.position_mrad = scaled_i32(
			exo_degrees_to_radians(command->position_deg), 1000.0f),
		.velocity_mrad_s = scaled_i32(
			exo_degrees_to_radians(command->velocity_deg_s), 1000.0f),
		.torque_mnm = scaled_i16(command->torque_ff_nm, 1000.0f),
		.kp_x100 = scaled_u16(command->kp, 100.0f),
		.kd_x1000 = scaled_u16(command->kd, 1000.0f),
	};
}

static exo_joint_command_t
joint_command_from_wire(const exo_ble_wire_joint_command_t *wire) {
	return (exo_joint_command_t){
		.position_deg = exo_radians_to_degrees(
			(float)wire->position_mrad / 1000.0f),
		.velocity_deg_s = exo_radians_to_degrees(
			(float)wire->velocity_mrad_s / 1000.0f),
		.torque_ff_nm = (float)wire->torque_mnm / 1000.0f,
		.kp = (float)wire->kp_x100 / 100.0f,
		.kd = (float)wire->kd_x1000 / 1000.0f,
	};
}

static bool encode_control_config_response(uint8_t request_id,
									   uint8_t *response,
									   size_t *response_len) {
	exo_controller_snapshot_t controller;
	if (!exo_controller_get_snapshot(&controller)) {
		return encode_ack(EXO_BLE_OPCODE_GET_CONTROL_CONFIG, request_id,
						  EXO_BLE_RESULT_INTERNAL_ERROR, 0U, response,
						  response_len);
	}
	uint8_t flags = EXO_BLE_CONTROL_CONFIG_FLAG_SELECTED_VALID;
	if (controller.armed && controller.active.mode != EXO_CONTROL_MODE_NONE) {
		flags |= EXO_BLE_CONTROL_CONFIG_FLAG_ACTIVE_VALID;
	}
	const exo_ble_wire_control_config_t config = {
		.selected_mode = wire_control_mode(controller.selected.mode),
		.active_mode = wire_control_mode(controller.active.mode),
		.selected_reference = wire_reference(controller.selected.reference),
		.active_reference = wire_reference(controller.active.reference),
		.config_flags = flags,
	};
	return exo_ble_protocol_encode_control_config(
		request_id, &config, response, EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
		response_len);
}

static bool encode_joint_command_response(const exo_ble_request_t *request,
									  uint8_t *response,
									  size_t *response_len) {
	exo_controller_snapshot_t controller;
	if (!exo_controller_get_snapshot(&controller)) {
		return encode_ack(request->opcode, request->request_id,
						  EXO_BLE_RESULT_INTERNAL_ERROR, 0U, response,
						  response_len);
	}
	const exo_joint_command_t *selected =
		request->side == EXO_BLE_SIDE_LEFT ? &controller.selected.command.left
										 : &controller.selected.command.right;
	const exo_ble_wire_joint_command_t wire =
		wire_joint_command(request->side, selected);
	return exo_ble_protocol_encode_joint_command(
		request->request_id, &wire, response,
		EXO_BLE_PROTOCOL_MAX_FRAME_SIZE, response_len);
}

/* 把[0,1)相位编码为0..9999；无效值必须发送0xFFFF，不能伪装成0%。 */
static uint16_t wire_phase_x10000(float phase, bool valid) {
	if (!valid || !isfinite(phase)) {
		return EXO_BLE_GAIT_FIXED_INVALID;
	}
	if (phase <= 0.0f) {
		return 0U;
	}
	if (phase >= 1.0f) {
		return EXO_BLE_GAIT_PHASE_X10000_MAX;
	}
	const uint16_t scaled = (uint16_t)llround((double)phase * 10000.0);
	return scaled > EXO_BLE_GAIT_PHASE_X10000_MAX
			   ? EXO_BLE_GAIT_PHASE_X10000_MAX
			   : scaled;
}

/* 置信度允许精确1.0000，因此上界与相位字段不同，是10000。 */
static uint16_t wire_confidence_x10000(float confidence, bool valid) {
	if (!valid || !isfinite(confidence)) {
		return EXO_BLE_GAIT_FIXED_INVALID;
	}
	if (confidence <= 0.0f) {
		return 0U;
	}
	if (confidence >= 1.0f) {
		return EXO_BLE_GAIT_CONFIDENCE_X10000_MAX;
	}
	return (uint16_t)llround((double)confidence * 10000.0);
}

/* 步频单位为step/min，线协议保留两位小数并把300.00作为显示上界。 */
static uint16_t wire_cadence_x100(float cadence_spm, bool valid) {
	if (!valid || !isfinite(cadence_spm)) {
		return EXO_BLE_GAIT_FIXED_INVALID;
	}
	if (cadence_spm <= 0.0f) {
		return 0U;
	}
	if (cadence_spm >= 300.0f) {
		return EXO_BLE_GAIT_CADENCE_X100_MAX;
	}
	return (uint16_t)llround((double)cadence_spm * 100.0);
}

/*
 * BLE worker只读取100 Hz任务已经发布的完整估计快照，不在通信任务里重复运行算法。
 * 左右相位允许单侧有效；overall_valid只有两侧都锁定时才置位。
 */
static bool encode_gait_response(uint8_t request_id, uint8_t *response,
								 size_t *response_len) {
	exo_control_loop_snapshot_t control;
	if (!exo_control_loop_get_snapshot(&control)) {
		return encode_ack(EXO_BLE_OPCODE_GET_GAIT_STATUS, request_id,
						  EXO_BLE_RESULT_INTERNAL_ERROR, 0U, response,
						  response_len);
	}

	const exo_gait_estimate_t *gait = &control.gait_estimate;
	const bool left_valid =
		(gait->valid_flags & EXO_GAIT_VALID_LEFT_PHASE) != 0U;
	const bool right_valid =
		(gait->valid_flags & EXO_GAIT_VALID_RIGHT_PHASE) != 0U;
	const bool cadence_valid =
		(gait->valid_flags & EXO_GAIT_VALID_CADENCE) != 0U;
	const bool confidence_valid =
		(gait->valid_flags & EXO_GAIT_VALID_CONFIDENCE) != 0U;

	uint8_t flags = 0U;
	if (gait->valid && left_valid && right_valid) {
		flags |= EXO_BLE_GAIT_FLAG_OVERALL_VALID;
	}
	if (left_valid) {
		flags |= EXO_BLE_GAIT_FLAG_LEFT_PHASE_VALID;
	}
	if (right_valid) {
		flags |= EXO_BLE_GAIT_FLAG_RIGHT_PHASE_VALID;
	}
	if (gait->motion_detected) {
		flags |= EXO_BLE_GAIT_FLAG_MOTION_DETECTED;
	}
	if ((gait->left_event_flags & EXO_GAIT_EVENT_PHASE_WRAP) != 0U) {
		flags |= EXO_BLE_GAIT_FLAG_LEFT_PHASE_WRAP;
	}
	if ((gait->right_event_flags & EXO_GAIT_EVENT_PHASE_WRAP) != 0U) {
		flags |= EXO_BLE_GAIT_FLAG_RIGHT_PHASE_WRAP;
	}

	const exo_ble_wire_gait_status_t status = {
		.estimator_id = wire_gait_estimator(gait->estimator_id),
		.estimator_state = wire_gait_state(gait->state),
		.gait_flags = flags,
		.phase_origin = wire_gait_origin(gait->phase_origin),
		.quality_flags =
			(uint16_t)(gait->quality_flags & EXO_BLE_GAIT_QUALITY_FLAGS_MASK),
		.left_phase_x10000 =
			wire_phase_x10000(gait->left_phase_01, left_valid),
		.right_phase_x10000 =
			wire_phase_x10000(gait->right_phase_01, right_valid),
		.cadence_x100 =
			wire_cadence_x100(gait->cadence_spm, cadence_valid),
		.confidence_x10000 = wire_confidence_x10000(
			gait->confidence_01, confidence_valid),
		.estimate_age_ms = gait->timestamp_us > 0
						   ? feedback_age_ms((uint64_t)gait->timestamp_us)
						   : UINT16_MAX,
	};
	return exo_ble_protocol_encode_gait_status(
		request_id, &status, response, EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
		response_len);
}

/* 将助力模块的内部诊断快照压成固定16-byte状态；这里不执行任何策略。 */
static bool encode_assist_response(uint8_t request_id, uint8_t *response,
								   size_t *response_len) {
	exo_assist_snapshot_t assist;
	if (!exo_assist_get_snapshot(&assist)) {
		return encode_ack(EXO_BLE_OPCODE_GET_ASSIST_CONFIG, request_id,
						  EXO_BLE_RESULT_INTERNAL_ERROR, 0U, response,
						  response_len);
	}

	uint8_t flags = 0U;
	if (assist.selected_valid) {
		flags |= EXO_BLE_ASSIST_FLAG_SELECTED_VALID;
	}
	if (assist.active_valid) {
		flags |= EXO_BLE_ASSIST_FLAG_ACTIVE_VALID;
	}
	if (assist.config_locked) {
		flags |= EXO_BLE_ASSIST_FLAG_CONFIG_LOCKED;
	}
	if (assist.strategy_ready) {
		flags |= EXO_BLE_ASSIST_FLAG_STRATEGY_READY;
	}
	if (assist.required_sensors_ready) {
		flags |= EXO_BLE_ASSIST_FLAG_SENSORS_READY;
	}
	if (assist.suggestion_valid) {
		flags |= EXO_BLE_ASSIST_FLAG_SUGGESTION_VALID;
	}
	if (assist.real_output_enabled) {
		flags |= EXO_BLE_ASSIST_FLAG_REAL_OUTPUT_ENABLED;
	}

	const exo_assist_config_t *selected = &assist.selected;
	const exo_assist_config_t *active = &assist.active;
	const exo_ble_wire_assist_config_t config = {
		.schema_version = EXO_ASSIST_CONFIG_SCHEMA_VERSION,
		.selected_mode = assist.selected_valid
						 ? wire_assist_mode(selected->mode)
						 : EXO_BLE_ASSIST_MODE_NONE,
		.active_mode = assist.active_valid
					   ? wire_assist_mode(active->mode)
					   : EXO_BLE_ASSIST_MODE_NONE,
		.requested_execution_state = assist.selected_valid
								 ? wire_assist_execution(
									   selected->requested_execution_state)
								 : EXO_BLE_ASSIST_EXEC_DISABLED,
		.execution_state = wire_assist_execution(assist.execution_state),
		.strategy_id = assist.selected_valid
					   ? wire_assist_strategy(selected->strategy_id)
					   : EXO_BLE_ASSIST_STRATEGY_NONE,
		.strategy_version = assist.strategy_version,
		.profile_id = assist.selected_valid ? selected->profile_id : 0U,
		.left_level_percent =
			assist.selected_valid ? selected->left_level_percent : 0U,
		.right_level_percent =
			assist.selected_valid ? selected->right_level_percent : 0U,
		.assist_flags = flags,
		.unavailable_reason =
			wire_assist_unavailable(assist.unavailable_reason),
		.config_generation = (uint8_t)assist.selected_generation,
		.required_sensor_mask = (uint16_t)assist.required_sensor_mask,
		.available_sensor_mask = (uint16_t)assist.available_sensor_mask,
	};
	return exo_ble_protocol_encode_assist_config(
		request_id, &config, response, EXO_BLE_PROTOCOL_MAX_FRAME_SIZE,
		response_len);
}

static exo_ble_wire_result_t controller_error_result(esp_err_t error) {
	if (error == ESP_OK) {
		return EXO_BLE_RESULT_OK;
	}
	if (error == ESP_ERR_INVALID_ARG) {
		return EXO_BLE_RESULT_INVALID_PAYLOAD;
	}
	if (error == ESP_ERR_INVALID_STATE) {
		return EXO_BLE_RESULT_REJECTED_BUSY;
	}
	return EXO_BLE_RESULT_INTERNAL_ERROR;
}

/* ACTIVE等“格式正确但本阶段不支持”的请求应返回NOT_READY，而不是伪装成坏帧。 */
static exo_ble_wire_result_t assist_error_result(esp_err_t error) {
	if (error == ESP_ERR_NOT_SUPPORTED) {
		return EXO_BLE_RESULT_REJECTED_NOT_READY;
	}
	return controller_error_result(error);
}

/* SET 请求必须先取得整机 IDLE 维护权，BLE worker 不能绕过状态机直接改运行配置。 */
static exo_ble_wire_result_t
execute_control_config_command(const exo_ble_request_t *request) {
	if (!exo_system_try_begin_maintenance()) {
		return EXO_BLE_RESULT_REJECTED_BUSY;
	}
	exo_control_mode_t mode;
	exo_reference_source_t reference;
	esp_err_t error = ESP_ERR_INVALID_ARG;
	if (control_mode_from_wire(request->control_config.selected_mode, &mode) &&
		reference_from_wire(request->control_config.selected_reference,
							&reference)) {
		error = exo_controller_set_mode(mode, reference);
	}
	exo_system_end_maintenance();
	return controller_error_result(error);
}

static exo_ble_wire_result_t
execute_joint_command(const exo_ble_request_t *request) {
	if (!exo_system_try_begin_maintenance()) {
		return EXO_BLE_RESULT_REJECTED_BUSY;
	}
	const exo_joint_command_t command =
		joint_command_from_wire(&request->joint_command);
	const esp_err_t error = exo_controller_set_joint_command(
		joint_from_wire_side(request->joint_command.side), &command);
	exo_system_end_maintenance();
	return controller_error_result(error);
}

/*
 * 高层策略配置也必须经过整机IDLE维护门。阶段9只执行DISABLED/SHADOW；ACTIVE
 * 可被解码以返回明确的NOT_READY + ACTIVE_NOT_ALLOWED，但绝不会进入selected。
 */
static exo_ble_wire_result_t
execute_assist_config_command(const exo_ble_request_t *request) {
	if (!exo_system_try_begin_maintenance()) {
		return EXO_BLE_RESULT_REJECTED_BUSY;
	}

	exo_assist_mode_t mode;
	exo_assist_execution_state_t execution;
	exo_assist_strategy_id_t strategy;
	esp_err_t error = ESP_ERR_INVALID_ARG;
	if (assist_mode_from_wire(request->assist_config.selected_mode, &mode) &&
		assist_execution_from_wire(
			request->assist_config.requested_execution_state, &execution) &&
		assist_strategy_from_wire(request->assist_config.strategy_id,
								  &strategy)) {
		const exo_assist_config_t config = {
			.schema_version = request->assist_config.schema_version,
			.mode = mode,
			.requested_execution_state = execution,
			.strategy_id = strategy,
			.profile_id = request->assist_config.profile_id,
			.left_level_percent =
				request->assist_config.left_level_percent,
			.right_level_percent =
				request->assist_config.right_level_percent,
			.reserved_flags = request->assist_config.reserved_flags,
		};
		error = exo_assist_set_config(&config);
	}
	exo_system_end_maintenance();
	return assist_error_result(error);
}

/*
 * CAPTURE_ZERO/CLEAR_ZERO 的业务适配。维护互斥保证它们不会与 START/STOP
 * 硬件序列并发；操作只修改选定关节的 RAM 软件零位，不发送机械置零命令。
 */
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

/*
 * 已通过协议验证的请求分发中心：
 *   GET_* 查询并编码快照；
 *   START/STOP 只向异步整机状态机提交请求；
 *   CAPTURE/CLEAR_ZERO 在维护作用域内更新 RAM 零位。
 * ACCEPTED 不等于已经 RUNNING/IDLE，手机仍需继续轮询 SYSTEM_STATUS。
 */
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
	case EXO_BLE_OPCODE_GET_CONTROL_CONFIG:
		return encode_control_config_response(request->request_id, response,
									  response_len);
	case EXO_BLE_OPCODE_GET_JOINT_COMMAND:
		return encode_joint_command_response(request, response, response_len);
	case EXO_BLE_OPCODE_GET_GAIT_STATUS:
		return encode_gait_response(request->request_id, response, response_len);
	case EXO_BLE_OPCODE_GET_ASSIST_CONFIG:
		return encode_assist_response(request->request_id, response,
								  response_len);
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
	case EXO_BLE_OPCODE_SET_CONTROL_CONFIG:
		return encode_ack(request->opcode, request->request_id,
						  execute_control_config_command(request), 0U, response,
						  response_len);
	case EXO_BLE_OPCODE_SET_JOINT_COMMAND:
		return encode_ack(request->opcode, request->request_id,
						  execute_joint_command(request), 0U, response,
						  response_len);
	case EXO_BLE_OPCODE_SET_ASSIST_CONFIG: {
		const exo_ble_wire_result_t result =
			execute_assist_config_command(request);
		const uint8_t detail = result == EXO_BLE_RESULT_REJECTED_NOT_READY
						   ? (uint8_t)
							 EXO_BLE_ASSIST_UNAVAILABLE_ACTIVE_NOT_ALLOWED
						   : 0U;
		return encode_ack(request->opcode, request->request_id, result, detail,
						  response, response_len);
	}
	default:
		return encode_ack(request->opcode, request->request_id,
						  EXO_BLE_RESULT_UNKNOWN_OPCODE, 0U, response,
						  response_len);
	}
}

/*
 * 判断异步工作是否仍属于当前连接会话。session_generation 在连接、断开或 Host
 * reset 时递增；即使 connection_handle 后来被协议栈复用，旧 generation 也不会
 * 通过检查。require_subscription=true 时还要求手机仍订阅 Response Notify。
 */
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

/* 统一累计“向 NimBLE 提交 Notify”成功或失败的次数。 */
static void record_notify_result(bool success) {
	portENTER_CRITICAL(&s_lock);
	if (success) {
		++s_snapshot.notify_success_count;
	} else {
		++s_snapshot.notify_failure_count;
	}
	portEXIT_CRITICAL(&s_lock);
}

/*
 * 从 Response characteristic 主动 Notify 一帧响应。每次尝试前重查会话，避免
 * 把旧连接的响应发给新连接；内存暂缺或发送失败时按固定次数短暂重试。
 * rc==0 表示 NimBLE 已接受发送，后续空口发送失败另由 NOTIFY_TX 事件计数。
 */
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

/* 在当前 generation 内按 request_id 查找最近响应。 */
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

/*
 * 把完整请求和已编码响应写入固定深度环形缓存。必须在 Notify 前缓存：如果首次
 * Notify 丢失，手机重发同一帧时仍能取得相同响应，而不会再次执行 START/归零。
 */
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

/* 相同 request_id 只有在整帧长度和每个字节都相同时才算合法重复请求。 */
static bool cache_request_matches(const exo_ble_cache_entry_t *entry,
								  const exo_ble_rx_item_t *item) {
	return entry->request_length == item->length &&
		   memcmp(entry->request, item->bytes, item->length) == 0;
}

/*
 * BLE 命令 worker 的永久循环。每个队列项依次经过：
 *   1. 当前 session/连接/Notify 订阅检查；
 *   2. 纯协议解码；错误帧尽可能返回相关联 ACK；
 *   3. request_id 缓存判断：相同帧重放缓存，不同帧返回 ID_CONFLICT；
 *   4. 首次请求进入业务分发；
 *   5. 编码结果先缓存，再 Notify。
 *
 * 因为业务都在 worker 而非 GATT 回调中执行，Command Write 能快速返回给 NimBLE；
 * worker 也不是周期任务，只在手机写入命令时被队列唤醒。
 */
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

/*
 * Command characteristic 的 GATT Write 回调。这里运行在 NimBLE
 * 上下文，必须短小： 仅确认连接与 Response Notify 已订阅、限制帧长、展平
 * mbuf、零等待入静态队列。 它不解码 opcode、不访问电机、不执行
 * START/STOP，也不等待队列空间。
 */
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

/*
 * Response characteristic 只允许 Notify，不提供手机主动
 * Read/Write；真正的响应由 notify_response() 通过保存的 value handle
 * 推送，因此其他访问返回不支持。
 */
static int response_access(uint16_t connection_handle, uint16_t attr_handle,
						   struct ble_gatt_access_ctxt *context, void *arg) {
	(void)connection_handle;
	(void)attr_handle;
	(void)context;
	(void)arg;
	return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
}

/*
 * Product Service 的两个 characteristic：
 *   Command  = Write，手机到 ESP32；
 *   Response = Notify，ESP32 到手机。
 * 末尾全零项是 NimBLE GATT 定义数组的结束标记。
 */
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

/* 把上述 characteristic 归入一个 Primary Product Service。 */
static const struct ble_gatt_svc_def s_product_services[] = {
	{
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &s_product_service_uuid.u,
		.characteristics = s_product_characteristics,
	},
	{0},
};

/* 先初始化标准 GAP/GATT 服务，再计算并注册产品服务所需的协议栈资源。 */
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

/*
 * GAP 管理扫描、连接和链路事件；它不处理产品命令内容。
 *
 * CONNECT：开启新 session generation，记录 handle，等待手机订阅 Notify；
 * DISCONNECT：再递增 generation，使旧队列项/缓存失效，然后恢复广播；
 * SUBSCRIBE：记录 Response Notify 是否已订阅；
 * MTU：仅记录协商值，v1 帧仍保证不超过默认 20-byte ATT 载荷；
 * NOTIFY_TX：统计 NimBLE 接受后发生的异步发送失败。
 *
 * 断开事件不会调用
 * exo_system_request_stop()，因此不会隐式停止已经运行的外骨骼。
 */
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

/*
 * 发布可连接、可发现的 BLE 广播：Service UUID 放在广播包，设备名 ESP-EXO 放在
 * scan response。广播持续到连接，连接失败、断开或本轮广播结束后会重新启动。
 */
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

/*
 * NimBLE Host reset 会终止当前传输会话：递增 generation、清连接/订阅状态。
 * 这只处理 BLE 链路，不清软件零位，也不向整机状态机提交 STOP。
 */
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

/* Host 与控制器同步后确定本机地址类型，标记可用并开始产品广播。 */
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

/* ESP-IDF 为 NimBLE 创建的 Host 任务入口；协议事件循环退出后释放其 RTOS 资源。
 */
static void host_task(void *arg) {
	(void)arg;
	nimble_port_run();
	nimble_port_freertos_deinit();
}

/* NimBLE 依赖 NVS；遇到页面耗尽或版本不兼容时擦除并重新初始化。 */
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

/*
 * 一次性初始化顺序：NVS -> NimBLE -> 回调/GATT/设备名 -> 静态队列 ->
 * 唯一 BLE worker -> boot_id/诊断快照 -> NimBLE Host 任务。
 * boot_id 在每次 ESP32 启动时随机生成且非 0，手机可据此识别设备重启。
 */
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

/*
 * 供 UART 工程诊断读取 BLE 状态。主体快照在临界区中复制，worker 栈余量随后
 * 从 FreeRTOS 读取；函数不向手机发送数据，也不改变连接或会话。
 */
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
