#ifndef EXO_SYSTEM_H
#define EXO_SYSTEM_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	EXO_SYSTEM_STATE_INIT = 0,
	EXO_SYSTEM_STATE_IDLE,
	EXO_SYSTEM_STATE_STARTING,
	EXO_SYSTEM_STATE_RUNNING,
	EXO_SYSTEM_STATE_STOPPING,
	EXO_SYSTEM_STATE_ERROR,
} exo_system_state_t;

typedef enum {
	EXO_SYSTEM_STEP_NONE = 0,
	EXO_SYSTEM_STEP_CHECK_PRECONDITIONS,
	EXO_SYSTEM_STEP_DISABLE_MOTORS,
	EXO_SYSTEM_STEP_SET_MOTION_MODE,
	EXO_SYSTEM_STEP_ENABLE_MOTORS,
	EXO_SYSTEM_STEP_SEND_ZERO_OUTPUT,
	EXO_SYSTEM_STEP_WAIT_FRESH_FEEDBACK,
	EXO_SYSTEM_STEP_ARM_CONTROLLER,
	EXO_SYSTEM_STEP_DISARM_CONTROLLER,
	EXO_SYSTEM_STEP_RELEASE_OUTPUT,
	EXO_SYSTEM_STEP_VERIFY_DISABLED,
} exo_system_step_t;

typedef enum {
	EXO_START_BLOCK_NONE = 0,
	EXO_START_BLOCK_INITIALIZING,
	EXO_START_BLOCK_SYSTEM_NOT_IDLE,
	EXO_START_BLOCK_OPERATION_BUSY,
	EXO_START_BLOCK_CONTROL_TASK_NOT_READY,
	EXO_START_BLOCK_TWAI_NOT_RUNNING,
	EXO_START_BLOCK_LEFT_MAPPING_INVALID,
	EXO_START_BLOCK_RIGHT_MAPPING_INVALID,
	EXO_START_BLOCK_LEFT_DIRECTION_INVALID,
	EXO_START_BLOCK_RIGHT_DIRECTION_INVALID,
	EXO_START_BLOCK_LEFT_ZERO_INVALID,
	EXO_START_BLOCK_RIGHT_ZERO_INVALID,
	EXO_START_BLOCK_LEFT_FEEDBACK_INVALID,
	EXO_START_BLOCK_RIGHT_FEEDBACK_INVALID,
	EXO_START_BLOCK_LEFT_OFFLINE,
	EXO_START_BLOCK_RIGHT_OFFLINE,
	EXO_START_BLOCK_LEFT_MOTOR_FAULT,
	EXO_START_BLOCK_RIGHT_MOTOR_FAULT,
	EXO_START_BLOCK_SYSTEM_ERROR,
} exo_start_block_reason_t;

typedef enum {
	EXO_REQUEST_NONE = 0,
	EXO_REQUEST_ACCEPTED,
	EXO_REQUEST_ALREADY_SATISFIED,
	EXO_REQUEST_ALREADY_PENDING,
	EXO_REQUEST_REJECTED_NOT_READY,
	EXO_REQUEST_REJECTED_BUSY,
	EXO_REQUEST_REJECTED_ERROR,
	EXO_REQUEST_INTERNAL_ERROR,
} exo_request_result_t;

typedef struct {
	bool initialized;
	exo_system_state_t state;
	exo_system_step_t active_step;
	bool ready_to_start;
	exo_start_block_reason_t start_block_reason;
	bool run_requested;
	bool output_allowed;
	bool maintenance_active;

	bool executor_task_alive;
	uintptr_t executor_task_identity;

	uint32_t transition_count;
	uint32_t start_request_count;
	uint32_t stop_request_count;
	uint32_t start_success_count;
	uint32_t start_failure_count;
	uint32_t start_cancel_count;
	uint32_t stop_success_count;
	uint32_t stop_failure_count;
	uint32_t rejected_request_count;

	exo_request_result_t last_request_result;
	esp_err_t last_error;
	exo_system_step_t last_failed_step;
	uint64_t state_entered_us;
	uint64_t last_request_us;
	uint64_t last_failure_us;
	uint64_t last_completed_us;
} exo_system_snapshot_t;

/* Creates the static, event-driven executor task and leaves the system INIT. */
esp_err_t exo_system_init(void);

/* Publishes IDLE after all application subsystems have been initialized. */
esp_err_t exo_system_finish_initialization(void);

/* These request APIs never transmit CAN frames, delay, or wait for feedback. */
exo_request_result_t exo_system_request_start(void);
exo_request_result_t exo_system_request_stop(void);

/* The sole product output gate, called only by the permanent 100 Hz task. */
esp_err_t exo_system_process_control_tick(int64_t now_us);

/* Copies one coherent system snapshot; readiness is derived when requested. */
bool exo_system_get_snapshot(exo_system_snapshot_t *snapshot);

/*
 * Serializes console maintenance with START/STOP hardware sequences. A
 * successful begin keeps the mutex until exo_system_end_maintenance().
 */
bool exo_system_try_begin_maintenance(void);
void exo_system_end_maintenance(void);

const char *exo_system_state_name(exo_system_state_t state);
const char *exo_system_step_name(exo_system_step_t step);
const char *exo_system_start_block_reason_name(exo_start_block_reason_t reason);
const char *exo_system_request_result_name(exo_request_result_t result);

#ifdef __cplusplus
}
#endif

#endif /* EXO_SYSTEM_H */
