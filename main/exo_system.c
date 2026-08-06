#include "exo_system.h"

#include <math.h>
#include <string.h>

#include "driver/twai.h"
#include "esp_timer.h"
#include "exo_control_loop.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hip_joint.h"
#include "impedance_control.h"
#include "rs01_driver.h"

#define EXO_EXECUTOR_TASK_STACK_BYTES 4096U
#define EXO_EXECUTOR_TASK_PRIORITY 6U
#define EXO_EXECUTOR_TASK_CORE 0
#define EXO_START_DISABLE_DELAY_MS 50U
#define EXO_START_MODE_DELAY_MS 20U
#define EXO_START_ENABLE_DELAY_MS 50U
#define EXO_RELEASE_DELAY_MS 20U
#define EXO_DISABLE_DELAY_MS 50U
#define EXO_FEEDBACK_POLL_MS 10U
#define EXO_FEEDBACK_WAIT_MS 500U

typedef struct {
	esp_err_t error;
	bool canceled;
	exo_start_block_reason_t block_reason;
} start_sequence_result_t;

static StaticTask_t s_executor_task_buffer;
static StackType_t
	s_executor_task_stack[EXO_EXECUTOR_TASK_STACK_BYTES / sizeof(StackType_t)];
static StaticSemaphore_t s_operation_mutex_buffer;
static SemaphoreHandle_t s_operation_mutex;
static TaskHandle_t s_executor_task;
static exo_system_snapshot_t s_snapshot;
static bool s_initialized;
static uint32_t s_maintenance_generation;
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;

static uint64_t now_us(void) { return (uint64_t)esp_timer_get_time(); }

static void set_state_locked(exo_system_state_t state, uint64_t timestamp_us) {
	if (s_snapshot.state == state) {
		return;
	}
	s_snapshot.state = state;
	s_snapshot.state_entered_us = timestamp_us;
	++s_snapshot.transition_count;
}

static bool request_is_rejected(exo_request_result_t result) {
	return result == EXO_REQUEST_REJECTED_NOT_READY ||
		   result == EXO_REQUEST_REJECTED_BUSY ||
		   result == EXO_REQUEST_REJECTED_ERROR ||
		   result == EXO_REQUEST_INTERNAL_ERROR;
}

static void record_request_locked(exo_request_result_t result,
								  uint64_t timestamp_us) {
	s_snapshot.last_request_result = result;
	s_snapshot.last_request_us = timestamp_us;
	if (request_is_rejected(result)) {
		++s_snapshot.rejected_request_count;
	}
}

static void set_active_step(exo_system_step_t step) {
	portENTER_CRITICAL(&s_state_lock);
	s_snapshot.active_step = step;
	portEXIT_CRITICAL(&s_state_lock);
}

static void set_start_block_reason(exo_start_block_reason_t reason) {
	portENTER_CRITICAL(&s_state_lock);
	s_snapshot.start_block_reason = reason;
	portEXIT_CRITICAL(&s_state_lock);
}

static exo_start_block_reason_t
validate_joint(exo_hip_joint_id_t joint, const hip_joint_snapshot_t *snapshot) {
	const bool left = joint == EXO_HIP_JOINT_LEFT;
	if (!snapshot->mapping_valid) {
		return left ? EXO_START_BLOCK_LEFT_MAPPING_INVALID
					: EXO_START_BLOCK_RIGHT_MAPPING_INVALID;
	}
	if (!snapshot->direction_valid) {
		return left ? EXO_START_BLOCK_LEFT_DIRECTION_INVALID
					: EXO_START_BLOCK_RIGHT_DIRECTION_INVALID;
	}
	if (!snapshot->zero_valid) {
		return left ? EXO_START_BLOCK_LEFT_ZERO_INVALID
					: EXO_START_BLOCK_RIGHT_ZERO_INVALID;
	}
	if (!snapshot->feedback_valid) {
		return left ? EXO_START_BLOCK_LEFT_FEEDBACK_INVALID
					: EXO_START_BLOCK_RIGHT_FEEDBACK_INVALID;
	}
	if (!snapshot->online) {
		return left ? EXO_START_BLOCK_LEFT_OFFLINE
					: EXO_START_BLOCK_RIGHT_OFFLINE;
	}
	if (!snapshot->joint_valid) {
		return left ? EXO_START_BLOCK_LEFT_FEEDBACK_INVALID
					: EXO_START_BLOCK_RIGHT_FEEDBACK_INVALID;
	}

	rs01_feedback_snapshot_t motor;
	if (!rs01_get_feedback(snapshot->actuator, &motor) || !motor.valid) {
		return left ? EXO_START_BLOCK_LEFT_FEEDBACK_INVALID
					: EXO_START_BLOCK_RIGHT_FEEDBACK_INVALID;
	}
	if (!motor.online) {
		return left ? EXO_START_BLOCK_LEFT_OFFLINE
					: EXO_START_BLOCK_RIGHT_OFFLINE;
	}
	if (motor.faults.present) {
		return left ? EXO_START_BLOCK_LEFT_MOTOR_FAULT
					: EXO_START_BLOCK_RIGHT_MOTOR_FAULT;
	}
	return EXO_START_BLOCK_NONE;
}

static exo_start_block_reason_t hardware_start_block_reason(void) {
	exo_control_loop_snapshot_t control;
	if (!exo_control_loop_get_snapshot(&control) || !control.initialized ||
		!control.task_alive) {
		return EXO_START_BLOCK_CONTROL_TASK_NOT_READY;
	}

	twai_status_info_t twai_status;
	if (twai_get_status_info(&twai_status) != ESP_OK ||
		twai_status.state != TWAI_STATE_RUNNING) {
		return EXO_START_BLOCK_TWAI_NOT_RUNNING;
	}

	hip_joint_snapshot_t left;
	hip_joint_snapshot_t right;
	if (!hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT, &left)) {
		return EXO_START_BLOCK_LEFT_MAPPING_INVALID;
	}
	if (!hip_joint_get_snapshot(EXO_HIP_JOINT_RIGHT, &right)) {
		return EXO_START_BLOCK_RIGHT_MAPPING_INVALID;
	}

	exo_start_block_reason_t reason = validate_joint(EXO_HIP_JOINT_LEFT, &left);
	if (reason != EXO_START_BLOCK_NONE) {
		return reason;
	}
	return validate_joint(EXO_HIP_JOINT_RIGHT, &right);
}

static bool start_still_requested(void) {
	portENTER_CRITICAL(&s_state_lock);
	const bool requested = s_snapshot.state == EXO_SYSTEM_STATE_STARTING &&
						   s_snapshot.run_requested;
	portEXIT_CRITICAL(&s_state_lock);
	return requested;
}

static bool wait_start_delay_ms(uint32_t delay_ms) {
	TickType_t total_ticks = pdMS_TO_TICKS(delay_ms);
	if (delay_ms > 0U && total_ticks == 0U) {
		total_ticks = 1U;
	}
	const TickType_t start_tick = xTaskGetTickCount();
	while ((xTaskGetTickCount() - start_tick) < total_ticks) {
		const TickType_t elapsed = xTaskGetTickCount() - start_tick;
		(void)ulTaskNotifyTake(pdTRUE, total_ticks - elapsed);
		if (!start_still_requested()) {
			return false;
		}
	}
	return start_still_requested();
}

static void executor_delay_ms(uint32_t delay_ms) {
	TickType_t total_ticks = pdMS_TO_TICKS(delay_ms);
	if (delay_ms > 0U && total_ticks == 0U) {
		total_ticks = 1U;
	}
	const TickType_t start_tick = xTaskGetTickCount();
	while ((xTaskGetTickCount() - start_tick) < total_ticks) {
		const TickType_t elapsed = xTaskGetTickCount() - start_tick;
		(void)ulTaskNotifyTake(pdTRUE, total_ticks - elapsed);
	}
}

static bool get_motor_pair(rs01_feedback_snapshot_t *feedback_a,
						   rs01_feedback_snapshot_t *feedback_b) {
	return rs01_get_feedback(EXO_ACTUATOR_A, feedback_a) &&
		   rs01_get_feedback(EXO_ACTUATOR_B, feedback_b);
}

static bool
running_feedback_is_confirmed(uint32_t count_a, uint32_t count_b,
							  rs01_feedback_snapshot_t *feedback_a,
							  rs01_feedback_snapshot_t *feedback_b) {
	if (!get_motor_pair(feedback_a, feedback_b)) {
		return false;
	}
	return feedback_a->valid && feedback_b->valid && feedback_a->online &&
		   feedback_b->online && feedback_a->feedback_count != count_a &&
		   feedback_b->feedback_count != count_b &&
		   feedback_a->enabled_requested && feedback_b->enabled_requested &&
		   feedback_a->control_mode == RS01_CONTROL_MODE_MOTION &&
		   feedback_b->control_mode == RS01_CONTROL_MODE_MOTION &&
		   feedback_a->mode == RS01_MODE_STATUS_RUNNING &&
		   feedback_b->mode == RS01_MODE_STATUS_RUNNING &&
		   !feedback_a->faults.present && !feedback_b->faults.present;
}

static bool stopped_feedback_is_confirmed(uint32_t count_a, uint32_t count_b) {
	rs01_feedback_snapshot_t feedback_a;
	rs01_feedback_snapshot_t feedback_b;
	if (!get_motor_pair(&feedback_a, &feedback_b)) {
		return false;
	}
	return feedback_a.valid && feedback_b.valid && feedback_a.online &&
		   feedback_b.online && feedback_a.feedback_count != count_a &&
		   feedback_b.feedback_count != count_b &&
		   !feedback_a.enabled_requested && !feedback_b.enabled_requested &&
		   feedback_a.mode == RS01_MODE_STATUS_RESET &&
		   feedback_b.mode == RS01_MODE_STATUS_RESET;
}

static esp_err_t
wait_for_running_feedback(uint32_t count_a, uint32_t count_b,
						  rs01_feedback_snapshot_t *feedback_a,
						  rs01_feedback_snapshot_t *feedback_b) {
	const TickType_t timeout_ticks = pdMS_TO_TICKS(EXO_FEEDBACK_WAIT_MS);
	const TickType_t poll_ticks = pdMS_TO_TICKS(EXO_FEEDBACK_POLL_MS);
	const TickType_t start_tick = xTaskGetTickCount();
	for (;;) {
		if (!start_still_requested()) {
			return ESP_ERR_INVALID_STATE;
		}
		if (running_feedback_is_confirmed(count_a, count_b, feedback_a,
										  feedback_b)) {
			return ESP_OK;
		}
		const TickType_t elapsed = xTaskGetTickCount() - start_tick;
		if (elapsed >= timeout_ticks) {
			return ESP_ERR_TIMEOUT;
		}
		const TickType_t remaining = timeout_ticks - elapsed;
		(void)ulTaskNotifyTake(pdTRUE,
							   remaining < poll_ticks ? remaining : poll_ticks);
	}
}

static esp_err_t wait_for_stopped_feedback(uint32_t count_a, uint32_t count_b) {
	const TickType_t timeout_ticks = pdMS_TO_TICKS(EXO_FEEDBACK_WAIT_MS);
	const TickType_t poll_ticks = pdMS_TO_TICKS(EXO_FEEDBACK_POLL_MS);
	const TickType_t start_tick = xTaskGetTickCount();
	for (;;) {
		if (stopped_feedback_is_confirmed(count_a, count_b)) {
			return ESP_OK;
		}
		const TickType_t elapsed = xTaskGetTickCount() - start_tick;
		if (elapsed >= timeout_ticks) {
			return ESP_ERR_TIMEOUT;
		}
		const TickType_t remaining = timeout_ticks - elapsed;
		(void)ulTaskNotifyTake(pdTRUE,
							   remaining < poll_ticks ? remaining : poll_ticks);
	}
}

static start_sequence_result_t start_result(esp_err_t error, bool canceled,
											exo_start_block_reason_t reason) {
	const start_sequence_result_t result = {
		.error = error,
		.canceled = canceled,
		.block_reason = reason,
	};
	return result;
}

static start_sequence_result_t canceled_start(void) {
	return start_result(ESP_OK, true, EXO_START_BLOCK_NONE);
}

static start_sequence_result_t failed_start(esp_err_t error) {
	return start_result(error, false, EXO_START_BLOCK_NONE);
}

static start_sequence_result_t execute_start_sequence(void) {
	set_active_step(EXO_SYSTEM_STEP_CHECK_PRECONDITIONS);
	if (!start_still_requested()) {
		return canceled_start();
	}
	const exo_start_block_reason_t reason = hardware_start_block_reason();
	if (reason != EXO_START_BLOCK_NONE) {
		set_start_block_reason(reason);
		return start_result(ESP_ERR_INVALID_STATE, false, reason);
	}

	rs01_feedback_snapshot_t feedback_a;
	rs01_feedback_snapshot_t feedback_b;
	if (!get_motor_pair(&feedback_a, &feedback_b)) {
		return failed_start(ESP_ERR_INVALID_STATE);
	}

	set_active_step(EXO_SYSTEM_STEP_DISABLE_MOTORS);
	esp_err_t err = rs01_disable(EXO_ACTUATOR_A);
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!wait_start_delay_ms(EXO_START_DISABLE_DELAY_MS)) {
		return canceled_start();
	}
	err = rs01_disable(EXO_ACTUATOR_B);
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!wait_start_delay_ms(EXO_START_DISABLE_DELAY_MS)) {
		return canceled_start();
	}

	set_active_step(EXO_SYSTEM_STEP_SET_MOTION_MODE);
	err = rs01_set_control_mode(EXO_ACTUATOR_A, RS01_CONTROL_MODE_MOTION);
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!wait_start_delay_ms(EXO_START_MODE_DELAY_MS)) {
		return canceled_start();
	}
	err = rs01_set_control_mode(EXO_ACTUATOR_B, RS01_CONTROL_MODE_MOTION);
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!wait_start_delay_ms(EXO_START_MODE_DELAY_MS)) {
		return canceled_start();
	}

	set_active_step(EXO_SYSTEM_STEP_ENABLE_MOTORS);
	err = rs01_enable(EXO_ACTUATOR_A);
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!wait_start_delay_ms(EXO_START_ENABLE_DELAY_MS)) {
		return canceled_start();
	}
	err = rs01_enable(EXO_ACTUATOR_B);
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!wait_start_delay_ms(EXO_START_ENABLE_DELAY_MS)) {
		return canceled_start();
	}

	if (!get_motor_pair(&feedback_a, &feedback_b) || !feedback_a.valid ||
		!feedback_b.valid || !feedback_a.online || !feedback_b.online ||
		!isfinite(feedback_a.sample.position_rad) ||
		!isfinite(feedback_b.sample.position_rad)) {
		return failed_start(ESP_ERR_INVALID_STATE);
	}
	const uint32_t count_a = feedback_a.feedback_count;
	const uint32_t count_b = feedback_b.feedback_count;

	set_active_step(EXO_SYSTEM_STEP_SEND_ZERO_OUTPUT);
	err = rs01_send_motion(EXO_ACTUATOR_A, feedback_a.sample.position_rad, 0.0f,
						   0.0f, 0.0f, 0.0f);
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!start_still_requested()) {
		return canceled_start();
	}
	err = rs01_send_motion(EXO_ACTUATOR_B, feedback_b.sample.position_rad, 0.0f,
						   0.0f, 0.0f, 0.0f);
	if (err != ESP_OK) {
		return failed_start(err);
	}

	set_active_step(EXO_SYSTEM_STEP_WAIT_FRESH_FEEDBACK);
	err = wait_for_running_feedback(count_a, count_b, &feedback_a, &feedback_b);
	if (err != ESP_OK) {
		return start_still_requested() ? failed_start(err) : canceled_start();
	}
	if (!isfinite(feedback_a.sample.position_rad) ||
		!isfinite(feedback_b.sample.position_rad)) {
		return failed_start(ESP_ERR_INVALID_STATE);
	}

	set_active_step(EXO_SYSTEM_STEP_ARM_CONTROLLER);
	err = impedance_control_arm(feedback_a.sample.position_rad,
								feedback_b.sample.position_rad,
								(int64_t)now_us());
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!start_still_requested()) {
		return canceled_start();
	}
	return start_result(ESP_OK, false, EXO_START_BLOCK_NONE);
}

static esp_err_t first_error(esp_err_t current, esp_err_t candidate) {
	return current == ESP_OK ? candidate : current;
}

static esp_err_t execute_stop_sequence(void) {
	set_active_step(EXO_SYSTEM_STEP_DISARM_CONTROLLER);
	esp_err_t command_error = impedance_control_disarm();

	rs01_feedback_snapshot_t feedback_a;
	rs01_feedback_snapshot_t feedback_b;
	const bool have_feedback = get_motor_pair(&feedback_a, &feedback_b);

	set_active_step(EXO_SYSTEM_STEP_RELEASE_OUTPUT);
	if (have_feedback && feedback_a.valid && feedback_a.online &&
		isfinite(feedback_a.sample.position_rad)) {
		command_error = first_error(
			command_error,
			rs01_send_motion(EXO_ACTUATOR_A, feedback_a.sample.position_rad,
							 0.0f, 0.0f, 0.0f, 0.0f));
	} else {
		command_error = first_error(command_error, ESP_ERR_INVALID_STATE);
	}
	executor_delay_ms(EXO_RELEASE_DELAY_MS);
	if (have_feedback && feedback_b.valid && feedback_b.online &&
		isfinite(feedback_b.sample.position_rad)) {
		command_error = first_error(
			command_error,
			rs01_send_motion(EXO_ACTUATOR_B, feedback_b.sample.position_rad,
							 0.0f, 0.0f, 0.0f, 0.0f));
	} else {
		command_error = first_error(command_error, ESP_ERR_INVALID_STATE);
	}
	executor_delay_ms(EXO_RELEASE_DELAY_MS);

	uint32_t count_a = 0U;
	uint32_t count_b = 0U;
	if (get_motor_pair(&feedback_a, &feedback_b)) {
		count_a = feedback_a.feedback_count;
		count_b = feedback_b.feedback_count;
	} else {
		command_error = first_error(command_error, ESP_ERR_INVALID_STATE);
	}

	set_active_step(EXO_SYSTEM_STEP_DISABLE_MOTORS);
	command_error = first_error(command_error, rs01_disable(EXO_ACTUATOR_A));
	executor_delay_ms(EXO_DISABLE_DELAY_MS);
	command_error = first_error(command_error, rs01_disable(EXO_ACTUATOR_B));
	executor_delay_ms(EXO_DISABLE_DELAY_MS);

	set_active_step(EXO_SYSTEM_STEP_VERIFY_DISABLED);
	const esp_err_t confirmation_error =
		wait_for_stopped_feedback(count_a, count_b);
	/* Fresh REST feedback is authoritative even if a redundant release or
	 * disable enqueue reported an earlier error. ERROR is reserved for a
	 * cleanup whose final disabled state cannot be confirmed. */
	return confirmation_error == ESP_OK
			   ? ESP_OK
			   : first_error(command_error, confirmation_error);
}

static void finalize_stop(esp_err_t stop_error, esp_err_t preserved_error,
						  bool count_explicit_stop) {
	const uint64_t timestamp_us = now_us();
	portENTER_CRITICAL(&s_state_lock);
	const exo_system_step_t failed_step = s_snapshot.active_step;
	s_snapshot.run_requested = false;
	s_snapshot.output_allowed = false;
	s_snapshot.active_step = EXO_SYSTEM_STEP_NONE;
	s_snapshot.last_completed_us = timestamp_us;
	if (stop_error == ESP_OK) {
		set_state_locked(EXO_SYSTEM_STATE_IDLE, timestamp_us);
		if (count_explicit_stop) {
			++s_snapshot.stop_success_count;
		}
		s_snapshot.last_error = preserved_error;
	} else {
		set_state_locked(EXO_SYSTEM_STATE_ERROR, timestamp_us);
		++s_snapshot.stop_failure_count;
		s_snapshot.last_error = stop_error;
		s_snapshot.last_failed_step = failed_step;
		s_snapshot.last_failure_us = timestamp_us;
		s_snapshot.start_block_reason = EXO_START_BLOCK_SYSTEM_ERROR;
	}
	portEXIT_CRITICAL(&s_state_lock);
}

static void handle_start(void) {
	if (xSemaphoreTake(s_operation_mutex, portMAX_DELAY) != pdTRUE) {
		return;
	}

	if (!start_still_requested()) {
		const esp_err_t stop_error = execute_stop_sequence();
		xSemaphoreGive(s_operation_mutex);
		finalize_stop(stop_error, ESP_OK, true);
		return;
	}

	const start_sequence_result_t result = execute_start_sequence();
	bool canceled = result.canceled;
	if (result.error == ESP_OK && !result.canceled) {
		const uint64_t timestamp_us = now_us();
		bool published = false;
		portENTER_CRITICAL(&s_state_lock);
		if (s_snapshot.state == EXO_SYSTEM_STATE_STARTING &&
			s_snapshot.run_requested) {
			s_snapshot.output_allowed = true;
			s_snapshot.active_step = EXO_SYSTEM_STEP_NONE;
			s_snapshot.start_block_reason = EXO_START_BLOCK_NONE;
			s_snapshot.last_error = ESP_OK;
			s_snapshot.last_completed_us = timestamp_us;
			++s_snapshot.start_success_count;
			set_state_locked(EXO_SYSTEM_STATE_RUNNING, timestamp_us);
			published = true;
		}
		portEXIT_CRITICAL(&s_state_lock);
		if (published) {
			xSemaphoreGive(s_operation_mutex);
			return;
		}
		/* STOP won the state lock after arm but before RUNNING publication. */
		canceled = true;
	}

	const uint64_t timestamp_us = now_us();
	portENTER_CRITICAL(&s_state_lock);
	const exo_system_step_t failed_step = s_snapshot.active_step;
	/* A STOP that published STOPPING before this result is classified as the
	 * winner of the transition, even if a driver call returned concurrently. */
	if (s_snapshot.state == EXO_SYSTEM_STATE_STOPPING) {
		canceled = true;
	}
	s_snapshot.output_allowed = false;
	s_snapshot.run_requested = false;
	if (s_snapshot.state != EXO_SYSTEM_STATE_STOPPING) {
		set_state_locked(EXO_SYSTEM_STATE_STOPPING, timestamp_us);
	}
	if (!canceled) {
		++s_snapshot.start_failure_count;
		s_snapshot.last_error = result.error;
		s_snapshot.last_failed_step = failed_step;
		s_snapshot.last_failure_us = timestamp_us;
		if (result.block_reason != EXO_START_BLOCK_NONE) {
			s_snapshot.start_block_reason = result.block_reason;
		}
	}
	portEXIT_CRITICAL(&s_state_lock);

	const esp_err_t stop_error = execute_stop_sequence();
	xSemaphoreGive(s_operation_mutex);
	finalize_stop(stop_error, canceled ? ESP_OK : result.error, canceled);
}

static void handle_stop(void) {
	if (xSemaphoreTake(s_operation_mutex, portMAX_DELAY) != pdTRUE) {
		return;
	}
	const esp_err_t error = execute_stop_sequence();
	xSemaphoreGive(s_operation_mutex);
	finalize_stop(error, ESP_OK, true);
}

static void executor_task(void *arg) {
	(void)arg;
	const TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
	portENTER_CRITICAL(&s_state_lock);
	s_snapshot.executor_task_alive = true;
	s_snapshot.executor_task_identity = (uintptr_t)current_task;
	portEXIT_CRITICAL(&s_state_lock);

	for (;;) {
		(void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		for (;;) {
			portENTER_CRITICAL(&s_state_lock);
			const exo_system_state_t state = s_snapshot.state;
			const bool run_requested = s_snapshot.run_requested;
			portEXIT_CRITICAL(&s_state_lock);

			if (state == EXO_SYSTEM_STATE_STARTING && run_requested) {
				handle_start();
				continue;
			}
			if (state == EXO_SYSTEM_STATE_STOPPING) {
				handle_stop();
				continue;
			}
			break;
		}
	}
}

esp_err_t exo_system_init(void) {
	const uint64_t timestamp_us = now_us();
	portENTER_CRITICAL(&s_state_lock);
	if (s_initialized) {
		portEXIT_CRITICAL(&s_state_lock);
		return ESP_OK;
	}
	portEXIT_CRITICAL(&s_state_lock);

	SemaphoreHandle_t mutex =
		xSemaphoreCreateMutexStatic(&s_operation_mutex_buffer);
	if (mutex == NULL) {
		return ESP_ERR_NO_MEM;
	}

	portENTER_CRITICAL(&s_state_lock);
	memset(&s_snapshot, 0, sizeof(s_snapshot));
	s_snapshot.initialized = true;
	s_snapshot.state = EXO_SYSTEM_STATE_INIT;
	s_snapshot.active_step = EXO_SYSTEM_STEP_NONE;
	s_snapshot.start_block_reason = EXO_START_BLOCK_INITIALIZING;
	s_snapshot.last_request_result = EXO_REQUEST_NONE;
	s_snapshot.last_error = ESP_OK;
	s_snapshot.state_entered_us = timestamp_us;
	s_maintenance_generation = 0U;
	s_operation_mutex = mutex;
	s_initialized = true;
	portEXIT_CRITICAL(&s_state_lock);

	TaskHandle_t task = xTaskCreateStaticPinnedToCore(
		executor_task, "exo_executor", EXO_EXECUTOR_TASK_STACK_BYTES, NULL,
		EXO_EXECUTOR_TASK_PRIORITY, s_executor_task_stack,
		&s_executor_task_buffer, EXO_EXECUTOR_TASK_CORE);
	if (task == NULL) {
		portENTER_CRITICAL(&s_state_lock);
		s_initialized = false;
		s_operation_mutex = NULL;
		memset(&s_snapshot, 0, sizeof(s_snapshot));
		portEXIT_CRITICAL(&s_state_lock);
		return ESP_ERR_NO_MEM;
	}

	portENTER_CRITICAL(&s_state_lock);
	s_executor_task = task;
	s_snapshot.executor_task_identity = (uintptr_t)task;
	portEXIT_CRITICAL(&s_state_lock);
	return ESP_OK;
}

esp_err_t exo_system_finish_initialization(void) {
	const uint64_t timestamp_us = now_us();
	portENTER_CRITICAL(&s_state_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_state_lock);
		return ESP_ERR_INVALID_STATE;
	}
	if (s_snapshot.state == EXO_SYSTEM_STATE_IDLE) {
		portEXIT_CRITICAL(&s_state_lock);
		return ESP_OK;
	}
	if (s_snapshot.state != EXO_SYSTEM_STATE_INIT) {
		portEXIT_CRITICAL(&s_state_lock);
		return ESP_ERR_INVALID_STATE;
	}
	s_snapshot.run_requested = false;
	s_snapshot.output_allowed = false;
	s_snapshot.start_block_reason = EXO_START_BLOCK_NONE;
	set_state_locked(EXO_SYSTEM_STATE_IDLE, timestamp_us);
	portEXIT_CRITICAL(&s_state_lock);
	return ESP_OK;
}

static exo_request_result_t start_result_for_state_locked(void) {
	switch (s_snapshot.state) {
	case EXO_SYSTEM_STATE_INIT:
		s_snapshot.start_block_reason = EXO_START_BLOCK_INITIALIZING;
		return EXO_REQUEST_REJECTED_NOT_READY;
	case EXO_SYSTEM_STATE_STARTING:
		return EXO_REQUEST_ALREADY_PENDING;
	case EXO_SYSTEM_STATE_RUNNING:
		return EXO_REQUEST_ALREADY_SATISFIED;
	case EXO_SYSTEM_STATE_STOPPING:
		s_snapshot.start_block_reason = EXO_START_BLOCK_SYSTEM_NOT_IDLE;
		return EXO_REQUEST_REJECTED_BUSY;
	case EXO_SYSTEM_STATE_ERROR:
		s_snapshot.start_block_reason = EXO_START_BLOCK_SYSTEM_ERROR;
		return EXO_REQUEST_REJECTED_ERROR;
	case EXO_SYSTEM_STATE_IDLE:
	default:
		return EXO_REQUEST_NONE;
	}
}

exo_request_result_t exo_system_request_start(void) {
	const uint64_t request_timestamp_us = now_us();
	portENTER_CRITICAL(&s_state_lock);
	++s_snapshot.start_request_count;
	if (!s_initialized || s_executor_task == NULL) {
		record_request_locked(EXO_REQUEST_INTERNAL_ERROR, request_timestamp_us);
		portEXIT_CRITICAL(&s_state_lock);
		return EXO_REQUEST_INTERNAL_ERROR;
	}
	exo_request_result_t result = start_result_for_state_locked();
	if (result != EXO_REQUEST_NONE) {
		record_request_locked(result, request_timestamp_us);
		portEXIT_CRITICAL(&s_state_lock);
		return result;
	}
	portEXIT_CRITICAL(&s_state_lock);

	if (xSemaphoreTake(s_operation_mutex, 0) != pdTRUE) {
		portENTER_CRITICAL(&s_state_lock);
		result = start_result_for_state_locked();
		if (result == EXO_REQUEST_NONE) {
			result = EXO_REQUEST_REJECTED_BUSY;
			s_snapshot.start_block_reason = EXO_START_BLOCK_OPERATION_BUSY;
		}
		record_request_locked(result, request_timestamp_us);
		portEXIT_CRITICAL(&s_state_lock);
		return result;
	}

	portENTER_CRITICAL(&s_state_lock);
	result = start_result_for_state_locked();
	portEXIT_CRITICAL(&s_state_lock);
	if (result != EXO_REQUEST_NONE) {
		portENTER_CRITICAL(&s_state_lock);
		record_request_locked(result, request_timestamp_us);
		portEXIT_CRITICAL(&s_state_lock);
		xSemaphoreGive(s_operation_mutex);
		return result;
	}

	const exo_start_block_reason_t reason = hardware_start_block_reason();
	const uint64_t timestamp_us = now_us();
	portENTER_CRITICAL(&s_state_lock);
	if (reason != EXO_START_BLOCK_NONE) {
		s_snapshot.start_block_reason = reason;
		result = EXO_REQUEST_REJECTED_NOT_READY;
	} else {
		s_snapshot.run_requested = true;
		s_snapshot.output_allowed = false;
		s_snapshot.active_step = EXO_SYSTEM_STEP_NONE;
		s_snapshot.start_block_reason = EXO_START_BLOCK_NONE;
		set_state_locked(EXO_SYSTEM_STATE_STARTING, timestamp_us);
		result = EXO_REQUEST_ACCEPTED;
	}
	record_request_locked(result, request_timestamp_us);
	const TaskHandle_t task = s_executor_task;
	portEXIT_CRITICAL(&s_state_lock);
	xSemaphoreGive(s_operation_mutex);

	if (result == EXO_REQUEST_ACCEPTED) {
		xTaskNotifyGive(task);
	}
	return result;
}

exo_request_result_t exo_system_request_stop(void) {
	const uint64_t timestamp_us = now_us();
	TaskHandle_t task = NULL;
	exo_request_result_t result;
	portENTER_CRITICAL(&s_state_lock);
	++s_snapshot.stop_request_count;
	if (!s_initialized || s_executor_task == NULL) {
		record_request_locked(EXO_REQUEST_INTERNAL_ERROR, timestamp_us);
		portEXIT_CRITICAL(&s_state_lock);
		return EXO_REQUEST_INTERNAL_ERROR;
	}

	switch (s_snapshot.state) {
	case EXO_SYSTEM_STATE_INIT:
	case EXO_SYSTEM_STATE_IDLE:
		result = EXO_REQUEST_ALREADY_SATISFIED;
		break;
	case EXO_SYSTEM_STATE_STOPPING:
		result = EXO_REQUEST_ALREADY_PENDING;
		break;
	case EXO_SYSTEM_STATE_STARTING:
		/* Record cancellation at the STOP linearization point. This also
		 * covers STOP before the executor has begun the START sequence. */
		++s_snapshot.start_cancel_count;
		s_snapshot.run_requested = false;
		s_snapshot.output_allowed = false;
		set_state_locked(EXO_SYSTEM_STATE_STOPPING, timestamp_us);
		result = EXO_REQUEST_ACCEPTED;
		task = s_executor_task;
		break;
	case EXO_SYSTEM_STATE_RUNNING:
	case EXO_SYSTEM_STATE_ERROR:
		s_snapshot.run_requested = false;
		s_snapshot.output_allowed = false;
		set_state_locked(EXO_SYSTEM_STATE_STOPPING, timestamp_us);
		result = EXO_REQUEST_ACCEPTED;
		task = s_executor_task;
		break;
	default:
		result = EXO_REQUEST_INTERNAL_ERROR;
		break;
	}
	record_request_locked(result, timestamp_us);
	portEXIT_CRITICAL(&s_state_lock);

	if (task != NULL) {
		xTaskNotifyGive(task);
	}
	return result;
}

esp_err_t exo_system_process_control_tick(int64_t now) {
	portENTER_CRITICAL(&s_state_lock);
	const bool output_allowed = s_snapshot.state == EXO_SYSTEM_STATE_RUNNING &&
								s_snapshot.run_requested &&
								s_snapshot.output_allowed;
	portEXIT_CRITICAL(&s_state_lock);
	return output_allowed ? impedance_control_process_tick(now) : ESP_OK;
}

bool exo_system_get_snapshot(exo_system_snapshot_t *snapshot) {
	if (snapshot == NULL) {
		return false;
	}

	portENTER_CRITICAL(&s_state_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_state_lock);
		memset(snapshot, 0, sizeof(*snapshot));
		return false;
	}
	*snapshot = s_snapshot;
	const uint32_t maintenance_generation = s_maintenance_generation;
	portEXIT_CRITICAL(&s_state_lock);

	snapshot->ready_to_start = false;
	if (snapshot->state == EXO_SYSTEM_STATE_INIT) {
		snapshot->start_block_reason = EXO_START_BLOCK_INITIALIZING;
		return true;
	}
	if (snapshot->state == EXO_SYSTEM_STATE_ERROR) {
		snapshot->start_block_reason = EXO_START_BLOCK_SYSTEM_ERROR;
		return true;
	}
	if (snapshot->state != EXO_SYSTEM_STATE_IDLE) {
		snapshot->start_block_reason = EXO_START_BLOCK_SYSTEM_NOT_IDLE;
		return true;
	}
	if (snapshot->maintenance_active) {
		snapshot->start_block_reason = EXO_START_BLOCK_OPERATION_BUSY;
		return true;
	}
	const exo_start_block_reason_t reason = hardware_start_block_reason();

	portENTER_CRITICAL(&s_state_lock);
	const bool unchanged =
		s_snapshot.state == EXO_SYSTEM_STATE_IDLE &&
		s_snapshot.transition_count == snapshot->transition_count &&
		!s_snapshot.run_requested && !s_snapshot.output_allowed &&
		!s_snapshot.maintenance_active &&
		s_maintenance_generation == maintenance_generation;
	portEXIT_CRITICAL(&s_state_lock);
	if (unchanged) {
		snapshot->start_block_reason = reason;
		snapshot->ready_to_start = reason == EXO_START_BLOCK_NONE;
	} else {
		/* Do not combine the old IDLE copy with fields from a transition that
		 * raced the readiness reads. Return one newer coherent system copy and
		 * conservatively leave readiness false for this call. */
		portENTER_CRITICAL(&s_state_lock);
		*snapshot = s_snapshot;
		portEXIT_CRITICAL(&s_state_lock);
		snapshot->ready_to_start = false;
		if (snapshot->state == EXO_SYSTEM_STATE_INIT) {
			snapshot->start_block_reason = EXO_START_BLOCK_INITIALIZING;
		} else if (snapshot->state == EXO_SYSTEM_STATE_ERROR) {
			snapshot->start_block_reason = EXO_START_BLOCK_SYSTEM_ERROR;
		} else if (snapshot->state != EXO_SYSTEM_STATE_IDLE) {
			snapshot->start_block_reason = EXO_START_BLOCK_SYSTEM_NOT_IDLE;
		} else {
			snapshot->start_block_reason = EXO_START_BLOCK_OPERATION_BUSY;
		}
	}
	return true;
}

bool exo_system_try_begin_maintenance(void) {
	if (s_operation_mutex == NULL ||
		xSemaphoreTake(s_operation_mutex, 0) != pdTRUE) {
		return false;
	}

	portENTER_CRITICAL(&s_state_lock);
	const bool allowed =
		s_initialized && s_snapshot.state == EXO_SYSTEM_STATE_IDLE &&
		!s_snapshot.run_requested && !s_snapshot.output_allowed;
	if (allowed) {
		s_snapshot.maintenance_active = true;
		++s_maintenance_generation;
	}
	portEXIT_CRITICAL(&s_state_lock);
	if (!allowed) {
		xSemaphoreGive(s_operation_mutex);
	}
	return allowed;
}

void exo_system_end_maintenance(void) {
	if (s_operation_mutex != NULL) {
		portENTER_CRITICAL(&s_state_lock);
		s_snapshot.maintenance_active = false;
		++s_maintenance_generation;
		portEXIT_CRITICAL(&s_state_lock);
		(void)xSemaphoreGive(s_operation_mutex);
	}
}

const char *exo_system_state_name(exo_system_state_t state) {
	switch (state) {
	case EXO_SYSTEM_STATE_INIT:
		return "INIT";
	case EXO_SYSTEM_STATE_IDLE:
		return "IDLE";
	case EXO_SYSTEM_STATE_STARTING:
		return "STARTING";
	case EXO_SYSTEM_STATE_RUNNING:
		return "RUNNING";
	case EXO_SYSTEM_STATE_STOPPING:
		return "STOPPING";
	case EXO_SYSTEM_STATE_ERROR:
		return "ERROR";
	default:
		return "UNKNOWN";
	}
}

const char *exo_system_step_name(exo_system_step_t step) {
	switch (step) {
	case EXO_SYSTEM_STEP_NONE:
		return "NONE";
	case EXO_SYSTEM_STEP_CHECK_PRECONDITIONS:
		return "CHECK_PRECONDITIONS";
	case EXO_SYSTEM_STEP_DISABLE_MOTORS:
		return "DISABLE_MOTORS";
	case EXO_SYSTEM_STEP_SET_MOTION_MODE:
		return "SET_MOTION_MODE";
	case EXO_SYSTEM_STEP_ENABLE_MOTORS:
		return "ENABLE_MOTORS";
	case EXO_SYSTEM_STEP_SEND_ZERO_OUTPUT:
		return "SEND_ZERO_OUTPUT";
	case EXO_SYSTEM_STEP_WAIT_FRESH_FEEDBACK:
		return "WAIT_FRESH_FEEDBACK";
	case EXO_SYSTEM_STEP_ARM_CONTROLLER:
		return "ARM_CONTROLLER";
	case EXO_SYSTEM_STEP_DISARM_CONTROLLER:
		return "DISARM_CONTROLLER";
	case EXO_SYSTEM_STEP_RELEASE_OUTPUT:
		return "RELEASE_OUTPUT";
	case EXO_SYSTEM_STEP_VERIFY_DISABLED:
		return "VERIFY_DISABLED";
	default:
		return "UNKNOWN";
	}
}

const char *
exo_system_start_block_reason_name(exo_start_block_reason_t reason) {
	switch (reason) {
	case EXO_START_BLOCK_NONE:
		return "NONE";
	case EXO_START_BLOCK_INITIALIZING:
		return "INITIALIZING";
	case EXO_START_BLOCK_SYSTEM_NOT_IDLE:
		return "SYSTEM_NOT_IDLE";
	case EXO_START_BLOCK_OPERATION_BUSY:
		return "OPERATION_BUSY";
	case EXO_START_BLOCK_CONTROL_TASK_NOT_READY:
		return "CONTROL_TASK_NOT_READY";
	case EXO_START_BLOCK_TWAI_NOT_RUNNING:
		return "TWAI_NOT_RUNNING";
	case EXO_START_BLOCK_LEFT_MAPPING_INVALID:
		return "LEFT_MAPPING_INVALID";
	case EXO_START_BLOCK_RIGHT_MAPPING_INVALID:
		return "RIGHT_MAPPING_INVALID";
	case EXO_START_BLOCK_LEFT_DIRECTION_INVALID:
		return "LEFT_DIRECTION_INVALID";
	case EXO_START_BLOCK_RIGHT_DIRECTION_INVALID:
		return "RIGHT_DIRECTION_INVALID";
	case EXO_START_BLOCK_LEFT_ZERO_INVALID:
		return "LEFT_ZERO_INVALID";
	case EXO_START_BLOCK_RIGHT_ZERO_INVALID:
		return "RIGHT_ZERO_INVALID";
	case EXO_START_BLOCK_LEFT_FEEDBACK_INVALID:
		return "LEFT_FEEDBACK_INVALID";
	case EXO_START_BLOCK_RIGHT_FEEDBACK_INVALID:
		return "RIGHT_FEEDBACK_INVALID";
	case EXO_START_BLOCK_LEFT_OFFLINE:
		return "LEFT_OFFLINE";
	case EXO_START_BLOCK_RIGHT_OFFLINE:
		return "RIGHT_OFFLINE";
	case EXO_START_BLOCK_LEFT_MOTOR_FAULT:
		return "LEFT_MOTOR_FAULT";
	case EXO_START_BLOCK_RIGHT_MOTOR_FAULT:
		return "RIGHT_MOTOR_FAULT";
	case EXO_START_BLOCK_SYSTEM_ERROR:
		return "SYSTEM_ERROR";
	default:
		return "UNKNOWN";
	}
}

const char *exo_system_request_result_name(exo_request_result_t result) {
	switch (result) {
	case EXO_REQUEST_NONE:
		return "NONE";
	case EXO_REQUEST_ACCEPTED:
		return "ACCEPTED";
	case EXO_REQUEST_ALREADY_SATISFIED:
		return "ALREADY_SATISFIED";
	case EXO_REQUEST_ALREADY_PENDING:
		return "ALREADY_PENDING";
	case EXO_REQUEST_REJECTED_NOT_READY:
		return "REJECTED_NOT_READY";
	case EXO_REQUEST_REJECTED_BUSY:
		return "REJECTED_BUSY";
	case EXO_REQUEST_REJECTED_ERROR:
		return "REJECTED_ERROR";
	case EXO_REQUEST_INTERNAL_ERROR:
		return "INTERNAL_ERROR";
	default:
		return "UNKNOWN";
	}
}
