#include "exo_control_loop.h"

#include <limits.h>
#include <string.h>

#include "esp_timer.h"
#include "exo_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hip_joint.h"
#include "impedance_control.h"

#define EXO_CONTROL_TASK_STACK_BYTES 4096U
#define EXO_CONTROL_TASK_PRIORITY 9U
#define EXO_CONTROL_TASK_CORE 1
#define EXO_CONTROL_PERIOD_TOLERANCE_US 1000U

typedef enum {
	CONTROL_LOOP_NOT_INITIALIZED = 0,
	CONTROL_LOOP_INITIALIZING,
	CONTROL_LOOP_INITIALIZED,
} control_loop_init_state_t;

typedef struct {
	exo_control_loop_snapshot_t snapshot;
	uint64_t period_total_us;
	uint64_t period_sample_count;
	int64_t previous_cycle_start_us;
} control_loop_diagnostics_t;

static StaticTask_t s_control_task_buffer;
static StackType_t
	s_control_task_stack[EXO_CONTROL_TASK_STACK_BYTES / sizeof(StackType_t)];
static control_loop_init_state_t s_init_state;
static exo_control_loop_snapshot_t s_published_snapshot;
static portMUX_TYPE s_diagnostics_lock = portMUX_INITIALIZER_UNLOCKED;

static uint32_t clamp_duration_us(int64_t duration_us) {
	if (duration_us <= 0) {
		return 0U;
	}
	if ((uint64_t)duration_us > UINT32_MAX) {
		return UINT32_MAX;
	}
	return (uint32_t)duration_us;
}

static int32_t clamp_period_error_us(int64_t error_us) {
	if (error_us > INT32_MAX) {
		return INT32_MAX;
	}
	if (error_us < INT32_MIN) {
		return INT32_MIN;
	}
	return (int32_t)error_us;
}

static exo_control_loop_joint_status_t
joint_status_from_snapshot(const hip_joint_snapshot_t *snapshot,
						   bool snapshot_available) {
	exo_control_loop_joint_status_t status = {0};
	if (snapshot_available) {
		status.feedback_count = snapshot->source_feedback_count;
		status.feedback_timestamp_us = snapshot->feedback_timestamp_us;
		status.joint_valid = snapshot->joint_valid;
	}
	return status;
}

static void update_cycle_diagnostics(
	control_loop_diagnostics_t *diagnostics, int64_t cycle_start_us,
	esp_err_t output_result, const hip_joint_snapshot_t *left_snapshot,
	bool left_available, const hip_joint_snapshot_t *right_snapshot,
	bool right_available, bool stack_sample_available,
	uint32_t stack_high_water_mark_bytes) {
	exo_control_loop_snapshot_t *snapshot = &diagnostics->snapshot;
	if (diagnostics->previous_cycle_start_us > 0) {
		const uint32_t period_us = clamp_duration_us(
			cycle_start_us - diagnostics->previous_cycle_start_us);
		const int32_t period_error_us = clamp_period_error_us(
			(int64_t)period_us - snapshot->target_period_us);
		const uint32_t abs_period_error_us =
			period_error_us < 0 ? (uint32_t)(-(int64_t)period_error_us)
								: (uint32_t)period_error_us;
		diagnostics->period_total_us += period_us;
		++diagnostics->period_sample_count;
		snapshot->period_sample_count = diagnostics->period_sample_count;
		if (abs_period_error_us > snapshot->period_tolerance_us) {
			++snapshot->period_out_of_tolerance_count;
		}
		snapshot->last_period_us = period_us;
		if (snapshot->min_period_us == 0U ||
			period_us < snapshot->min_period_us) {
			snapshot->min_period_us = period_us;
		}
		if (period_us > snapshot->max_period_us) {
			snapshot->max_period_us = period_us;
		}
		snapshot->mean_period_us = (uint32_t)(diagnostics->period_total_us /
											  diagnostics->period_sample_count);
		snapshot->last_period_error_us = period_error_us;
		if (abs_period_error_us > snapshot->max_abs_period_error_us) {
			snapshot->max_abs_period_error_us = abs_period_error_us;
		}
	}

	diagnostics->previous_cycle_start_us = cycle_start_us;
	snapshot->last_cycle_start_us = (uint64_t)cycle_start_us;
	++snapshot->cycle_count;
	snapshot->last_output_error = output_result;
	if (output_result != ESP_OK) {
		++snapshot->output_error_count;
	}
	snapshot->left = joint_status_from_snapshot(left_snapshot, left_available);
	snapshot->right =
		joint_status_from_snapshot(right_snapshot, right_available);
	if (stack_sample_available) {
		snapshot->stack_high_water_mark_bytes = stack_high_water_mark_bytes;
	}
}

static void update_timing_diagnostics(control_loop_diagnostics_t *diagnostics,
									  uint32_t execution_us,
									  uint64_t deadline_misses,
									  uint64_t skipped_cycles) {
	exo_control_loop_snapshot_t *snapshot = &diagnostics->snapshot;
	snapshot->last_execution_us = execution_us;
	if (execution_us > snapshot->max_execution_us) {
		snapshot->max_execution_us = execution_us;
	}
	snapshot->deadline_miss_count += deadline_misses;
	snapshot->skipped_cycle_count += skipped_cycles;

	/* The task prepares a complete cycle locally. Readers therefore see either
	 * the previous complete cycle or this complete cycle, never mixed fields.
	 */
	portENTER_CRITICAL(&s_diagnostics_lock);
	s_published_snapshot = *snapshot;
	portEXIT_CRITICAL(&s_diagnostics_lock);
}

static void control_task(void *arg) {
	const TickType_t period_ticks = (TickType_t)(uintptr_t)arg;
	const uint32_t stack_sample_period_cycles =
		(uint32_t)configTICK_RATE_HZ / (uint32_t)period_ticks;
	TickType_t last_wake_tick = xTaskGetTickCount();
	const TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
	uint32_t stack_sample_countdown = 0U;
	uint32_t stack_high_water_mark_bytes = 0U;
	control_loop_diagnostics_t diagnostics = {0};

	portENTER_CRITICAL(&s_diagnostics_lock);
	diagnostics.snapshot = s_published_snapshot;
	portEXIT_CRITICAL(&s_diagnostics_lock);
	diagnostics.snapshot.task_alive = true;
	diagnostics.snapshot.task_handle_identity = (uintptr_t)current_task;

	for (;;) {
		xTaskDelayUntil(&last_wake_tick, period_ticks);
		const int64_t cycle_start_us = esp_timer_get_time();

		hip_joint_snapshot_t left_snapshot = {0};
		hip_joint_snapshot_t right_snapshot = {0};
		const bool left_available =
			hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT, &left_snapshot);
		const bool right_available =
			hip_joint_get_snapshot(EXO_HIP_JOINT_RIGHT, &right_snapshot);

		const esp_err_t output_result =
			impedance_control_process_tick(cycle_start_us);
		const bool sample_stack = stack_sample_countdown == 0U;
		if (sample_stack) {
			stack_high_water_mark_bytes =
				(uint32_t)uxTaskGetStackHighWaterMark(NULL);
			stack_sample_countdown = stack_sample_period_cycles - 1U;
		} else {
			--stack_sample_countdown;
		}

		/* Include joint reads, control output, the periodic stack scan, and the
		 * main diagnostic update in the measured control workload. */
		update_cycle_diagnostics(&diagnostics, cycle_start_us, output_result,
								 &left_snapshot, left_available,
								 &right_snapshot, right_available, sample_stack,
								 stack_high_water_mark_bytes);
		const int64_t cycle_end_us = esp_timer_get_time();
		const TickType_t after_cycle_tick = xTaskGetTickCount();
		const uint32_t execution_us =
			clamp_duration_us(cycle_end_us - cycle_start_us);
		const TickType_t lag_ticks = after_cycle_tick - last_wake_tick;
		uint64_t deadline_misses = 0U;
		uint64_t skipped_cycles = 0U;
		if (lag_ticks >= period_ticks) {
			deadline_misses = 1U;
			skipped_cycles = (uint64_t)(lag_ticks / period_ticks);
			last_wake_tick = after_cycle_tick;
		}

		/* This final bounded publication is intentionally the only operation
		 * after the full-workload timing sample. */
		update_timing_diagnostics(&diagnostics, execution_us, deadline_misses,
								  skipped_cycles);
	}
}

esp_err_t exo_control_loop_init(void) {
	const uint32_t frequency_hz = g_exo_hardware_config.control_frequency_hz;
	if (frequency_hz == 0U || frequency_hz > (uint32_t)configTICK_RATE_HZ ||
		((uint32_t)configTICK_RATE_HZ % frequency_hz) != 0U) {
		return ESP_ERR_INVALID_ARG;
	}

	portENTER_CRITICAL(&s_diagnostics_lock);
	if (s_init_state == CONTROL_LOOP_INITIALIZED) {
		portEXIT_CRITICAL(&s_diagnostics_lock);
		return ESP_OK;
	}
	if (s_init_state == CONTROL_LOOP_INITIALIZING) {
		portEXIT_CRITICAL(&s_diagnostics_lock);
		return ESP_OK;
	}
	s_init_state = CONTROL_LOOP_INITIALIZING;
	memset(&s_published_snapshot, 0, sizeof(s_published_snapshot));
	s_published_snapshot.initialized = true;
	s_published_snapshot.configured_frequency_hz = frequency_hz;
	s_published_snapshot.target_period_us = 1000000U / frequency_hz;
	s_published_snapshot.period_tolerance_us = EXO_CONTROL_PERIOD_TOLERANCE_US;
	s_published_snapshot.last_output_error = ESP_OK;
	portEXIT_CRITICAL(&s_diagnostics_lock);

	const TickType_t period_ticks =
		(TickType_t)((uint32_t)configTICK_RATE_HZ / frequency_hz);
	TaskHandle_t task_handle = xTaskCreateStaticPinnedToCore(
		control_task, "exo_control", EXO_CONTROL_TASK_STACK_BYTES,
		(void *)(uintptr_t)period_ticks, EXO_CONTROL_TASK_PRIORITY,
		s_control_task_stack, &s_control_task_buffer, EXO_CONTROL_TASK_CORE);
	if (task_handle == NULL) {
		portENTER_CRITICAL(&s_diagnostics_lock);
		s_init_state = CONTROL_LOOP_NOT_INITIALIZED;
		memset(&s_published_snapshot, 0, sizeof(s_published_snapshot));
		portEXIT_CRITICAL(&s_diagnostics_lock);
		return ESP_ERR_NO_MEM;
	}

	portENTER_CRITICAL(&s_diagnostics_lock);
	s_init_state = CONTROL_LOOP_INITIALIZED;
	s_published_snapshot.task_handle_identity = (uintptr_t)task_handle;
	portEXIT_CRITICAL(&s_diagnostics_lock);
	return ESP_OK;
}

bool exo_control_loop_get_snapshot(exo_control_loop_snapshot_t *snapshot) {
	if (snapshot == NULL) {
		return false;
	}

	portENTER_CRITICAL(&s_diagnostics_lock);
	if (s_init_state != CONTROL_LOOP_INITIALIZED) {
		portEXIT_CRITICAL(&s_diagnostics_lock);
		memset(snapshot, 0, sizeof(*snapshot));
		return false;
	}
	*snapshot = s_published_snapshot;
	portEXIT_CRITICAL(&s_diagnostics_lock);
	return true;
}
