#ifndef EXO_CONTROL_LOOP_H
#define EXO_CONTROL_LOOP_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint32_t feedback_count;
	uint64_t feedback_timestamp_us;
	bool joint_valid;
} exo_control_loop_joint_status_t;

typedef struct {
	bool initialized;
	bool task_alive;
	uintptr_t task_handle_identity;

	uint32_t configured_frequency_hz;
	uint32_t target_period_us;
	uint32_t period_tolerance_us;
	uint64_t cycle_count;
	uint64_t period_sample_count;
	uint64_t period_out_of_tolerance_count;
	uint64_t last_cycle_start_us;
	uint32_t last_period_us;
	uint32_t min_period_us;
	uint32_t max_period_us;
	uint32_t mean_period_us;
	int32_t last_period_error_us;
	uint32_t max_abs_period_error_us;
	uint32_t last_execution_us;
	uint32_t max_execution_us;

	uint64_t deadline_miss_count;
	uint64_t skipped_cycle_count;
	uint64_t output_error_count;
	esp_err_t last_output_error;

	exo_control_loop_joint_status_t left;
	exo_control_loop_joint_status_t right;
	uint32_t stack_high_water_mark_bytes;
} exo_control_loop_snapshot_t;

/* Creates the permanent, statically allocated control task at most once. */
esp_err_t exo_control_loop_init(void);

/* Copies a coherent diagnostic snapshot without changing control state. */
bool exo_control_loop_get_snapshot(exo_control_loop_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* EXO_CONTROL_LOOP_H */
