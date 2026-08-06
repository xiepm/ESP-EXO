#ifndef HIP_JOINT_H
#define HIP_JOINT_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "exo_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	exo_hip_joint_id_t joint;
	exo_side_t side;
	exo_actuator_id_t actuator;
	exo_direction_t direction;

	/* Both samples refer to feedback_timestamp_us. */
	exo_motion_sample_t raw_actuator_sample;
	exo_motion_sample_t joint_sample;
	float software_zero_motor_rad;

	uint64_t feedback_timestamp_us;
	uint32_t source_feedback_count;
	uint64_t zero_source_timestamp_us;
	uint32_t zero_source_feedback_count;

	bool mapping_valid;
	bool direction_valid;
	bool zero_valid;
	bool feedback_valid;
	bool online;
	bool joint_valid;
} hip_joint_snapshot_t;

/* Initializes the two static hip objects from g_exo_hardware_config. */
esp_err_t hip_joint_init(void);

/*
 * Returns true when the request itself is valid. Inspect joint_valid before
 * using joint_sample for control.
 */
bool hip_joint_get_snapshot(exo_hip_joint_id_t joint,
							hip_joint_snapshot_t *snapshot);

/*
 * Captures the current RS01 output position as a RAM-only software offset.
 * This function does not transmit CAN frames or alter the motor's zero.
 */
esp_err_t hip_joint_set_zero_from_current(exo_hip_joint_id_t joint);

/* Clears only the selected joint's RAM offset. */
esp_err_t hip_joint_clear_zero(exo_hip_joint_id_t joint);

#ifdef __cplusplus
}
#endif

#endif /* HIP_JOINT_H */
