#ifndef HOST_FAKE_EXO_JOINT_COMMAND_H
#define HOST_FAKE_EXO_JOINT_COMMAND_H
#include <stdbool.h>
#include "esp_err.h"
#include "exo_types.h"
#include "rs01_driver.h"
typedef struct {
	esp_err_t map_error;
	esp_err_t actuator_error[EXO_ACTUATOR_COUNT];
	bool actuator_attempted[EXO_ACTUATOR_COUNT];
} exo_joint_command_send_result_t;
bool exo_joint_command_is_valid(const exo_joint_command_t *command);
esp_err_t exo_joint_command_map_pair(
	const exo_joint_command_pair_t *joint_commands,
	rs01_motion_command_t actuator_commands[EXO_ACTUATOR_COUNT]);
esp_err_t exo_joint_command_try_send_pair(
	const exo_joint_command_pair_t *joint_commands);
esp_err_t exo_joint_command_try_send_pair_detailed(
	const exo_joint_command_pair_t *joint_commands,
	exo_joint_command_send_result_t *send_result);
#endif
