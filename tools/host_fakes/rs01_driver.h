#ifndef HOST_FAKE_RS01_DRIVER_H
#define HOST_FAKE_RS01_DRIVER_H
#include "exo_types.h"
typedef struct {
	float position_rad;
	float velocity_rad_s;
	float torque_nm;
	float kp;
	float kd;
} rs01_motion_command_t;
#endif
