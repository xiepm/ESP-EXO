#pragma once
#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif
/* Legacy API names remain stable; both private and MIT use these ranges. */
#if defined(CONFIG_EXO_MOTOR_RS05) && CONFIG_EXO_MOTOR_RS05
#define EXO_MOTOR_WIRE_SPEED_MAX 50.0f
#define EXO_MOTOR_WIRE_TORQUE_MAX 5.5f
#define EXO_MOTOR_MODEL_NAME "RobStride05"
#else
#define EXO_MOTOR_WIRE_SPEED_MAX 44.0f
#define EXO_MOTOR_WIRE_TORQUE_MAX 17.0f
#define EXO_MOTOR_MODEL_NAME "RobStride01"
#endif
