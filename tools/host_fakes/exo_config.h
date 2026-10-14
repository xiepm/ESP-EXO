#ifndef HOST_FAKE_EXO_CONFIG_H
#define HOST_FAKE_EXO_CONFIG_H
#include "sine_reference.h"
typedef struct {
	sine_reference_config_t reference;
	float kp;
	float kd;
} exo_sine_impedance_config_t;
extern const exo_sine_impedance_config_t g_exo_sine_impedance_config;
extern const sine_reference_config_t g_exo_alternating_sine_test_config;
#endif
