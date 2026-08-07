#ifndef SINE_REFERENCE_H
#define SINE_REFERENCE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	float amplitude_rad;
	float frequency_hz;
	float ramp_time_s;
} sine_reference_config_t;

typedef struct {
	float offset_rad;
	float ramp;
} sine_reference_sample_t;

/* Pure validation: this module has no ESP-IDF, RTOS, driver, or I/O dependency.
 */
bool sine_reference_config_is_valid(const sine_reference_config_t *config);

/*
 * Evaluates the retained sinusoidal position offset at elapsed_us. Non-positive
 * elapsed time is treated as the trajectory origin and produces zero offset.
 */
bool sine_reference_evaluate(const sine_reference_config_t *config,
							 int64_t elapsed_us,
							 sine_reference_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* SINE_REFERENCE_H */
