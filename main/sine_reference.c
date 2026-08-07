#include "sine_reference.h"

#include <math.h>
#include <stddef.h>

#define SINE_REFERENCE_TWO_PI_D 6.283185307179586476925286766559

static float quintic_ramp(float elapsed_s, float ramp_time_s) {
	if (ramp_time_s <= 0.0f) {
		return 1.0f;
	}
	if (elapsed_s <= 0.0f) {
		return 0.0f;
	}
	if (elapsed_s >= ramp_time_s) {
		return 1.0f;
	}

	const float s = elapsed_s / ramp_time_s;
	const float s2 = s * s;
	const float s3 = s2 * s;
	return s3 * (10.0f - 15.0f * s + 6.0f * s2);
}

bool sine_reference_config_is_valid(const sine_reference_config_t *config) {
	return config != NULL && isfinite(config->amplitude_rad) &&
		   isfinite(config->frequency_hz) && isfinite(config->ramp_time_s) &&
		   config->amplitude_rad >= 0.0f && config->frequency_hz > 0.0f &&
		   config->ramp_time_s >= 0.0f;
}

bool sine_reference_evaluate(const sine_reference_config_t *config,
							 int64_t elapsed_us,
							 sine_reference_sample_t *sample) {
	if (sample == NULL) {
		return false;
	}
	*sample = (sine_reference_sample_t){0};
	if (!sine_reference_config_is_valid(config)) {
		return false;
	}

	const int64_t clamped_elapsed_us = elapsed_us > 0 ? elapsed_us : 0;
	const float elapsed_s = (float)clamped_elapsed_us / 1000000.0f;
	const float ramp = quintic_ramp(elapsed_s, config->ramp_time_s);
	/* Wrap by complete cycles before sinf() so long runs do not accumulate a
	 * large single-precision phase. The first 60 seconds are regression-tested
	 * against the original unwrapped expression. */
	const double cycles =
		(double)clamped_elapsed_us * (double)config->frequency_hz / 1000000.0;
	const double wrapped_cycles = cycles - floor(cycles);
	const float phase_rad = (float)(SINE_REFERENCE_TWO_PI_D * wrapped_cycles);
	const float shaped_sine = ramp * sinf(phase_rad);
	const float offset_rad = config->amplitude_rad * shaped_sine;
	if (!isfinite(ramp) || !isfinite(offset_rad)) {
		return false;
	}

	sample->offset_rad = offset_rad;
	sample->ramp = ramp;
	return true;
}
