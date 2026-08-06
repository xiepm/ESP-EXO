#include "impedance_control.h"

#include <math.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rs01_driver.h"

static const char *TAG = "impedance";

/* Parameters from the demo2 reference. */
#define IMP_KP 0.2f
#define IMP_KD 0.10f
#define IMP_AMPLITUDE_RAD 1.570796327f /* 90 degrees */
#define IMP_FREQ_HZ 0.5f
#define IMP_RAMP_TIME_S 3.0f

typedef enum {
	IMPEDANCE_STATE_IDLE = 0,
	IMPEDANCE_STATE_PREPARING,
	IMPEDANCE_STATE_RUNNING,
	IMPEDANCE_STATE_STOPPING,
} impedance_state_t;

static float s_center[EXO_ACTUATOR_COUNT];
static int64_t s_start_us;
static impedance_state_t s_state;
static bool s_tick_active;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static exo_can_node_id_t actuator_can_id(exo_actuator_id_t actuator) {
	exo_can_node_id_t can_id = 0;
	(void)rs01_get_can_id(actuator, &can_id);
	return can_id;
}

/* Quintic ramp from demo2: r(s) = 10s^3 - 15s^4 + 6s^5. */
static float quintic_ramp(float elapsed, float ramp_duration) {
	if (ramp_duration <= 0.0f) {
		return 1.0f;
	}
	if (elapsed <= 0.0f) {
		return 0.0f;
	}
	if (elapsed >= ramp_duration) {
		return 1.0f;
	}

	const float s = elapsed / ramp_duration;
	const float s2 = s * s;
	const float s3 = s2 * s;
	return s3 * (10.0f - 15.0f * s + 6.0f * s2);
}

static bool tick_is_active(void) {
	portENTER_CRITICAL(&s_lock);
	const bool active = s_tick_active;
	portEXIT_CRITICAL(&s_lock);
	return active;
}

esp_err_t impedance_control_process_tick(int64_t now_us) {
	float center_a;
	float center_b;
	int64_t start_us;

	portENTER_CRITICAL(&s_lock);
	if (s_state != IMPEDANCE_STATE_RUNNING) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_OK;
	}
	if (s_tick_active) {
		portEXIT_CRITICAL(&s_lock);
		return ESP_ERR_INVALID_STATE;
	}

	s_tick_active = true;
	center_a = s_center[EXO_ACTUATOR_A];
	center_b = s_center[EXO_ACTUATOR_B];
	start_us = s_start_us;
	portEXIT_CRITICAL(&s_lock);

	const float elapsed = (float)(now_us - start_us) / 1000000.0f;
	const float omega = 2.0f * (float)M_PI * IMP_FREQ_HZ;
	const float ramp = quintic_ramp(elapsed, IMP_RAMP_TIME_S);
	const float phase = omega * elapsed;
	const float sin_val = ramp * sinf(phase);

	/* Actuator A remains anti-phase; actuator B remains in-phase. */
	const float target_a = center_a - IMP_AMPLITUDE_RAD * sin_val;
	const float target_b = center_b + IMP_AMPLITUDE_RAD * sin_val;

	/* Attempt both outputs so one actuator's TX error cannot starve the other.
	 */
	const esp_err_t err_a = rs01_try_send_motion(EXO_ACTUATOR_A, target_a, 0.0f,
												 0.0f, IMP_KP, IMP_KD);
	const esp_err_t err_b = rs01_try_send_motion(EXO_ACTUATOR_B, target_b, 0.0f,
												 0.0f, IMP_KP, IMP_KD);

	portENTER_CRITICAL(&s_lock);
	s_tick_active = false;
	portEXIT_CRITICAL(&s_lock);

	return err_a != ESP_OK ? err_a : err_b;
}

esp_err_t impedance_control_start(void) {
	portENTER_CRITICAL(&s_lock);
	if (s_state != IMPEDANCE_STATE_IDLE) {
		portEXIT_CRITICAL(&s_lock);
		ESP_LOGW(TAG, "Cannot start while impedance control is active");
		return ESP_ERR_INVALID_STATE;
	}
	s_state = IMPEDANCE_STATE_PREPARING;
	portEXIT_CRITICAL(&s_lock);

	ESP_LOGI(TAG, "Starting dual-motor impedance control...");
	ESP_LOGI(TAG, "  Motors: 0x%02X (anti-phase) + 0x%02X (in-phase)",
			 actuator_can_id(EXO_ACTUATOR_A), actuator_can_id(EXO_ACTUATOR_B));
	ESP_LOGI(TAG, "  KP=%.3f  KD=%.3f  Amp=90 deg  Freq=%.1fHz  Ramp=%.1fs",
			 (double)IMP_KP, (double)IMP_KD, (double)IMP_FREQ_HZ,
			 (double)IMP_RAMP_TIME_S);

	/* 1. Disable both motors. */
	esp_err_t err = rs01_disable(EXO_ACTUATOR_A);
	if (err != ESP_OK) {
		goto start_failed;
	}
	vTaskDelay(pdMS_TO_TICKS(50));
	err = rs01_disable(EXO_ACTUATOR_B);
	if (err != ESP_OK) {
		goto start_failed;
	}
	vTaskDelay(pdMS_TO_TICKS(50));

	/* 2. Select motion control mode for both motors. */
	err = rs01_set_control_mode(EXO_ACTUATOR_A, RS01_CONTROL_MODE_MOTION);
	if (err != ESP_OK) {
		goto start_failed;
	}
	vTaskDelay(pdMS_TO_TICKS(20));
	err = rs01_set_control_mode(EXO_ACTUATOR_B, RS01_CONTROL_MODE_MOTION);
	if (err != ESP_OK) {
		goto start_failed;
	}
	vTaskDelay(pdMS_TO_TICKS(20));

	/* 3. Enable both motors. */
	err = rs01_enable(EXO_ACTUATOR_A);
	if (err != ESP_OK) {
		goto start_failed;
	}
	vTaskDelay(pdMS_TO_TICKS(50));
	err = rs01_enable(EXO_ACTUATOR_B);
	if (err != ESP_OK) {
		goto start_failed;
	}
	vTaskDelay(pdMS_TO_TICKS(50));

	/* 4. Send zero-gain frames to establish fresh feedback. */
	float init_pos_a = 0.0f;
	float init_pos_b = 0.0f;
	rs01_feedback_snapshot_t feedback_a;
	rs01_feedback_snapshot_t feedback_b;
	if (rs01_get_feedback(EXO_ACTUATOR_A, &feedback_a) && feedback_a.valid) {
		init_pos_a = feedback_a.sample.position_rad;
	}
	if (rs01_get_feedback(EXO_ACTUATOR_B, &feedback_b) && feedback_b.valid) {
		init_pos_b = feedback_b.sample.position_rad;
	}

	err = rs01_send_motion(EXO_ACTUATOR_A, init_pos_a, 0.0f, 0.0f, 0.0f, 0.0f);
	if (err != ESP_OK) {
		goto start_failed;
	}
	err = rs01_send_motion(EXO_ACTUATOR_B, init_pos_b, 0.0f, 0.0f, 0.0f, 0.0f);
	if (err != ESP_OK) {
		goto start_failed;
	}

	/* 5. Wait for feedback, then capture the trajectory centers locally. */
	vTaskDelay(pdMS_TO_TICKS(300));
	const bool has_feedback_a =
		rs01_get_feedback(EXO_ACTUATOR_A, &feedback_a) && feedback_a.valid;
	const bool has_feedback_b =
		rs01_get_feedback(EXO_ACTUATOR_B, &feedback_b) && feedback_b.valid;
	const float center_a =
		has_feedback_a ? feedback_a.sample.position_rad : 0.0f;
	const float center_b =
		has_feedback_b ? feedback_b.sample.position_rad : 0.0f;

	if (!has_feedback_a) {
		ESP_LOGW(TAG, "No feedback from motor 0x%02X, using pos=0",
				 actuator_can_id(EXO_ACTUATOR_A));
	}
	if (!has_feedback_b) {
		ESP_LOGW(TAG, "No feedback from motor 0x%02X, using pos=0",
				 actuator_can_id(EXO_ACTUATOR_B));
	}

	ESP_LOGI(TAG,
			 "Center positions: A=%.4f rad (%.2f deg), B=%.4f rad (%.2f deg)",
			 (double)center_a, (double)(center_a * 180.0 / M_PI),
			 (double)center_b, (double)(center_b * 180.0 / M_PI));

	/* Publish all trajectory state atomically, with RUNNING written last. */
	const int64_t start_us = esp_timer_get_time();
	portENTER_CRITICAL(&s_lock);
	s_center[EXO_ACTUATOR_A] = center_a;
	s_center[EXO_ACTUATOR_B] = center_b;
	s_start_us = start_us;
	s_state = IMPEDANCE_STATE_RUNNING;
	portEXIT_CRITICAL(&s_lock);

	ESP_LOGI(TAG, "Impedance control ready for external periodic ticks");
	return ESP_OK;

start_failed:
	(void)rs01_disable(EXO_ACTUATOR_A);
	(void)rs01_disable(EXO_ACTUATOR_B);
	portENTER_CRITICAL(&s_lock);
	s_state = IMPEDANCE_STATE_IDLE;
	portEXIT_CRITICAL(&s_lock);
	ESP_LOGE(TAG, "Failed to start impedance control: %s",
			 esp_err_to_name(err));
	return err;
}

esp_err_t impedance_control_stop(void) {
	float safe_a;
	float safe_b;

	/* Block new ticks before performing any release or disable operation. */
	portENTER_CRITICAL(&s_lock);
	if (s_state != IMPEDANCE_STATE_RUNNING) {
		portEXIT_CRITICAL(&s_lock);
		ESP_LOGW(TAG, "Impedance control is not running");
		return ESP_ERR_INVALID_STATE;
	}
	s_state = IMPEDANCE_STATE_STOPPING;
	safe_a = s_center[EXO_ACTUATOR_A];
	safe_b = s_center[EXO_ACTUATOR_B];
	portEXIT_CRITICAL(&s_lock);

	/* Let an already-entered tick finish before releasing either motor. */
	while (tick_is_active()) {
		vTaskDelay(1);
	}

	ESP_LOGI(TAG, "Stopping impedance control...");

	/* Send zero-gain frames at the latest positions to release torque. */
	rs01_feedback_snapshot_t feedback;
	if (rs01_get_feedback(EXO_ACTUATOR_A, &feedback) && feedback.valid) {
		safe_a = feedback.sample.position_rad;
	}
	if (rs01_get_feedback(EXO_ACTUATOR_B, &feedback) && feedback.valid) {
		safe_b = feedback.sample.position_rad;
	}

	(void)rs01_send_motion(EXO_ACTUATOR_A, safe_a, 0.0f, 0.0f, 0.0f, 0.0f);
	vTaskDelay(pdMS_TO_TICKS(20));
	(void)rs01_send_motion(EXO_ACTUATOR_B, safe_b, 0.0f, 0.0f, 0.0f, 0.0f);
	vTaskDelay(pdMS_TO_TICKS(20));

	(void)rs01_disable(EXO_ACTUATOR_A);
	vTaskDelay(pdMS_TO_TICKS(50));
	(void)rs01_disable(EXO_ACTUATOR_B);
	vTaskDelay(pdMS_TO_TICKS(50));

	portENTER_CRITICAL(&s_lock);
	s_state = IMPEDANCE_STATE_IDLE;
	portEXIT_CRITICAL(&s_lock);

	ESP_LOGI(TAG, "Impedance control stopped, motors disabled");
	return ESP_OK;
}

bool impedance_control_is_running(void) {
	portENTER_CRITICAL(&s_lock);
	const bool running = s_state == IMPEDANCE_STATE_RUNNING;
	portEXIT_CRITICAL(&s_lock);
	return running;
}
