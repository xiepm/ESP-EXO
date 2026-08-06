#include "impedance_control.h"
#include "exo_config.h"
#include "rs01_driver.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "impedance";

/* ================================================================
 *  Parameters — from demo2 reference
 * ================================================================ */

#define IMP_KP                  0.2f
#define IMP_KD                  0.10f
#define IMP_AMPLITUDE_RAD       1.570796327f   /*  90° */
#define IMP_FREQ_HZ             0.5f
#define IMP_RAMP_TIME_S         3.0f

/* ================================================================
 *  State
 * ================================================================ */

static esp_timer_handle_t  s_timer;
static float                s_center[EXO_ACTUATOR_COUNT];
static int64_t              s_start_us;
static volatile bool        s_running;
static portMUX_TYPE         s_lock = portMUX_INITIALIZER_UNLOCKED;

static exo_can_node_id_t actuator_can_id(exo_actuator_id_t actuator)
{
    exo_can_node_id_t can_id = 0;
    (void)rs01_get_can_id(actuator, &can_id);
    return can_id;
}

/* ================================================================
 *  Quintic ramp — same formula as demo2
 *    r(s) = 10s³ - 15s⁴ + 6s⁵
 *  Scaled float version for MCU
 * ================================================================ */
static float quintic_ramp(float elapsed, float ramp_duration)
{
    if (ramp_duration <= 0.0f) return 1.0f;
    if (elapsed <= 0.0f)      return 0.0f;
    if (elapsed >= ramp_duration) return 1.0f;

    float s = elapsed / ramp_duration;
    float s2 = s * s;
    float s3 = s2 * s;
    /* 10s³ - 15s⁴ + 6s⁵ = s³ * (10 - 15s + 6s²) */
    return s3 * (10.0f - 15.0f * s + 6.0f * s2);
}

/* ================================================================
 *  Timer callback — fired at the configured control rate
 * ================================================================ */
static void timer_cb(void *arg)
{
    float elapsed = (float)(esp_timer_get_time() - s_start_us) / 1000000.0f;

    float omega   = 2.0f * (float)M_PI * IMP_FREQ_HZ;
    float ramp    = quintic_ramp(elapsed, IMP_RAMP_TIME_S);
    float phase   = omega * elapsed;
    float sin_val = ramp * sinf(phase);

    /* Actuator A: anti-phase; actuator B: in-phase. */
    float target_a = s_center[EXO_ACTUATOR_A] - IMP_AMPLITUDE_RAD * sin_val;
    float target_b = s_center[EXO_ACTUATOR_B] + IMP_AMPLITUDE_RAD * sin_val;

    /* Send control frames: pos, spd=0, kp, kd, tq_ff=0
     * (velocity=0 → pure impedance, no trajectory feed-forward) */
    (void)rs01_send_motion(EXO_ACTUATOR_A, target_a, 0.0f, 0.0f,
                           IMP_KP, IMP_KD);
    (void)rs01_send_motion(EXO_ACTUATOR_B, target_b, 0.0f, 0.0f,
                           IMP_KP, IMP_KD);
}

/* ================================================================
 *  Public API
 * ================================================================ */

esp_err_t impedance_control_start(void)
{
    portENTER_CRITICAL(&s_lock);
    if (s_running) {
        portEXIT_CRITICAL(&s_lock);
        ESP_LOGW(TAG, "Already running");
        return ESP_ERR_INVALID_STATE;
    }
    s_running = true;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "Starting dual-motor impedance control...");
    ESP_LOGI(TAG, "  Motors: 0x%02X (anti-phase) + 0x%02X (in-phase)",
             actuator_can_id(EXO_ACTUATOR_A),
             actuator_can_id(EXO_ACTUATOR_B));
    ESP_LOGI(TAG, "  KP=%.3f  KD=%.3f  Amp=%.0f°  Freq=%.1fHz  Ramp=%.1fs",
             (double)IMP_KP, (double)IMP_KD, 90.0,
             (double)IMP_FREQ_HZ, (double)IMP_RAMP_TIME_S);

    /* 1. Disable both motors */
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

    /* 2. Set MOVE_CONTROL_MODE for both motors */
    err = rs01_set_control_mode(EXO_ACTUATOR_A,
                                RS01_CONTROL_MODE_MOTION);
    if (err != ESP_OK) {
        goto start_failed;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    err = rs01_set_control_mode(EXO_ACTUATOR_B,
                                RS01_CONTROL_MODE_MOTION);
    if (err != ESP_OK) {
        goto start_failed;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 3. Enable both motors */
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

    /* 4. Send initial zero-gain frames to get feedback flowing */
    float init_pos_a = 0.0f;
    float init_pos_b = 0.0f;

    /* Try to read positions from per-motor state */
    rs01_feedback_snapshot_t feedback_a;
    rs01_feedback_snapshot_t feedback_b;
    if (rs01_get_feedback(EXO_ACTUATOR_A, &feedback_a) && feedback_a.valid) {
        init_pos_a = feedback_a.sample.position_rad;
    }
    if (rs01_get_feedback(EXO_ACTUATOR_B, &feedback_b) && feedback_b.valid) {
        init_pos_b = feedback_b.sample.position_rad;
    }

    err = rs01_send_motion(EXO_ACTUATOR_A, init_pos_a, 0.0f, 0.0f,
                           0.0f, 0.0f);
    if (err != ESP_OK) {
        goto start_failed;
    }
    err = rs01_send_motion(EXO_ACTUATOR_B, init_pos_b, 0.0f, 0.0f,
                           0.0f, 0.0f);
    if (err != ESP_OK) {
        goto start_failed;
    }

    /* 5. Wait for feedback to arrive and read center positions */
    vTaskDelay(pdMS_TO_TICKS(300));

    bool has_feedback_a =
        rs01_get_feedback(EXO_ACTUATOR_A, &feedback_a) && feedback_a.valid;
    bool has_feedback_b =
        rs01_get_feedback(EXO_ACTUATOR_B, &feedback_b) && feedback_b.valid;

    portENTER_CRITICAL(&s_lock);
    s_center[EXO_ACTUATOR_A] =
        has_feedback_a ? feedback_a.sample.position_rad : 0.0f;
    s_center[EXO_ACTUATOR_B] =
        has_feedback_b ? feedback_b.sample.position_rad : 0.0f;
    portEXIT_CRITICAL(&s_lock);

    if (!has_feedback_a) {
        ESP_LOGW(TAG, "No feedback from motor 0x%02X, using pos=0",
                 actuator_can_id(EXO_ACTUATOR_A));
    }
    if (!has_feedback_b) {
        ESP_LOGW(TAG, "No feedback from motor 0x%02X, using pos=0",
                 actuator_can_id(EXO_ACTUATOR_B));
    }

    ESP_LOGI(TAG, "Center positions: A=%.4f rad (%.2f°), B=%.4f rad (%.2f°)",
             (double)s_center[EXO_ACTUATOR_A],
             (double)(s_center[EXO_ACTUATOR_A] * 180.0 / M_PI),
             (double)s_center[EXO_ACTUATOR_B],
             (double)(s_center[EXO_ACTUATOR_B] * 180.0 / M_PI));

    /* 6. Create and start periodic timer */
    esp_timer_create_args_t timer_cfg = {
        .callback = timer_cb,
        .arg      = NULL,
        .name     = "impedance_tmr",
    };
    err = esp_timer_create(&timer_cfg, &s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
        goto start_failed;
    }

    s_start_us = esp_timer_get_time();
    uint64_t control_period_us =
        1000000ULL / g_exo_hardware_config.control_frequency_hz;
    err = esp_timer_start_periodic(s_timer, control_period_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(err));
        esp_timer_delete(s_timer);
        s_timer = NULL;
        goto start_failed;
    }

    ESP_LOGI(TAG, "Impedance control running @ %lu Hz",
             (unsigned long)g_exo_hardware_config.control_frequency_hz);
    return ESP_OK;

start_failed:
    if (s_timer != NULL) {
        (void)esp_timer_stop(s_timer);
        (void)esp_timer_delete(s_timer);
        s_timer = NULL;
    }
    (void)rs01_disable(EXO_ACTUATOR_A);
    (void)rs01_disable(EXO_ACTUATOR_B);
    portENTER_CRITICAL(&s_lock);
    s_running = false;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGE(TAG, "Failed to start impedance control: %s",
             esp_err_to_name(err));
    return err;
}

esp_err_t impedance_control_stop(void)
{
    portENTER_CRITICAL(&s_lock);
    if (!s_running) {
        portEXIT_CRITICAL(&s_lock);
        ESP_LOGW(TAG, "Not running");
        return ESP_ERR_INVALID_STATE;
    }
    s_running = false;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "Stopping impedance control...");

    /* 1. Stop and delete timer */
    if (s_timer != NULL) {
        esp_timer_stop(s_timer);
        esp_timer_delete(s_timer);
        s_timer = NULL;
    }

    /* 2. Send zero-gain frames to release torque */
    float safe_a = s_center[EXO_ACTUATOR_A];
    float safe_b = s_center[EXO_ACTUATOR_B];
    rs01_feedback_snapshot_t feedback;
    if (rs01_get_feedback(EXO_ACTUATOR_A, &feedback) && feedback.valid) {
        safe_a = feedback.sample.position_rad;
    }
    if (rs01_get_feedback(EXO_ACTUATOR_B, &feedback) && feedback.valid) {
        safe_b = feedback.sample.position_rad;
    }

    (void)rs01_send_motion(EXO_ACTUATOR_A, safe_a, 0.0f, 0.0f,
                           0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(20));
    (void)rs01_send_motion(EXO_ACTUATOR_B, safe_b, 0.0f, 0.0f,
                           0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 3. Disable both motors */
    (void)rs01_disable(EXO_ACTUATOR_A);
    vTaskDelay(pdMS_TO_TICKS(50));
    (void)rs01_disable(EXO_ACTUATOR_B);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "Impedance control stopped, motors disabled");
    return ESP_OK;
}

bool impedance_control_is_running(void)
{
    portENTER_CRITICAL(&s_lock);
    bool running = s_running;
    portEXIT_CRITICAL(&s_lock);
    return running;
}
