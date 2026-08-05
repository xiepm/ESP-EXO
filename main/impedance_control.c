#include "impedance_control.h"
#include "robstride_control.h"
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
#define IMP_CONTROL_PERIOD_US   10000          /* 100 Hz */

/* ---- Motor CAN IDs ---- */
#define MOTOR_A_ID              0x10           /* anti-phase */
#define MOTOR_B_ID              0x20           /* in-phase  */

/* ================================================================
 *  State
 * ================================================================ */

static esp_timer_handle_t  s_timer;
static float                s_center[2];      /* [0]=0x10, [1]=0x20 */
static int64_t              s_start_us;
static volatile bool        s_running;
static portMUX_TYPE         s_lock = portMUX_INITIALIZER_UNLOCKED;

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
 *  Timer callback — fired at 100 Hz
 * ================================================================ */
static void timer_cb(void *arg)
{
    float elapsed = (float)(esp_timer_get_time() - s_start_us) / 1000000.0f;

    float omega   = 2.0f * (float)M_PI * IMP_FREQ_HZ;
    float ramp    = quintic_ramp(elapsed, IMP_RAMP_TIME_S);
    float phase   = omega * elapsed;
    float sin_val = ramp * sinf(phase);

    /* Motor 0x10: anti-phase  →  q_ref = center - A * sin(ωt) */
    float target_a = s_center[0] - IMP_AMPLITUDE_RAD * sin_val;
    /* Motor 0x20: in-phase    →  q_ref = center + A * sin(ωt) */
    float target_b = s_center[1] + IMP_AMPLITUDE_RAD * sin_val;

    /* Send control frames: pos, spd=0, kp, kd, tq_ff=0
     * (velocity=0 → pure impedance, no trajectory feed-forward) */
    RobStride_Impedance_SendFrame(MOTOR_A_ID, target_a, 0.0f, IMP_KP, IMP_KD, 0.0f);
    RobStride_Impedance_SendFrame(MOTOR_B_ID, target_b, 0.0f, IMP_KP, IMP_KD, 0.0f);
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
             MOTOR_A_ID, MOTOR_B_ID);
    ESP_LOGI(TAG, "  KP=%.3f  KD=%.3f  Amp=%.0f°  Freq=%.1fHz  Ramp=%.1fs",
             (double)IMP_KP, (double)IMP_KD, 90.0,
             (double)IMP_FREQ_HZ, (double)IMP_RAMP_TIME_S);

    /* 1. Disable both motors */
    RobStride_motor_reset(MOTOR_A_ID);
    vTaskDelay(pdMS_TO_TICKS(50));
    RobStride_motor_reset(MOTOR_B_ID);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 2. Set MOVE_CONTROL_MODE for both motors */
    Set_RobStride_Motor_parameter(MOTOR_A_ID, 0x7005, 0, 'j');
    vTaskDelay(pdMS_TO_TICKS(20));
    Set_RobStride_Motor_parameter(MOTOR_B_ID, 0x7005, 0, 'j');
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 3. Enable both motors */
    RobStride_motor_enable(MOTOR_A_ID);
    vTaskDelay(pdMS_TO_TICKS(50));
    RobStride_motor_enable(MOTOR_B_ID);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 4. Send initial zero-gain frames to get feedback flowing */
    float init_pos_a = 0.0f;
    float init_pos_b = 0.0f;

    /* Try to read positions from per-motor state */
    if (g_motor_states[MOTOR_A_ID].valid) {
        init_pos_a = g_motor_states[MOTOR_A_ID].position;
    }
    if (g_motor_states[MOTOR_B_ID].valid) {
        init_pos_b = g_motor_states[MOTOR_B_ID].position;
    }

    RobStride_Impedance_SendFrame(MOTOR_A_ID, init_pos_a, 0.0f, 0.0f, 0.0f, 0.0f);
    RobStride_Impedance_SendFrame(MOTOR_B_ID, init_pos_b, 0.0f, 0.0f, 0.0f, 0.0f);

    /* 5. Wait for feedback to arrive and read center positions */
    vTaskDelay(pdMS_TO_TICKS(300));

    portENTER_CRITICAL(&s_lock);
    if (g_motor_states[MOTOR_A_ID].valid) {
        s_center[0] = g_motor_states[MOTOR_A_ID].position;
    } else {
        s_center[0] = 0.0f;
        ESP_LOGW(TAG, "No feedback from motor 0x%02X, using pos=0", MOTOR_A_ID);
    }
    if (g_motor_states[MOTOR_B_ID].valid) {
        s_center[1] = g_motor_states[MOTOR_B_ID].position;
    } else {
        s_center[1] = 0.0f;
        ESP_LOGW(TAG, "No feedback from motor 0x%02X, using pos=0", MOTOR_B_ID);
    }
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "Center positions: A=%.4f rad (%.2f°), B=%.4f rad (%.2f°)",
             (double)s_center[0], (double)(s_center[0] * 180.0 / M_PI),
             (double)s_center[1], (double)(s_center[1] * 180.0 / M_PI));

    /* 6. Create and start periodic timer */
    esp_timer_create_args_t timer_cfg = {
        .callback = timer_cb,
        .arg      = NULL,
        .name     = "impedance_tmr",
    };
    esp_err_t err = esp_timer_create(&timer_cfg, &s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
        s_running = false;
        return err;
    }

    s_start_us = esp_timer_get_time();
    err = esp_timer_start_periodic(s_timer, IMP_CONTROL_PERIOD_US);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(err));
        esp_timer_delete(s_timer);
        s_timer = NULL;
        s_running = false;
        return err;
    }

    ESP_LOGI(TAG, "Impedance control running @ %d Hz", 1000000 / IMP_CONTROL_PERIOD_US);
    return ESP_OK;
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
    float safe_a = s_center[0];
    float safe_b = s_center[1];
    if (g_motor_states[MOTOR_A_ID].valid) {
        safe_a = g_motor_states[MOTOR_A_ID].position;
    }
    if (g_motor_states[MOTOR_B_ID].valid) {
        safe_b = g_motor_states[MOTOR_B_ID].position;
    }

    RobStride_Impedance_SendFrame(MOTOR_A_ID, safe_a, 0.0f, 0.0f, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(20));
    RobStride_Impedance_SendFrame(MOTOR_B_ID, safe_b, 0.0f, 0.0f, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 3. Disable both motors */
    RobStride_motor_reset(MOTOR_A_ID);
    vTaskDelay(pdMS_TO_TICKS(50));
    RobStride_motor_reset(MOTOR_B_ID);
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
