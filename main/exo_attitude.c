#include "exo_attitude.h"

#include <math.h>
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define DEG_TO_RAD 0.01745329251994329577f
#define RAD_TO_DEG 57.295779513082320876f
#define CALIBRATION_US 2000000
#define CALIBRATION_MIN_SAMPLES 100
#define MAX_SAMPLE_GAP_US 100000
#define ACCEL_FEEDBACK_GAIN 2.5f
/* 2026-11-11 confirmed stationary: Y-axis sigma up to 1.1983 dps over 2s.
 * Keep acceleration/endpoint tests independent to reject actual board motion. */
#define CALIBRATION_GYRO_STD_DPS 1.5f

typedef struct {
    float q[4];
    float bias[3];
    bool tilt_initialized;
    bool calibrated;
    bool magnetic_reference;
    float magnetic_norm;
    int64_t previous_us;
    int64_t calibration_start_us;
    uint32_t calibration_count;
    float accel_mean[3], accel_m2[3], gyro_mean[3], gyro_m2[3];
    float first_accel[3];
    uint32_t calibration_reject_mask;
    uint32_t calibration_rejections;
    uint32_t last_calibration_samples;
    float last_calibration_gyro_std_dps[3];
    float last_calibration_accel_std_g[3];
    float last_calibration_gravity_delta_g;
    uint32_t updates;
    uint32_t discontinuities;
} filter_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static exo_attitude_status_t s_status;
static exo_attitude_mag_calibration_t s_mag;
static bool s_mag_enabled;
static bool s_mag_fusion_enabled;
static bool s_calibration_disabled;
static uint32_t s_generation;
/* The sampler is the sole writer of these two objects. */
static uint32_t s_seen_generation = UINT32_MAX;
static filter_t s_filter;

static float norm3(const float *v)
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static bool finite3(const float *v)
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static bool valid_mag_matrix(const float matrix[3][3])
{
    float a[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!isfinite(matrix[i][j]) || fabsf(matrix[i][j]) > 10.0f ||
                fabsf(matrix[i][j] - matrix[j][i]) > 0.00001f) return false;
            a[i][j] = 0.5f * (matrix[i][j] + matrix[j][i]);
        }
    }
    /* Jacobi diagonalization checks principal gains, not just matrix entries.
     * A symmetric matrix with every gain in [0.1,10] is positive definite. */
    for (int iteration = 0; iteration < 20; ++iteration) {
        int p = 0, q = 1;
        if (fabsf(a[0][2]) > fabsf(a[p][q])) { p = 0; q = 2; }
        if (fabsf(a[1][2]) > fabsf(a[p][q])) { p = 1; q = 2; }
        if (fabsf(a[p][q]) < 0.000001f) break;
        const float angle = 0.5f * atan2f(2.0f * a[p][q], a[q][q] - a[p][p]);
        const float c = cosf(angle), s = sinf(angle);
        const float pp = a[p][p], qq = a[q][q], pq = a[p][q];
        for (int k = 0; k < 3; ++k) {
            if (k == p || k == q) continue;
            const float kp = a[k][p], kq = a[k][q];
            a[k][p] = a[p][k] = c * kp - s * kq;
            a[k][q] = a[q][k] = s * kp + c * kq;
        }
        a[p][p] = c * c * pp - 2.0f * s * c * pq + s * s * qq;
        a[q][q] = s * s * pp + 2.0f * s * c * pq + c * c * qq;
        a[p][q] = a[q][p] = 0.0f;
    }
    for (int i = 0; i < 3; ++i) {
        float radius = 0.0f;
        for (int j = 0; j < 3; ++j) if (j != i) radius += fabsf(a[i][j]);
        if (a[i][i] - radius < 0.1f - 0.00001f ||
            a[i][i] + radius > 10.0f + 0.00001f) return false;
    }
    return true;
}

static float wrap_pi(float angle)
{
    while (angle > 3.14159265358979323846f) angle -= 6.28318530717958647692f;
    while (angle < -3.14159265358979323846f) angle += 6.28318530717958647692f;
    return angle;
}

static void initialize_tilt(filter_t *f, const float *a)
{
    const float roll = atan2f(a[1], a[2]);
    const float pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
    const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    f->q[0] = cr * cp;
    f->q[1] = sr * cp;
    f->q[2] = cr * sp;
    f->q[3] = -sr * sp;
    f->tilt_initialized = true;
}

static void reset_filter(filter_t *f, uint32_t discontinuities)
{
    memset(f, 0, sizeof(*f));
    f->q[0] = 1.0f;
    f->discontinuities = discontinuities;
}

static void reset_calibration(filter_t *f)
{
    f->calibration_count = 0;
    f->calibration_start_us = 0;
    memset(f->accel_mean, 0, sizeof(f->accel_mean));
    memset(f->accel_m2, 0, sizeof(f->accel_m2));
    memset(f->gyro_mean, 0, sizeof(f->gyro_mean));
    memset(f->gyro_m2, 0, sizeof(f->gyro_m2));
}

static void record_calibration_window(filter_t *f, const float *a)
{
    f->last_calibration_samples = f->calibration_count;
    float gravity_change[3] = {0};
    for (int i = 0; i < 3; ++i) {
        const float divisor = f->calibration_count > 1 ? (float)(f->calibration_count - 1) : 1.0f;
        f->last_calibration_gyro_std_dps[i] = sqrtf(fmaxf(0.0f, f->gyro_m2[i] / divisor));
        f->last_calibration_accel_std_g[i] = sqrtf(fmaxf(0.0f, f->accel_m2[i] / divisor));
        if (f->calibration_count) gravity_change[i] = a[i] - f->first_accel[i];
    }
    f->last_calibration_gravity_delta_g = norm3(gravity_change);
}

static void calibrate(filter_t *f, const float *a, const float *g, int64_t time_us)
{
    const float accel_norm = norm3(a);
    /* The measured board has about 7 deg/s combined zero bias. A tight
     * absolute-rate threshold would prevent calibration indefinitely. */
    uint32_t reject = 0;
    if (accel_norm < 0.90f || accel_norm > 1.10f) reject |= EXO_ATTITUDE_CAL_REJECT_ACCEL_NORM;
    if (norm3(g) > 20.0f) reject |= EXO_ATTITUDE_CAL_REJECT_GYRO_NORM;
    if (reject) {
        record_calibration_window(f, a);
        f->calibration_reject_mask = reject;
        ++f->calibration_rejections;
        reset_calibration(f);
        return;
    }
    if (!f->calibration_count) {
        f->calibration_start_us = time_us;
        memcpy(f->first_accel, a, sizeof(f->first_accel));
    }
    ++f->calibration_count;
    for (int i = 0; i < 3; ++i) {
        float delta = a[i] - f->accel_mean[i];
        f->accel_mean[i] += delta / (float)f->calibration_count;
        f->accel_m2[i] += delta * (a[i] - f->accel_mean[i]);
        delta = g[i] - f->gyro_mean[i];
        f->gyro_mean[i] += delta / (float)f->calibration_count;
        f->gyro_m2[i] += delta * (g[i] - f->gyro_mean[i]);
    }
    if (time_us - f->calibration_start_us < CALIBRATION_US ||
        f->calibration_count < CALIBRATION_MIN_SAMPLES) return;

    record_calibration_window(f, a);
    for (int i = 0; i < 3; ++i) {
        if (f->last_calibration_gyro_std_dps[i] > CALIBRATION_GYRO_STD_DPS) reject |= EXO_ATTITUDE_CAL_REJECT_GYRO_STD;
        if (f->last_calibration_accel_std_g[i] > 0.015f) reject |= EXO_ATTITUDE_CAL_REJECT_ACCEL_STD;
    }
    if (f->last_calibration_gravity_delta_g > 0.03f) reject |= EXO_ATTITUDE_CAL_REJECT_GRAVITY_DELTA;
    f->calibration_reject_mask = reject;
    if (reject) {
        ++f->calibration_rejections;
        reset_calibration(f);
        return;
    }
    /* A six-axis IMU cannot distinguish constant rotation about gravity from
     * gyro bias: the operator must actually keep the board stationary. */
    memcpy(f->bias, f->gyro_mean, sizeof(f->bias));
    initialize_tilt(f, f->accel_mean);
    f->calibrated = true;
}

static void euler(const float *q, float *roll, float *pitch, float *yaw)
{
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                  1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    float sine_pitch = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (sine_pitch > 1.0f) sine_pitch = 1.0f;
    if (sine_pitch < -1.0f) sine_pitch = -1.0f;
    *pitch = asinf(sine_pitch);
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

static void integrate(filter_t *f, const float *a, const float *g, float dt, bool use_accel)
{
    float rate[3];
    for (int i = 0; i < 3; ++i) rate[i] = (g[i] - f->bias[i]) * DEG_TO_RAD;
    const float *q = f->q;
    if (use_accel) {
        const float inv_norm = 1.0f / norm3(a);
        const float gravity[] = {
            2.0f * (q[1] * q[3] - q[0] * q[2]),
            2.0f * (q[0] * q[1] + q[2] * q[3]),
            q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3],
        };
        /* Measured gravity cross estimated gravity, both in the body frame. */
        rate[0] += ACCEL_FEEDBACK_GAIN * inv_norm * (a[1] * gravity[2] - a[2] * gravity[1]);
        rate[1] += ACCEL_FEEDBACK_GAIN * inv_norm * (a[2] * gravity[0] - a[0] * gravity[2]);
        rate[2] += ACCEL_FEEDBACK_GAIN * inv_norm * (a[0] * gravity[1] - a[1] * gravity[0]);
    }
    /* Finite rotation avoids first-order integration drift at high rates. */
    const float speed = norm3(rate);
    const float half_angle = 0.5f * speed * dt;
    const float factor = speed > 0.000001f ? sinf(half_angle) / speed : 0.5f * dt;
    const float d[] = {cosf(half_angle), rate[0] * factor, rate[1] * factor, rate[2] * factor};
    const float next[] = {
        q[0] * d[0] - q[1] * d[1] - q[2] * d[2] - q[3] * d[3],
        q[0] * d[1] + q[1] * d[0] + q[2] * d[3] - q[3] * d[2],
        q[0] * d[2] - q[1] * d[3] + q[2] * d[0] + q[3] * d[1],
        q[0] * d[3] + q[1] * d[2] - q[2] * d[1] + q[3] * d[0],
    };
    const float inv_norm = 1.0f / sqrtf(next[0] * next[0] + next[1] * next[1] +
                                     next[2] * next[2] + next[3] * next[3]);
    for (int i = 0; i < 4; ++i) f->q[i] = next[i] * inv_norm;
}

static void rotate_world_yaw(filter_t *f, float delta)
{
    const float c = cosf(delta * 0.5f), s = sinf(delta * 0.5f);
    const float w = f->q[0], x = f->q[1], y = f->q[2], z = f->q[3];
    f->q[0] = c * w - s * z;
    f->q[1] = c * x - s * y;
    f->q[2] = c * y + s * x;
    f->q[3] = c * z + s * w;
}

void exo_attitude_update(const exo_sensor_status_t *sample)
{
    if (!sample) return;
    uint32_t generation;
    bool mag_enabled;
    bool mag_fusion_enabled;
    bool calibration_disabled;
    exo_attitude_mag_calibration_t mag;
    portENTER_CRITICAL(&s_lock);
    generation = s_generation;
    calibration_disabled = s_calibration_disabled;
    mag_enabled = s_mag_enabled;
    mag_fusion_enabled = s_mag_fusion_enabled;
    mag = s_mag;
    portEXIT_CRITICAL(&s_lock);
    filter_t *f = &s_filter;
    if (generation != s_seen_generation) {
        reset_filter(f, 0);
        s_seen_generation = generation;
    }
    if (calibration_disabled) return;
    const exo_sensor_device_status_t *imu = &sample->qmi;
    const int64_t now = esp_timer_get_time();
    if (!sample->bus_ready || !imu->configured || !imu->valid || imu->timestamp_us <= 0 ||
        now < imu->timestamp_us || now - imu->timestamp_us > EXO_SENSOR_FRESH_US ||
        !finite3(imu->value) || !finite3(imu->value + 3)) {
        portENTER_CRITICAL(&s_lock);
        if (generation == s_generation) s_status.valid = false;
        portEXIT_CRITICAL(&s_lock);
        return;
    }
    if (f->previous_us == imu->timestamp_us) return; /* Never integrate a duplicate. */
    int64_t interval = imu->timestamp_us - f->previous_us;
    if (f->previous_us && (interval <= 0 || interval > MAX_SAMPLE_GAP_US)) {
        reset_filter(f, f->discontinuities + 1);
        interval = 0;
    }
    const float dt = f->previous_us ? (float)interval * 0.000001f : 0.0f;
    f->previous_us = imu->timestamp_us;
    const float accel_norm = norm3(imu->value);
    const bool use_accel = accel_norm >= 0.85f && accel_norm <= 1.15f;
    if (!f->tilt_initialized && use_accel) initialize_tilt(f, imu->value);
    const bool was_calibrated = f->calibrated;
    if (!f->calibrated) {
        calibrate(f, imu->value, imu->value + 3, imu->timestamp_us);
        if (!f->calibrated && use_accel) initialize_tilt(f, imu->value);
    }
    if (was_calibrated && f->tilt_initialized && dt > 0.0f) {
        integrate(f, imu->value, imu->value + 3, dt, use_accel);
    }
    ++f->updates;
    exo_attitude_status_t status = {
        .valid = f->calibrated && f->tilt_initialized,
        .gyro_calibrated = f->calibrated,
        .accel_used = use_accel,
        .mag_calibrated = mag_enabled,
        .updates = f->updates,
        .calibration_samples = f->calibration_count,
        .calibration_reject_mask = f->calibration_reject_mask,
        .calibration_rejections = f->calibration_rejections,
        .last_calibration_samples = f->last_calibration_samples,
        .last_calibration_gravity_delta_g = f->last_calibration_gravity_delta_g,
        .discontinuities = f->discontinuities,
        .timestamp_us = imu->timestamp_us,
    };
    if (f->calibrated) status.calibration_progress_pct = 100;
    else if (f->calibration_count) {
        uint32_t progress = (uint32_t)((imu->timestamp_us - f->calibration_start_us) * 100 / CALIBRATION_US);
        status.calibration_progress_pct = progress < 100 ? progress : 99;
    }
    float roll, pitch, yaw;
    euler(f->q, &roll, &pitch, &yaw);
    const exo_sensor_device_status_t *m = &sample->qmc;
    status.mag_available = m->configured && m->valid && m->timestamp_us > 0 &&
                           m->timestamp_us <= now && now - m->timestamp_us <= EXO_SENSOR_FRESH_US &&
                           finite3(m->value);
    if (status.mag_available) {
        status.mag_timestamp_us = m->timestamp_us;
        float corrected[3];
        for (int i = 0; i < 3; ++i) {
            corrected[i] = m->value[i];
            if (!mag_enabled) continue;
            if (mag.use_matrix) {
                corrected[i] = 0.0f;
                for (int j = 0; j < 3; ++j) {
                    corrected[i] += mag.matrix[i][j] * (m->value[j] - mag.offset_ut[j]);
                }
            } else {
                corrected[i] = (m->value[i] - mag.offset_ut[i]) * mag.scale[i];
            }
        }
        float body[3];
        for (int i = 0; i < 3; ++i) {
            const int axis = mag_enabled ? mag.axis[i] : i + 1;
            const int source = (axis < 0 ? -axis : axis) - 1;
            body[i] = corrected[source];
            if (axis < 0) body[i] = -body[i];
        }
        status.mag_norm_ut = norm3(body);
        const float cp = cosf(pitch), sp = sinf(pitch), cr = cosf(roll), sr = sinf(roll);
        const float horizontal_x = cp * body[0] + sp * sr * body[1] + sp * cr * body[2];
        const float horizontal_y = cr * body[1] - sr * body[2];
        status.mag_heading_valid = f->tilt_initialized && status.mag_norm_ut >= 15.0f &&
                                   status.mag_norm_ut <= 85.0f &&
                                   hypotf(horizontal_x, horizontal_y) >= 5.0f;
        if (status.mag_heading_valid) {
            const float heading = atan2f(-horizontal_y, horizontal_x);
            status.mag_heading_deg = heading * RAD_TO_DEG;
            const float innovation = wrap_pi(heading - yaw);
            const bool consistent = !f->magnetic_reference ||
                (status.mag_norm_ut >= 0.7f * f->magnetic_norm &&
                 status.mag_norm_ut <= 1.3f * f->magnetic_norm && fabsf(innovation) <= 35.0f * DEG_TO_RAD);
            if (mag_fusion_enabled && f->calibrated && consistent) {
                const float correction = f->magnetic_reference ? innovation * (1.0f - expf(-dt / 2.0f)) : innovation;
                rotate_world_yaw(f, correction);
                if (!f->magnetic_reference) f->magnetic_norm = status.mag_norm_ut;
                f->magnetic_reference = true;
                status.mag_fused = true;
                euler(f->q, &roll, &pitch, &yaw);
            }
        }
    }
    status.yaw_magnetic_reference = f->magnetic_reference;
    status.roll_deg = roll * RAD_TO_DEG;
    status.pitch_deg = pitch * RAD_TO_DEG;
    status.yaw_deg = yaw * RAD_TO_DEG;
    memcpy(status.gyro_bias_dps, f->bias, sizeof(status.gyro_bias_dps));
    memcpy(status.last_calibration_gyro_std_dps, f->last_calibration_gyro_std_dps,
           sizeof(status.last_calibration_gyro_std_dps));
    memcpy(status.last_calibration_accel_std_g, f->last_calibration_accel_std_g,
           sizeof(status.last_calibration_accel_std_g));
    memcpy(status.quaternion, f->q, sizeof(status.quaternion));
    portENTER_CRITICAL(&s_lock);
    if (generation == s_generation) s_status = status;
    portEXIT_CRITICAL(&s_lock);
}

void exo_attitude_get_status(exo_attitude_status_t *status)
{
    if (!status) return;
    portENTER_CRITICAL(&s_lock);
    *status = s_status;
    status->calibration_disabled = s_calibration_disabled;
    status->mag_fusion_enabled = s_mag_fusion_enabled;
    memcpy(status->mag_offset_ut, s_mag.offset_ut, sizeof(status->mag_offset_ut));
    memcpy(status->mag_scale, s_mag.scale, sizeof(status->mag_scale));
    for (int i = 0; i < 3; ++i) status->mag_axis[i] = s_mag_enabled ? s_mag.axis[i] : i + 1;
    status->mag_matrix_enabled = s_mag_enabled && s_mag.use_matrix;
    memcpy(status->mag_matrix, s_mag.matrix, sizeof(status->mag_matrix));
    portEXIT_CRITICAL(&s_lock);
    const int64_t now = esp_timer_get_time();
    if (!status->timestamp_us || now < status->timestamp_us ||
        now - status->timestamp_us > EXO_SENSOR_FRESH_US) {
        status->valid = false;
        status->accel_used = false;
        status->mag_fused = false;
    }
    if (!status->mag_timestamp_us || now < status->mag_timestamp_us ||
        now - status->mag_timestamp_us > EXO_SENSOR_FRESH_US) {
        status->mag_available = false;
        status->mag_heading_valid = false;
        status->mag_fused = false;
    }
}

esp_err_t exo_attitude_enable_mag_fusion(void)
{
    portENTER_CRITICAL(&s_lock);
    if (s_calibration_disabled) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_NOT_SUPPORTED;
    }
    memset(&s_mag, 0, sizeof(s_mag));
    s_mag_enabled = false;
    s_mag_fusion_enabled = true;
    ++s_generation;
    memset(&s_status, 0, sizeof(s_status));
    s_status.quaternion[0] = 1.0f;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

void exo_attitude_disable_calibration(void)
{
    portENTER_CRITICAL(&s_lock);
    s_calibration_disabled = true;
    memset(&s_mag, 0, sizeof(s_mag));
    s_mag_enabled = false;
    s_mag_fusion_enabled = false;
    ++s_generation;
    memset(&s_status, 0, sizeof(s_status));
    s_status.quaternion[0] = 1.0f;
    portEXIT_CRITICAL(&s_lock);
}

void exo_attitude_reset(void)
{
    portENTER_CRITICAL(&s_lock);
    ++s_generation;
    memset(&s_status, 0, sizeof(s_status));
    s_status.quaternion[0] = 1.0f;
    s_status.mag_calibrated = s_mag_enabled;
    portEXIT_CRITICAL(&s_lock);
}

esp_err_t exo_attitude_set_mag_calibration(const exo_attitude_mag_calibration_t *calibration)
{
    if (calibration) {
        unsigned used_axes = 0;
        for (int i = 0; i < 3; ++i) {
            const int axis = calibration->axis[i];
            const unsigned source = (unsigned)(axis < 0 ? -axis : axis);
            if (source < 1 || source > 3 || (used_axes & (1U << source)) ||
                !isfinite(calibration->offset_ut[i]) || fabsf(calibration->offset_ut[i]) > 2000.0f ||
                (!calibration->use_matrix && (!isfinite(calibration->scale[i]) ||
                 calibration->scale[i] < 0.1f || calibration->scale[i] > 10.0f))) {
                return ESP_ERR_INVALID_ARG;
            }
            used_axes |= 1U << source;
        }
        if (calibration->use_matrix && !valid_mag_matrix(calibration->matrix)) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    portENTER_CRITICAL(&s_lock);
    if (s_calibration_disabled && calibration) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (calibration) {
        s_mag = *calibration;
        if (s_mag.use_matrix) {
            for (int i = 0; i < 3; ++i) s_mag.scale[i] = s_mag.matrix[i][i];
        } else {
            memset(s_mag.matrix, 0, sizeof(s_mag.matrix));
        }
    } else {
        memset(&s_mag, 0, sizeof(s_mag));
    }
    s_mag_enabled = calibration != NULL;
    s_mag_fusion_enabled = calibration != NULL;
    ++s_generation;
    memset(&s_status, 0, sizeof(s_status));
    s_status.quaternion[0] = 1.0f;
    s_status.mag_calibrated = s_mag_enabled;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}
