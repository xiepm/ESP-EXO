/* Host test: compile the actual firmware estimator with clock/lock-only stubs.
 * gcc -std=c11 -Wall -Wextra -Werror -Itools/attitude-test -Imain
 *     main/exo_attitude.c tools/test_attitude.c -lm -o build/test_attitude.exe
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "exo_attitude.h"

#define PI 3.14159265358979323846f
#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)
#define NEAR(actual, expected, tolerance) CHECK(fabsf((actual) - (expected)) < (tolerance))

static int64_t clock_us = 1000000;
static exo_sensor_status_t sample;
static const float bias[3] = {-5.3f, -4.9f, -0.85f};
int64_t esp_timer_get_time(void) { return clock_us; }

static exo_attitude_status_t status(void)
{
    exo_attitude_status_t result;
    exo_attitude_get_status(&result);
    return result;
}

static void tick(void)
{
    clock_us += 10000;
    sample.qmi.timestamp_us = clock_us;
    if (sample.qmc.valid) sample.qmc.timestamp_us = clock_us;
    exo_attitude_update(&sample);
}

static void setup(void)
{
    CHECK(exo_attitude_set_mag_calibration(NULL) == ESP_OK);
    memset(&sample, 0, sizeof(sample));
    sample.bus_ready = sample.qmi.configured = sample.qmi.valid = true;
    sample.qmi.value[2] = 0.936f; /* Measured board acceleration scale. */
    memcpy(sample.qmi.value + 3, bias, sizeof(bias));
}

static void settle(void)
{
    for (int i = 0; i < 210; ++i) tick();
    CHECK(status().valid && status().gyro_calibrated);
    for (int i = 0; i < 3; ++i) NEAR(status().gyro_bias_dps[i], bias[i], 0.001f);
}

/* Generate body gravity/magnetic vectors from a known body-to-world Euler pose. */
static void pose(float roll_deg, float pitch_deg, float yaw_deg, float *a, float *m)
{
    const float r = roll_deg * PI / 180.0f, p = pitch_deg * PI / 180.0f, y = yaw_deg * PI / 180.0f;
    const float cr = cosf(r), sr = sinf(r), cp = cosf(p), sp = sinf(p), cy = cosf(y), sy = sinf(y);
    a[0] = -sp; a[1] = sr * cp; a[2] = cr * cp;
    /* Earth magnetic vector (40,0,20) uT. */
    m[0] = 40.0f * cy * cp - 20.0f * sp;
    m[1] = 40.0f * (cy * sp * sr - sy * cr) + 20.0f * cp * sr;
    m[2] = 40.0f * (cy * sp * cr + sy * sr) + 20.0f * cp * cr;
}

static void test_tilt_and_bias(void)
{
    setup();
    float unused[3];
    pose(30.0f, -20.0f, 0.0f, sample.qmi.value, unused);
    settle();
    NEAR(status().roll_deg, 30.0f, 0.01f);
    NEAR(status().pitch_deg, -20.0f, 0.01f);
    NEAR(status().yaw_deg, 0.0f, 0.01f);
    CHECK(!status().yaw_magnetic_reference && !status().mag_fused);
    for (int i = 0; i < 1000; ++i) tick();
    NEAR(status().yaw_deg, 0.0f, 0.02f); /* Real board's large bias was removed. */
    exo_attitude_status_t s = status();
    float norm = 0.0f;
    for (int i = 0; i < 4; ++i) norm += s.quaternion[i] * s.quaternion[i];
    NEAR(norm, 1.0f, 0.00001f);
}

static void test_yaw_and_acceleration_gate(void)
{
    setup(); settle();
    sample.qmi.value[5] = bias[2] + 90.0f;
    sample.qmi.value[2] = 3.0f; /* Reject acceleration feedback during impact. */
    for (int i = 0; i < 100; ++i) tick();
    CHECK(!status().accel_used && status().valid);
    NEAR(status().yaw_deg, 90.0f, 0.02f);
    NEAR(status().roll_deg, 0.0f, 0.01f);
    sample.qmi.value[2] = 1.0f;
    sample.qmi.value[5] = bias[2];
    for (int i = 0; i < 200; ++i) tick();
    NEAR(status().yaw_deg, 90.0f, 0.02f);
    CHECK(status().accel_used);
}

static void test_acceleration_correction_sign(void)
{
    setup(); settle();
    float unused[3];
    pose(25.0f, 15.0f, 0.0f, sample.qmi.value, unused);
    for (int i = 0; i < 500; ++i) tick();
    NEAR(status().roll_deg, 25.0f, 0.03f);
    NEAR(status().pitch_deg, 15.0f, 0.03f);
}

static void test_time_and_invalid_input(void)
{
    setup(); settle();
    const uint32_t count = status().updates;
    exo_attitude_update(&sample);
    CHECK(status().updates == count);
    clock_us += EXO_SENSOR_FRESH_US + 1;
    CHECK(!status().valid);
    tick();
    CHECK(!status().valid && status().discontinuities == 1);
    for (int i = 0; i < 205; ++i) tick();
    CHECK(status().valid);
    sample.qmi.value[3] = NAN;
    tick();
    CHECK(!status().valid);
    sample.qmi.value[3] = bias[0];
    tick();
    CHECK(status().valid && isfinite(status().yaw_deg));
    clock_us -= 50000;
    tick();
    CHECK(!status().valid && status().discontinuities == 2);
}

static void test_moving_calibration(void)
{
    setup();
    for (int i = 0; i < 450; ++i) {
        sample.qmi.value[3] = bias[0] + ((i & 1) ? 2.0f : -2.0f);
        tick();
    }
    CHECK(!status().gyro_calibrated && !status().valid);
    sample.qmi.value[3] = bias[0];
    for (int i = 0; i < 430; ++i) tick();
    CHECK(status().valid);
}

static void test_measured_stationary_noise(void)
{
    /* Historical noise and the latest confirmed-stationary board measurement. */
    const float noise_levels[] = {0.762f, 1.2f};
    for (size_t level = 0; level < sizeof(noise_levels) / sizeof(noise_levels[0]); ++level) {
        setup();
        for (int i = 0; i < 210; ++i) {
            sample.qmi.value[4] = bias[1] + ((i & 1) ? noise_levels[level] : -noise_levels[level]);
            tick();
        }
        CHECK(status().valid && status().gyro_calibrated);
        CHECK(status().calibration_reject_mask == 0);
        NEAR(status().last_calibration_gyro_std_dps[1], noise_levels[level], 0.01f);
        for (int axis = 0; axis < 3; ++axis) {
            NEAR(status().gyro_bias_dps[axis], bias[axis], 0.01f);
        }
    }
}

static void test_magnetic_setup(void)
{
    setup();
    float body[3];
    pose(30.0f, -20.0f, 60.0f, sample.qmi.value, body);
    memcpy(sample.qmc.value, body, sizeof(body));
    sample.qmc.configured = sample.qmc.valid = true;
    settle();
    CHECK(status().mag_available && status().mag_heading_valid);
    NEAR(status().mag_heading_deg, 60.0f, 0.01f);
    CHECK(!status().mag_calibrated && !status().mag_fused);
    NEAR(status().yaw_deg, 0.0f, 0.01f);

    exo_attitude_mag_calibration_t config = {
        .axis = {2, -1, 3}, .offset_ut = {1, -2, 3}, .scale = {2, 0.5f, 1},
    };
    /* Distorted chip axes to exercise both calibration and installation map. */
    sample.qmc.value[0] = -body[1] / config.scale[0] + config.offset_ut[0];
    sample.qmc.value[1] = body[0] / config.scale[1] + config.offset_ut[1];
    sample.qmc.value[2] = body[2] / config.scale[2] + config.offset_ut[2];
    CHECK(exo_attitude_set_mag_calibration(&config) == ESP_OK);
    settle();
    CHECK(status().mag_fused && status().yaw_magnetic_reference && status().mag_calibrated);
    NEAR(status().yaw_deg, 60.0f, 0.01f);
    NEAR(status().roll_deg, 30.0f, 0.01f);
    NEAR(status().pitch_deg, -20.0f, 0.01f);
    sample.qmc.value[0] = 300.0f; /* Magnetic interference rejects this update. */
    tick();
    CHECK(!status().mag_fused && !status().mag_heading_valid && status().valid);
    NEAR(status().yaw_deg, 60.0f, 0.01f);
    sample.qmc.valid = false;
    tick();
    CHECK(!status().mag_available && !status().mag_fused && status().valid);
    CHECK(status().yaw_magnetic_reference);
    config.axis[0] = 1; /* Duplicate source axis. */
    CHECK(exo_attitude_set_mag_calibration(&config) == ESP_ERR_INVALID_ARG);
    config.axis[0] = 2; config.scale[0] = NAN;
    CHECK(exo_attitude_set_mag_calibration(&config) == ESP_ERR_INVALID_ARG);
    CHECK(exo_attitude_set_mag_calibration(NULL) == ESP_OK);
    CHECK(!status().valid && !status().mag_calibrated && !status().yaw_magnetic_reference);
}

static void test_full_magnetic_matrix(void)
{
    setup();
    float body[3];
    pose(30.0f, -20.0f, 60.0f, sample.qmi.value, body);
    sample.qmc.configured = sample.qmc.valid = true;
    exo_attitude_mag_calibration_t config = {
        .axis = {2, -1, 3}, .offset_ut = {11, -7, 3}, .use_matrix = true,
        .matrix = {{1.5f, 0.25f, 0.1f}, {0.25f, 0.75f, -0.08f}, {0.1f, -0.08f, 1.25f}},
    };
    /* Undo the known mounting and coupled soft-iron matrix, then add offset.
     * Diagonal-only correction cannot recover this heading or field norm. */
    const float chip[] = {-body[1], body[0], body[2]};
    const float a = 1.5f, b = 0.25f, c = 0.1f, d = 0.75f, e = -0.08f, f = 1.25f;
    const float determinant = a * (d * f - e * e) - b * (b * f - e * c) + c * (b * e - d * c);
    sample.qmc.value[0] = ((d * f - e * e) * chip[0] + (c * e - b * f) * chip[1] +
                           (b * e - c * d) * chip[2]) / determinant + config.offset_ut[0];
    sample.qmc.value[1] = ((c * e - b * f) * chip[0] + (a * f - c * c) * chip[1] +
                           (b * c - a * e) * chip[2]) / determinant + config.offset_ut[1];
    sample.qmc.value[2] = ((b * e - c * d) * chip[0] + (b * c - a * e) * chip[1] +
                           (a * d - b * b) * chip[2]) / determinant + config.offset_ut[2];
    CHECK(exo_attitude_set_mag_calibration(&config) == ESP_OK);
    settle();
    exo_attitude_status_t s = status();
    CHECK(s.mag_matrix_enabled && s.mag_calibrated && s.mag_fused);
    NEAR(s.roll_deg, 30.0f, 0.01f);
    NEAR(s.pitch_deg, -20.0f, 0.01f);
    NEAR(s.yaw_deg, 60.0f, 0.01f);
    NEAR(s.mag_norm_ut, sqrtf(2000.0f), 0.001f);
    for (int i = 0; i < 3; ++i) {
        NEAR(s.mag_scale[i], config.matrix[i][i], 0.00001f);
        NEAR(s.mag_offset_ut[i], config.offset_ut[i], 0.00001f);
        for (int j = 0; j < 3; ++j) NEAR(s.mag_matrix[i][j], config.matrix[i][j], 0.00001f);
    }
    clock_us += 10000;
    sample.qmi.timestamp_us = sample.qmc.timestamp_us = clock_us;
    const exo_sensor_status_t original = sample;
    exo_attitude_update(&sample);
    CHECK(memcmp(&original, &sample, sizeof(sample)) == 0);

    const exo_attitude_mag_calibration_t good = config;
    config.matrix[0][1] = config.matrix[1][0] = NAN;
    CHECK(exo_attitude_set_mag_calibration(&config) == ESP_ERR_INVALID_ARG);
    config = good;
    config.matrix[0][1] = config.matrix[1][0] = 2.0f; /* Negative eigenvalue. */
    CHECK(exo_attitude_set_mag_calibration(&config) == ESP_ERR_INVALID_ARG);
    config = good;
    config.matrix[0][1] = 0.1f; /* Not symmetric. */
    CHECK(exo_attitude_set_mag_calibration(&config) == ESP_ERR_INVALID_ARG);
    config = good;
    config.matrix[2][2] = 0.09f;
    CHECK(exo_attitude_set_mag_calibration(&config) == ESP_ERR_INVALID_ARG);
    config.matrix[2][2] = 10.1f;
    CHECK(exo_attitude_set_mag_calibration(&config) == ESP_ERR_INVALID_ARG);
    CHECK(status().mag_matrix_enabled);
    NEAR(status().mag_matrix[0][1], good.matrix[0][1], 0.00001f);

    /* Public gain bounds are inclusive, and legacy diagonal settings still
     * work after matrix mode without retaining stale cross-axis terms. */
    memset(config.matrix, 0, sizeof(config.matrix));
    config.matrix[0][0] = 0.1f;
    config.matrix[1][1] = 1.0f;
    config.matrix[2][2] = 10.0f;
    CHECK(exo_attitude_set_mag_calibration(&config) == ESP_OK);
    config = good;
    config.use_matrix = false;
    config.scale[0] = config.scale[1] = config.scale[2] = 1.0f;
    CHECK(exo_attitude_set_mag_calibration(&config) == ESP_OK);
    CHECK(!status().mag_matrix_enabled);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) CHECK(status().mag_matrix[i][j] == 0.0f);
    }
    CHECK(exo_attitude_set_mag_calibration(NULL) == ESP_OK);
    CHECK(!status().mag_matrix_enabled && !status().mag_calibrated);
}

static void test_raw_magnetic_fusion(void)
{
    setup();
    float body[3];
    pose(30.0f, -20.0f, 60.0f, sample.qmi.value, body);
    memcpy(sample.qmc.value, body, sizeof(body));
    sample.qmc.configured = sample.qmc.valid = true;
    const exo_attitude_mag_calibration_t old_config = {
        .axis = {2, -1, 3}, .offset_ut = {1, -2, 3}, .scale = {2, 0.5f, 1},
    };
    CHECK(exo_attitude_set_mag_calibration(&old_config) == ESP_OK);
    CHECK(exo_attitude_enable_mag_fusion() == ESP_OK);
    settle();
    exo_attitude_status_t s = status();
    CHECK(s.mag_fusion_enabled && !s.mag_calibrated && s.mag_fused);
    CHECK(s.mag_available && s.mag_heading_valid && s.yaw_magnetic_reference);
    for (int i = 0; i < 3; ++i) {
        CHECK(s.mag_offset_ut[i] == 0.0f);
        CHECK(s.mag_scale[i] == 0.0f);
        for (int j = 0; j < 3; ++j) CHECK(s.mag_matrix[i][j] == 0.0f);
    }
    CHECK(!s.mag_matrix_enabled);
    NEAR(s.roll_deg, 30.0f, 0.01f);
    NEAR(s.pitch_deg, -20.0f, 0.01f);
    NEAR(s.yaw_deg, 60.0f, 0.01f);
    NEAR(s.mag_norm_ut, sqrtf(2000.0f), 0.001f);

    /* Raw fusion reads the physical sensor vectors without correcting them. */
    clock_us += 10000;
    sample.qmi.timestamp_us = sample.qmc.timestamp_us = clock_us;
    const exo_sensor_status_t original = sample;
    exo_attitude_update(&sample);
    CHECK(memcmp(&original, &sample, sizeof(sample)) == 0);

    const float weak[] = {1, 2, 3};
    memcpy(sample.qmc.value, weak, sizeof(weak));
    tick();
    CHECK(status().valid && status().mag_available && !status().mag_heading_valid);
    CHECK(!status().mag_fused && status().mag_fusion_enabled);
    sample.qmc.value[0] = 100.0f;
    sample.qmc.value[1] = sample.qmc.value[2] = 0.0f;
    tick();
    CHECK(status().valid && !status().mag_heading_valid && !status().mag_fused);

    /* A strong vertical field still cannot supply a horizontal heading. */
    for (int i = 0; i < 3; ++i) sample.qmc.value[i] = 40.0f * sample.qmi.value[i];
    tick();
    CHECK(status().mag_available && !status().mag_heading_valid && !status().mag_fused);

    /* A stale magnetic sample must not stop fresh six-axis propagation. */
    memcpy(sample.qmc.value, body, sizeof(body));
    clock_us += 10000;
    sample.qmi.timestamp_us = clock_us;
    sample.qmc.timestamp_us = clock_us - EXO_SENSOR_FRESH_US - 1;
    exo_attitude_update(&sample);
    CHECK(status().valid && !status().mag_available && !status().mag_fused);
    tick();
    CHECK(status().mag_fused && !status().mag_calibrated);
    NEAR(status().yaw_deg, 60.0f, 0.02f);

    CHECK(exo_attitude_set_mag_calibration(NULL) == ESP_OK);
    CHECK(!status().mag_fusion_enabled && !status().mag_fused);
    settle();
    CHECK(!status().mag_fused && !status().yaw_magnetic_reference);
    CHECK(exo_attitude_enable_mag_fusion() == ESP_OK);
    settle();
    CHECK(status().mag_fusion_enabled && status().mag_fused && !status().mag_calibrated);
    NEAR(status().yaw_deg, 60.0f, 0.02f);
}

static void check_calibration_disabled(void)
{
    const exo_attitude_status_t s = status();
    CHECK(s.calibration_disabled);
    CHECK(!s.valid && !s.gyro_calibrated && !s.mag_calibrated);
    CHECK(!s.mag_fused && !s.yaw_magnetic_reference);
    CHECK(!s.mag_fusion_enabled);
    CHECK(!s.mag_matrix_enabled);
    CHECK(s.calibration_samples == 0 && s.calibration_progress_pct == 0);
    for (int i = 0; i < 3; ++i) {
        CHECK(s.gyro_bias_dps[i] == 0.0f);
        CHECK(s.mag_offset_ut[i] == 0.0f);
        CHECK(s.mag_scale[i] == 0.0f);
    }
}

/* This mode is intentionally one-way until reboot, so run this test last. */
static void test_disable_all_calibration(void)
{
    setup();
    const exo_attitude_mag_calibration_t config = {
        .axis = {1, 2, 3}, .offset_ut = {1, -2, 3}, .scale = {2, 0.5f, 1},
    };
    sample.qmc.configured = sample.qmc.valid = true;
    const int16_t raw[] = {1500, -750, 375};
    const float value[] = {40, -20, 10};
    memcpy(sample.qmc.raw, raw, sizeof(raw));
    memcpy(sample.qmc.value, value, sizeof(value));
    CHECK(exo_attitude_set_mag_calibration(&config) == ESP_OK);
    settle();
    const exo_attitude_status_t before = status();
    CHECK(!before.calibration_disabled && before.mag_calibrated);
    for (int i = 0; i < 3; ++i) {
        CHECK(before.mag_offset_ut[i] == config.offset_ut[i]);
        CHECK(before.mag_scale[i] == config.scale[i]);
    }

    exo_attitude_disable_calibration();
    check_calibration_disabled();
    /* The estimator cannot change raw sensor data or physical-unit values. */
    const exo_sensor_status_t original = sample;
    exo_attitude_update(&sample);
    CHECK(memcmp(&original, &sample, sizeof(sample)) == 0);
    for (int i = 0; i < 310; ++i) tick();
    check_calibration_disabled();
    CHECK(memcmp(sample.qmc.raw, raw, sizeof(raw)) == 0);
    CHECK(memcmp(sample.qmc.value, value, sizeof(value)) == 0);

    exo_attitude_reset();
    for (int i = 0; i < 210; ++i) tick();
    check_calibration_disabled();
    CHECK(exo_attitude_set_mag_calibration(NULL) == ESP_OK);
    check_calibration_disabled();
    CHECK(exo_attitude_set_mag_calibration(&config) == ESP_ERR_NOT_SUPPORTED);
    CHECK(exo_attitude_enable_mag_fusion() == ESP_ERR_NOT_SUPPORTED);
    for (int i = 0; i < 210; ++i) tick();
    check_calibration_disabled();
}

int main(void)
{
    test_tilt_and_bias();
    test_yaw_and_acceleration_gate();
    test_acceleration_correction_sign();
    test_time_and_invalid_input();
    test_moving_calibration();
    test_measured_stationary_noise();
    test_magnetic_setup();
    test_full_magnetic_matrix();
    test_raw_magnetic_fusion();
    test_disable_all_calibration();
    puts("PASS: tilt, real-board gyro bias, yaw, dynamic acceleration, correction sign, timestamps, motion rejection, calibrated magnetic mounting and interference, full soft-iron matrix and validation, raw magnetic fusion and rejection gates, locked raw mode clears all calibration");
    return 0;
}
