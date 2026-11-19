#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "exo_sensors.h"

#define EXO_ATTITUDE_CAL_REJECT_GYRO_STD (1U << 0)
#define EXO_ATTITUDE_CAL_REJECT_ACCEL_STD (1U << 1)
#define EXO_ATTITUDE_CAL_REJECT_GRAVITY_DELTA (1U << 2)
#define EXO_ATTITUDE_CAL_REJECT_ACCEL_NORM (1U << 3)
#define EXO_ATTITUDE_CAL_REJECT_GYRO_NORM (1U << 4)

/* Native QMI right-handed axes. Quaternion [w,x,y,z] rotates body to world;
 * Euler convention is Rz(yaw)*Ry(pitch)*Rx(roll), degrees, yaw in [-180,180].
 * World gravity/specific-force reference is +Z. Default yaw is relative to the
 * end of stationary gyro calibration; magnetic north requires explicit setup. */
typedef struct {
    bool valid;
    bool calibration_disabled;
    bool gyro_calibrated;
    bool accel_used;
    bool mag_available;
    bool mag_heading_valid;
    bool mag_calibrated;
    bool mag_fusion_enabled;
    bool mag_fused;
    bool yaw_magnetic_reference;
    uint32_t updates;
    uint32_t calibration_samples;
    uint32_t calibration_progress_pct;
    /* Last evaluated calibration window survives a rejected-window restart.
     * A successful window has reject_mask=0. Standard deviations are sample
     * estimates (N-1 denominator), in the same units as sensor values. */
    uint32_t calibration_reject_mask;
    uint32_t calibration_rejections;
    uint32_t last_calibration_samples;
    float last_calibration_gyro_std_dps[3];
    float last_calibration_accel_std_g[3];
    float last_calibration_gravity_delta_g;
    uint32_t discontinuities;
    int64_t timestamp_us;
    int64_t mag_timestamp_us;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_bias_dps[3];
    /* Actual stored magnetic correction parameters, even while disabled. */
    float mag_offset_ut[3];
    float mag_scale[3];
    int8_t mag_axis[3];
    bool mag_matrix_enabled;
    float mag_matrix[3][3];
    float quaternion[4];
    /* Without magnetic calibration, uncorrected native chip axes are assumed. */
    float mag_heading_deg;
    float mag_norm_ut;
} exo_attitude_status_t;

typedef struct {
    /* After chip-register sign correction: map chip axes to QMI body axes.
     * Each entry is a unique signed 1/2/3 (e.g. {2,-1,3}). Calibration offset
     * and scale/matrix apply in chip axes BEFORE this mapping. */
    int8_t axis[3];
    float offset_ut[3];
    /* Legacy diagonal gains when use_matrix=false. Matrix mode stores its
     * diagonal here for status; correction uses the entire matrix instead. */
    float scale[3];
    bool use_matrix;
    /* Symmetric positive-definite soft-iron correction in chip axes. */
    float matrix[3][3];
} exo_attitude_mag_calibration_t;

/* Called serially by the sampler for each new complete QMI sample. */
void exo_attitude_update(const exo_sensor_status_t *sample);
void exo_attitude_get_status(exo_attitude_status_t *status);
/* Enable magnetic yaw correction using uncorrected native chip axes. Clears
 * magnetic calibration and restarts stationary gyro calibration. The caller
 * must verify sensor mounting axes; this does not mark the magnetometer calibrated. */
esp_err_t exo_attitude_enable_mag_fusion(void);
/* Raw measurement mode: clear all software calibration and pause the estimator.
 * Calibration cannot be enabled again until reboot. Sensor sampling continues. */
void exo_attitude_disable_calibration(void);
/* Recalibrate gyro while stationary for >=2 seconds; clears yaw reference. */
void exo_attitude_reset(void);
/* Only supply independently measured mounting/calibration values. NULL clears
 * magnetic corrections and disables fusion. Raw mode rejects non-NULL settings.
 * Outside raw mode, changing these settings starts fresh gyro calibration. */
esp_err_t exo_attitude_set_mag_calibration(const exo_attitude_mag_calibration_t *calibration);
