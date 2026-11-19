#pragma once

#include "exo_attitude.h"

/* Board calibration measured 2026-11-11 on COM3, QMC5883P +/-8G.
 * Source: .mklink/logs/mag-calibration-20261111/fit.json
 * Apply in QMC axes, then map into QMI body axes: [Y, -X, Z].
 * Symmetric soft-iron matrix has determinant one; absolute gain is unchanged.
 * Held-out radial RMS: 1.0383%; reference field in capture: 50.411575 uT.
 * Fit JSON SHA256: 7abd0949ffb6991eb15cccdf7d99935076b2778dff8e1e90da0ffa07f4d23394
 */
static const exo_attitude_mag_calibration_t EXO_BOARD_MAG_CALIBRATION = {
    .axis = {2, -1, 3},
    .offset_ut = {-2.27864476f, 1.57011358f, 3.44335084f},
    .use_matrix = true,
    .matrix = {
        {0.994442598f, -0.00772373771f, 0.00125102536f},
        {-0.00772373771f, 1.00148852f, -0.000509031141f},
        {0.00125102536f, -0.000509031141f, 1.00415582f}
    },
};
