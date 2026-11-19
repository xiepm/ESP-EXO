#pragma once
#include <stdint.h>
void exo_board_init(void);
/* Cached snapshots only: page 0 Euler deg, 1 accel g, 2 gyro deg/s, 3 mag uT.
 * Wire flags: valid=1, gyro calibrated=2, magnetic fusion=4, configured=8.
 * age_ms=65535 denotes missing/saturated. No I2C transactions in this call. */
void exo_board_read(uint8_t page, uint8_t *flags, uint16_t *age_ms, float values[3]);
void exo_board_print_status(void);
