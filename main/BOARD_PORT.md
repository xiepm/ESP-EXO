# exo_v2.1 peripheral port (2026-11-17)

Source: user's known-working `saltedfish12138/exo_v2.1`, downloaded to
`D:/ESP32/esp_can/exo_v2.1-main/exo_v2.1-main`. That reference tree is unchanged.
`exo_sensors.[ch]`, `exo_attitude.[ch]`, `exo_mag_calibration.h` and
`exo_smbus.[ch]` were copied from its `main/`. Host attitude tests and their
minimal clock/lock fakes were copied to `tools/board_attitude*`.

The product retains its binary BLE service, calibration NVS namespaces,
START/STOP state machine and single 100Hz motor command path. The reference
NUS console, power-on commands and second motor controller were NOT imported.
Version0.11.13 did not raise GPIO42 based on the reported independent supply.
The approved GPIO42 A/B test subsequently proved this assumption wrong:
with GPIO42 off, both reads failed; with GPIO42 on, both motors replied,
TXconfirmed2/RX6/TEC0/REC0. See the diagnosis record.

Version0.11.14 initializes GPIO42 LOW before output configuration, then raises
it only after NVS, CAN/RX, state machine and control initialization succeed.
The application stays INIT/output-disabled for3s while motor electronics boot,
then enters IDLE and runs the original bounded feedback requests. No enable or
motion command is issued by the board-power module. Supply remains on in IDLE
and after STOP so encoder feedback/calibration remain available. Actual motor
enable and torque still require the existing explicit START. This supply
behavior is an intentional change from0.11.13. The user separately authorized
flashing0.11.14 and maintaining standby power on2026-11-17; deployment and
read-only target verification completed. It is not the diagnostic timer.

Current configuration (0.11.17): `CONFIG_EXO_BOARD_V21=y`,
`CONFIG_EXO_MOTOR_RS05` disabled. The user explicitly identified the installed
motors as **RS01** on2026-11-17. The earlier RS05 inference from the reference
program was incorrect for this device. RS01 wire ranges are speed ±44rad/s,
torque ±17Nm; position/KP/KD ranges are unchanged. Board revision must not
select a motor model. RS05 remains an explicit alternative (50rad/s,5.5Nm).
These constants control wire scaling in BOTH directions. Regular RS01 AFO
now accepts up to15Nm; RS05 and historical trial builds retain their5Nm cap.

I2C0 SDA8/SCL9 100kHz: QMI8658A 0x6B, QMC5883P 0x2C, 100Hz background
sampling (priority5 vs control priority9). QMC explicitly exits reset. The
estimator uses the same measured board magnetic matrix/axis mapping as the
reference. Gyro bias estimates anew from >=2s stationary data; it is unrelated
to persisted motor direction and wearable zero. Board attitude never substitutes
for hip encoders or marks missing CAN feedback valid. Invalid/stale snapshots
are not displayed as zero angles. Sensor reads are bounded by driver timeouts;
BLE reads cached snapshots only, not I2C.

I2C1 SDA15/SCL16 initializes SMBus without scanning or sending transactions.
Read/write APIs and PEC support are available internally; no arbitrary bus-write
UI is exposed. `board_status` reports cached sensor/attitude/SMBus diagnostics.

BLE v1 opcode 0x27, request payload one byte page0..3. Response payload16:
page u8, flags u8 (valid1, gyro-calibrated2, mag-fused4, configured8), age-ms
u16 LE (65535 missing/saturated), three IEEE754 float32 LE. Pages: Euler degrees,
accelerometer g, gyro degrees/s, magnetic field microtesla. Valid age <250ms.
Firmware0.11.13/0.11.14 gates this optional extension; old firmware receives no new
requests. Four pages are separate snapshots, not a synchronized four-page frame.
Web labels the data as read snapshots and shows read time/age.

Validation: `tools/board_v21_test.ps1`; `npm test` and `npx tsc --noEmit` in
web-bluetooth-console; target `idf.py build`. Actual CAN result and flash/NVS
evidence are in docs/phase10/EXO_V21_CAN_20261117.md and output/can/20261117.
