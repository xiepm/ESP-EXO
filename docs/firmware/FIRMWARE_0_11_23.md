# 0.11.23 / AFO strategy 12 — archived hardware-test version

Archived on 2026-11-19 (Asia/Shanghai). This is the exact application flashed on 2026-11-18, not a new firmware build. It is a development/test snapshot, not a gait-validation release.

## Contents and download

- [Application BIN](../../firmware/0.11.23/exo_0.11.23.bin) — download the raw file, application offset `0x10000`.
- [ELF for debugging](../../firmware/0.11.23/exo_0.11.23.elf).
- [Frozen source ZIP](../../firmware/0.11.23/source_snapshot.zip) and [build manifest](../../firmware/0.11.23/manifest.json).
- Source at repository paths is byte-identical to all 291 entries in the candidate manifest. Unchanged supporting files are inherited from the parent commit.
- [USB and web recording guide](../phase10/AFO_USB_RECORDING_0_11_23.md).
- [Flash verification](evidence/0.11.23/flash_result.json).

Application SHA256: `646290f22fd5f6c7cf53a46509e292ea8b0a3c587b0ce0f1fa61ca7c74e11646`.
ELF SHA256: `927b04156f3832fae8372bcb3d756eec298ded5aa4bc004b2d9ef79f5aaccf0a`.

## Hardware and behavior

ESP32-S3, RS01 motors, CAN TX GPIO4 / RX GPIO5 at 1 Mbps; GPIO42 standby power retained. AFO strategy 12 is the v22r3 implementation. Working window is -40 to 90 degrees and 400 degrees/s; hard-fault interlocks remain. Existing mode/level/torque-cap/save/START workflow and persisted direction/zero storage remain.

0.11.23 adds bounded 100 Hz diagnostic history, read-only `afo_trace` USB retrieval, and combined web recording. It does not provide evidence that every real gait cycle receives assistance. Recorded/interpolated replay torque is a calculated command, not measured human interaction torque.

## Verification and unresolved issue

Candidate evidence records 11 native regression groups, 8 system-health cases, 206 frontend tests, TypeScript and ESP-IDF 5.5 build checks. Replay outputs match v22r3 across 163128 leg samples. These were completed before this archival upload; no rebuild was performed for upload.

Application flash verification passed; the 32 KiB partition/NVS/PHY region was identical before and after flashing. Device readback reported 0.11.23 with the expected ELF hash, IDLE, output disabled and zero START requests. One USB diagnostic batch returned 24 paired ticks. Private NVS backups are not included in this archive.

**Known unresolved BLE connection failure:** after flashing, browser GATT connection attempts failed. A separate Windows BLE client also timed out while no `afo_trace` commands were being sent. In that observation, the device session generation stayed zero, BLE telemetry stayed stopped, and control deadline misses were zero; advertising was seen at approximately -88 dBm. Earlier scanning observed approximately -84 dBm. This argues against active USB diagnostic throughput being required to reproduce the failure; it does not rule out resident tracing/resource effects or establish the root cause. A controlled firmware/radio comparison has not been completed. Sustained simultaneous USB/BLE throughput and real-gait stability remain unverified.

The original candidate manifest intentionally retains its preflash `BUILT_UNFLASHED` status; the linked flash evidence records the subsequent successful deployment. Do not interpret the candidate's build-time status as the final device state.

## Updating an already configured board

This is an application-only artifact. Enumerate the board's current serial port; do not assume COM7. Confirm IDLE/output disabled before updating. Use the project's ESP-IDF 5.5 esptool environment to write only the application at `0x10000`, verify it, and preserve the bootloader/partition/NVS/PHY areas. Do not use erase-all. This archive operation performs no flashing, START, or motor enable.
