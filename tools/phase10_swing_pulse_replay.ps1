param()
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$drive = $repoRoot.Substring(0,1).ToLowerInvariant()
$tail = $repoRoot.Substring(2).Replace('\','/')
$wslRoot = "/mnt/$drive$tail"
# 仅编译主机夹具；无串口/BLE/刷写/运动动作。
$command = "set -eu; cd '$wslRoot'; mkdir -p build/phase10_host; " +
    'gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itools/host_fakes -Imain/include ' +
    'tools/phase10_swing_pulse_replay.c main/exo_sensor_snapshot.c main/gait_estimator.c ' +
    'main/exo_assist.c main/exo_swing_pulse.c main/exo_phase_profile.c main/exo_ble_protocol.c ' +
    'main/exo_controller.c main/exo_command_composer.c main/sine_reference.c -lm ' +
    '-o build/phase10_host/phase10_swing_pulse_replay; ./build/phase10_host/phase10_swing_pulse_replay'
& wsl.exe sh -lc $command
if($LASTEXITCODE -ne 0) { throw "Swing pulse replay failed: $LASTEXITCODE" }
