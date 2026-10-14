param()

$ErrorActionPreference = 'Stop'
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path (Join-Path $scriptDir '..')).Path
$drive = $repoRoot.Substring(0, 1).ToLowerInvariant()
$tail = $repoRoot.Substring(2).Replace('\', '/')
$wslRoot = "/mnt/$drive$tail"

$command = "set -eu; cd '$wslRoot'; " +
    'mkdir -p build/phase10_host; ' +
    'gcc -std=c11 -Wall -Wextra -Werror -pedantic ' +
    '-Itools/host_fakes -Imain/include ' +
    'tools/phase10_assist_integration_replay.c ' +
    'main/exo_sensor_snapshot.c main/gait_estimator.c main/exo_assist.c main/exo_swing_pulse.c ' +
    'main/exo_phase_profile.c main/exo_controller.c ' +
    'main/exo_command_composer.c main/sine_reference.c -lm ' +
    '-o build/phase10_host/phase10_assist_integration_replay; ' +
    './build/phase10_host/phase10_assist_integration_replay'

& wsl.exe sh -lc $command
if ($LASTEXITCODE -ne 0) {
    throw "Phase 10 assist integration replay failed with exit code $LASTEXITCODE"
}
