param()

$ErrorActionPreference = 'Stop'
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path (Join-Path $scriptDir '..')).Path
$drive = $repoRoot.Substring(0, 1).ToLowerInvariant()
$tail = $repoRoot.Substring(2).Replace('\', '/')
$wslRoot = "/mnt/$drive$tail"

if (-not $wslRoot) {
    throw 'Cannot convert the repository path to WSL. Please install WSL.'
}

# Keep generated host-test files in the existing build directory.
# One shell line works in both Windows PowerShell 5.1 and PowerShell 7.
$command = "set -eu; cd '$wslRoot'; " +
    'mkdir -p build/phase9_host; ' +
    'gcc -std=c11 -Wall -Wextra -Werror -pedantic ' +
    '-Imain/include tools/phase9_gait_replay.c ' +
    'main/exo_sensor_snapshot.c main/gait_estimator.c ' +
    'main/exo_ble_protocol.c -lm ' +
    '-o build/phase9_host/phase9_gait_replay; ' +
    './build/phase9_host/phase9_gait_replay'

& wsl.exe sh -lc $command
if ($LASTEXITCODE -ne 0) {
    throw "Phase 9 host replay failed with exit code $LASTEXITCODE"
}
