param([string]$ZigPath,[switch]$PeakOnly)
$ErrorActionPreference='Stop'
$repoRoot=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if(-not $ZigPath){$ZigPath=Join-Path $repoRoot 'build/native_toolchain/zig-windows-x86_64-0.13.0/zig.exe'}
$testOutput=Join-Path $repoRoot 'build/adaptive_host_native'
New-Item -ItemType Directory -Path $testOutput -Force | Out-Null
$core=@('main/exo_adaptive.c','main/exo_afo.c','main/exo_adaptive_event.c','main/exo_adaptive_profile.c','main/exo_adaptive_config.c','main/exo_adaptive_safety.c')
$integration=@('main/exo_adaptive_runtime.c','main/exo_assist.c','main/exo_swing_pulse.c','main/exo_phase_profile.c','main/exo_controller.c','main/exo_bench_peak.c','main/exo_command_composer.c','main/sine_reference.c','main/exo_sensor_snapshot.c','main/gait_estimator.c')
Push-Location $repoRoot
try {
    $exe=Join-Path $testOutput 'nominal_bench.exe'
    & $ZigPath cc -std=c11 -O1 -g -UNDEBUG -Wall -Wextra -Werror -pedantic -Itools/host_fakes -Itools/host_fakes/factory -Itools/host_fakes/system -Imain/include -DCONFIG_EXO_BENCH_PEAK_TEST=1 -DEXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=15000 tools/bench_peak_replay.c @core @integration -o $exe
    if($LASTEXITCODE -ne 0){throw 'Peak integration compilation failed'}
    & $exe
    if($LASTEXITCODE -ne 0){throw 'Peak integration checks failed'}
    if(-not $PeakOnly){
    $exe=Join-Path $testOutput 'nominal_cadence.exe'
    & $ZigPath cc -std=c11 -O1 -g -UNDEBUG -Wall -Wextra -Werror -pedantic -Imain/include tools/adaptive_cadence_replay.c @core -o $exe
    if($LASTEXITCODE -ne 0){throw 'Cadence compilation failed'}
    & $exe
    if($LASTEXITCODE -ne 0){throw 'Cadence checks failed'}
    $exe=Join-Path $testOutput 'nominal_protocol.exe'
    & $ZigPath cc -std=c11 -O1 -g -UNDEBUG -Wall -Wextra -Werror -pedantic -Imain/include tools/nominal_protocol_replay.c main/exo_ble_protocol.c -o $exe
    if($LASTEXITCODE -ne 0){throw 'Nominal protocol compilation failed'}
    & $exe
    if($LASTEXITCODE -ne 0){throw 'Nominal protocol checks failed'}
    }
    Write-Output 'PASS nominal candidate native checks; no hardware accessed; no ASan/UBSan in this native fallback'
} finally {Pop-Location}
