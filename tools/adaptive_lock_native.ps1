param([string]$ZigPath)
$ErrorActionPreference='Stop'
$repoRoot=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if(-not $ZigPath) { $ZigPath=Join-Path $repoRoot 'build/native_toolchain/zig-windows-x86_64-0.13.0/zig.exe' }
if(-not (Test-Path -LiteralPath $ZigPath)) { throw 'Provide -ZigPath to a verified local Zig compiler' }
$nativeOutput=Join-Path $repoRoot 'build/adaptive_host_native'
New-Item -ItemType Directory -Path $nativeOutput -Force | Out-Null
$sources=@('tools/adaptive_lock_replay.c','main/exo_adaptive_runtime.c','main/exo_adaptive.c','main/exo_afo.c',
    'main/exo_adaptive_event.c','main/exo_adaptive_profile.c','main/exo_adaptive_config.c','main/exo_adaptive_safety.c',
    'main/exo_assist.c','main/exo_swing_pulse.c','main/exo_phase_profile.c','main/exo_controller.c','main/exo_bench_peak.c',
    'main/exo_command_composer.c','main/sine_reference.c','main/exo_sensor_snapshot.c','main/gait_estimator.c')
Push-Location $repoRoot
try {
    $executable=Join-Path $nativeOutput 'lock_diagnostics.exe'
    # Native fallback only; no device access and no product-source compatibility shims.
    & $ZigPath cc -std=c11 -O1 -g -Wall -Wextra -Werror -pedantic `
        -Itools/host_fakes -Itools/host_fakes/factory -Itools/host_fakes/system -Imain/include `
        -DEXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=500 @sources -o $executable
    if($LASTEXITCODE -ne 0) { throw 'Native diagnostic compilation/link failed' }
    & $executable
    if($LASTEXITCODE -ne 0) { throw 'Native diagnostic check failed' }
    Write-Output 'PASS native diagnostic check; Zig C compiler; no ASan/UBSan in this fallback'
} finally { Pop-Location }
