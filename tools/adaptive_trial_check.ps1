param([Parameter(Mandatory=$true)][string]$OutputDirectory)
$ErrorActionPreference='Stop'
$repoRoot=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$resultDir=[IO.Path]::GetFullPath($OutputDirectory)
if(Test-Path -LiteralPath $resultDir){throw 'Refusing to replace existing check output'}
New-Item -ItemType Directory -Path $resultDir -ErrorAction Stop | Out-Null
$zig=Join-Path $repoRoot 'build/native_toolchain/zig-windows-x86_64-0.13.0/zig.exe'
$core=@('main/exo_adaptive.c','main/exo_afo.c','main/exo_adaptive_event.c','main/exo_adaptive_profile.c','main/exo_adaptive_config.c','main/exo_adaptive_safety.c')
$runtime=@('main/exo_adaptive_runtime.c','main/exo_assist.c','main/exo_swing_pulse.c','main/exo_phase_profile.c','main/exo_controller.c','main/exo_bench_peak.c','main/exo_command_composer.c','main/sine_reference.c','main/exo_sensor_snapshot.c','main/gait_estimator.c')
$cases=@(
    @{Name='walking';Source='tools/adaptive_walking_replay.c';Flags=@();Sources=$core;Args=@()},
    @{Name='regular_mode';Source='tools/adaptive_mode_replay.c';Flags=@('-DCONFIG_EXO_ADAPTIVE_ASSIST_MODE=1');Sources=$core+$runtime;Args=@()},
    @{Name='core';Source='tools/adaptive_replay.c';Flags=@();Sources=$core;Args=@()},
    @{Name='fault';Source='tools/adaptive_fault_replay.c';Flags=@();Sources=$core;Args=@()},
    @{Name='lock';Source='tools/adaptive_lock_replay.c';Flags=@('-DEXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=500');Sources=$core+$runtime;Args=@()},
    @{Name='active500';Source='tools/adaptive_active_replay.c';Flags=@('-DEXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=500');Sources=$core+$runtime;Args=@()},
    @{Name='active15000';Source='tools/adaptive_active_replay.c';Flags=@('-DEXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=15000');Sources=$core+$runtime;Args=@()},
    @{Name='trial';Source='tools/adaptive_trial_replay.c';Flags=@('-DCONFIG_EXO_ADAPTIVE_BOUNDED_TRIAL=1');Sources=$core+$runtime;Args=@((Join-Path $resultDir 'trial_preset.bin'))},
    @{Name='probe5';Source='tools/adaptive_worn_entry_probe.c';Flags=@();Sources=$core;Args=@((Join-Path $resultDir 'probe5.csv'),'5')},
    @{Name='wire';Source='tools/nominal_protocol_replay.c';Flags=@();Sources=@('main/exo_ble_protocol.c');Args=@()}
)
Push-Location $repoRoot
try {
    foreach($case in $cases) {
        $exe=Join-Path $resultDir ($case.Name+'.exe')
        $compileArgs=@('cc','-std=c11','-O1','-g','-UNDEBUG','-Wall','-Wextra','-Werror','-pedantic','-Itools/host_fakes','-Itools/host_fakes/factory','-Itools/host_fakes/system','-Imain/include')+$case.Flags+@($case.Source)+$case.Sources+@('-o',$exe)
        & $zig @compileArgs *> (Join-Path $resultDir ($case.Name+'_build.log'))
        if($LASTEXITCODE -ne 0){throw ('Compilation failed: '+$case.Name)}
        $runArgs=$case.Args
        & $exe @runArgs *> (Join-Path $resultDir ($case.Name+'_run.log'))
        if($LASTEXITCODE -ne 0){throw ('Check failed: '+$case.Name)}
        Write-Output ('PASS '+$case.Name)
    }
} finally {Pop-Location}
