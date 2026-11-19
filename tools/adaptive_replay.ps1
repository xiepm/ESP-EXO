param([switch]$Sanitize)
$ErrorActionPreference='Stop'
$repoRoot=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$wslRoot='/mnt/'+$repoRoot.Substring(0,1).ToLowerInvariant()+$repoRoot.Substring(2).Replace('\','/')
$flags=if($Sanitize){'-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer'}else{'-O2'}
$core='main/exo_adaptive.c main/exo_afo.c main/exo_adaptive_event.c main/exo_adaptive_profile.c main/exo_adaptive_config.c main/exo_adaptive_safety.c'
# 只运行主机编译和回放。无串口、BLE、烧录或运动命令。
$commands=@"
set -eu
cd '$wslRoot'
mkdir -p build/adaptive_host
gcc -std=c11 $flags -Wall -Wextra -Werror -pedantic -Imain/include tools/adaptive_replay.c $core -lm -o build/adaptive_host/adaptive_replay
./build/adaptive_host/adaptive_replay build/adaptive_host/asymmetric.csv
gcc -std=c11 $flags -Wall -Wextra -Werror -pedantic -Itools/host_fakes -Itools/host_fakes/factory -Itools/host_fakes/system -Imain/include tools/adaptive_integration_replay.c main/exo_adaptive_runtime.c $core main/exo_assist.c main/exo_swing_pulse.c main/exo_phase_profile.c main/exo_controller.c main/exo_bench_peak.c main/exo_command_composer.c main/sine_reference.c main/exo_sensor_snapshot.c main/gait_estimator.c -lm -o build/adaptive_host/integration
./build/adaptive_host/integration
gcc -std=c11 $flags -Wall -Wextra -Werror -pedantic -Imain/include tools/adaptive_fault_replay.c $core -lm -o build/adaptive_host/fault_replay
./build/adaptive_host/fault_replay
gcc -std=c11 $flags -Wall -Wextra -Werror -pedantic -DEXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=500 -Itools/host_fakes -Itools/host_fakes/factory -Itools/host_fakes/system -Imain/include tools/adaptive_active_replay.c main/exo_adaptive_runtime.c $core main/exo_assist.c main/exo_swing_pulse.c main/exo_phase_profile.c main/exo_controller.c main/exo_bench_peak.c main/exo_command_composer.c main/sine_reference.c main/exo_sensor_snapshot.c main/gait_estimator.c -lm -o build/adaptive_host/active_500
./build/adaptive_host/active_500
gcc -std=c11 $flags -Wall -Wextra -Werror -pedantic -DEXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=15000 -Itools/host_fakes -Itools/host_fakes/factory -Itools/host_fakes/system -Imain/include tools/adaptive_active_replay.c main/exo_adaptive_runtime.c $core main/exo_assist.c main/exo_swing_pulse.c main/exo_phase_profile.c main/exo_controller.c main/exo_bench_peak.c main/exo_command_composer.c main/sine_reference.c main/exo_sensor_snapshot.c main/gait_estimator.c -lm -o build/adaptive_host/active_15000
./build/adaptive_host/active_15000
gcc -std=c11 $flags -Wall -Wextra -Werror -pedantic -DEXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=500 -Itools/host_fakes -Itools/host_fakes/factory -Itools/host_fakes/system -Imain/include tools/adaptive_lock_replay.c main/exo_adaptive_runtime.c $core main/exo_assist.c main/exo_swing_pulse.c main/exo_phase_profile.c main/exo_controller.c main/exo_bench_peak.c main/exo_command_composer.c main/sine_reference.c main/exo_sensor_snapshot.c main/gait_estimator.c -lm -o build/adaptive_host/lock_diagnostics
./build/adaptive_host/lock_diagnostics
"@
wsl.exe sh -lc $commands
if($LASTEXITCODE -ne 0){throw "Adaptive host replay failed: $LASTEXITCODE"}
