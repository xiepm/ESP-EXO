param()
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$drive = $repoRoot.Substring(0, 1).ToLowerInvariant()
$tail = $repoRoot.Substring(2).Replace('\', '/')
$wslRoot = "/mnt/$drive$tail"
# 仅在WSL编译/运行真实状态机主机回放，不打开串口、不发送BLE/CAN。
$command = "set -eu; cd '$wslRoot'; " +
    'mkdir -p build/phase10_host; ' +
    'gcc -std=c11 -Wall -Wextra -Werror -pedantic -ffunction-sections -fdata-sections ' +
    '-Itools/host_fakes/system -Imain/include -Itools/host_fakes ' +
    'tools/phase10_runtime_fault_replay.c main/exo_sensor_snapshot.c -Wl,--gc-sections -lm ' +
    '-o build/phase10_host/phase10_runtime_fault_replay; ' +
    './build/phase10_host/phase10_runtime_fault_replay'
& wsl.exe sh -lc $command
if ($LASTEXITCODE -ne 0) { throw "Runtime fault replay failed: $LASTEXITCODE" }
