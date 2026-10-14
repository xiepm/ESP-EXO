param()
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$drive = $repoRoot.Substring(0, 1).ToLowerInvariant()
$wslRoot = '/mnt/' + $drive + $repoRoot.Substring(2).Replace('\', '/')
# 仅主机编译与测试，不打开串口，不发送BLE或CAN。
$command = "set -eu; cd '$wslRoot'; mkdir -p build/phase10_host; " +
    'gcc -std=c11 -Wall -Wextra -Werror -pedantic -Imain/include ' +
    'tools/phase10_assist_limit_replay.c main/exo_ble_protocol.c ' +
    '-o build/phase10_host/phase10_assist_limit_replay; ' +
    './build/phase10_host/phase10_assist_limit_replay'
& wsl.exe sh -lc $command
if ($LASTEXITCODE -ne 0) { throw 'Assistance torque limit protocol replay failed' }
