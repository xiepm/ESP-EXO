param()
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$drive = $repoRoot.Substring(0, 1).ToLowerInvariant()
$tail = $repoRoot.Substring(2).Replace('\', '/')
$wslRoot = "/mnt/$drive$tail"
# 仅运行主机回放，绝不打开串口、烧录或驱动电机。
$command = "set -eu; cd '$wslRoot'; " +
    'mkdir -p build/phase10_host; ' +
    'gcc -std=c11 -Wall -Wextra -Werror -pedantic ' +
    '-Itools/host_fakes/factory -Itools/host_fakes/system -Imain/include -Itools/host_fakes ' +
    'tools/phase10_direction_teaching_replay.c main/exo_ble_protocol.c -lm ' +
    '-o build/phase10_host/phase10_direction_teaching_replay; ' +
    './build/phase10_host/phase10_direction_teaching_replay'
& wsl.exe sh -lc $command
if ($LASTEXITCODE -ne 0) { throw "Direction teaching replay failed: $LASTEXITCODE" }
