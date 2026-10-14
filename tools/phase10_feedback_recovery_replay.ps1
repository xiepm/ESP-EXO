param()
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$drive = $repoRoot.Substring(0, 1).ToLowerInvariant()
$tail = $repoRoot.Substring(2).Replace('\', '/')
$wslRoot = "/mnt/$drive$tail"
# 仅编译真实C模块+硬件桩，不打开串口，也不请求CAN/运动。
$command = "set -eu; cd '$wslRoot'; mkdir -p build/phase10_host; " +
    'for name in recovery frames protocol; do ' +
    'gcc -std=c11 -Wall -Wextra -Werror -pedantic -ffunction-sections -fdata-sections ' +
    '-Itools/host_fakes/system -Imain/include -Itools/host_fakes ' +
    'tools/phase10_feedback_${name}_replay.c main/exo_ble_protocol.c -Wl,--gc-sections -lm ' +
    '-o build/phase10_host/phase10_feedback_${name}_replay; ' +
    './build/phase10_host/phase10_feedback_${name}_replay; done'
& wsl.exe --exec sh -lc $command
if ($LASTEXITCODE -ne 0) { throw "Feedback recovery replay failed: $LASTEXITCODE" }
