param()
$ErrorActionPreference = 'Stop'
$taskRepo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$taskWsl = '/mnt/' + $taskRepo.Substring(0,1).ToLowerInvariant() + $taskRepo.Substring(2).Replace('\','/')
# 仅编译和回放内存反馈，不打开串口/BLE，不烧录，不操作电机。
$taskCommand = "set -eu; cd '$taskWsl'; mkdir -p build/phase10_host; " +
  'gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itools/host_fakes/factory ' +
  '-Itools/host_fakes/system -Imain/include -Itools/host_fakes ' +
  'tools/phase10_pair_zero_replay.c main/exo_ble_protocol.c -lm ' +
  '-o build/phase10_host/phase10_pair_zero_replay; ./build/phase10_host/phase10_pair_zero_replay'
& wsl.exe sh -lc $taskCommand
if ($LASTEXITCODE -ne 0) { throw "Pair-zero replay failed: $LASTEXITCODE" }
