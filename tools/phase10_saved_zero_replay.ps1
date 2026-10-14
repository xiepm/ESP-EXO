param()
$ErrorActionPreference = 'Stop'
$taskRepo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$taskWsl = '/mnt/' + $taskRepo.Substring(0,1).ToLowerInvariant() + $taskRepo.Substring(2).Replace('\','/')
# 只运行主机测试，不打开串口、不烧录、不向硬件发送任何命令。
$taskCommand = "set -eu; cd '$taskWsl'; mkdir -p build/phase10_host; " +
  'gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itools/host_fakes/factory ' +
  '-Itools/host_fakes/system -Imain/include -Itools/host_fakes ' +
  'tools/phase10_saved_zero_replay.c main/exo_ble_protocol.c -lm ' +
  '-o build/phase10_host/phase10_saved_zero_replay; ./build/phase10_host/phase10_saved_zero_replay'
& wsl.exe sh -lc $taskCommand
if ($LASTEXITCODE -ne 0) { throw "Saved-zero replay failed: $LASTEXITCODE" }
