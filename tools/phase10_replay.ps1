param()

$ErrorActionPreference = 'Stop'
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path (Join-Path $scriptDir '..')).Path
$drive = $repoRoot.Substring(0, 1).ToLowerInvariant()
$tail = $repoRoot.Substring(2).Replace('\', '/')
$wslRoot = "/mnt/$drive$tail"

if (-not $wslRoot) {
    throw 'Cannot convert the repository path to WSL. Please install WSL.'
}

# 直接编译固件中的曲线和合成器实现；生成物只放入已有 build/ 目录。

$command = "set -eu; cd '$wslRoot'; " +
    'mkdir -p build/phase10_host; ' +
    'gcc -std=c11 -Wall -Wextra -Werror -pedantic ' +
    '-Imain/include tools/phase10_profile_replay.c ' +
    'main/exo_phase_profile.c main/exo_command_composer.c -lm ' +
    '-o build/phase10_host/phase10_profile_replay; ' +
    './build/phase10_host/phase10_profile_replay'

& wsl.exe sh -lc $command
if ($LASTEXITCODE -ne 0) {
    throw "Phase 10 host replay failed with exit code $LASTEXITCODE"
}
