param([switch]$Nominal15,[switch]$Release)
$ErrorActionPreference='Stop'
$repoRoot=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
. (Join-Path $PSScriptRoot 'activate_esp_idf.ps1')
$benchBuildPath=Join-Path $repoRoot 'build/adaptive_bench'
New-Item -ItemType Directory -Path $benchBuildPath -Force | Out-Null
$benchConfigPath=Join-Path $benchBuildPath 'sdkconfig'
$benchConfigText=[IO.File]::ReadAllText((Join-Path $repoRoot 'sdkconfig'))
$benchConfigText=$benchConfigText -replace '(?m)^# CONFIG_EXO_ADAPTIVE_BENCH_ACTIVE is not set\r?$', 'CONFIG_EXO_ADAPTIVE_BENCH_ACTIVE=y'
$benchConfigText=$benchConfigText -replace '(?m)^CONFIG_EXO_ADAPTIVE_BENCH_ACTIVE=.*\r?$', 'CONFIG_EXO_ADAPTIVE_BENCH_ACTIVE=y'
if($benchConfigText -notmatch '(?m)^CONFIG_EXO_ADAPTIVE_BENCH_ACTIVE=y\r?$') {
    $benchConfigText += "`nCONFIG_EXO_ADAPTIVE_BENCH_ACTIVE=y`n"
}
[IO.File]::WriteAllText($benchConfigPath,$benchConfigText,[Text.UTF8Encoding]::new($false))
if($Release -and -not $Nominal15){throw 'Release requires -Nominal15'}
$variantFlags=@{EXO_ADAPTIVE_NOMINAL_15NM=$Nominal15.IsPresent;EXO_ADAPTIVE_ACTIVE_RELEASE=$Release.IsPresent;EXO_BENCH_PEAK_TEST=($Nominal15.IsPresent -and -not $Release.IsPresent)}
foreach($flag in $variantFlags.Keys){
    $benchConfigText=$benchConfigText -replace ('(?m)^(# )?CONFIG_'+$flag+'(=.*| is not set)\r?\n?'),''
    $benchConfigText+="`n"+$(if($variantFlags[$flag]){'CONFIG_'+$flag+'=y'}else{'# CONFIG_'+$flag+' is not set'})+"`n"
}
[IO.File]::WriteAllText($benchConfigPath,$benchConfigText,[Text.UTF8Encoding]::new($false))
# 单独SDKCONFIG/构建目录保留普通固件SHADOW配置；此工具没有任何烧录/串口命令。
Push-Location $repoRoot
try {
    idf.py -B $benchBuildPath "-DSDKCONFIG=$benchConfigPath" reconfigure
    if($LASTEXITCODE -ne 0) { throw 'Candidate reconfigure failed' }
    cmake --build $benchBuildPath -- -j2
    if($LASTEXITCODE -ne 0) { throw "Adaptive bench build failed: $LASTEXITCODE" }
} finally { Pop-Location }
