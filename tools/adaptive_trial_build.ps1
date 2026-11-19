param([string]$BuildDirectory)
$ErrorActionPreference='Stop'
$repoRoot=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if(-not $BuildDirectory){$BuildDirectory=Join-Path $repoRoot ('build/adaptive_trial_'+(Get-Date -Format 'yyyyMMdd_HHmmss_fff'))}
$trialBuild=[IO.Path]::GetFullPath($BuildDirectory)
$buildRoot=[IO.Path]::GetFullPath((Join-Path $repoRoot 'build'))+[IO.Path]::DirectorySeparatorChar
if(-not $trialBuild.StartsWith($buildRoot,[StringComparison]::OrdinalIgnoreCase)){throw 'Build directory must be inside this repository build directory'}
if(Test-Path -LiteralPath $trialBuild){throw 'Refusing to replace an existing build directory'}
New-Item -ItemType Directory -Path $trialBuild -ErrorAction Stop | Out-Null
$trialConfig=[IO.File]::ReadAllText((Join-Path $repoRoot 'sdkconfig'))
$flags=@{EXO_ADAPTIVE_BOUNDED_TRIAL=$true;EXO_ADAPTIVE_BENCH_ACTIVE=$false;EXO_ADAPTIVE_NOMINAL_15NM=$false;EXO_ADAPTIVE_ACTIVE_RELEASE=$false;EXO_BENCH_PEAK_TEST=$false;EXO_ENGINEERING_GAIT_STIMULUS=$false}
foreach($flag in $flags.Keys){
    $trialConfig=$trialConfig -replace ('(?m)^(# )?CONFIG_'+$flag+'(=.*| is not set)\r?\n?'),''
    $trialConfig+="`n"+$(if($flags[$flag]){'CONFIG_'+$flag+'=y'}else{'# CONFIG_'+$flag+' is not set'})+"`n"
}
$trialConfigPath=Join-Path $trialBuild 'sdkconfig'
[IO.File]::WriteAllText($trialConfigPath,$trialConfig,[Text.UTF8Encoding]::new($false))
. (Join-Path $PSScriptRoot 'activate_esp_idf.ps1')
Push-Location $repoRoot
try {
    idf.py -B $trialBuild "-DSDKCONFIG=$trialConfigPath" reconfigure
    if($LASTEXITCODE -ne 0){throw 'Trial reconfigure failed'}
    cmake --build $trialBuild -- -j2
    if($LASTEXITCODE -ne 0){throw 'Trial build failed'}
    Write-Output "TRIAL_BUILD=$trialBuild"
} finally {Pop-Location}
# This script has no serial, flash, device-configuration or START commands.
