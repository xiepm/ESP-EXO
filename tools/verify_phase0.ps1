[CmdletBinding()]
param(
    [string]$IdfProfilePath = 'C:\Espressif\tools\Microsoft.v5.5.PowerShell_profile.ps1',
    [switch]$SkipClean,
    [switch]$Flash,
    [string]$Port
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path -LiteralPath $IdfProfilePath -PathType Leaf)) {
    throw "ESP-IDF activation script not found: $IdfProfilePath"
}
if ($Flash -and [string]::IsNullOrWhiteSpace($Port)) {
    throw 'Use -Port COMx together with -Flash.'
}

$phase0RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
. $IdfProfilePath
Set-StrictMode -Version Latest

function Invoke-Phase0Idf {
    param([Parameter(Mandatory)][string[]]$Arguments)

    & idf.py @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "idf.py $($Arguments -join ' ') failed with exit code $LASTEXITCODE"
    }
}

Push-Location $phase0RepoRoot
try {
    $phase0IdfVersion = (& idf.py --version | Out-String).Trim()
    if ($LASTEXITCODE -ne 0 -or $phase0IdfVersion -notmatch 'v5\.5') {
        throw "Expected ESP-IDF v5.5, got: $phase0IdfVersion"
    }

    if (-not $SkipClean) {
        Invoke-Phase0Idf -Arguments @('fullclean')
    }
    Invoke-Phase0Idf -Arguments @('build')
    Invoke-Phase0Idf -Arguments @('size')

    $phase0Outputs = @(
        'build\twai_network_master.bin',
        'build\twai_network_master.elf',
        'build\bootloader\bootloader.bin',
        'build\partition_table\partition-table.bin'
    )

    $phase0Evidence = foreach ($phase0RelativePath in $phase0Outputs) {
        $phase0OutputPath = Join-Path $phase0RepoRoot $phase0RelativePath
        if (-not (Test-Path -LiteralPath $phase0OutputPath -PathType Leaf)) {
            throw "Required build output missing: $phase0RelativePath"
        }
        $phase0File = Get-Item -LiteralPath $phase0OutputPath
        [PSCustomObject]@{
            Path   = $phase0RelativePath -replace '\\', '/'
            Bytes  = $phase0File.Length
            SHA256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $phase0OutputPath).Hash
        }
    }

    $phase0App = $phase0Evidence | Where-Object Path -eq 'build/twai_network_master.bin'
    if ($phase0App.Bytes -gt 1MB) {
        throw "Application binary exceeds the 1 MiB baseline partition: $($phase0App.Bytes) bytes"
    }

    [PSCustomObject]@{
        Result   = 'PASS'
        ESP_IDF  = $phase0IdfVersion
        Target   = 'esp32s3'
        GitHead  = (git rev-parse HEAD)
        GitState = (git status --short | Out-String).Trim()
    } | Format-List
    $phase0Evidence | Format-Table -AutoSize

    if ($Flash) {
        Invoke-Phase0Idf -Arguments @('-p', $Port, 'flash')
        Write-Host "Flash completed on $Port. Run 'idf.py -p $Port monitor' and follow docs/baseline/SMOKE_TEST.md."
    }
}
finally {
    Pop-Location
}
