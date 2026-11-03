[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$Command = 'calibration-gui',
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$CommandArgs
)

$ErrorActionPreference = 'Stop'

if ($Command -in @('help', '-h', '--help')) {
    Write-Output 'Calibration GUI:'
    Write-Output '  .\qcopter.ps1 calibration-gui'
    Write-Output '  .\qcopter.ps1 calibration-gui -UploadFirmware'
    Write-Output '  .\qcopter.ps1 calibration-gui -UploadFirmware -Check -Seconds 20'
    return
}
if ($Command -ne 'calibration-gui') {
    throw "Unknown command '$Command'. Run .\qcopter.ps1 help."
}

$launcher = Join-Path $PSScriptRoot 'tools\start_calibration.ps1'
if (-not (Test-Path -LiteralPath $launcher -PathType Leaf)) {
    throw "Calibration launcher is missing: $launcher"
}
$shellPath = (Get-Process -Id $PID).Path
& $shellPath -NoProfile -File $launcher @CommandArgs
if ($LASTEXITCODE -ne 0) {
    throw "Calibration GUI command failed with exit code $LASTEXITCODE."
}
