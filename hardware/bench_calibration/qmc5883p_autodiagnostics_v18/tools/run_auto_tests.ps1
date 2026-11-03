<#
 IntelQCopter v18 unattended bench diagnosis. Windows PowerShell 5.1 compatible.
 Examples:
 .\tools\run_auto_tests.ps1 -Port COM3 -Suite core -PowerMode warm
 .\tools\run_auto_tests.ps1 -Port COM3 -Suite softreset -PowerMode warm
 .\tools\run_auto_tests.ps1 -Port COM3 -Tests 'A0,F1,B1,F2' -DryRun
#>
[CmdletBinding()]
param(
  [string]$Port = 'COM3',
  [ValidateSet('triage','core','rates','wifi','uart','i2c','failure','softreset','full')][string]$Suite = 'triage',
  [string]$Tests = '',
  [ValidateSet('warm','ykush','command')][string]$PowerMode = 'warm',
  [string]$YkushExe = 'ykush.exe',
  [int]$YkushPort = 1,
  [string]$YkushSerial = '',
  [string]$PowerCommands = '',
  [int]$Repeat = 1,
  [int]$WaitPortSeconds = 0,
  [int]$MinSeconds = 25,
  [int]$MaxSeconds = 120,
  [string]$Pio = '',
  [switch]$ListPorts,
  [switch]$ListTests,
  [switch]$DryRun,
  [switch]$StopOnError
)
$ErrorActionPreference = 'Stop'
$Project = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$Script = Join-Path $PSScriptRoot 'auto_tests.py'
$VenvPython = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\python.exe'
$RunArguments = @($Script, '--port', $Port, '--suite', $Suite, '--power-mode', $PowerMode, '--repeat', "$Repeat", '--connect-timeout', "$WaitPortSeconds", '--min-sec', "$MinSeconds", '--max-sec', "$MaxSeconds")
if ($Tests) { $RunArguments += @('--tests', $Tests) }
if ($YkushExe) { $RunArguments += @('--ykush-exe', $YkushExe) }
if ($YkushPort) { $RunArguments += @('--ykush-port', "$YkushPort") }
if ($YkushSerial) { $RunArguments += @('--ykush-serial', $YkushSerial) }
if ($PowerCommands) { $RunArguments += @('--power-commands', $PowerCommands) }
if ($Pio) { $RunArguments += @('--pio', $Pio) }
if ($ListPorts) { $RunArguments += '--list-ports' }
if ($ListTests) { $RunArguments += '--list-tests' }
if ($DryRun) { $RunArguments += '--dry-run' }
if ($StopOnError) { $RunArguments += '--stop-on-error' }
Push-Location $Project
try {
  if (Test-Path -LiteralPath $VenvPython) {
    & $VenvPython @RunArguments
  } elseif (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3 @RunArguments
  } elseif (Get-Command python -ErrorAction SilentlyContinue) {
    & python @RunArguments
  } else {
    throw 'Python is not installed. Install Python 3 and pyserial.'
  }
  if ($LASTEXITCODE -ne 0) { throw "Automatic test runner returned exit code $LASTEXITCODE" }
} finally {
  Pop-Location
}
