param([string]$TracePath, [string]$KalmanPath, [int]$KalmanTrials = 500)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$zig = Join-Path $repoRoot ".venv-tests\Scripts\python-zig.exe"
$cache = Join-Path $repoRoot "build\zig-cache"
$outputDir = Join-Path $repoRoot "build\host-tests"
$testSource = Join-Path $repoRoot "flight_controller\tests\test_flight_controller.c"
$testExe = Join-Path $outputDir "flight_controller_tests.exe"

# Each firmware is one C file. Check duplicated wire definitions stay identical.
$wirePattern = '(?s)/\* BEGIN WIRE PROTOCOL v1:.*?/\* END WIRE PROTOCOL v1 \*/'
$flightText = (Get-Content -LiteralPath (Join-Path $repoRoot 'flight_controller\flight_controller.c') -Raw).Replace("`r`n", "`n")
$handlerText = (Get-Content -LiteralPath (Join-Path $repoRoot 'handler_controller\handler_controller.c') -Raw).Replace("`r`n", "`n")
$flightWire = [regex]::Match($flightText, $wirePattern)
$handlerWire = [regex]::Match($handlerText, $wirePattern)
if (-not $flightWire.Success -or -not $handlerWire.Success -or $flightWire.Value -cne $handlerWire.Value) {
    throw 'Wire protocol definitions differ between the two single-file firmware projects.'
}
Write-Output 'Single-file protocol consistency check passed.'

if (-not (Test-Path $zig)) { throw "Test compiler missing. Create .venv-tests and install ziglang first." }
New-Item -ItemType Directory -Force $cache, $outputDir | Out-Null
$env:ZIG_GLOBAL_CACHE_DIR = $cache
& $zig cc $testSource -std=c11 -O2 -Wall -Wextra -Werror -o $testExe -lm
if ($LASTEXITCODE -ne 0) { throw "Host test compilation failed ($LASTEXITCODE)." }
& $testExe
if ($LASTEXITCODE -ne 0) { throw "Flight-controller host tests failed ($LASTEXITCODE)." }
if ($TracePath) {
    $traceParent = Split-Path -Parent $TracePath
    if ($traceParent) { New-Item -ItemType Directory -Force $traceParent | Out-Null }
    & $testExe --trace $TracePath
    if ($LASTEXITCODE -ne 0) { throw "Flight-controller trace generation failed ($LASTEXITCODE)." }
}
if ($KalmanPath) {
    $kalmanParent = Split-Path -Parent $KalmanPath
    if ($kalmanParent) { New-Item -ItemType Directory -Force $kalmanParent | Out-Null }
    & $testExe --kalman-mc $KalmanPath $KalmanTrials
    if ($LASTEXITCODE -ne 0) { throw "Kalman Monte Carlo run failed ($LASTEXITCODE)." }
}
