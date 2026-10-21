param(
    [ValidateSet("flight", "controller")]
    [string]$Target = "flight",
    [string]$Board = "",
    [switch]$Clean
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$sdkRoot = "C:\WinLibs\pico-sdk"
$toolsRoot = "C:\WinLibs\pico-toolchain"
$pythonExe = Join-Path $env:LOCALAPPDATA "Programs\Python\Python310\python.exe"

if (-not (Test-Path (Join-Path $sdkRoot "external\pico_sdk_import.cmake"))) {
    throw "Pico SDK not found at '$sdkRoot'. Install Pico SDK 2.3.0 there first."
}

$cmakeExe = Get-ChildItem -Path $toolsRoot -Filter cmake.exe -File -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
$ninjaExe = Get-ChildItem -Path $toolsRoot -Filter ninja.exe -File -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
$compilerExe = Get-ChildItem -Path $toolsRoot -Filter arm-none-eabi-gcc.exe -File -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
$picotoolConfig = Get-ChildItem -Path (Join-Path $toolsRoot "picotool") -Filter picotoolConfig.cmake -File -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $cmakeExe -or -not $ninjaExe -or -not $compilerExe -or -not $picotoolConfig -or -not (Test-Path $pythonExe)) {
    throw "Pico build tools are incomplete under '$toolsRoot'. Run tools/setup/setup_pico_sdk.ps1."
}

$env:PICO_SDK_PATH = $sdkRoot
$env:PATH = @(
    $cmakeExe.DirectoryName,
    $ninjaExe.DirectoryName,
    $compilerExe.DirectoryName,
    $env:PATH
) -join ';'

if (-not $Board) {
    if ($Target -eq "flight") { $Board = "pico2" }
    else { $Board = "pico" }
}

if ($Target -eq "flight") {
    $platform = "rp2350"
    $buildDir = Join-Path $repoRoot "build\flight-rp2350"
} else {
    $platform = "rp2040"
    $buildDir = Join-Path $repoRoot "build\controller-rp2040"
}
if ($Clean -and (Test-Path $buildDir)) { Remove-Item -LiteralPath $buildDir -Recurse -Force }

& $cmakeExe.FullName -S $repoRoot -B $buildDir -G Ninja `
    "-DQCOPTER_TARGET=$Target" "-DPICO_PLATFORM=$platform" `
    "-DPICO_BOARD=$Board" "-DPICO_SDK_PATH=$sdkRoot" `
    "-Dpicotool_DIR=$($picotoolConfig.DirectoryName)" `
    "-DPython3_EXECUTABLE=$pythonExe"
if ($LASTEXITCODE -ne 0) { throw "CMake configuration failed with exit code $LASTEXITCODE." }

& $cmakeExe.FullName --build $buildDir
if ($LASTEXITCODE -ne 0) { throw "Firmware build failed with exit code $LASTEXITCODE." }

Write-Host "Build complete. No firmware was flashed. Output is in '$buildDir'."
