$ErrorActionPreference = "Stop"
$installRoot = "C:\WinLibs"
$sdkPath = Join-Path $installRoot "pico-sdk"
$toolPath = Join-Path $installRoot "pico-toolchain"
$installerPath = Join-Path $installRoot "install_pico_tools.py"
$python = Join-Path $env:LOCALAPPDATA "Programs\Python\Python310\python.exe"

if (-not (Test-Path $python)) {
    throw "Python 3.10 is needed to run the official Pico SDK tools installer."
}
if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    throw "Git is required to clone the Pico SDK and its submodules."
}
if (-not (Test-Path $installRoot)) {
    New-Item -ItemType Directory -Path $installRoot | Out-Null
}
if (-not (Test-Path (Join-Path $sdkPath "external\pico_sdk_import.cmake"))) {
    & git clone --depth 1 --branch 2.3.0 --recurse-submodules `
        https://github.com/raspberrypi/pico-sdk.git $sdkPath
    if ($LASTEXITCODE -ne 0) { throw "Pico SDK clone failed." }
}
if (-not (Test-Path $installerPath)) {
    Invoke-WebRequest `
        -Uri "https://raw.githubusercontent.com/raspberrypi/pico-sdk-tools/main/installer/install_pico_tools.py" `
        -OutFile $installerPath
}

& $python $installerPath 2.3.0 --install-dir $toolPath --platform win32_x64 `
    --no-sdk --no-picorc --no-rc-include `
    --only "arm-toolchain cmake ninja pico-sdk-tools picotool openocd"
if ($LASTEXITCODE -ne 0) { throw "Pico SDK tools installation failed." }

Write-Host "Pico SDK 2.3.0 and build/debug tools are installed under C:\WinLibs."
Write-Host "Build with: .\tools\build_firmware.ps1 -Target flight"
