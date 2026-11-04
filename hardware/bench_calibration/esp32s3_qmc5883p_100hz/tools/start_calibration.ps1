<#
IntelQCopter competition calibration, Windows PowerShell 5.1.
Wi-Fi OFF / QMC5883P 100Hz / UART 921600 / 500Hz IMU target.

  .\tools\start_calibration.ps1
  .\tools\start_calibration.ps1 -UploadFirmware
  .\tools\start_calibration.ps1 -Port COM3 -UploadFirmware
  .\tools\start_calibration.ps1 -Check -Seconds 20
  .\tools\start_calibration.ps1 -Wifi                 # Wi-Fi calibration, no cable needed
  .\tools\start_calibration.ps1 -UploadFirmware -Wifi # Build+upload via COM, then calibrate via Wi-Fi
#>
[CmdletBinding()]
param(
    [string]$Port = '',
    [string]$Output = '',
    [int]$GyroSamples = 30000,
    [int]$AccelSamples = 5000,
    [int]$MagTurnSamples = 5000,
    [int]$MagTumbleSamples = 15000,
    [int]$BmpSamples = 1000,
    [ValidateRange(0,30)][double]$SettleSeconds = 3.0,
    [switch]$SkipBmp,
    [switch]$SkipLidar,
    [string]$LidarPort = '',
    [int]$LidarSamples = 300,
    [switch]$MagOnly,
    [switch]$MagProbe,
    [switch]$Wifi,  # Enable ESP32-S3 Wi-Fi AP calibration (UDP)
    [switch]$InstallDependencies,
    [Alias('Flash')][switch]$UploadFirmware,
    [switch]$Check,
    [ValidateRange(2,300)][int]$Seconds = 20
)
$ErrorActionPreference = 'Stop'
$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
. (Join-Path $PSScriptRoot 'resolve_platformio.ps1')
. (Join-Path $PSScriptRoot 'resolve_serial_port.ps1')
Push-Location $Root
try {
    $PythonExe=$null; $PythonPrefix=@()
    # The Windows py launcher can exist without registering an installed
    # Python (for example, this machine has Python 3.15 but `py -3` fails).
    # Probe each shared interpreter before choosing it so COM detection and
    # the GUI use the same working runtime.
    if (Get-Command py -ErrorAction SilentlyContinue) {
        $probe = & py -3 -c 'import sys; print(sys.executable)' 2>$null
        if ($LASTEXITCODE -eq 0 -and $probe) {
            $PythonExe='py'; $PythonPrefix=@('-3')
        }
    }
    if (-not $PythonExe -and (Get-Command python -ErrorAction SilentlyContinue)) {
        $pythonCommand = Get-Command python -ErrorAction SilentlyContinue
        & $pythonCommand.Source -c 'import sys' 2>$null
        if ($LASTEXITCODE -eq 0) { $PythonExe=$pythonCommand.Source }
    }
    if (-not $PythonExe) { throw 'Python 3 not found. Install Python 3 with pyserial.' }

    if ($InstallDependencies) {
        & $PythonExe @PythonPrefix '-m' 'pip' 'install' '-r' (Join-Path $Root 'requirements.txt')
        if ($LASTEXITCODE -ne 0) { throw 'Python requirements installation failed.' }
    }
    if ($LidarPort -and $Port -and $LidarPort -ieq $Port) {
        throw 'The LiDAR and ESP32 cannot use the same serial port.'
    }

    # Wi-Fi mode: ESP32-S3 acts as Access Point, PC connects via UDP.
    # No COM port needed for calibration - only for firmware upload.
    # Build first (cable connected), then remove cable and calibrate via Wi-Fi.
    if ($Wifi -and -not $UploadFirmware) {
        $selectedPort = ''
        Write-Host 'Wi-Fi calibration mode (no COM port needed):' -ForegroundColor Cyan
        Write-Host '  1. Build+upload firmware with: .\tools\start_calibration.ps1 -UploadFirmware -Wifi' -ForegroundColor Cyan
        Write-Host '  2. Power cycle the ESP32-S3 board' -ForegroundColor Cyan
        Write-Host '  3. Disconnect USB cable' -ForegroundColor Cyan
        Write-Host '  4. Connect PC to Wi-Fi AP: IntelQCopter-Cal (password: IntelQCopter2026)' -ForegroundColor Cyan
        Write-Host '  5. Run: .\tools\start_calibration.ps1 -Wifi' -ForegroundColor Cyan
    } else {
        $selectedPort=Select-IqSerialPort -Port $Port -PythonExe $PythonExe -PythonPrefix $PythonPrefix -Root $Root -WaitSeconds 5
    }

    if ($Wifi) {
        Write-Host 'Wi-Fi mode ENABLED: ESP32-S3 AP calibration (UDP).' -ForegroundColor Cyan
        Write-Host '  AP: IntelQCopter-Cal | Password: IntelQCopter2026 | UDP port: 19005' -ForegroundColor Cyan
    } else {
        Write-Host ('Selected UART: {0} at 921600 baud; Wi-Fi OFF' -f $selectedPort) -ForegroundColor Cyan
    }

    if ($UploadFirmware) {
        $configText=[System.IO.File]::ReadAllText((Join-Path $Root 'platformio.ini'))
        # Inspect only the full-calibration section. The MAG-only common env is
        # intentionally text-based and appears before the probe-specific env.
        $fullSection = [regex]::Match($configText,'(?ms)^\[env:esp32-s3-calibration-500hz\]\s*(.*?)(?=^\[|\z)')
        if (-not $fullSection.Success) { throw 'Full calibration environment is missing from platformio.ini.' }
        $fullConfig = $fullSection.Groups[1].Value
        if ($fullConfig -match '(?m)^\s*-DIQ_MAG_TEXT_DIAGNOSTIC=1\s*$' -or
            $fullConfig -match '(?m)^\s*-DIQ_WIFI_ENABLED=1\s*$') {
            throw 'This project must have IQ_MAG_TEXT_DIAGNOSTIC=0 and IQ_WIFI_ENABLED=0.'
        }
        $PioExe = Find-IntelQCopterPlatformIO
        Write-Host 'Compiling and uploading full sensor calibration firmware...' -ForegroundColor Cyan
        $envName = if ($Wifi) { 'esp32-s3-calibration-500hz-wifi' } else { 'esp32-s3-calibration-500hz' }
        & $PioExe 'run' '-e' $envName '-t' 'upload' '--upload-port' $selectedPort
        if ($LASTEXITCODE -ne 0) { throw 'Calibration firmware upload failed.' }
        # Windows can change the COM number after an ESP32 reset. Detect again.
        $selectedPort = Select-IqSerialPort -Port $selectedPort -PythonExe $PythonExe -PythonPrefix $PythonPrefix -Root $Root -WaitSeconds 40 -AllowChangedPort
    } else {
        Write-Host 'Using installed firmware. No compilation or upload.' -ForegroundColor Cyan
    }

    if ($Check) {
        Write-Host 'Checking raw binary UART sensor stream (no GUI)...' -ForegroundColor Cyan
        if ($Wifi) {
            throw '-Check is not supported in Wi-Fi mode. Use USB serial for preflight checks.'
        }
        & $PythonExe @PythonPrefix (Join-Path $Root 'tools\uart_preflight.py') '--port' $selectedPort '--seconds' "$Seconds"
        if ($LASTEXITCODE -ne 0) { throw 'UART preflight failed. Check output and sensor wiring before calibration.' }
        return
    }

    Write-Host 'Starting full calibration GUI with automatic stages:' -ForegroundColor Cyan
    Write-Host "  Gyro stationary: $GyroSamples samples"
    Write-Host "  Accel 6 faces: $AccelSamples samples per face"
    Write-Host "  Mag yaw/pitch/roll: $MagTurnSamples samples per rotation"
    Write-Host "  Mag 3D tumble: $MagTumbleSamples samples"
    Write-Host "  Barometer: $BmpSamples samples"
    Write-Host "  Preparation: $SettleSeconds seconds per stage"
    Write-Host '  Motor output DISABLED; remove propellers.' -ForegroundColor Yellow
    if ($Wifi) {
        $a=@((Join-Path $Root 'tools\launch_calibration_gui.py'),
            '--wifi','--gyro-samples',"$GyroSamples",
            '--accel-samples',"$AccelSamples",'--mag-turn-samples',"$MagTurnSamples",
            '--mag-tumble-samples',"$MagTumbleSamples",'--bmp-samples',"$BmpSamples",
            '--lidar-samples',"$LidarSamples",'--settle-seconds',"$SettleSeconds")
    } else {
        $a=@((Join-Path $Root 'tools\launch_calibration_gui.py'),
            '--port',$selectedPort,'--gyro-samples',"$GyroSamples",
            '--accel-samples',"$AccelSamples",'--mag-turn-samples',"$MagTurnSamples",
            '--mag-tumble-samples',"$MagTumbleSamples",'--bmp-samples',"$BmpSamples",
            '--lidar-samples',"$LidarSamples",'--settle-seconds',"$SettleSeconds")
    }
    if ($Output) { $a+=@('--output',$Output) }
    if ($SkipBmp) { $a+=@('--skip-bmp') }
    # LiDAR stages remain available in the GUI even with no LiDAR COM port.
    # Users can connect a separate UART adapter or skip all LiDAR in the GUI.
    if ($SkipLidar) { $a+='--skip-lidar' }
    if ($LidarPort) { $a+=@('--lidar-port',$LidarPort) }
    if ($MagOnly) { $a+='--mag-only' }
    & $PythonExe @PythonPrefix @a
    if ($LASTEXITCODE -ne 0) { throw "Calibration application exited with code $LASTEXITCODE." }
} finally {
    Pop-Location
}
