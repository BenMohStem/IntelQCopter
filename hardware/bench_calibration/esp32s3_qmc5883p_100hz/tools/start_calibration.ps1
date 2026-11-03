<#
IntelQCopter competition calibration, Windows PowerShell 5.1.
Wi-Fi OFF / QMC5883P 100Hz / UART 921600 / 500Hz IMU target.

  .\tools\start_calibration.ps1
  .\tools\start_calibration.ps1 -UploadFirmware
  .\tools\start_calibration.ps1 -Port COM3 -UploadFirmware
  .\tools\start_calibration.ps1 -Check -Seconds 20
  .\tools\start_calibration.ps1 -MagProbe -Seconds 20
#>
[CmdletBinding()]
param(
    [string]$Port = '',
    [string]$Output = '',
    [int]$GyroSamples = 30000,
    [int]$AccelSamples = 5000,
    [int]$MagTurnSamples = 2000,
    [int]$MagTumbleSamples = 8000,
    [int]$BmpSamples = 1000,
    [ValidateRange(0,30)][double]$SettleSeconds = 3.0,
    [switch]$SkipBmp,
    [switch]$SkipLidar,
    [string]$LidarPort = '',
    [int]$LidarSamples = 300,
    [switch]$MagOnly,
    [switch]$MagProbe,
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
    # If no COM port was specified, use the detected CH343. Do not pass empty
    # '--upload-port' values to PlatformIO (the old launcher error).
    $selectedPort=Select-IqSerialPort -Port $Port -PythonExe $PythonExe -PythonPrefix $PythonPrefix -Root $Root -WaitSeconds 5
    if ($MagProbe) {
        # This is the EXACT v13 MAG-only source in its own PlatformIO env.
        # It is deliberately text at 115200 baud rather than binary 921600.
        $PioExe = Find-IntelQCopterPlatformIO
        Write-Host 'MAG-ONLY 100Hz probe (v13 reference). Uploading diagnostic firmware...' -ForegroundColor Cyan
        & $PioExe 'run' '-e' 'esp32-s3-mag-probe' '-t' 'upload' '--upload-port' $selectedPort
        if ($LASTEXITCODE -ne 0) { throw 'MAG probe firmware upload failed.' }
        Write-Host 'IMPORTANT: Unplug USB and EVERY other board power source for 10 seconds.' -ForegroundColor Yellow
        Write-Host 'Reconnect after 10 seconds, then press Enter.' -ForegroundColor Yellow
        [void](Read-Host 'Power cycle complete')
        $selectedPort = Select-IqSerialPort -Port $selectedPort -PythonExe $PythonExe -PythonPrefix $PythonPrefix -Root $Root -WaitSeconds 40 -AllowChangedPort
        & $PythonExe @PythonPrefix (Join-Path $Root 'tools\mag_probe_capture.py') '--port' $selectedPort '--seconds' "$Seconds"
        if ($LASTEXITCODE -ne 0) { throw 'MAG probe recorded no valid fresh measurements. See saved text log.' }
        Write-Host 'Probe finished. Reflash full binary calibration next: .\tools\start_calibration.ps1 -UploadFirmware -Check' -ForegroundColor Yellow
        return
    }

    Write-Host ('Selected UART: {0} at 921600 baud; Wi-Fi OFF' -f $selectedPort) -ForegroundColor Cyan

    if ($UploadFirmware) {
        $configText=[System.IO.File]::ReadAllText((Join-Path $Root 'platformio.ini'))
        # Check ONLY the full-calibration env, not the separate MAG probe env.
        $fullConfig = ($configText -split '(?m)^\[env:esp32-s3-mag-probe\]')[0]
        if ($fullConfig -match '(?m)^\s*-DIQ_MAG_TEXT_DIAGNOSTIC=1\s*$' -or
            $fullConfig -match '(?m)^\s*-DIQ_WIFI_ENABLED=1\s*$') {
            throw 'This project must have IQ_MAG_TEXT_DIAGNOSTIC=0 and IQ_WIFI_ENABLED=0.'
        }
        $PioExe = Find-IntelQCopterPlatformIO
        Write-Host 'Compiling and uploading full sensor calibration firmware...' -ForegroundColor Cyan
        & $PioExe 'run' '-e' 'esp32-s3-calibration-500hz' '-t' 'upload' '--upload-port' $selectedPort
        if ($LASTEXITCODE -ne 0) { throw 'Calibration firmware upload failed.' }
        # Windows can change the COM number after an ESP32 reset. Detect again.
        $selectedPort = Select-IqSerialPort -Port $selectedPort -PythonExe $PythonExe -PythonPrefix $PythonPrefix -Root $Root -WaitSeconds 40 -AllowChangedPort
    } else {
        Write-Host 'Using installed firmware. No compilation or upload.' -ForegroundColor Cyan
    }

    if ($Check) {
        Write-Host 'Checking raw binary UART sensor stream (no GUI)...' -ForegroundColor Cyan
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
    $a=@((Join-Path $Root 'tools\live_calibration_gui.py'),
        '--port',$selectedPort,'--gyro-samples',"$GyroSamples",
        '--accel-samples',"$AccelSamples",'--mag-turn-samples',"$MagTurnSamples",
        '--mag-tumble-samples',"$MagTumbleSamples",'--bmp-samples',"$BmpSamples",
        '--lidar-samples',"$LidarSamples",'--settle-seconds',"$SettleSeconds")
    if ($Output) { $a+=@('--output',$Output) }
    if ($SkipBmp) { $a+='--skip-bmp' }
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
