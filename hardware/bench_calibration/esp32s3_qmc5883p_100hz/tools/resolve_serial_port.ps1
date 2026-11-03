# ASCII / Windows PowerShell 5.1 compatible. Requires pyserial.
function Get-IqSerialPorts {
    param([string]$PythonExe,[string[]]$PythonPrefix,[string]$Root)
    $portLister = Join-Path $Root 'tools\list_serial_ports.py'
    $raw = & $PythonExe @PythonPrefix $portLister
    if ($LASTEXITCODE -ne 0) { throw 'Cannot enumerate COM ports. Check Python and pyserial.' }
    $obj = ($raw | Out-String) | ConvertFrom-Json
    return @($obj.ports | Where-Object { $_ -ne $null })
}

function Select-IqSerialPort {
    param(
        [string]$Port,
        [string]$PythonExe,
        [string[]]$PythonPrefix,
        [string]$Root,
        [int]$WaitSeconds = 0,
        [switch]$AllowChangedPort
    )
    $stopAt = (Get-Date).AddSeconds($WaitSeconds)
    $lastReport = Get-Date '2000-01-01'
    while ($true) {
        $ports = @(Get-IqSerialPorts -PythonExe $PythonExe -PythonPrefix $PythonPrefix -Root $Root)
        if ($Port) {
            $exact = @($ports | Where-Object { $_.device -ieq $Port })
            if ($exact.Count -eq 1) { return [string]$exact[0].device }
        }
        $targets = @($ports | Where-Object { $_.is_target })
        if ((-not $Port -or $AllowChangedPort) -and $targets.Count -eq 1) {
            Write-Host ("Detected ESP32 CH343 automatically: {0} ({1})" -f $targets[0].device,$targets[0].description) -ForegroundColor Cyan
            return [string]$targets[0].device
        }
        if (-not $Port -and $ports.Count -eq 1) {
            Write-Host ("Only serial port found: {0} ({1})" -f $ports[0].device,$ports[0].description) -ForegroundColor Cyan
            return [string]$ports[0].device
        }
        if ((Get-Date) -lt $stopAt) {
            if (((Get-Date)-$lastReport).TotalSeconds -ge 5) {
                Write-Host 'Waiting for the ESP32 serial port to appear...' -ForegroundColor Yellow
                $lastReport=Get-Date
            }
            Start-Sleep -Seconds 1
            continue
        }
        if ($ports.Count -eq 0) {
            throw 'No COM ports detected. Reconnect USB / check CH343 driver and Device Manager.'
        }
        if ($Port -and -not $AllowChangedPort) {
            throw ("Requested port {0} not detected. Available: {1}" -f $Port,($ports.device -join ', '))
        }
        if ($Port -and $AllowChangedPort) {
            throw ("Port {0} did not return after upload; available: {1}" -f $Port,($ports.device -join ', '))
        }
        Write-Host 'Select the ESP32 USB-UART COM port:' -ForegroundColor Yellow
        for ($i=0; $i -lt $ports.Count; $i++) {
            Write-Host ('  [{0}] {1}  {2}' -f ($i+1),$ports[$i].device,$ports[$i].description)
        }
        $answer=Read-Host 'Enter a number'
        $selected=0
        if (-not [int]::TryParse($answer,[ref]$selected) -or $selected -lt 1 -or $selected -gt $ports.Count) {
            throw 'Invalid COM selection.'
        }
        return [string]$ports[$selected-1].device
    }
}
