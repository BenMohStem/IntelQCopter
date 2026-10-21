$ErrorActionPreference='Stop'
$root=Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$config=Get-Content -LiteralPath (Join-Path $root '.local/config.json') -Raw | ConvertFrom-Json
try {
 $response=Invoke-RestMethod 'http://127.0.0.1:8088/settings' -TimeoutSec 2
 if(-not $response.fields){throw 'Port 8088 belongs to another service'}
} catch {
 $localDir=Join-Path $root '.local'
 New-Item -ItemType Directory -Force -Path $localDir | Out-Null
 $script=Join-Path $root 'simulation/gazebo/station_manager.py'
 Start-Process -FilePath $config.python -ArgumentList @('-u',('"'+$script+'"')) -WorkingDirectory $root -WindowStyle Hidden -RedirectStandardOutput (Join-Path $localDir 'station-manager.log') -RedirectStandardError (Join-Path $localDir 'station-manager-error.log') | Out-Null
}
Write-Output 'Open http://127.0.0.1:8088 for launch, next-start settings, recordings and the guide.'
