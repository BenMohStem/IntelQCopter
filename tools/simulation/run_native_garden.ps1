param([ValidateSet('Gazebo')][string]$Engine='Gazebo',
      [ValidatePattern('^[a-zA-Z0-9_-]+$')][string]$RunPrefix=(Get-Date -Format 'yyyyMMdd-HHmmss'),
      [switch]$WithoutRos)
$ErrorActionPreference='Stop'
$taskRoot=Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
Set-Location $taskRoot
$taskLogs=Join-Path $taskRoot 'artifacts/test-runs/native-local'
New-Item -ItemType Directory -Force $taskLogs | Out-Null
# Historical environment name: Torch/OpenCV/YOLO Python, not the Isaac application.
$taskPy=Join-Path $taskRoot '.venv-isaacsim/Scripts/python.exe'
$taskPixi='C:/WinLibs/pixi/pixi.exe'
$taskName="$RunPrefix-gazebo"
$taskEngineLog=Join-Path $taskLogs "$taskName-engine.log"
$taskRos=$null;$taskProcess=$null
function Wait-TaskLog($process,$path,$pattern,$seconds){
 $deadline=(Get-Date).AddSeconds($seconds)
 while(-not ((Get-Content $path -Raw -ErrorAction SilentlyContinue) -match $pattern)){
  if($process.HasExited -or (Get-Date) -gt $deadline){throw "Process did not become ready: $path"}
  Start-Sleep -Milliseconds 500
 }
}
try {
 & $taskPy sim/gazebo/build_world.py
 if($LASTEXITCODE -ne 0){throw 'SDF authoring failed'}
 $taskProcess=Start-Process $taskPixi -ArgumentList @('run','--manifest-path','sim/gazebo/pixi.toml','python','-u','sim/gazebo/engine_server.py') -WorkingDirectory $taskRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput $taskEngineLog -RedirectStandardError (Join-Path $taskLogs "$taskName-engine.stderr.log")
 Wait-TaskLog $taskProcess $taskEngineLog 'live engine ready' 180
 if(-not $WithoutRos){
  $taskRosLog=Join-Path $taskLogs "$taskName-ros.log"
  $taskRos=Start-Process $taskPixi -ArgumentList @('run','--manifest-path','C:/dev/pixi.toml','cmd','/d','/c','tools\ros2\ros2_windows.cmd','python','-u','sim/engines/ros_bridge.py','--engine','gazebo') -WorkingDirectory $taskRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput $taskRosLog -RedirectStandardError (Join-Path $taskLogs "$taskName-ros.stderr.log")
  $null=$taskRos.Handle
  Wait-TaskLog $taskRos $taskRosLog 'listening at' 45
 }
 $taskClientArgs=@('sim/gazebo/run_mission.py','--name',$taskName,'--video')
 if(-not $WithoutRos){$taskClientArgs+='--ros'}
 $taskClient=Start-Process $taskPy -ArgumentList $taskClientArgs -WorkingDirectory $taskRoot -WindowStyle Hidden -Wait -PassThru -RedirectStandardOutput (Join-Path $taskLogs "$taskName-client.log") -RedirectStandardError (Join-Path $taskLogs "$taskName-client.stderr.log")
 Get-Content (Join-Path $taskLogs "$taskName-client.log")
 if($taskClient.ExitCode -ne 0){throw "Mission failed. Original logs retained: $taskName"}
 if(-not $WithoutRos){
  if(-not $taskRos.WaitForExit(10000)){throw 'DDS bridge did not finish writing evidence'}
  if($taskRos.ExitCode -ne 0){throw 'DDS bridge failed'}
  Copy-Item 'artifacts/simulation/2026-10-20-engines/gazebo-ros2-transport.json' "artifacts/simulation/2026-10-20-engines/$taskName-transport.json"
 }
 & $taskPy tools/reports/write_native_engine_report.py $taskName --label 'Gazebo native physics / RGB / standalone Jazzy DDS'
 if($LASTEXITCODE -ne 0){throw 'Evidence generation failed'}
} finally {
 foreach($p in @($taskRos,$taskProcess)){
  if($p -and -not $p.HasExited){Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue}
 }
}
