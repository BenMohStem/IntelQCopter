param([ValidateSet('dx12','vulkan')][string]$Backend='dx12',[int]$TimeoutSeconds=600,[string]$RunName='dx12-render')
$ErrorActionPreference='Stop'
$taskRepo=Split-Path -Parent $PSScriptRoot
$taskOut=Join-Path $taskRepo "artifacts/test-runs/2026-10-20-isaac-retry/$RunName"
New-Item -ItemType Directory -Force $taskOut | Out-Null
$taskIsaac='C:\IsaacSim\app'
$taskArgs=@("`"$taskIsaac\apps\isaacsim.exp.base.kit`"",'--no-window',
 '--/app/settings/persistent=false','--/renderer/multiGpu/enabled=false',
 '--/renderer/activeGpu=0','--/app/renderer/resolution/width=320','--/app/renderer/resolution/height=240',
 "--/log/file=`"$taskOut\$Backend-kit.log`"",'--exec',"`"$taskRepo\sim\isaac\retry_probe.py`"")
if($Backend -eq 'dx12'){$taskArgs+='--/app/vulkan=false';$taskArgs+='--d3d12'}
else{$taskArgs+='--/app/vulkan=true';$taskArgs+='--vulkan'}
$taskArgs | Set-Content (Join-Path $taskOut "$Backend-arguments.txt")
$env:OMNI_KIT_ACCEPT_EULA='YES' # Accepted by the user earlier in this chat.
$env:QCOPTER_ISAAC_PROBE_DIR=$taskOut
$taskProcess=Start-Process -FilePath "$taskIsaac\kit\kit.exe" -ArgumentList $taskArgs -WorkingDirectory $taskIsaac -WindowStyle Hidden -PassThru -RedirectStandardOutput "$taskOut\$Backend-stdout.log" -RedirectStandardError "$taskOut\$Backend-stderr.log"
$taskProcess.Id | Set-Content (Join-Path $taskOut "$Backend-pid.txt")
if(-not $taskProcess.WaitForExit($TimeoutSeconds*1000)){
 Stop-Process -Id $taskProcess.Id -Force
 "Timeout after $TimeoutSeconds seconds; only this probe process was stopped." | Set-Content (Join-Path $taskOut "$Backend-outcome.txt")
}else{"Exited: $($taskProcess.ExitCode)" | Set-Content (Join-Path $taskOut "$Backend-outcome.txt")}
Get-Content (Join-Path $taskOut "$Backend-outcome.txt")
