param([switch]$SkipVideo)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
Set-Location $root
$python=Join-Path $root '.venv-isaacsim\Scripts\python.exe'
function Run-Checked([string]$exe,[string[]]$arguments) {
    & $exe @arguments
    if($LASTEXITCODE -ne 0){throw "Step failed: $exe $arguments"}
}
& "$PSScriptRoot\build_sitl.ps1"
& "$PSScriptRoot\run_flight_controller_tests.ps1" -KalmanPath 'artifacts/simulation/2026-10-20/angle-mc.csv'
Run-Checked "$root\build\host-tests\flight_controller_tests.exe" @('--altitude-mc','artifacts/simulation/2026-10-20/altitude-mc.csv')
Run-Checked $python @('sim/sitl/run.py')
Run-Checked $python @('sim/sitl/analysis_tests.py')
Run-Checked 'C:\IsaacSim\app\kit\python\python.exe' @('sim/ros2_integration_test.py')
Run-Checked $python @('sim/yolo_reference_test.py')
Run-Checked $python @('sim/yolo_onnx_test.py')
Run-Checked $python @('sim/isaac/build_stage.py')
if(-not $SkipVideo){Run-Checked $python @('sim/sitl/visualize.py')}
Run-Checked $python @('tools/write_modelling_results.py')
Write-Output 'Evidence runs complete. Isaac physics was not invoked; see its recorded startup failure.'
