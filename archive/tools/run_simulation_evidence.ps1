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
Run-Checked 'C:\WinLibs\pixi\pixi.exe' @('run','--manifest-path','C:/dev/pixi.toml','cmd','/d','/c','tools\ros2_windows.cmd','python','sim/ros2_integration_test.py')
Run-Checked $python @('sim/yolo_reference_test.py')
Run-Checked $python @('sim/yolo_onnx_test.py')
if(-not $SkipVideo){Run-Checked $python @('sim/sitl/visualize.py')}
Run-Checked $python @('tools/write_modelling_results.py')
$tectonic=Join-Path $root 'build/tools/tectonic.exe'
if(Test-Path $tectonic) {
    $env:TECTONIC_CACHE_DIR=Join-Path $root 'build/tectonic-cache'
    Run-Checked $tectonic @('-X','compile','--outdir','docs/modelisation','--outfmt','pdf','--keep-logs','docs/modelisation/main.tex')
} else {
    Write-Warning 'LaTeX sources were regenerated; PDF not rebuilt because build/tools/tectonic.exe is absent.'
}
Run-Checked $python @('tools/validate_simulation_artifacts.py')
Write-Output 'Evidence runs complete using standalone Jazzy; current native garden workflow is tools/run_native_garden.ps1.'
