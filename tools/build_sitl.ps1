$ErrorActionPreference='Stop'
$repoRoot=Split-Path -Parent $PSScriptRoot
$compiler=Join-Path $repoRoot '.venv-tests\Scripts\python-zig.exe'
$out=Join-Path $repoRoot 'build\sitl'
New-Item -ItemType Directory -Force $out | Out-Null
$env:ZIG_GLOBAL_CACHE_DIR=Join-Path $repoRoot 'build\zig-cache'
& $compiler cc (Join-Path $repoRoot 'flight_controller\flight_controller.c') -std=c11 -O2 -shared -DQCOPTER_HOST_TEST -DQCOPTER_SITL_DLL -o (Join-Path $out 'flight_core.dll') -lm
if($LASTEXITCODE -ne 0){throw 'C flight core shared build failed'}
$package=Join-Path $repoRoot 'companion\ros2_ws\src\qcopter_autonomy'
& $compiler c++ (Join-Path $package 'src\navigation.cpp') (Join-Path $package 'src\mission.cpp') -I (Join-Path $package 'include') -std=c++17 -O2 -shared -DQCOPTER_NAV_DLL -o (Join-Path $out 'navigation.dll')
if($LASTEXITCODE -ne 0){throw 'C++ navigation shared build failed'}
Write-Output 'SITL libraries built: production C flight core and common C++ navigation core.'
