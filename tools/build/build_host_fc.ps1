param([string]$OutputPath,[string]$SourcePath)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$configPath=Join-Path $root '.local/config.json'
$config=if(Test-Path -LiteralPath $configPath){Get-Content -LiteralPath $configPath -Raw | ConvertFrom-Json}else{$null}
$zigCommand=Get-Command zig -ErrorAction SilentlyContinue
$zig=if($config -and $config.zig){$config.zig}elseif($zigCommand){$zigCommand.Source}else{'C:/WinLibs/zig/zig.exe'}
if(-not $SourcePath){$SourcePath=Join-Path $root 'firmware/src/flight_controller.cpp'}
if(-not $OutputPath){$OutputPath=Join-Path $root 'build/sitl/esp32_flight_core.dll'}
New-Item -ItemType Directory -Force (Split-Path -Parent $OutputPath) | Out-Null
$env:ZIG_GLOBAL_CACHE_DIR=Join-Path $root 'build/zig-cache'
$language=if([IO.Path]::GetExtension($SourcePath) -eq '.c'){'c11'}else{'gnu++17'}
& $zig cc $SourcePath -I (Join-Path $root 'firmware/include') "-std=$language" -O2 -Wall -Wextra -Werror -Wno-missing-field-initializers -DFC_CORE_ONLY -DFC_BUILD_DLL -shared -o $OutputPath -lm
if($LASTEXITCODE -ne 0){throw 'ESP32 C host build failed'}
Write-Output 'Built the active ESP32 C core for the Gazebo host process; no hardware flashed.'
