$ErrorActionPreference='Stop'
$kicadPython=Join-Path $env:LOCALAPPDATA 'Programs/KiCad/10.0/bin/python.exe'
if(-not(Test-Path -LiteralPath $kicadPython)){throw 'KiCad10 Python not found; install KiCad or run check_photo_pinouts.py with its Python.'}
& $kicadPython (Join-Path $PSScriptRoot 'check_photo_pinouts.py')
if($LASTEXITCODE -ne 0){throw 'PCB pin audit failed'}
