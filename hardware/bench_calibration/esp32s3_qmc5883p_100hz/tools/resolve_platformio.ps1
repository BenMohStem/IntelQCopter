# Resolve the PlatformIO executable used by VS Code without requiring a PATH edit.
# Dot-source from the two launchers after determining $Root.
function Find-IntelQCopterPlatformIO {
    $candidates = @()
    if ($env:PLATFORMIO_EXE) { $candidates += $env:PLATFORMIO_EXE }
    $candidates += (Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe')
    foreach ($candidate in $candidates) {
        if ($candidate -and (Test-Path -LiteralPath $candidate -PathType Leaf)) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    foreach ($name in @('pio', 'platformio')) {
        $cmd = Get-Command $name -ErrorAction SilentlyContinue
        if ($cmd -and $cmd.Source) { return $cmd.Source }
    }
    throw ('PlatformIO CLI was not found. Expected: ' +
           (Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe') +
           '. In VS Code PlatformIO, open its terminal or reinstall the extension. ' +
           'To override, set $env:PLATFORMIO_EXE to the full platformio.exe path.')
}
