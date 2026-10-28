param(
    [string]$Command = 'help',
    [switch]$Describe,
    [Parameter(ValueFromRemainingArguments = $true)][string[]]$CommandArgs
)
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$project = Get-Content -LiteralPath (Join-Path $projectRoot 'project.json') -Raw | ConvertFrom-Json
if ($Command -eq 'help') {
    Write-Output "$($project.name) [$($project.branch)]"
    $project.commands.PSObject.Properties | ForEach-Object { '{0,-20} {1}' -f $_.Name, $_.Value.description }
    return
}
$entry = $project.commands.PSObject.Properties[$Command]
if (-not $entry) { throw "Unknown command '$Command'. Run .\qcopter.ps1 help." }
$spec = $entry.Value
$localConfig = Join-Path $projectRoot '.local/config.json'
$local = if (Test-Path -LiteralPath $localConfig) { Get-Content -LiteralPath $localConfig -Raw | ConvertFrom-Json } else { $null }
$python = if ($env:QCOPTER_PYTHON) { $env:QCOPTER_PYTHON } elseif ($local -and $local.python) { $local.python } else { $null }
if (-not $python) {
    $systemPython = Get-Command python, python3 -CommandType Application -ErrorAction SilentlyContinue |
        Where-Object { $_.Source -notlike '*WindowsApps*' } | Select-Object -First 1
    if ($systemPython) { $python = $systemPython.Source }
}
if (-not $python -and $env:LOCALAPPDATA) {
    foreach ($version in @('Python315', 'Python314', 'Python310')) {
        $candidate = Join-Path $env:LOCALAPPDATA "Programs/Python/$version/python.exe"
        if (Test-Path -LiteralPath $candidate) { $python = $candidate; break }
    }
}
if (-not $python) { throw 'System Python not found. Install it or set QCOPTER_PYTHON / .local/config.json explicitly.' }
$script = if ($spec.path) { Join-Path $projectRoot $spec.path } else { $null }
if ($script -and -not (Test-Path -LiteralPath $script)) { throw "Command target is missing: $script" }
$prefix = @($spec.args)
if ($Describe) {
    [pscustomobject]@{ command=$Command; runtime=$spec.runtime; path=$script; args=@($prefix)+@($CommandArgs); root=$projectRoot } | ConvertTo-Json -Depth 4
    return
}
$previous = Get-Location
try {
    Set-Location -LiteralPath $projectRoot
    $global:LASTEXITCODE = 0
    switch ($spec.runtime) {
        'powershell' {
            # A native shell invocation binds named arguments from the catalog.
            # String-array splatting directly into a PS script treats them as values.
            $shellPath = (Get-Process -Id $PID).Path
            & $shellPath -NoProfile -File $script @prefix @CommandArgs
        }
        'cmd' { & $script @prefix @CommandArgs }
        'python' { & $python $script @prefix @CommandArgs }
        'node' { & node $script @prefix @CommandArgs }
        'pio' {
            $pio = Join-Path $env:USERPROFILE '.platformio/penv/Scripts/platformio.exe'
            if (-not (Test-Path -LiteralPath $pio)) { $pio = 'pio' }
            & $pio @prefix @CommandArgs
        }
        default { throw "Unsupported runtime: $($spec.runtime)" }
    }
    if ($LASTEXITCODE -ne 0) { throw "Command '$Command' failed with exit code $LASTEXITCODE." }
} finally { Set-Location -LiteralPath $previous.Path }
