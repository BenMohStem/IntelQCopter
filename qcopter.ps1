param([string]$Command='help',[Parameter(ValueFromRemainingArguments=$true)][string[]]$CommandArgs)
$ErrorActionPreference='Stop'
$root=$PSScriptRoot
$catalog=Get-Content -LiteralPath (Join-Path $root 'project.json') -Raw | ConvertFrom-Json
if($Command -eq 'help'){$catalog.commands.PSObject.Properties | ForEach-Object {'{0,-18} {1}' -f $_.Name,$_.Value.description};return}
$entry=$catalog.commands.PSObject.Properties[$Command]
if(-not $entry){throw 'Unknown command; run .\qcopter.ps1 help'}
$spec=$entry.Value
$python=Join-Path $env:USERPROFILE '.platformio/penv/Scripts/python.exe'
$pio=Join-Path $env:USERPROFILE '.platformio/penv/Scripts/platformio.exe'
$old=Get-Location
try{
 Set-Location -LiteralPath $root
 if($spec.runtime -eq 'python'){& $python (Join-Path $root $spec.path) @($spec.args) @CommandArgs}
 elseif($spec.runtime -eq 'pio'){& $pio @($spec.args) @CommandArgs}
 else{throw 'Unsupported command runtime'}
 if($LASTEXITCODE -ne 0){throw "Command failed: $Command ($LASTEXITCODE)"}
}finally{Set-Location -LiteralPath $old.Path}
