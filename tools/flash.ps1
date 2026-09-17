param([string]$Port = 'COM9')
$cli = "$env:LOCALAPPDATA\arduino-cli\arduino-cli.exe"
$sketch = "$PSScriptRoot\..\firmware\gyrobot_v5"
& $cli compile -b arduino:avr:nano $sketch
if ($LASTEXITCODE -ne 0) { exit 1 }
& $cli upload -p $Port -b arduino:avr:nano $sketch
if ($LASTEXITCODE -ne 0) { exit 1 }
Write-Host "OK: v5 залита в $Port"
