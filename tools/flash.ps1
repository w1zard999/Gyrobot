# Компиляция и заливка gyrobot_tumbller в Arduino Nano по USB.
# Перед заливкой вынуть HC-05 из разъёма (иначе "not in sync").
param([string]$Port = 'COM10')
$cli = "$env:LOCALAPPDATA\arduino-cli\arduino-cli.exe"
$sketch = "$PSScriptRoot\..\firmware\gyrobot_tumbller"
& $cli compile -b arduino:avr:nano $sketch
if ($LASTEXITCODE -ne 0) { exit 1 }
& $cli upload -p $Port -b arduino:avr:nano $sketch
if ($LASTEXITCODE -ne 0) { Write-Host "Не залилось: HC-05 вынут? Порт $Port верный? (arduino-cli board list)"; exit 1 }
Write-Host "OK: gyrobot_tumbller залит в $Port. Вставь HC-05, положи робота на бок и жди рывка колёс."
