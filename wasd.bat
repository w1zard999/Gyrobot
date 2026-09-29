@echo off
chcp 65001 >nul
title Gyrobot WASD
rem Управление роботом с клавиатуры по Bluetooth (HC-05, исходящий порт COM13).
rem Если порт другой: wasd.bat COM7
set PORT=%1
if "%PORT%"=="" set PORT=COM13
python "%~dp0tools\wasd.py" --port %PORT%
pause
