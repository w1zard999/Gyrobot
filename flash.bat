@echo off
chcp 65001 >nul
title Gyrobot flash
rem Прошивка по USB. ПЕРЕД ЗАЛИВКОЙ ВЫНУТЬ HC-05 — его TX на D0 глушит загрузчик.
rem Если порт другой: flash.bat COM5
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\flash.ps1" %*
pause
