@echo off
chcp 65001 >nul
title Gyrobot WASD
rem Управление роботом с клавиатуры (Windows). Порт ищется сам; вручную: wasd.bat --port COM13
python "%~dp0tools\wasd.py" %*
if errorlevel 1 pause
