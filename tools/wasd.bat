@echo off
chcp 65001 >nul
title Gyrobot WASD
python "%~dp0wasd.py" --port COM13
pause
