#!/usr/bin/env sh
# Управление роботом с клавиатуры (Linux / macOS). Порт ищется сам; вручную: ./wasd.sh --port /dev/rfcomm0
cd "$(dirname "$0")" || exit 1
PY=$(command -v python3 || command -v python) || { echo "Нужен Python 3"; exit 1; }
exec "$PY" tools/wasd.py "$@"
