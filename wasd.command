#!/usr/bin/env sh
# macOS: двойной клик в Finder открывает Терминал и запускает клиент
cd "$(dirname "$0")" || exit 1
./wasd.sh "$@"
status=$?
if [ $status -ne 0 ]; then printf "Нажми Enter, чтобы закрыть окно"; read -r _; fi
exit $status
