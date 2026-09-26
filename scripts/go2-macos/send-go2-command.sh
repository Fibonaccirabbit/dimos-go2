#!/bin/zsh
set -euo pipefail

script_dir="${0:A:h}"
install_dir="${script_dir:h:h}"
if (( $# == 0 )); then
  print -u2 '用法: zsh send-go2-command.sh "读取点云状态，不要移动"'
  exit 1
fi
source "${DIMOS_VENV_PATH:-$install_dir/.venv}/bin/activate"
cd "$install_dir"
exec dimos agent-send "$*"
