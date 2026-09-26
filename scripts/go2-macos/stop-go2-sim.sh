#!/bin/zsh
set -euo pipefail

script_dir="${0:A:h}"
install_dir="${script_dir:h:h}"
source "${DIMOS_VENV_PATH:-$install_dir/.venv}/bin/activate"
cd "$install_dir"
export PYTHONPATH="$install_dir${PYTHONPATH:+:$PYTHONPATH}"

exec dimos stop
