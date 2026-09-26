#!/bin/zsh
set -euo pipefail

script_dir="${0:A:h}"
install_dir="${script_dir:h:h}"
source "${DIMOS_VENV_PATH:-$install_dir/.venv}/bin/activate"
cd "$install_dir"

export PYTHONPATH="$install_dir${PYTHONPATH:+:$PYTHONPATH}"
export DIMOS_SKIP_SYSTEM_CONFIG=1

blueprints=("$@")
if (( ${#blueprints[@]} == 0 )); then
  blueprints=(unitree-go2)
fi

exec dimos \
  --transport lcm \
  --rerun-open web \
  --simulation dimsim \
  --dimsim-scene empty \
  run --daemon "${blueprints[@]}"
