#!/bin/zsh
set -euo pipefail

script_dir="${0:A:h}"
install_dir="${script_dir:h:h}"
exec zsh "$script_dir/run-go2-capabilities.sh" cockpit
