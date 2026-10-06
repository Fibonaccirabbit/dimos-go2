#!/bin/zsh
# UR7e cockpit: workspace camera + Agent chat in the browser.
# Usage: run-ur7e-cockpit.sh [--allow-motion] [--allow-vision]
#   --allow-motion  enable hover / return-to-start moves (onsite supervision required)
#   --allow-vision  let the Agent send workspace camera frames to DeepSeek (locate/observe)
#
# Lab settings come from the environment or ${UR7E_ENV_FILE:-~/.config/dimos/ur7e.env}:
#   UR7E_SSH_TARGET          user@host of the Ubuntu ROS workstation
#   UR7E_REMOTE_ROOT         bridge directory on the workstation (docs/ur7e/bringup.md)
#   UR7E_CONTROL_PATH        authenticated SSH ControlMaster socket
#   UR7E_CUROBO_ROOT         cuRobo checkout (with .venv) on the workstation
#   UR7E_CAMERA_CALIBRATION  local JSON with candidates[0].base_link_from_color_optical
#   DIMOS_VENV_PATH          dimOS virtualenv (default: <repo>/.venv)
set -euo pipefail

script_dir="${0:A:h}"
install_dir="${script_dir:h:h}"
env_file="${UR7E_ENV_FILE:-$HOME/.config/dimos/ur7e.env}"
if [[ -f "$env_file" ]]; then
  set -a
  source "$env_file"
  set +a
fi
for name in UR7E_SSH_TARGET UR7E_REMOTE_ROOT UR7E_CONTROL_PATH UR7E_CUROBO_ROOT UR7E_CAMERA_CALIBRATION; do
  if [[ -z "${(P)name:-}" ]]; then
    print -u2 "Missing $name (set it or add it to $env_file)."
    exit 1
  fi
done

source "${DIMOS_VENV_PATH:-$install_dir/.venv}/bin/activate"
cd "$install_dir"
export PYTHONPATH="$install_dir${PYTHONPATH:+:$PYTHONPATH}"
export DIMOS_SKIP_SYSTEM_CONFIG=1
export NO_PROXY="127.0.0.1,localhost,api.deepseek.com${NO_PROXY:+,$NO_PROXY}"
export DEEPSEEK_API_KEY="$(security find-generic-password -s dimos-deepseek -w 2>/dev/null)"
if [[ -z "$DEEPSEEK_API_KEY" ]]; then
  print -u2 "Missing macOS Keychain service 'dimos-deepseek'."
  exit 1
fi

if ! ssh -S "$UR7E_CONTROL_PATH" -O check "$UR7E_SSH_TARGET" 2>/dev/null; then
  print -u2 "No SSH master at $UR7E_CONTROL_PATH. Open one first (password typed in the terminal only):"
  print -u2 "  ssh -M -S $UR7E_CONTROL_PATH -o ControlPersist=2h -fN $UR7E_SSH_TARGET"
  exit 1
fi

allow_motion=false
allow_vision=false
agent_prompt="$(python -c 'from dimos.robot.universal_robots.system_prompt import UR7E_SYSTEM_PROMPT as p; print(p)')"
for arg in "$@"; do
  case "$arg" in
    --allow-motion) allow_motion=true ;;
    --allow-vision)
      allow_vision=true
      agent_prompt+=' The operator has authorized sharing workspace camera images with the model: use observe for visual questions and describe only what the latest image shows.'
      ;;
    *) print -u2 'Usage: run-ur7e-cockpit.sh [--allow-motion] [--allow-vision]'; exit 2 ;;
  esac
done

# The fixed white-box move stays available to scripts but is hidden from the Agent,
# so it has to locate targets itself. Without vision it cannot locate anything.
typeset -a extra_args
excluded_tools='["agent_send","ur7e_hover_above_white_box"]'
if [[ "$allow_vision" == false ]]; then
  extra_args+=(--disable observe-skill)
  excluded_tools='["agent_send","ur7e_hover_above_white_box","ur7e_locate_object"]'
fi

exec dimos --transport zenoh --viewer none --robot-id ur7e-workstation \
  run --daemon ur7e-ros-agentic-cockpit "${extra_args[@]}" \
  --open-browser=false \
  --ur7eskillcontainer.address="$UR7E_SSH_TARGET" \
  --ur7eskillcontainer.remote-root="$UR7E_REMOTE_ROOT" \
  --ur7eskillcontainer.control-path="$UR7E_CONTROL_PATH" \
  --ur7eskillcontainer.curobo-root="$UR7E_CUROBO_ROOT" \
  --ur7eskillcontainer.allow-motion="$allow_motion" \
  --ur7ecameramodule.address="$UR7E_SSH_TARGET" \
  --ur7ecameramodule.remote-root="$UR7E_REMOTE_ROOT" \
  --ur7ecameramodule.control-path="$UR7E_CONTROL_PATH" \
  --ur7ecameramodule.camera-calibration="$UR7E_CAMERA_CALIBRATION" \
  --mcpclient.model=deepseek-flash \
  --mcpclient.model-base-url=https://api.deepseek.com \
  --mcpclient.model-api-key-env=DEEPSEEK_API_KEY \
  --mcpclient.model-use-responses-api=true \
  --mcpclient.excluded-tools="$excluded_tools" \
  --mcpclient.trace-dir="${UR7E_TRACE_DIR:-$install_dir/logs/ur7e-agent-trace}" \
  --mcpclient.system-prompt="$agent_prompt"
