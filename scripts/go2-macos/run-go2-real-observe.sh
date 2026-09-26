#!/bin/zsh
set -euo pipefail

script_dir="${0:A:h}"
install_dir="${script_dir:h:h}"
source "${DIMOS_VENV_PATH:-$install_dir/.venv}/bin/activate"
cd "$install_dir"
export PYTHONPATH="$install_dir${PYTHONPATH:+:$PYTHONPATH}"
export DIMOS_SKIP_SYSTEM_CONFIG=1
# Keep local RPC and DeepSeek on the working direct path without changing
# macOS or Clash settings; retain any caller-specified bypass entries.
export NO_PROXY="127.0.0.1,localhost,api.deepseek.com${NO_PROXY:+,$NO_PROXY}"
export DEEPSEEK_API_KEY="$(security find-generic-password -s dimos-deepseek -w 2>/dev/null)"
if [[ -z "$DEEPSEEK_API_KEY" ]]; then
  print -u2 "Missing macOS Keychain service 'dimos-deepseek'."
  exit 1
fi

agent_prompt='You observe a PHYSICAL Go2 over Ethernet with movement LOCKED. Reply in the user language using text. You have observation, real pointcloud health, native dimOS costmap inspection and PATH PREVIEW ONLY. preview_relative_path can display a proposed path from current real pose using forward/left distances in meters, but cannot execute it. Inspect navigation_preview_status before planning. Never claim movement, arrival or successful VLN. Report blocked/unknown/stale starts honestly; never clear unknown space. Read battery via get_battery_soc and visual facts via observe; never guess absent data. Do not call observe or send camera images to the model until the user explicitly authorizes sharing PHYSICAL camera images with DeepSeek. Local video preview is independent of this permission. No simulation memory, saved apartment map or invented world coordinates may be used. No speech, voice input or autonomous monitoring.'
if [[ "${2:-}" == --allow-vision ]]; then
  agent_prompt+=' The operator has explicitly authorized sharing PHYSICAL Go2 camera images with DeepSeek for requested visual questions; use observe to answer them. This grants no movement permission.'
elif [[ -n "${2:-}" ]]; then
  print -u2 'Usage: run-go2-real-observe.sh [robot-ip] [--allow-vision]'
  exit 2
fi

exec dimos --transport zenoh --viewer none --simulation='' --no-replay \
  --robot-ip "${1:-192.168.123.161}" --robot-id go2-edu-real-sensors \
  run --daemon unitree-go2-observe-cockpit \
  --go2connection.read-only=true \
  --go2connection.lidar=false \
  --go2ros2lidar.control-path="${DIMOS_GO2_SSH_CONTROL_PATH:-/private/tmp/dimos-go2-edu-readonly.sock}" \
  --voxelgridmapper.device=CPU:0 \
  --open-browser=false \
  --mcpclient.model=deepseek-flash \
  --mcpclient.model-base-url=https://api.deepseek.com \
  --mcpclient.model-api-key-env=DEEPSEEK_API_KEY \
  --mcpclient.model-use-responses-api=true \
  --mcpclient.excluded-tools='["agent_send"]' \
  --mcpclient.trace-dir="$install_dir/artifacts/real-observe-agent-trace" \
  --mcpclient.system-prompt="$agent_prompt"
