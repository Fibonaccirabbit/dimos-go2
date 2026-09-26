#!/bin/zsh
set -euo pipefail

script_dir="${0:A:h}"
install_dir="${script_dir:h:h}"
deepseek_key="$(security find-generic-password -s dimos-deepseek -w 2>/dev/null)"
agent_prompt='You control a simulated Unitree Go2 with a front camera. Reply concisely in the same language as the user. Use current_time for time questions. Always call observe before each visual query or visual navigation action, even if previous images are in the conversation: the scene may have changed. The observe image is delivered before your next model call; use its accompanying image immediately, without waiting or asking for another frame. Describe only what the latest image shows. Do not infer metric distances from a single image. For visual navigation, use the requested small metric step and choose its direction from the latest image; move_to relative x is forward and relative y is left. After moving, call observe once to verify the new view, then report the outcome and finish. For metric navigation commands call move_to. Never call agent_send. Confirm a movement only after its tool returns and report actual coordinates rather than assuming the requested displacement was exact.'

if [[ -z "$deepseek_key" ]]; then
  print -u2 "DeepSeek API key is missing from the macOS Keychain service 'dimos-deepseek'."
  exit 1
fi

export DEEPSEEK_API_KEY="$deepseek_key"
unset deepseek_key

exec zsh "$script_dir/run-go2-sim.sh" \
  unitree-go2 \
  mcp-server \
  unitree-skill-container \
  observe-skill \
  mcp-client \
  --mcpclient.model=deepseek-flash \
  --mcpclient.model-base-url=https://api.deepseek.com \
  --mcpclient.model-api-key-env=DEEPSEEK_API_KEY \
  --mcpclient.model-use-responses-api=true \
  --mcpclient.excluded-tools='["agent_send"]' \
  --mcpclient.trace-dir="$install_dir/artifacts/vision-trace" \
  --mcpclient.system-prompt="$agent_prompt"
