#!/bin/zsh
set -euo pipefail

script_dir="${0:A:h}"
install_dir="${script_dir:h:h}"
source "${DIMOS_VENV_PATH:-$install_dir/.venv}/bin/activate"
cd "$install_dir"
export PYTHONPATH="$install_dir${PYTHONPATH:+:$PYTHONPATH}"
export DIMOS_SKIP_SYSTEM_CONFIG=1
export DEEPSEEK_API_KEY="$(security find-generic-password -s dimos-deepseek -w 2>/dev/null)"
if [[ -z "$DEEPSEEK_API_KEY" ]]; then
  print -u2 "Missing macOS Keychain service 'dimos-deepseek'."
  exit 1
fi

memory_dir="$install_dir/artifacts/apartment-memory"
agent_prompt='You operate a simulated Go2 in an apartment through dimOS native tools. Reply in the user language using text; speak is disabled. Always observe before visual questions or navigation. Use navigate_with_text for semantic targets, translating semantic memory queries to English descriptions for CLIP. Use tag_location only when the user requests a named location. Never invent target world coordinates, use simulator metadata as perception, or claim arrival merely because navigation was started. After starting navigation, use wait between navigation_status checks, and only report arrival when goal_reached is true and state is idle. Observe again to describe the actual destination. If still following a path, report it is in progress; stop_navigation cancels it. Only report visual facts from the latest image. You may use look_out_for when explicitly requested, then stop it when done. Never call agent_send, follow_person, or sport tricks. If semantic memory has no confident match, report that rather than guessing. Metric move_to uses relative x forward and y left. This is a capability test, not real hardware.'

typeset -a viewer_args blueprint_args
case "${1:-legacy}" in
  legacy)
    viewer_args=(--rerun-open web)
    blueprint_args=(unitree-go2-agentic --disable web-input)
    trace_dir="$install_dir/artifacts/apartment-agent-trace"
    ;;
  cockpit)
    viewer_args=(--viewer none --robot-id go2-apartment-sim)
    blueprint_args=(unitree-go2-agentic-cockpit --disable voice-input --disable websocket-vis-module --open-browser=false)
    trace_dir="$install_dir/artifacts/cockpit-agent-trace"
    ;;
  *)
    print -u2 'Usage: run-go2-capabilities.sh [legacy|cockpit]'
    exit 2
    ;;
esac

exec dimos \
  --transport lcm \
  "${viewer_args[@]}" \
  --simulation dimsim \
  --dimsim-scene apartment \
  --detection-model deepseek \
  run --daemon "${blueprint_args[@]}" \
  --disable speak-skill \
  --spatialmemory.embedding-providers='["CPUExecutionProvider"]' \
  --spatialmemory.min-distance-threshold=0.35 \
  --spatialmemory.min-rotation-threshold=0.35 \
  --spatialmemory.min-time-threshold=2 \
  --spatialmemory.db-path="$memory_dir/chroma" \
  --spatialmemory.visual-memory-path="$memory_dir/visual_memory.pkl" \
  --spatialmemory.output-dir="$memory_dir/images" \
  --spatialmemory.new-memory=false \
  --mcpclient.model=deepseek-flash \
  --mcpclient.model-base-url=https://api.deepseek.com \
  --mcpclient.model-api-key-env=DEEPSEEK_API_KEY \
  --mcpclient.model-use-responses-api=true \
  --mcpclient.excluded-tools='["agent_send","follow_person","execute_sport_command"]' \
  --mcpclient.trace-dir="$trace_dir" \
  --mcpclient.system-prompt="$agent_prompt"
