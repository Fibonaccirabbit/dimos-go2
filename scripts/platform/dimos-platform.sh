#!/bin/zsh
# One entry point for every adapted robot. Each target starts a dimOS cockpit
# (camera, map/state, Agent chat) at http://127.0.0.1:7780/ and opens it in Chrome.
#
#   dimos-platform.sh go2-sim                       Go2 in the DimSim apartment (navigation, memory, Agent)
#   dimos-platform.sh go2-real [--allow-vision]     Physical Go2 EDU over Ethernet, sensors only
#   dimos-platform.sh ur7e [--allow-motion] [--allow-vision]   UR7e arm via the Ubuntu workstation
#   dimos-platform.sh stop | status
set -euo pipefail

script_dir="${0:A:h}"
install_dir="${script_dir:h:h}"
scripts="$install_dir/scripts"
dimos="${DIMOS_VENV_PATH:-$install_dir/.venv}/bin/dimos"
url="http://127.0.0.1:7780/"
target="${1:-}"
(( $# )) && shift

stop_running() {
  "$dimos" stop >/dev/null 2>&1 || true
}

wait_and_open() {
  for _ in {1..90}; do
    if curl -s --noproxy '*' -o /dev/null --max-time 1 "$url"; then
      # The cockpit streams over WebTransport, which Safari lacks: use Chrome.
      if [[ -d "/Applications/Google Chrome.app" ]]; then
        open -a "Google Chrome" "$url"
      else
        print -u2 "Open $url in Chrome or Edge (Safari does not support WebTransport)."
      fi
      print "Cockpit ready: $url"
      return 0
    fi
    sleep 2
  done
  print -u2 "Cockpit did not come up; check: $dimos log"
  return 1
}

case "$target" in
  go2-sim)
    stop_running
    zsh "$scripts/go2-macos/run-go2-capabilities.sh" cockpit
    ;;
  go2-real)
    stop_running
    zsh "$scripts/go2-macos/run-go2-real-observe.sh" 192.168.123.161 "$@"
    ;;
  ur7e)
    stop_running
    zsh "$scripts/ur7e/run-ur7e-cockpit.sh" "$@"
    ;;
  stop)
    stop_running
    print "Stopped."
    exit 0
    ;;
  status)
    exec "$dimos" status
    ;;
  *)
    sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
wait_and_open
