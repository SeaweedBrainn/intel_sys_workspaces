#!/usr/bin/env bash
# Print ready-to-open dashboard links for this robot.
#
# Run on the robot, or from your laptop over ssh:
#   ssh jetson '~/intel_sys_workspaces/scripts/dashboard_url.sh'
#
# Usage: dashboard_url.sh [control|view|both]   (default: both)
#   control  full access: drive, path, autonomy, e-stop
#   view     read-only: status and map
set -euo pipefail

PORT="${DASHBOARD_PORT:-8080}"
TS_IP="$(tailscale ip -4 2>/dev/null | head -1 || true)"
HOST="${DASHBOARD_LINK_HOST:-${TS_IP:-127.0.0.1}}"
CONTROL_FILE="${DASHBOARD_TOKEN_FILE:-$HOME/.config/robot-dashboard/token}"
VIEW_FILE="$(dirname "$CONTROL_FILE")/view_token"

show() {  # label, token file
  if [[ ! -s "$2" ]]; then
    echo "$1: no token yet. Run scripts/dashboard.sh once to create it."
    return
  fi
  echo "$1: http://${HOST}:${PORT}/?token=$(cat "$2")"
}

case "${1:-both}" in
  control) show control "$CONTROL_FILE" ;;
  view) show view "$VIEW_FILE" ;;
  both) show control "$CONTROL_FILE"; show view "$VIEW_FILE" ;;
  *) echo "usage: $0 [control|view|both]" >&2; exit 2 ;;
esac
