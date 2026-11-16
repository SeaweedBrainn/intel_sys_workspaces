#!/usr/bin/env bash
# Start the robot dashboard in the rover container (host networking, Zenoh overlay).
#
# Usage:
#   scripts/dashboard.sh            # start in background, print URL and token
#   scripts/dashboard.sh --stop     # stop it (zeroes the wheels on the way out)
#
# Reach it over Tailscale at http://<tailscale-ip>:${DASHBOARD_PORT:-8080}/ ,
# or over SSH: ssh -L 8080:localhost:8080 <user>@<robot>  then http://localhost:8080/
# The STM32 stack must already be running for live data and drive commands
# (see docs/TESTING.md).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
NAME=robot-dashboard
PORT="${DASHBOARD_PORT:-8080}"

if [[ "${1:-}" == "--stop" ]]; then
  docker stop -t 5 "$NAME" >/dev/null 2>&1 && echo "dashboard stopped" || echo "dashboard not running"
  exit 0
fi

docker rm -f "$NAME" >/dev/null 2>&1 || true

# Bind to the Tailscale address by default so only the tailnet can reach it,
# not every interface (the tailnet also includes other people's devices).
# Falls back to localhost if Tailscale is not up.
TS_IP="$(tailscale ip -4 2>/dev/null | head -1 || true)"
HOST_BIND="${DASHBOARD_HOST:-${TS_IP:-127.0.0.1}}"

docker compose -f docker-compose.yml -f docker-compose.zenoh.yml run -d --rm --name "$NAME" \
  -e DASHBOARD_HOST="$HOST_BIND" \
  -e DASHBOARD_PORT="$PORT" \
  ${DASHBOARD_TOKEN:+-e DASHBOARD_TOKEN="$DASHBOARD_TOKEN"} \
  intel-sys-rover bash -c 'source /opt/ros/humble/setup.bash && source /workspaces/intel_sys/install/setup.bash && exec python3 /workspaces/intel_sys/scripts/dashboard/dashboard.py' >/dev/null

sleep 6
echo "dashboard bound to ${HOST_BIND}:${PORT}"
echo "  tailnet:    http://${TS_IP:-<tailscale-ip>}:${PORT}/"
echo "  ssh tunnel: ssh -L ${PORT}:localhost:${PORT} <user>@<robot>  then http://localhost:${PORT}/"
echo "  token:      docker logs ${NAME} 2>&1 | grep token"
docker logs "$NAME" 2>&1 | grep -E "token|serving" || echo "dashboard still starting; run: docker logs $NAME"
