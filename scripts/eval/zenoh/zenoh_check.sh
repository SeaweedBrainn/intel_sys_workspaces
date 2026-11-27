#!/bin/bash
# Zenoh middleware checks for the two open bugs in docs/ROADMAP.md:
#   1. subscribers that stop receiving after their publisher (or the router) restarts
#   2. a subscription that goes deaf while the same process receives heavy traffic
# Runs its own router on port 7457 with copies of docker/zenoh/*.json5, so the robot's
# router and nodes are not touched. Run it inside any stack container, e.g.
#   docker exec -it localization bash /workspaces/intel_sys/scripts/eval/zenoh/zenoh_check.sh
# Each line prints messages per second; a healthy run shows 10 (pubsub) or 10/12 (heavy)
# after every restart. On x86 (sim container, rmw_zenoh_cpp 0.1.11) all checks pass.
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)
source /ros_entrypoint_intel_sys.sh 2>/dev/null || source /opt/ros/humble/setup.bash
if (exec 3<>/dev/tcp/127.0.0.1/7457) 2>/dev/null; then
  echo "port 7457 is in use (a router left from an earlier run?); stop it first"; exit 1
fi
W=$(mktemp -d /tmp/zenoh_check.XXXX)
sed 's#tcp/127.0.0.1:7447#tcp/127.0.0.1:7457#' "$REPO/docker/zenoh/router.json5" > "$W/router.json5"
sed 's#tcp/127.0.0.1:7447#tcp/127.0.0.1:7457#' "$REPO/docker/zenoh/session.json5" > "$W/shm_on.json5"
sed 's/shared_memory: { enabled: true }/shared_memory: { enabled: false }/' "$W/shm_on.json5" > "$W/shm_off.json5"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ZENOH_ROUTER_CONFIG_URI="$W/router.json5" ZENOH_ROUTER_CHECK_ATTEMPTS=5
cd "$W"
echo "rmw_zenoh_cpp: $(dpkg-query -W -f='${Version}' ros-humble-rmw-zenoh-cpp 2>/dev/null)  arch: $(uname -m)  work dir: $W"

# The router binary itself, not `ros2 run` (whose PID is a wrapper), so only our router is killed.
ZENOHD="$(ros2 pkg prefix rmw_zenoh_cpp)/lib/rmw_zenoh_cpp/rmw_zenohd"
router() { "$ZENOHD" > "router_$1.log" 2>&1 & ROUTER=$!; sleep 3; }
stop_router() { kill -9 "$ROUTER" 2>/dev/null; wait "$ROUTER" 2>/dev/null; }
last() { for q in rel be tl; do printf '  %-3s %s msg/s\n' "$q" "$(tail -1 "sub_$q.log" | awk '{print $3}')"; done; }
pubs() { for q in rel be tl; do python3 "$HERE/pubsub.py" pub "$q" > /dev/null 2>&1 & done; }
trap 'pkill -f "$HERE/pubsub.py"; pkill -f "$HERE/heavy.py"; [ -n "${ROUTER:-}" ] && kill -9 "$ROUTER" 2>/dev/null' EXIT

for shm in on off; do
  export ZENOH_SESSION_CONFIG_URI="$W/shm_$shm.json5"
  echo; echo "===== shared memory $shm"
  router "$shm"
  for q in rel be tl; do python3 "$HERE/pubsub.py" sub "$q" > "sub_$q.log" 2>&1 & done
  sleep 3; pubs; sleep 6
  echo "publishers up:"; last
  for sig in INT KILL; do
    pkill -"$sig" -f "$HERE/pubsub.py pub"; sleep 3; pubs; sleep 8
    echo "publishers restarted after SIG$sig:"; last
  done
  stop_router; sleep 4; router "${shm}_2"; sleep 12
  echo "router restarted:"; last
  pkill -f "$HERE/pubsub.py"; sleep 1

  python3 "$HERE/heavy.py" sub > heavy.log 2>&1 &
  sleep 2; python3 "$HERE/heavy.py" pub 4915200 10 > /dev/null 2>&1 &
  sleep 25
  tail -n 20 heavy.log | awk '{c+=$2; l+=$4; n++} END {printf "heavy traffic (50 MB/s camera-sized + LiDAR-sized cloud, one subscriber): cam %.1f Hz, lidar %.1f Hz\n", c/n, l/n}'
  pkill -f "$HERE/heavy.py"; stop_router; sleep 2
  grep -h -i -E "shm|segment|error" router_${shm}*.log 2>/dev/null | cut -c1-200 | sort | uniq -c | head -5
done
