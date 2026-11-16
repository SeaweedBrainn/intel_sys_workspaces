#!/usr/bin/env bash
# Compare RMW implementations with the same latency probe.
#
# Runs latency_probe under rmw_zenoh_cpp (env from docker/zenoh.env) and
# rmw_cyclonedds_cpp, writing one JSON per (mode, rmw) to results/.
#
# Usage (inside the container, with the robot or sim running):
#   ./run_docker.sh bash scripts/compare_rmw.sh loopback
#   ./run_docker.sh bash scripts/compare_rmw.sh stamp --topic /imu_raw --type sensor_msgs/msg/Imu
#
# Env:
#   SECONDS_PER_RUN   default 10
#   ZENOH_ENV         default docker/zenoh.env (must exist; owned by the Zenoh track)
#   RESULTS_DIR       default results/ at repo root
set -uo pipefail

REPO_ROOT="${REPO_ROOT:-/workspaces/intel_sys}"
PROBE="$REPO_ROOT/scripts/latency_probe/latency_probe.py"
RESULTS_DIR="${RESULTS_DIR:-$REPO_ROOT/results}"
ZENOH_ENV="${ZENOH_ENV:-$REPO_ROOT/docker/zenoh.env}"
SECONDS_PER_RUN="${SECONDS_PER_RUN:-10}"

[[ $# -ge 1 ]] || { echo "usage: $0 <probe-mode> [probe args...]"; exit 2; }
MODE="$1"; shift
PROBE_ARGS=("$MODE" --seconds "$SECONDS_PER_RUN" "$@")
TAG="$(date +%Y%m%d-%H%M%S)"
mkdir -p "$RESULTS_DIR"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"
set -u

run_one() {
  local rmw="$1" out="$2"
  echo "== $rmw ($MODE) -> $out =="
  # Each RMW runs in a subshell so environment changes do not leak between runs.
  (
    if [[ "$rmw" == "rmw_zenoh_cpp" ]]; then
      if [[ ! -f "$ZENOH_ENV" ]]; then
        echo "[FAIL] $ZENOH_ENV missing; cannot run zenoh leg"
        exit 3
      fi
      set -a; source "$ZENOH_ENV"; set +a
    fi
    export RMW_IMPLEMENTATION="$rmw"
    python3 "$PROBE" "${PROBE_ARGS[@]}" --out "$out"
  )
  local rc=$?
  if [[ $rc -eq 0 ]]; then echo "[PASS] $rmw run ok"; else echo "[FAIL] $rmw run rc=$rc"; fi
  return $rc
}

FAILED=0
run_one rmw_zenoh_cpp      "$RESULTS_DIR/${TAG}_${MODE}_zenoh.json"    || FAILED=1
run_one rmw_cyclonedds_cpp "$RESULTS_DIR/${TAG}_${MODE}_cyclonedds.json" || FAILED=1

# Side-by-side summary of the p50/p95/p99 fields.
echo
echo "== summary ($TAG, $MODE) =="
python3 - "$RESULTS_DIR/${TAG}_${MODE}_zenoh.json" "$RESULTS_DIR/${TAG}_${MODE}_cyclonedds.json" <<'PY'
import json, os, sys
rows = []
for path in sys.argv[1:]:
    if not os.path.exists(path):
        rows.append((os.path.basename(path), "missing"))
        continue
    d = json.load(open(path))
    rows.append((d["rmw"], f'n={d["samples"]} rate={d["rate_hz"]}Hz '
                 f'p50={d["p50_ms"]} p95={d["p95_ms"]} p99={d["p99_ms"]} max={d["max_ms"]} ms'))
for name, text in rows:
    print(f"{name:22s} {text}")
PY

exit "$FAILED"
