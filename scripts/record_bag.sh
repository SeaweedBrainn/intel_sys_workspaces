#!/usr/bin/env bash
# Record a bag of the robot's topics into bags/<name>_<date>, until Ctrl-C.
#
#   scripts/record_bag.sh loop_test          # everything except localization outputs
#   scripts/record_bag.sh loop_test --all    # also Point-LIO and RTAB-Map outputs
#
# Runs in a one-off container on the rover image with the Zenoh settings, so it
# sees the robot's stacks. The drivers must be running (scripts/robot.sh up drivers, or any mode).
set -euo pipefail
cd "$(dirname "$0")/.."
NAME="${1:?usage: $0 <name> [--all]}"
OUT="bags/${NAME}_$(date +%Y%m%d_%H%M%S)"
EXCLUDE=(-x '/(point_lio|rtabmap)/.*')
[[ "${2:-}" == "--all" ]] && EXCLUDE=()
mkdir -p bags
# Detached with a fixed name, so Ctrl-C (or a kill of this script) stops the recorder
# with SIGINT and the bag is closed properly.
scripts/robot.sh run -d --rm --no-deps --name bag-recorder -v "$PWD/bags:/workspaces/intel_sys/bags" lidar \
  ros2 bag record -a "${EXCLUDE[@]}" -o "/workspaces/intel_sys/$OUT" >/dev/null
stop() { docker kill -s INT bag-recorder >/dev/null 2>&1 || true; }
trap stop INT TERM
echo "recording to $OUT (Ctrl-C to stop)"
docker logs -f bag-recorder 2>&1 | grep --line-buffered -v "Watchdog Confirmator" &
while docker inspect bag-recorder >/dev/null 2>&1; do sleep 1; done
echo "saved $OUT"
