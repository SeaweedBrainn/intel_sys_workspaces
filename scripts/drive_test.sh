#!/usr/bin/env bash
# Short, speed-capped drive test. MOVES THE ROBOT.
#
# Safety:
#   - Refuses to run without --confirm-wheels-free.
#   - Hard cap: linear speed <= 0.15 m/s (MAX_V), duration <= 2 s (MAX_T).
#   - A trap publishes zero Twist on any exit, including Ctrl-C and errors.
#
# Usage (inside container):
#   ./run_docker.sh bash scripts/drive_test.sh --confirm-wheels-free
#   ./run_docker.sh bash scripts/drive_test.sh --confirm-wheels-free --speed 0.10 --duration 1.5
#
# Checks: /odom/wheel reports displacement in the commanded direction.
set -uo pipefail

MAX_V=0.15
MAX_T=2.0
SPEED=0.10
DURATION=1.5
CONFIRMED=0
ODOM_TOPIC="${ODOM_TOPIC:-/odom/wheel}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --confirm-wheels-free) CONFIRMED=1; shift ;;
    --speed) SPEED="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    *) echo "unknown arg: $1"; exit 2 ;;
  esac
done

if [[ "$CONFIRMED" -ne 1 ]]; then
  echo "Refusing to move the robot. Re-run with --confirm-wheels-free after"
  echo "confirming the wheels are clear (or the robot is suspended and clear)."
  exit 2
fi

# Enforce caps regardless of what was passed.
awk "BEGIN{exit !($SPEED <= $MAX_V && $SPEED > 0)}" || { echo "speed must be in (0, $MAX_V]"; exit 2; }
awk "BEGIN{exit !($DURATION <= $MAX_T && $DURATION > 0)}" || { echo "duration must be in (0, $MAX_T]"; exit 2; }

cd "${WS:-/workspaces/intel_sys}" || exit 1
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
[[ -f install/setup.bash ]] && source install/setup.bash
set -u

stop_robot() {
  # Publish several zero commands so a dropped message cannot leave motion running.
  for _ in 1 2 3; do
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1
  done
  echo "[INFO] cmd_vel zeroed"
}
trap stop_robot EXIT INT TERM

# Capture start odometry.
START="$(timeout 5 ros2 topic echo --once "$ODOM_TOPIC" 2>/dev/null | grep -A3 'position:' | grep 'x:' | head -1 | awk '{print $2}')"
if [[ -z "$START" ]]; then
  echo "[FAIL] no odometry on $ODOM_TOPIC; aborting before motion"
  exit 1
fi
echo "[INFO] start x=$START on $ODOM_TOPIC; driving +x at ${SPEED} m/s for ${DURATION}s"

# Publish at 10 Hz for DURATION seconds, then stop.
COUNT=$(awk "BEGIN{printf \"%d\", $DURATION * 10}")
timeout "$(awk "BEGIN{print $DURATION + 3}")" ros2 topic pub -r 10 -t "$COUNT" /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: $SPEED, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
sleep 1

END="$(timeout 5 ros2 topic echo --once "$ODOM_TOPIC" 2>/dev/null | grep -A3 'position:' | grep 'x:' | head -1 | awk '{print $2}')"
DX="$(awk "BEGIN{print $END - $START}")"
EXPECTED="$(awk "BEGIN{print $SPEED * $DURATION}")"
echo "[INFO] dx=${DX} m, expected ~${EXPECTED} m"

# Pass if displacement is positive and within 50% of expected.
if awk "BEGIN{exit !($DX > 0 && $DX > 0.5 * $EXPECTED && $DX < 1.5 * $EXPECTED)}"; then
  echo "[PASS] wheel odometry tracked commanded motion"
  exit 0
fi
echo "[FAIL] wheel odometry displacement ${DX} m outside tolerance"
exit 1
