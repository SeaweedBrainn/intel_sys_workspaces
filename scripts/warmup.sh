#!/usr/bin/env bash
# On-robot warmup. Run inside the rover container. READ-ONLY: this script
# never publishes motion commands. Motion lives in scripts/drive_test.sh.
#
# Usage (host):   ./run_docker.sh bash scripts/warmup.sh
# Each stage prints PASS / FAIL / SKIP. SKIP = device absent, not broken.
# Exits nonzero only if a stage FAILs.
set -uo pipefail

WS="${WS:-/workspaces/intel_sys}"
RATE_WINDOW="${RATE_WINDOW:-3}"   # seconds to sample topic rates
cd "$WS" 2>/dev/null || true
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
[[ -f install/setup.bash ]] && source install/setup.bash
set -u

FAILS=0
pass() { echo "[PASS] $1"; }
fail() { echo "[FAIL] $1"; FAILS=$((FAILS + 1)); }
skip() { echo "[SKIP] $1"; }

topic_exists() { ros2 topic list 2>/dev/null | grep -qx "$1"; }

# Hz over RATE_WINDOW seconds. Prints a number or nothing.
topic_hz() {
  timeout "$((RATE_WINDOW + 4))" ros2 topic hz "$1" --window 50 2>/dev/null \
    | grep -m1 'average rate' | awk '{print $3}'
}

# Start the hardware bridge only if the serial link exists. Launched in the
# background so stage (a) can observe it; killed on exit.
BRIDGE_PID=""
cleanup() { [[ -n "$BRIDGE_PID" ]] && kill "$BRIDGE_PID" 2>/dev/null; }
trap cleanup EXIT

echo "== Stage a: STM32 serial and bridge =="
if [[ -e /dev/rrc ]]; then
  if ! ros2 node list 2>/dev/null | grep -q ros_robot_controller; then
    ros2 run intel_sys_hardware stm32_bridge --ros-args \
      -r __node:=ros_robot_controller -p port:=/dev/rrc -p baudrate:=1000000 \
      > /tmp/warmup_bridge.log 2>&1 &
    BRIDGE_PID=$!
    sleep 5
  fi
  if ros2 node list 2>/dev/null | grep -q ros_robot_controller; then
    pass "stm32 bridge node running (/dev/rrc present)"
  else
    fail "stm32 bridge did not start; see /tmp/warmup_bridge.log"
  fi
else
  fail "/dev/rrc missing inside container (check compose /dev mount and udev)"
fi

echo "== Stage b: IMU rate (imu_raw) =="
if topic_exists /imu_raw || topic_exists /ros_robot_controller/imu_raw; then
  t=$(ros2 topic list | grep -m1 'imu_raw$')
  hz="$(topic_hz "$t")"
  # The bridge timer is 0.02 s (50 Hz nominal); accept >= 90% of nominal.
  IMU_MIN_HZ="${IMU_MIN_HZ:-45}"
  if [[ -n "$hz" ]] && awk "BEGIN{exit !($hz >= $IMU_MIN_HZ)}"; then
    pass "$t rate ${hz} Hz (min ${IMU_MIN_HZ})"
  elif [[ -n "$hz" ]]; then
    fail "$t rate ${hz} Hz below ${IMU_MIN_HZ} Hz"
  else
    fail "$t exists but no messages in ${RATE_WINDOW}s"
  fi
else
  skip "no imu_raw topic (stm32 bridge not running)"
fi

echo "== Stage c: motor command path (no motion) =="
if topic_exists /cmd_vel && topic_exists /set_motor; then
  pass "cmd_vel and set_motor topics present (commands available; not sent)"
else
  skip "cmd_vel/set_motor not both present; start mecanum_controller to enable"
fi

echo "== Stage d: TF completeness =="
TF_OUT="$(timeout 8 ros2 run tf2_ros tf2_echo base_footprint imu_link 2>&1 | head -20)"
if echo "$TF_OUT" | grep -q "Translation"; then
  pass "base_footprint -> imu_link resolves"
else
  skip "base_footprint -> imu_link not resolved (robot_state_publisher not running?)"
fi
for f in odom; do
  TF_OUT="$(timeout 8 ros2 run tf2_ros tf2_echo "$f" base_footprint 2>&1 | head -20)"
  if echo "$TF_OUT" | grep -q "Translation"; then
    pass "$f -> base_footprint resolves"
  else
    skip "$f -> base_footprint not resolved (no odometry source yet)"
  fi
done

echo "== Stage e: LiDAR and camera rates (SKIP if absent) =="
if topic_exists /lidar/points_raw; then
  hz="$(topic_hz /lidar/points_raw)"
  [[ -n "$hz" ]] && pass "/lidar/points_raw ${hz} Hz" || fail "/lidar/points_raw present, no data"
else
  skip "/lidar/points_raw absent (LiDAR offline)"
fi
if topic_exists /camera/image_raw; then
  hz="$(topic_hz /camera/image_raw)"
  [[ -n "$hz" ]] && pass "/camera/image_raw ${hz} Hz" || fail "/camera/image_raw present, no data"
else
  skip "/camera/image_raw absent (camera not launched or D435 not enumerated)"
fi

echo "== Stage f: mode report =="
MODES="wheel"
topic_exists /lidar/points_raw && MODES="$MODES lio"
topic_exists /camera/image_raw && MODES="$MODES vio"
if topic_exists /lidar/points_raw && topic_exists /camera/image_raw; then
  MODES="$MODES lvio"
fi
echo "[INFO] available localization modes: $MODES"
pass "mode report emitted"

echo
if [[ "$FAILS" -gt 0 ]]; then
  echo "warmup: $FAILS FAIL"
  exit 1
fi
echo "warmup: no FAILs"
exit 0
