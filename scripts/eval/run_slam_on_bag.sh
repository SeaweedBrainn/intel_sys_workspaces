#!/usr/bin/env bash
# Point-LIO + RTAB-Map on a recorded bag, with or without the D435 (CAMERA=1).
#
# Usage (inside the container, workspace built):
#   [CAMERA=1] [START_OFFSET=s] bash scripts/eval/run_slam_on_bag.sh <bag_dir> <out_prefix> [rate] [extra localization args]
#
# Plays the L2 cloud and IMU (and with CAMERA=1 the color image, the depth aligned
# to color and the camera info) with --clock. The robot description gives the
# sensor frames, camera optical frames included; the bag's /tf_static is not played
# (it has an older LiDAR mount). Records Point-LIO's raw odometry and RTAB-Map's optimized path, then
# appends RTAB-Map's link counts from the database to <out_prefix>.log.
set -uo pipefail
BAG=$1; OUT=$2; RATE=${3:-1.0}; shift 3 2>/dev/null || shift $#
WS=/workspaces/intel_sys
set +u
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
[[ -f /opt/intel_sys/third_party/local_setup.bash ]] && source /opt/intel_sys/third_party/local_setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-46}
cd "$WS"
mkdir -p "$(dirname "$OUT")"
DB="/tmp/$(basename "$OUT").db"
CAM=false; TOPICS="/lidar/points_raw /lidar/imu"
if [[ "${CAMERA:-0}" == "1" ]]; then
  CAM=true; TOPICS="$TOPICS /camera/image_raw /camera/depth/image_raw /camera/camera_info"
fi

cleanup() {
  kill -INT "$EVAL" "$LOC" "$DESC" 2>/dev/null; sleep 4
  pkill -9 -f "pointlio_mapping|point_lio_frame_adapter|lidar_body_filter|robot_state_publisher|rtabmap_slam/rtabmap" 2>/dev/null
}
trap cleanup EXIT

ros2 launch intel_sys_description robot_description.launch.py use_sim_time:=true > "$OUT.desc.log" 2>&1 &
DESC=$!
ros2 launch intel_sys_localization localization.launch.py use_sim_time:=true slam:=true slam_camera:=$CAM \
  database_path:="$DB" "$@" > "$OUT.loc.log" 2>&1 &
LOC=$!
python3 -u scripts/eval/lio_bag_eval.py --out "$OUT" --title "$(basename "$BAG") Point-LIO + RTAB-Map camera=$CAM $*" \
  --path-topic /rtabmap/mapPath > "$OUT.log" 2>&1 &
EVAL=$!
sleep 10
ros2 bag play "$BAG" --clock 200 -r "$RATE" --start-offset "${START_OFFSET:-0}" --topics $TOPICS > "$OUT.play.log" 2>&1
sleep 8
kill -INT "$EVAL" 2>/dev/null; wait "$EVAL" 2>/dev/null
pkill -INT -f "rtabmap_slam/rtabmap" 2>/dev/null; sleep 6   # let it write the database
python3 - "$DB" >> "$OUT.log" <<'PY'
import sqlite3
import sys
try:
    c = sqlite3.connect(sys.argv[1])
    nodes = c.execute('select count(*) from Node').fetchone()[0]
    types = dict(c.execute('select type, count(*) from Link group by type').fetchall())
    words = c.execute('select count(*) from Word').fetchone()[0]
    print(f'database: {nodes} nodes, {words} visual words, links by type {types} '
          f'(0 neighbor, 1 visual loop closure, 2 proximity)')
except Exception as e:  # noqa: BLE001
    print(f'database not readable: {e}')
PY
