#!/usr/bin/env bash
# Visual-only RTAB-Map (D435 RGB-D: visual odometry + mapping, no LiDAR) on a recorded bag.
#
# Usage (inside the container, workspace built):
#   bash scripts/run_vslam_on_bag.sh <bag_dir> <out_prefix> [rate] [extra rtabmap.launch.py args...]
#
# Plays the color image, the depth aligned to color, the camera info and the bag's
# /tf_static (camera frames) with --clock, runs rtabmap_launch with visual_odometry,
# records the visual odometry and the final map with scripts/lio_bag_eval.py, and
# appends to <out_prefix>.log the loop closures stored in the database and how
# often visual odometry was lost. START_OFFSET skips into the bag.
set -uo pipefail
BAG=$1; OUT=$2; RATE=${3:-1.0}; shift 3 2>/dev/null || shift $#
WS=/workspaces/intel_sys
set +u
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-45}
cd "$WS"
mkdir -p "$(dirname "$OUT")"
DB="/tmp/$(basename "$OUT").db"
rm -f "$DB"

cleanup() {
  kill -INT "$EVAL" 2>/dev/null; wait "$EVAL" 2>/dev/null
  kill -INT "$SLAM" 2>/dev/null; sleep 4
  pkill -9 -f "rgbd_odometry|rtabmap_slam/rtabmap --ros|rgbd_sync" 2>/dev/null
}
trap cleanup EXIT

ros2 launch rtabmap_launch rtabmap.launch.py use_sim_time:=true \
  rgb_topic:=/camera/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/camera_info \
  frame_id:=camera_link approx_sync:=true approx_sync_max_interval:=0.02 rgbd_sync:=true \
  visual_odometry:=true rtabmap_viz:=false rviz:=false database_path:="$DB" args:="-d" "$@" \
  > "$OUT.slam.log" 2>&1 &
SLAM=$!
python3 -u scripts/lio_bag_eval.py --out "$OUT" --title "$(basename "$BAG") visual RGB-D rate $RATE $*" \
  --odom-topic /rtabmap/odom --cloud-topic /rtabmap/cloud_map --cloud-latest > "$OUT.log" 2>&1 &
EVAL=$!
sleep 10
ros2 bag play "$BAG" --clock 200 -r "$RATE" --start-offset "${START_OFFSET:-0}" \
  --topics /camera/image_raw /camera/depth/image_raw /camera/camera_info /tf_static > "$OUT.play.log" 2>&1
sleep 8
kill -INT "$EVAL" 2>/dev/null; wait "$EVAL" 2>/dev/null
kill -INT "$SLAM" 2>/dev/null; sleep 6
python3 - "$DB" "$OUT.slam.log" >> "$OUT.log" <<'EOF'
import re
import sqlite3
import sys

db, log = sys.argv[1], sys.argv[2]
try:
    c = sqlite3.connect(db)
    nodes = c.execute('select count(*) from Node').fetchone()[0]
    types = dict(c.execute('select type, count(*) from Link group by type').fetchall())
    print(f'database: {nodes} nodes, links by type {types} (1 = global loop closure, 2 = proximity)')
except Exception as e:  # noqa: BLE001
    print(f'database not readable: {e}')
q = [int(m.group(1)) for m in re.finditer(r'quality=(\d+)', open(log, errors='ignore').read())]
if q:
    lost = sum(1 for v in q if v == 0)
    print(f'visual odometry: {len(q)} frames, lost {lost} ({100 * lost / len(q):.0f}%), median inliers {sorted(q)[len(q) // 2]}')
EOF
