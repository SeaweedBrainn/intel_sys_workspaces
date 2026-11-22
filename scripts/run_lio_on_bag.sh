#!/usr/bin/env bash
# Run Point-LIO (+ body filter + frame adapter) on a recorded bag and evaluate it.
#
# Usage (inside the container, workspace built):
#   bash scripts/run_lio_on_bag.sh <bag_dir> <out_prefix> [rate] [extra ros2 launch args...]
#   e.g. bash scripts/run_lio_on_bag.sh bags/sensor_data_..._orientation_3 results/lio_o3
#
# Plays only /lidar/points_raw and /lidar/imu with --clock, runs the robot
# description (for base_footprint -> lidar_link) and localization.launch.py
# with use_sim_time, records with scripts/lio_bag_eval.py, and writes
# <out_prefix>.png/.npz plus <out_prefix>.log. Point-LIO starts fresh each run.
set -uo pipefail
BAG=$1; OUT=$2; RATE=${3:-1.0}; shift 3 2>/dev/null || shift $#
WS=/workspaces/intel_sys
set +u
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
[[ -f /opt/intel_sys/third_party/local_setup.bash ]] && source /opt/intel_sys/third_party/local_setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43}
cd "$WS"
mkdir -p "$(dirname "$OUT")"

cleanup() {
  kill -INT "$EVAL" 2>/dev/null; wait "$EVAL" 2>/dev/null
  kill -INT "$LOC" "$DESC" 2>/dev/null; sleep 3
  pkill -9 -f "pointlio_mapping|point_lio_frame_adapter|lidar_body_filter|robot_state_publisher|icp_odometry" 2>/dev/null
}
trap cleanup EXIT

ros2 launch intel_sys_description robot_description.launch.py use_sim_time:=true > "$OUT.desc.log" 2>&1 &
DESC=$!
ros2 launch intel_sys_localization localization.launch.py use_sim_time:=true "$@" > "$OUT.loc.log" 2>&1 &
LOC=$!
# EVAL_ARGS: extra recorder options, e.g. for odometry:=icp
#   EVAL_ARGS="--odom-topic /odom --cloud-topic /odom_local_scan_map"
python3 -u scripts/lio_bag_eval.py --out "$OUT" --title "$(basename "$BAG") rate $RATE $*" ${EVAL_ARGS:-} > "$OUT.log" 2>&1 &
EVAL=$!
sleep 8
ros2 bag play "$BAG" --clock 200 -r "$RATE" --start-offset "${START_OFFSET:-0}" --topics /lidar/points_raw /lidar/imu > "$OUT.play.log" 2>&1
sleep 5
