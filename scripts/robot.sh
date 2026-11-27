#!/usr/bin/env bash
# Run docker compose on the robot's services (docker-compose.robot.yml) from the repo root.
#
#   scripts/robot.sh up            # drivers, health, Foxglove + mapping and exploring:
#                                  #   Point-LIO, RTAB-Map, Nav2 on its map, explorer (idle until Explore)
#   scripts/robot.sh up follow     # drivers, health, Foxglove + tag following:
#                                  #   Point-LIO, Nav2 in odom on the live LiDAR, follower (off until Follow)
#   scripts/robot.sh up drivers    # drivers (STM32, LiDAR, camera), robot description, health, Foxglove
#   scripts/robot.sh ros2 topic hz /odom   # ros2 CLI on the robot's middleware
#   scripts/robot.sh <compose args>        # anything else, e.g. ps, logs -f lidar, down
#
# `up` and `up follow` switch between the two modes: the other mode's stacks are stopped
# first. Start either with the robot standing still. Map options for `up`:
#   SLAM_LOCALIZATION=true  re-localize on the saved map instead of mapping
#   SLAM_CONTINUE=true      keep mapping into the saved map
#   SLAM_CAMERA=false       RTAB-Map on the LiDAR alone
set -euo pipefail
cd "$(dirname "$0")/.."
export COMPOSE_FILE=docker-compose.robot.yml
AUTONOMY=(explorer follow navigation localization)   # both modes' services, dependants first

running() { docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null | grep -q true; }

up_drivers() {
  export FOXGLOVE_ADDRESS="${FOXGLOVE_ADDRESS:-$(tailscale ip -4 2>/dev/null | head -1 || true)}"
  docker compose up -d zenohd stm32 lidar camera description health foxglove
}

# The camera has just been started; wait until it publishes (RTAB-Map with the camera
# stops updating without frames, and the follower needs them).
wait_for_camera() {
  for _ in 1 2 3; do
    docker exec camerastack /ros_entrypoint_intel_sys.sh \
      timeout 15 ros2 topic echo --once --no-daemon /camera/camera_info >/dev/null 2>&1 && return 0
  done
  echo "The camera is not publishing /camera/camera_info; check 'scripts/robot.sh logs camera'." >&2
  [[ "$1" == explore ]] && echo "To map without it: SLAM_CAMERA=false scripts/robot.sh up" >&2
  exit 1
}

# Make room for a mode. A localization already running in this mode is kept (restarting
# it would reset odometry and, when mapping, start a new map). Anything else from either
# mode is removed, stopped containers included, so none comes back with the other
# mode's settings.
prepare_mode() {
  local slam=$1
  if running localization; then
    if docker inspect -f '{{join .Config.Cmd " "}}' localization | grep -q "slam:=$slam"; then
      return 0
    fi
    echo "switching mode: stopping ${AUTONOMY[*]}"
  fi
  docker compose --profile explore --profile follow rm -s -f "${AUTONOMY[@]}" >/dev/null 2>&1 || true
}

# A fresh mapping session (the default) makes RTAB-Map delete its database, so move the
# previous map aside first. Not when re-localizing or continuing on it, and not when
# localization is already running (it isn't restarted).
keep_previous_map() {
  DATA="${ROBOT_DATA_DIR:-data}"; mkdir -p "$DATA"
  [[ "${SLAM_LOCALIZATION:-false}" == true || "${SLAM_CONTINUE:-false}" == true ]] && return 0
  if [[ -f "$DATA/rtabmap.db" ]] && ! running localization; then
    mv "$DATA/rtabmap.db" "$DATA/rtabmap_$(date +%Y%m%d_%H%M%S).db"
    echo "previous map kept as $(ls -t "$DATA"/rtabmap_*.db | head -1)"
  fi
}

if [[ "${1:-}" == "up" ]]; then
  case "${2:-}" in
    drivers) up_drivers
             exit 0 ;;
    "")      up_drivers
             [[ "${SLAM_CAMERA:-true}" == false ]] || wait_for_camera explore
             prepare_mode true
             keep_previous_map
             export SLAM=true
             exec docker compose --profile explore up -d --no-recreate localization navigation explorer ;;
    follow)  up_drivers
             wait_for_camera follow
             prepare_mode false
             export SLAM=false
             exec docker compose --profile follow up -d --no-recreate localization navigation follow ;;
    *)       echo "unknown mode '$2': use 'up', 'up follow' or 'up drivers'" >&2
             exit 2 ;;
  esac
fi
# ros2 inside the LiDAR container (Zenoh settings, workspace sourced), e.g.
#   scripts/robot.sh ros2 topic hz /lidar/points_raw
if [[ "${1:-}" == "ros2" ]]; then
  exec docker compose exec lidar /ros_entrypoint_intel_sys.sh "$@"
fi
exec docker compose "$@"
